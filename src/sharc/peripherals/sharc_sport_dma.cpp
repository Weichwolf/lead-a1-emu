// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_sport_dma.hpp"

#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace lead_a1 {
namespace {

constexpr std::uint32_t index_mask = 0x0fffffff;
constexpr std::uint32_t chain_mask = 0x1fffffff;
constexpr std::uint32_t pci = 0x80000;
constexpr std::uint32_t internal_bits = 0x7ffff;
constexpr unsigned int chain_clocks = 26;
constexpr std::uint32_t full_count = 65536;
constexpr std::uint32_t short_mask = 0xffff;

SharcClockGate gate(unsigned int sport) {
  constexpr std::array gates{SharcClockGate::sport01,
                             SharcClockGate::sport23,
                             SharcClockGate::sport45,
                             SharcClockGate::sport67};
  if (sport >= gates.size() * 2) { throw std::invalid_argument("Invalid SPORT DMA port identity"); }
  return gates[sport / 2];
}

std::uint32_t chain_address(std::uint32_t chain) {
  // PCI occupies encoded bit19; physical address bits27..19 live in28..20.
  constexpr std::uint32_t high = 0x1ff00000;
  const auto address = (chain & internal_bits) | ((chain & high) >> 1);
  return address == 0 || (chain & high) != 0 ? address : address | pci;
}

std::uint32_t count_value(std::uint16_t count) {
  return count == 0 ? full_count : count;
}

void validate_load(const SharcSportDmaState &state) {
  if (state.load) {
    const auto load = *state.load;
    if (state.active || !state.mode.enabled || !state.mode.chained || state.insertion ||
        load.received > load.words.size() || load.elapsed_cclk > chain_clocks ||
        (load.received == load.words.size() && load.elapsed_cclk == chain_clocks) ||
        load.index_address < pci || load.index_address > index_mask || !state.chain ||
        load.index_address != chain_address(state.chain.value_or(0))) {
      throw std::invalid_argument("Invalid SPORT DMA chain loading state");
    }
    for (unsigned int index = load.received; index < load.words.size(); ++index) {
      if (load.words[index] != 0) {
        throw std::invalid_argument("Invalid uncaptured SPORT DMA chain word");
      }
    }
  }
}

void validate(const SharcSportDmaState &state) {
  validate_load(state);
  if ((state.index && *state.index > index_mask) || (state.chain && *state.chain > chain_mask) ||
      state.remaining > full_count) {
    throw std::invalid_argument("Invalid SPORT DMA parameter width");
  }
  if (state.active && (!state.index || !state.modifier || !state.count || state.remaining == 0 ||
                       (!state.mode.enabled && !state.insertion))) {
    throw std::invalid_argument("Invalid active SPORT DMA parameters");
  }
  if ((!state.active && state.remaining != 0) ||
      (state.active && state.count != static_cast<std::uint16_t>(state.remaining & short_mask)) ||
      (state.insertion != (state.mode.chained && !state.mode.enabled))) {
    throw std::invalid_argument("Invalid SPORT DMA transfer phase");
  }
  if (state.mode.chained && !state.chain) {
    throw std::invalid_argument("Missing SPORT DMA chain pointer");
  }
  if ((state.transmit_buffer || state.transmit_shift) && !state.mode.transmit) {
    throw std::invalid_argument("Invalid SPORT DMA transmit ownership");
  }
  if (state.receive_buffer && state.mode.transmit) {
    throw std::invalid_argument("Invalid SPORT DMA receive ownership");
  }
}

unsigned int parameter(MemoryAccess access) {
  constexpr unsigned int bytes = 4;
  constexpr unsigned int span = 16;
  if (access.size != bytes || access.address % bytes != 0 || access.address >= span) {
    throw std::runtime_error("SPORT DMA parameters require aligned32-bit offsets0..12");
  }
  return static_cast<unsigned int>(access.address / bytes);
}

void start_standard(SharcSportDmaState &state) {
  if (!state.index || !state.modifier || !state.count) {
    throw std::runtime_error("Unknown SPORT DMA index/modifier/count");
  }
  state.remaining = count_value(*state.count);
  state.active = true;
}

void start_chain(SharcSportDmaState &state) {
  if (!state.chain) { throw std::runtime_error("Unknown SPORT DMA chain pointer"); }
  const auto address = chain_address(*state.chain);
  if (address != 0) { state.load = SharcSportDmaLoad{.index_address = address}; }
}

void finish_load(SharcSportDmaState &state) {
  if (!state.load || state.load->received != 4 || state.load->elapsed_cclk != chain_clocks) {
    return;
  }
  const auto words = state.load->words;
  state.index = words[0] & index_mask;
  state.modifier = static_cast<std::uint16_t>(words[1] & short_mask);
  state.count = static_cast<std::uint16_t>(words[2] & short_mask);
  state.chain = words[3] & chain_mask;
  state.load.reset();
  start_standard(state);
}

std::uint32_t current_chain(const SharcSportDmaState &state) {
  if (!state.mode.chained) { return 0; }
  if (!state.chain) { throw std::logic_error("Missing active SPORT DMA chain pointer"); }
  return *state.chain;
}

bool finish_word(SharcSportDmaState &state) {
  constexpr std::int64_t ring = 0x10000000;
  const auto index = state.index;
  const auto raw_modifier = state.modifier;
  if (!index || !raw_modifier || state.remaining == 0) {
    throw std::logic_error("Missing active SPORT DMA address/count");
  }
  const auto chain = current_chain(state);
  const auto modifier = std::bit_cast<std::int16_t>(*raw_modifier);
  state.index = static_cast<std::uint32_t>(std::int64_t{*index} + modifier + ring) & index_mask;
  --state.remaining;
  state.count = static_cast<std::uint16_t>(state.remaining & short_mask);
  if (state.remaining != 0) { return false; }
  state.active = false;
  const bool terminal = !state.mode.chained || state.insertion || chain_address(chain) == 0;
  const bool interrupt = terminal || (chain & pci) != 0;
  const bool deferred = terminal && state.mode.transmit && state.mode.external_completion;
  if (interrupt && !deferred) { state.interrupt_pending = true; }
  if (!terminal) { start_chain(state); }
  return interrupt && deferred;
}

void configure(SharcSportDmaState &state, SharcSportDmaMode mode) {
  if (state.load) {
    throw std::runtime_error("SPORT DMA mode change during TCB loading not qualified");
  }
  if (mode.transmit != state.mode.transmit &&
      (state.active || state.transmit_buffer || state.transmit_shift || state.receive_buffer)) {
    throw std::runtime_error("SPORT DMA direction change with owned data not qualified");
  }
  if (mode == state.mode) { return; }
  const auto previous = state.mode;
  if (mode.chained && !state.chain) { throw std::runtime_error("Unknown SPORT DMA chain pointer"); }
  if (mode.chained && !mode.enabled && !state.active && !state.insertion) {
    throw std::runtime_error("Initial SPORT DMA chain insertion not qualified");
  }
  state.mode = mode;
  state.insertion = mode.chained && !mode.enabled;
  if (!mode.enabled && !mode.chained) {
    state.active = false;
    state.remaining = 0;
    return;
  }
  if (state.active) {
    if (previous.chained != mode.chained) {
      throw std::runtime_error("SPORT DMA live chaining change not qualified");
    }
    return;
  }
  if (mode.enabled && (!previous.enabled || previous.chained != mode.chained)) {
    if (mode.chained) {
      start_chain(state);
    } else {
      start_standard(state);
    }
  }
}

}

SharcSportDma::SharcSportDma(SharcPower &power,
                             SharcMemory &memory,
                             unsigned int sport,
                             SharcSportDmaState initial)
    : power_(power), memory_(memory), gate_(gate(sport)), current_(initial) {
  validate(current_);
}

SharcSportDmaState SharcSportDma::state() const {
  return working();
}

SharcSportDmaState SharcSportDma::snapshot() const {
  if (staged_) { throw std::logic_error("SPORT DMA snapshot during active instruction"); }
  return current_;
}

void SharcSportDma::configure(SharcSportDmaMode mode) {
  require_clock();
  auto next = working();
  lead_a1::configure(next, mode);
  working() = next;
}

bool SharcSportDma::locks_iop_buses() const {
  return current_.load.has_value();
}

std::optional<SharcMemoryRequest> SharcSportDma::request() const {
  external_event();
  require_clock();
  if (current_.load) {
    const auto load = *current_.load;
    if (load.received == load.words.size()) { return {}; }
    return SharcMemoryRequest{
        .access = {.address = load.index_address - load.received, .width = SharcWidth::normal32}};
  }
  if (!current_.active || (current_.mode.transmit && current_.transmit_buffer) ||
      (!current_.mode.transmit && !current_.receive_buffer)) {
    return {};
  }
  const auto index = current_.index;
  if (!index) { throw std::logic_error("Missing active SPORT DMA address"); }
  if (*index < pci) {
    throw std::runtime_error("SPORT DMA low index interpretation not qualified: word-address=" +
                             std::to_string(*index));
  }
  if (current_.mode.transmit) {
    return SharcMemoryRequest{.access = {.address = *index, .width = SharcWidth::normal32}};
  }
  return SharcMemoryRequest{.access = {.address = *index, .width = SharcWidth::normal32},
                            .kind = AccessKind::write,
                            .value = *current_.receive_buffer};
}

void SharcSportDma::service(SharcMemoryBus &bus) {
  const auto expected = request();
  if (&bus.memory() != &memory_ || !expected || bus.grant(SharcMemoryMaster::io0) != expected) {
    throw std::runtime_error("SPORT DMA service requires its exact bound IOD0 memory grant");
  }
  auto next = current_;
  const auto value = static_cast<std::uint32_t>(bus.execute(SharcMemoryMaster::io0));
  if (next.load) {
    auto &load = *next.load;
    load.words[load.received++] = value;
    finish_load(next);
  } else {
    const bool transmit = next.mode.transmit;
    const bool interrupt = finish_word(next);
    if (transmit) {
      next.transmit_buffer = SharcSportDmaWord{.value = value, .interrupt_after_shift = interrupt};
      next.transmit_latch = value;
    } else {
      next.receive_buffer.reset();
    }
  }
  current_ = next;
}

void SharcSportDma::advance_cclk(std::uint32_t clocks) {
  external_event();
  require_clock();
  auto next = current_;
  if (next.load) {
    auto &elapsed = next.load->elapsed_cclk;
    elapsed = clocks >= chain_clocks - elapsed ? chain_clocks : elapsed + clocks;
    finish_load(next);
  }
  current_ = next;
}

void SharcSportDma::receive_word(std::uint32_t value) {
  external_event();
  require_clock();
  if (current_.mode.transmit || current_.receive_buffer) {
    throw std::runtime_error("SPORT DMA receive word requires its empty receive buffer");
  }
  current_.receive_buffer = value;
}

void SharcSportDma::transmit_word(std::uint32_t value) {
  require_clock();
  auto &state = working();
  if (!state.mode.transmit || state.transmit_buffer) {
    throw std::runtime_error("SPORT core TX requires its empty active-direction buffer");
  }
  state.transmit_buffer = SharcSportDmaWord{.value = value};
  state.transmit_latch = value;
}

std::uint32_t SharcSportDma::read_transmit_word() const {
  require_clock();
  const auto &state = working();
  if (!state.mode.transmit || !state.transmit_latch) {
    throw std::runtime_error("SPORT core TX read requires known active-direction data");
  }
  return *state.transmit_latch;
}

std::uint32_t SharcSportDma::read_receive_word(bool hang_disabled) {
  require_clock();
  auto &state = working();
  if (state.mode.transmit) { throw std::runtime_error("SPORT core RX direction is inactive"); }
  if (!state.receive_buffer) {
    if (!hang_disabled || !state.receive_latch) {
      throw std::runtime_error("SPORT core RX buffer is empty or old data is unknown");
    }
    return *state.receive_latch;
  }
  state.receive_latch = state.receive_buffer;
  state.receive_buffer.reset();
  return *state.receive_latch;
}

void SharcSportDma::flush_buffers() {
  external_event();
  require_clock();
  current_.transmit_buffer.reset();
  current_.transmit_shift.reset();
  current_.receive_buffer.reset();
  current_.transmit_latch.reset();
  current_.receive_latch.reset();
}

std::uint32_t SharcSportDma::start_transmit() {
  external_event();
  require_clock();
  if (!current_.mode.transmit || !current_.transmit_buffer || current_.transmit_shift) {
    throw std::runtime_error("SPORT DMA shifter requires buffered TX word and idle shift register");
  }
  current_.transmit_shift = current_.transmit_buffer;
  current_.transmit_buffer.reset();
  return current_.transmit_shift->value;
}

void SharcSportDma::finish_transmit() {
  external_event();
  require_clock();
  if (!current_.transmit_shift) { throw std::runtime_error("SPORT DMA has no transmitted word"); }
  if (current_.transmit_shift->interrupt_after_shift) { current_.interrupt_pending = true; }
  current_.transmit_shift.reset();
}

bool SharcSportDma::interrupt_request() const {
  return current_.interrupt_pending;
}

void SharcSportDma::acknowledge_interrupt() {
  external_event();
  current_.interrupt_pending = false;
}

std::uint64_t SharcSportDma::read(MemoryAccess access) {
  const auto index = parameter(access);
  require_clock();
  const auto &state = working();
  if (state.load) {
    throw std::runtime_error("SPORT DMA parameter read during TCB load not qualified");
  }
  const std::array<std::optional<std::uint32_t>, 4> values{
      state.index, state.modifier, state.count, state.chain};
  const auto value = values[index];
  if (!value) { throw std::runtime_error("Unknown SPORT DMA parameter read"); }
  return *value;
}

void SharcSportDma::write(MemoryAccess access, std::uint64_t value) {
  const auto index = parameter(access);
  require_clock();
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized SPORT DMA parameter write");
  }
  auto next = working();
  if (next.load || (next.active && index < 2)) {
    throw std::runtime_error("SPORT DMA live index/modifier or loading write not qualified");
  }
  const auto raw = static_cast<std::uint32_t>(value);
  switch (index) {
    case 0: next.index = raw & index_mask; break;
    case 1: next.modifier = static_cast<std::uint16_t>(raw & short_mask); break;
    case 2:
      next.count = static_cast<std::uint16_t>(raw & short_mask);
      if (next.active) { next.remaining = count_value(*next.count); }
      break;
    case 3:
      next.chain = raw & chain_mask;
      if (!next.active && next.mode.enabled && next.mode.chained) { start_chain(next); }
      break;
    default: throw std::logic_error("Invalid SPORT DMA register index");
  }
  working() = next;
}

void SharcSportDma::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested SPORT DMA instruction"); }
  staged_ = current_;
}

void SharcSportDma::commit_instruction() noexcept {
  if (staged_) { current_ = *staged_; }
  staged_.reset();
}

void SharcSportDma::abort_instruction() noexcept {
  staged_.reset();
}

void SharcSportDma::external_event() const {
  if (staged_ || memory_.active()) {
    throw std::logic_error("SPORT DMA external event during active instruction");
  }
}

void SharcSportDma::require_clock() const {
  if (!power_.enabled(gate_)) {
    throw std::runtime_error("SPORT DMA with stopped peripheral clock not qualified");
  }
}

SharcSportDmaState &SharcSportDma::working() {
  return staged_ ? *staged_ : current_;
}

const SharcSportDmaState &SharcSportDma::working() const {
  return staged_ ? *staged_ : current_;
}

}
