// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_sport.hpp"

#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_power.hpp"
#include "sharc/peripherals/sharc_sport_dma.hpp"
#include "sharc/peripherals/sharc_sport_internal.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr unsigned int dma_first = 16;
constexpr unsigned int dma_end = 24;
constexpr unsigned int buffers_end = 28;
constexpr unsigned int error_register = 12;
constexpr unsigned int selection_first = 4;
constexpr unsigned int compand_first = 8;

unsigned int register_word(MemoryAccess access) {
  constexpr unsigned int bytes = 4;
  if (access.size != bytes || access.address % bytes != 0 ||
      access.address >= buffers_end * bytes) {
    throw std::runtime_error("SPORT requires aligned32-bit canonical register offsets");
  }
  const auto word = static_cast<unsigned int>(access.address / bytes);
  if (word > error_register && word < dma_first) {
    throw std::runtime_error("Unimplemented SPORT register hole");
  }
  return word;
}

SharcClockGate clock_gate(unsigned int number) {
  constexpr unsigned int sports = 8;
  constexpr std::uint32_t first_pair = 0x40;
  if (number >= sports) { throw std::invalid_argument("Invalid SPORT identity"); }
  return static_cast<SharcClockGate>(first_pair << (number / 2));
}

SharcSportDmaMode dma_mode(const std::array<std::uint32_t, 3> &control, unsigned int channel) {
  constexpr unsigned int enable_shift = 18;
  constexpr unsigned int chain_shift = 19;
  constexpr std::uint32_t external = 2;
  return {.enabled = (control[0] & (1U << (enable_shift + channel * 2))) != 0,
          .chained = (control[0] & (1U << (chain_shift + channel * 2))) != 0,
          .transmit = (control[0] & sport_detail::transmit_bit) != 0,
          .external_completion = (control[1] & external) != 0};
}

std::uint32_t actual_dma_status(const std::array<SharcSportDma, 2> &channels, unsigned int number) {
  constexpr unsigned int active_shift = 24;
  constexpr unsigned int chain_shift = 28;
  std::uint32_t bits{};
  for (unsigned int channel = 0; channel < channels.size(); ++channel) {
    const auto state = channels[channel].state();
    const auto offset = ((number % 2) * 2) + channel;
    if (state.active) { bits |= 1U << (active_shift + offset); }
    if (state.load) { bits |= 1U << (chain_shift + offset); }
  }
  return bits;
}

void store_control(SharcSportState &state, unsigned int index, std::uint32_t value) {
  auto target = state.control;
  constexpr std::array masks{
      sport_detail::control_mask, 0xffffffffU, sport_detail::multichannel_mask};
  target[index] = value & masks[index];
  for (unsigned int channel = 0; channel < state.flush.size(); ++channel) {
    if (sport_detail::enabled(state.control, channel) && !sport_detail::enabled(target, channel)) {
      state.flush[channel] = sport_detail::flush_clocks;
    }
  }
  state.control = target;
}

void shift_controls(SharcSportState &state) {
  const auto before = sport_detail::effective(state);
  state.sampled[2] = state.sampled[1];
  state.sampled[1] = state.sampled[0];
  state.sampled[0] = state.control;
  state.sampled_select[1] = state.sampled_select[0];
  state.sampled_select[0] = state.select;
  state.sampled_compand[1] = state.sampled_compand[0];
  state.sampled_compand[0] = state.compand;
  const auto after = sport_detail::effective(state);
  constexpr std::uint32_t wire_mask = 0x0303ffff;
  constexpr std::uint32_t multi_mask = 0x00800fff;
  auto wire_fields = wire_mask;
  auto multi_fields = multi_mask;
  constexpr std::array<std::uint32_t, 2> standard_enable{1, 0x01000000};
  constexpr std::array<std::uint32_t, 2> multi_enable{1, 0x00800000};
  for (unsigned int channel = 0; channel < state.flush.size(); ++channel) {
    if (!sport_detail::enabled(state.control, channel) && state.flush[channel] == 0) {
      wire_fields &= ~standard_enable[channel];
      multi_fields &= ~multi_enable[channel];
    }
  }
  if (state.word_active && (((before[0] ^ after[0]) & wire_fields) != 0 ||
                            ((before[2] ^ after[2]) & multi_fields) != 0)) {
    throw std::runtime_error("Live SPORT wire configuration change not qualified");
  }
  if (!sport_detail::enabled(after, 0) && !sport_detail::enabled(after, 1)) {
    state.frame_active = false;
    state.word_active = false;
    state.word_selected = false;
    state.pending_frame.reset();
    state.frame_edge = false;
    state.bit = 0;
    state.slot = 0;
    state.shift.fill(0);
    state.signals = {};
  }
}

}

SharcSport::SharcSport(SharcPower &power,
                       SharcMemory &memory,
                       unsigned int sport,
                       const SharcSportSnapshot &initial)
    : power_(power),
      memory_(memory),
      number_(sport),
      gate_(clock_gate(sport)),
      current_(initial.serial),
      dma_{SharcSportDma{power, memory, sport, initial.dma[0]},
           SharcSportDma{power, memory, sport, initial.dma[1]}} {
  sport_detail::validate(current_);
}

SharcSportSnapshot SharcSport::snapshot() const {
  if (staged_) { throw std::logic_error("SPORT snapshot during active instruction"); }
  return {.serial = current_, .dma = {dma_[0].snapshot(), dma_[1].snapshot()}};
}

SharcSportSignals SharcSport::signals() const {
  require_clock();
  return sport_detail::outputs(current_);
}

SharcSportDma &SharcSport::dma(SharcSportChannel channel) {
  const auto index = static_cast<unsigned int>(channel);
  if (index >= dma_.size()) { throw std::invalid_argument("Invalid SPORT data channel"); }
  return dma_[index];
}

std::uint32_t SharcSport::dma_status() const {
  return current_.status[1];
}

bool SharcSport::interrupt_request() const {
  return current_.core_interrupt || dma_[0].interrupt_request() || dma_[1].interrupt_request();
}

void SharcSport::acknowledge_interrupt() {
  idle();
  current_.core_interrupt = false;
  for (auto &channel : dma_) { channel.acknowledge_interrupt(); }
}

void SharcSport::advance_sclk(std::uint32_t periods) {
  idle();
  require_clock();
  // Three stable samples exhaust this control/readiness reference. Wire bits
  // advance only through observe(), never through this elapsed-period API.
  constexpr unsigned int settle_samples = 3;
  auto next = current_;
  begin_instruction();
  try {
    for (std::uint32_t cycle = 0; cycle < std::min(periods, settle_samples); ++cycle) {
      shift_controls(next);
      const auto control = sport_detail::effective(next);
      for (unsigned int channel = 0; channel < dma_.size(); ++channel) {
        const auto mode = dma_mode(control, channel);
        if (mode != dma_[channel].state().mode) { dma_[channel].configure(mode); }
      }
    }
    current_ = next;
    for (auto &channel : dma_) { channel.commit_instruction(); }
    staged_.reset();
  } catch (...) {
    abort_instruction();
    throw;
  }
}

void SharcSport::advance_cclk(std::uint32_t clocks) {
  idle();
  require_clock();
  for (auto &channel : dma_) { channel.advance_cclk(clocks); }
  for (unsigned int channel = 0; channel < current_.flush.size(); ++channel) {
    auto &remaining = current_.flush[channel];
    if (remaining == 0) { continue; }
    if (clocks >= remaining) {
      dma_[channel].flush_buffers();
      current_.shift[channel] = 0;
      remaining = 0;
    } else {
      remaining -= clocks;
    }
  }
  if (!sport_detail::enabled(current_.control, 0) && !sport_detail::enabled(current_.control, 1) &&
      current_.flush[0] == 0 && current_.flush[1] == 0) {
    current_.word_active = false;
    current_.word_selected = false;
    current_.frame_active = false;
    current_.pending_frame.reset();
    current_.frame_edge = false;
    current_.bit = 0;
    current_.slot = 0;
    current_.signals = {};
  }
}

void SharcSport::advance_half_pclk(std::uint32_t half_periods) {
  idle();
  require_clock();
  const auto actual = actual_dma_status(dma_, number_);
  if (half_periods >= 2) {
    current_.status = {actual, actual};
  } else if (half_periods == 1) {
    current_.status[1] = current_.status[0];
    current_.status[0] = actual;
  }
}

void SharcSport::observe(SharcSportLines lines) {
  idle();
  require_clock();
  auto next = current_;
  const sport_detail::DmaStates before{dma_[0].snapshot(), dma_[1].snapshot()};
  const auto actions = sport_detail::wire(next, lines, before);
  for (unsigned int index = 0; index < actions.size(); ++index) {
    const auto action = actions[index];
    switch (action.kind) {
      case sport_detail::ActionKind::none: break;
      case sport_detail::ActionKind::transmit_start:
        static_cast<void>(dma_[index].start_transmit());
        if (!before[index].mode.enabled) { next.core_interrupt = true; }
        break;
      case sport_detail::ActionKind::transmit_finish: dma_[index].finish_transmit(); break;
      case sport_detail::ActionKind::receive:
        dma_[index].receive_word(action.value);
        if (!before[index].mode.enabled) { next.core_interrupt = true; }
        break;
      default: throw std::logic_error("Invalid SPORT serial action");
    }
  }
  current_ = next;
}

std::uint64_t SharcSport::read(MemoryAccess access) {
  const auto word = register_word(access);
  require_clock();
  const auto &state = working();
  if (word == 0) {
    if (state.word_active &&
        (sport_detail::effective(state)[0] & sport_detail::transmit_bit) == 0) {
      throw std::runtime_error("SPORT partial RX buffer-status readback not qualified");
    }
    return state.control[0] | sport_detail::buffer_status({dma_[0].state(), dma_[1].state()});
  }
  if (word == 1) { return state.control[1]; }
  if (word == 2) {
    constexpr unsigned int slot_shift = 16;
    return state.control[2] | (state.slot << slot_shift) | dma_status();
  }
  if (word == 3) { return state.divisor; }
  if (word < compand_first) { return state.select[word - selection_first]; }
  if (word < error_register) { return state.compand[word - compand_first]; }
  if (word == error_register) { return state.error_control; }
  if (word < dma_end) {
    return dma_[(word - dma_first) / 4].read({.address = ((word - dma_first) % 4) * 4, .size = 4});
  }
  constexpr std::uint32_t hang_disable = 0x800000;
  auto &channel = dma_[(word - dma_end) / 2];
  return word % 2 == 0
             ? channel.read_transmit_word()
             : channel.read_receive_word((sport_detail::effective(state)[0] & hang_disable) != 0);
}

void SharcSport::write(MemoryAccess access, std::uint64_t value) {
  const auto word = register_word(access);
  require_clock();
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized SPORT register write");
  }
  auto &state = working();
  const auto raw = static_cast<std::uint32_t>(value);
  if (word < 3) {
    store_control(state, word, raw);
    return;
  }
  if (word == 3) {
    state.divisor = raw;
    return;
  }
  if (word < compand_first) {
    state.select[word - selection_first] = raw;
    return;
  }
  if (word < error_register) {
    state.compand[word - compand_first] = raw;
    return;
  }
  if (word == error_register) {
    constexpr std::uint32_t acknowledge = 0x70;
    if ((raw & acknowledge) != 0) {
      throw std::runtime_error("SPORT error acknowledgement not qualified");
    }
    state.error_control = raw;
    return;
  }
  if (word < dma_end) {
    dma_[(word - dma_first) / 4].write({.address = ((word - dma_first) % 4) * 4, .size = 4}, raw);
    return;
  }
  if (word % 2 != 0) { throw std::runtime_error("SPORT RX buffer write not qualified"); }
  dma_[(word - dma_end) / 2].transmit_word(raw);
}

void SharcSport::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested SPORT instruction"); }
  dma_[0].begin_instruction();
  try {
    dma_[1].begin_instruction();
    staged_ = current_;
  } catch (...) {
    dma_[0].abort_instruction();
    throw;
  }
}

void SharcSport::commit_instruction() noexcept {
  if (staged_) { current_ = *staged_; }
  for (auto &channel : dma_) { channel.commit_instruction(); }
  staged_.reset();
}

void SharcSport::abort_instruction() noexcept {
  for (auto &channel : dma_) { channel.abort_instruction(); }
  staged_.reset();
}

void SharcSport::idle() const {
  if (staged_ || memory_.active()) {
    throw std::logic_error("SPORT event during active instruction");
  }
}

void SharcSport::require_clock() const {
  if (!power_.enabled(gate_)) {
    throw std::runtime_error("SPORT with stopped peripheral clock not qualified");
  }
}

SharcSportState &SharcSport::working() {
  return staged_ ? *staged_ : current_;
}

const SharcSportState &SharcSport::working() const {
  return staged_ ? *staged_ : current_;
}

}
