// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_pdap.hpp"

#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_pdap_internal.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t status_offset = 0x20;
constexpr std::uint32_t fifo_offset = 0x80;
constexpr unsigned int fifo_size_shift = 28;
constexpr std::uint32_t overflow_bit = 0x100;

void access_shape(MemoryAccess access) {
  if (access.size != 4 || access.address % 4 != 0) {
    throw std::out_of_range("PDAP requires aligned32-bit register access");
  }
}

unsigned int input_index(SharcPdapPort port) {
  switch (port) {
    case SharcPdapPort::dai: return 0;
    case SharcPdapPort::ami: return 1;
    default: throw std::invalid_argument("Invalid PDAP input port");
  }
}

bool core_fifo_enabled(const SharcPdapState &state) {
  return (state.control[0] & detail::idp_enable) != 0 &&
         (state.control[1] & detail::pdap_enable) != 0;
}

}

SharcPdap::SharcPdap(SharcPower &power, SharcPdapState initial) : power_(power), current_(initial) {
  detail::pdap_validate(current_);
  require_dma_owner(current_);
}

SharcPdap::SharcPdap(SharcPower &power, SharcMemory &memory, SharcPdapState initial)
    : power_(power), memory_(&memory), current_(initial) {
  detail::pdap_validate(current_);
}

SharcPdapState SharcPdap::state() const {
  return working();
}

SharcPdapState SharcPdap::snapshot() const {
  idle();
  return current_;
}

SharcPdapInterrupts SharcPdap::interrupts() const {
  require_clock();
  if (current_.flush_cclk != 0) { throw std::runtime_error("PDAP disable flush still pending"); }
  const auto threshold = current_.control[0] & 15;
  const bool dma_enabled =
      (current_.control[0] & detail::idp_dma) != 0 && (current_.control[2] & 0xff00U) != 0;
  std::uint8_t dma{};
  for (unsigned int channel = 0; channel < current_.dma.size(); ++channel) {
    if (current_.dma[channel].interrupt_pending) {
      dma |= static_cast<std::uint8_t>(1U << channel);
    }
  }
  return {.threshold =
              !dma_enabled && threshold < current_.fifo.size() && current_.count > threshold,
          .overflow = current_.overflow,
          .dma = dma};
}

void SharcPdap::idle() const {
  if (staged_) { throw std::logic_error("PDAP event during active instruction"); }
}

void SharcPdap::require_clock() const {
  if (!power_.enabled(SharcClockGate::dai)) {
    throw std::runtime_error("PDAP with stopped DAI clock not qualified");
  }
}

void SharcPdap::require_dma_owner(const SharcPdapState &state) const {
  if (memory_ == nullptr && ((state.control[0] & detail::idp_dma) != 0 ||
                             (state.control[2] & detail::idp_ping_pong) != 0)) {
    throw std::runtime_error("PDAP DMA/ping-pong requires its bound RAM owner");
  }
}

void SharcPdap::advance_cclk(std::uint32_t clocks) {
  idle();
  require_clock();
  if (current_.flush_cclk == 0) { return; }
  if (clocks < current_.flush_cclk) {
    current_.flush_cclk -= clocks;
    return;
  }
  detail::pdap_flush(current_, true);
  current_.flush_cclk = 0;
}

void SharcPdap::advance_sclk(std::uint32_t periods) {
  idle();
  require_clock();
  if (periods != 0) { current_.startup_sclk = 0; }
}

void SharcPdap::acknowledge_overflow() {
  idle();
  require_clock();
  current_.overflow = false;
}

void SharcPdap::observe(SharcPdapPort port, SharcPdapInput input) {
  idle();
  require_clock();
  detail::pdap_validate_input(input);
  const auto index = input_index(port);
  auto next = current_;
  const auto previous = next.inputs[index];
  next.inputs[index] = input;
  const auto selected = (next.control[1] & detail::pdap_select) != 0 ? 1U : 0U;
  if (!detail::pdap_active(next) || selected != index) {
    current_ = next;
    return;
  }
  if (next.flush_cclk != 0) { throw std::runtime_error("PDAP disable flush still pending"); }
  if (!input.clock) { throw std::runtime_error("Unknown PDAP acquisition clock"); }
  const bool active = (next.control[1] & detail::pdap_edge) == 0;
  if (*input.clock == active) {
    if (!previous.clock) { throw std::runtime_error("Unknown PDAP active-edge history"); }
    if (*previous.clock != *input.clock) {
      if (next.startup_sclk != 0) {
        throw std::runtime_error("PDAP startup input-clock period still pending");
      }
      detail::pdap_sample(next, input);
    }
  }
  current_ = next;
}

void SharcPdap::require_fifo(const SharcPdapState &state) {
  if (state.flush_cclk != 0) { throw std::runtime_error("PDAP disable flush still pending"); }
  if (!core_fifo_enabled(state)) {
    throw std::runtime_error("PDAP core FIFO requires enabled parallel input mode");
  }
}

bool SharcPdap::fifo_read_ready() const {
  require_clock();
  return core_fifo_enabled(current_) && current_.flush_cclk == 0 && current_.count != 0;
}

bool SharcPdap::fifo_write_ready() const {
  require_clock();
  return core_fifo_enabled(current_) && current_.flush_cclk == 0 &&
         current_.count < current_.fifo.size();
}

std::uint64_t SharcPdap::read(MemoryAccess access) {
  access_shape(access);
  require_clock();
  auto &state = working();
  if (access.address < 16) { return state.control[access.address / 4]; }
  if (state.flush_cclk != 0) { throw std::runtime_error("PDAP disable flush still pending"); }
  if (access.address == status_offset) {
    constexpr unsigned int dma_shift = 17;
    return (state.count << fifo_size_shift) | (state.overflow ? overflow_bit : 0U) |
           (std::uint32_t{detail::idp_dma_status(state)} << dma_shift);
  }
  if (access.address != fifo_offset) {
    throw std::out_of_range("Unimplemented PDAP/IDP register offset");
  }
  require_fifo(state);
  if (state.count == 0) {
    if ((state.control[0] & detail::idp_bhd) != 0) {
      throw std::runtime_error("PDAP BHD empty-FIFO read value not qualified");
    }
    throw std::runtime_error("PDAP empty-FIFO core hang requires IOP readiness coupling");
  }
  const auto value = state.fifo[state.head];
  detail::pdap_pop(state);
  return value;
}

void SharcPdap::control_write(unsigned int index, std::uint32_t value) {
  auto next = working();
  if (next.flush_cclk != 0) { throw std::runtime_error("PDAP disable flush still pending"); }
  const bool old_enabled = (next.control[0] & detail::idp_enable) != 0;
  const auto previous_pp = next.control[1];
  const auto previous_channels = next.control[2];
  next.control[index] = value;
  next.control[0] &= ~detail::idp_clear_overflow;
  next.control[1] &= ~detail::pdap_reset;
  next.control[2] &= ~detail::idp_fifo_clear;
  detail::pdap_validate_controls(next);
  require_dma_owner(next);
  detail::idp_dma_configure(next, working());
  const bool reset = index == 1 && (value & detail::pdap_reset) != 0;
  const bool enabled = (next.control[0] & detail::idp_enable) != 0;
  const bool changed = ((previous_pp ^ next.control[1]) & detail::pdap_configuration) != 0 ||
                       ((previous_channels ^ next.control[2]) & 1) != 0 || old_enabled != enabled;
  if (next.samples != 0 && changed && !reset) {
    throw std::runtime_error("PDAP reconfiguration with partial packing not qualified");
  }
  if (reset) {
    next.partial = 0;
    next.samples = 0;
  }
  if (index == 0 && (value & detail::idp_clear_overflow) != 0) { detail::pdap_flush(next, true); }
  if (index == 2 && (value & detail::idp_fifo_clear) != 0) { detail::pdap_flush(next, false); }
  if (old_enabled && !enabled) {
    next.startup_sclk = 0;
    next.flush_cclk = 2;
  } else if (!old_enabled && enabled) {
    next.startup_sclk = 1;
  }
  working() = next;
}

void SharcPdap::write(MemoryAccess access, std::uint64_t value) {
  access_shape(access);
  require_clock();
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::out_of_range("Oversized PDAP register write");
  }
  const auto word = static_cast<std::uint32_t>(value);
  if (access.address < 16) {
    control_write(access.address / 4, word);
    return;
  }
  if (access.address != fifo_offset) {
    throw std::out_of_range("Read-only or unimplemented PDAP/IDP register write");
  }
  auto &state = working();
  require_fifo(state);
  if (state.count == state.fifo.size()) {
    throw std::runtime_error((state.control[0] & detail::idp_bhd) != 0
                                 ? "PDAP BHD full-FIFO write behavior not qualified"
                                 : "PDAP full-FIFO core hang requires IOP readiness coupling");
  }
  detail::pdap_push(state, word);
}

void SharcPdap::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested PDAP instruction"); }
  staged_ = current_;
}

void SharcPdap::commit_instruction() noexcept {
  if (staged_) { current_ = *staged_; }
  staged_.reset();
}

void SharcPdap::abort_instruction() noexcept {
  staged_.reset();
}

SharcPdapState &SharcPdap::working() {
  return staged_ ? *staged_ : current_;
}

const SharcPdapState &SharcPdap::working() const {
  return staged_ ? *staged_ : current_;
}

}
