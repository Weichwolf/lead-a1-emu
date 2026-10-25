// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_power.hpp"

#include "lead_a1/memory.hpp"

#include <bit>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t shutdown_mask = 0x92fff;
constexpr unsigned int latency = 2;
constexpr unsigned int accelerator_shift = 17;

SharcAccelerator selection(std::uint32_t value) {
  constexpr std::uint32_t selector_mask = 3;
  const auto code = (value >> accelerator_shift) & selector_mask;
  if (code == selector_mask) {
    throw std::runtime_error("SHARC PMCTL1 reserved accelerator selection");
  }
  return static_cast<SharcAccelerator>(code);
}

void validate(const SharcPowerState &state) {
  if ((state.shutdown & ~shutdown_mask) != 0 || state.remaining_pclk > latency ||
      (state.remaining_pclk == 0 && state.half_elapsed) ||
      (state.accelerator && *state.accelerator > SharcAccelerator::fft)) {
    throw std::invalid_argument("Invalid SHARC PMCTL1 clock/accelerator state");
  }
  if (!state.control) {
    if (state.shutdown != 0 || state.accelerator || state.remaining_pclk != 0) {
      throw std::invalid_argument("Invalid SHARC PMCTL1 reset knowledge");
    }
    return;
  }
  const auto accelerator = selection(*state.control);
  if (state.remaining_pclk == 0 &&
      (state.shutdown != (*state.control & shutdown_mask) || state.accelerator != accelerator)) {
    throw std::invalid_argument("Invalid SHARC PMCTL1 settled outputs");
  }
}

void access_shape(MemoryAccess access) {
  if (access.address != 0 || access.size != 4) {
    throw std::runtime_error("SHARC PMCTL1 requires mapped offset0 and 32-bit access");
  }
}

}

SharcPower::SharcPower(SharcPowerState initial) : current_(initial) {
  validate(current_);
}

SharcPowerState SharcPower::state() const {
  return working();
}

SharcPowerState SharcPower::snapshot() const {
  if (staged_) { throw std::logic_error("SHARC PMCTL1 snapshot during active instruction"); }
  return current_;
}

bool SharcPower::enabled(SharcClockGate gate) const {
  const auto mask = static_cast<std::uint32_t>(gate);
  if (!std::has_single_bit(mask) || (mask & ~shutdown_mask) != 0) {
    throw std::invalid_argument("Invalid SHARC PMCTL1 clock output");
  }
  return (current_.shutdown & mask) == 0;
}

void SharcPower::advance_pclk(std::uint32_t clocks) {
  if (staged_) { throw std::logic_error("SHARC PMCTL1 PCLK during active instruction"); }
  if (current_.remaining_pclk == 0) { return; }
  if (!current_.control) { throw std::logic_error("Missing delivered SHARC PMCTL1 value"); }
  if (clocks < current_.remaining_pclk) {
    current_.remaining_pclk -= clocks;
    return;
  }
  const auto value = current_.control.value();
  const auto accelerator = selection(value);
  current_.shutdown = value & shutdown_mask;
  current_.accelerator = accelerator;
  current_.remaining_pclk = 0;
  current_.half_elapsed = false;
}

void SharcPower::advance_half_pclk(std::uint32_t half_cycles) {
  if (staged_) { throw std::logic_error("SHARC PMCTL1 half-PCLK during active instruction"); }
  if (current_.remaining_pclk == 0) { return; }
  const auto remaining = (current_.remaining_pclk * 2) - (current_.half_elapsed ? 1U : 0U);
  if (half_cycles >= remaining) {
    advance_pclk(current_.remaining_pclk);
    return;
  }
  const auto elapsed = half_cycles + (current_.half_elapsed ? 1U : 0U);
  current_.remaining_pclk -= elapsed / 2;
  current_.half_elapsed = elapsed % 2 != 0;
}

std::uint64_t SharcPower::read(MemoryAccess access) {
  access_shape(access);
  const auto &state = working();
  if (!state.control) { throw std::runtime_error("Unknown SHARC PMCTL1 reset/reserved readback"); }
  if (state.remaining_pclk != 0) {
    throw std::runtime_error("SHARC PMCTL1 unsettled readback timing not qualified");
  }
  return *state.control;
}

void SharcPower::write(MemoryAccess access, std::uint64_t value) {
  access_shape(access);
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized SHARC PMCTL1 write");
  }
  const auto control = static_cast<std::uint32_t>(value);
  static_cast<void>(selection(control));
  auto &state = working();
  if (state.remaining_pclk != 0) {
    throw std::runtime_error("SHARC PMCTL1 overlapping write timing not qualified");
  }
  state.control = control;
  state.remaining_pclk = latency;
}

void SharcPower::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested SHARC PMCTL1 instruction"); }
  staged_ = current_;
}

void SharcPower::commit_instruction() noexcept {
  if (staged_) {
    current_ = *staged_;
    staged_.reset();
  }
}

void SharcPower::abort_instruction() noexcept {
  staged_.reset();
}

SharcPowerState &SharcPower::working() {
  return staged_ ? *staged_ : current_;
}

const SharcPowerState &SharcPower::working() const {
  return staged_ ? *staged_ : current_;
}

}
