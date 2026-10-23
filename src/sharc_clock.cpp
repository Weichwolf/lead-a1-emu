// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_clock.hpp"

#include "lead_a1/memory.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t multiplier = 0x3f;
constexpr std::uint32_t divider = 0xc0;
constexpr std::uint32_t input_divider = 0x100;
constexpr std::uint32_t divider_enable = 0x200;
constexpr std::uint32_t clockout_enable = 0x1000;
constexpr std::uint32_t bypass = 0x8000;
constexpr std::uint32_t hardware_ratio = 0x30000;
constexpr std::uint32_t external_divider = 0x1c0000;
constexpr std::uint32_t writable = 0x1c91ff;
constexpr std::uint32_t divider_mask = divider | external_divider;
constexpr std::uint32_t pll_wait = 4096;
constexpr std::uint32_t divider_wait = 14;
constexpr std::uint32_t bypass_reentry_wait = 6;
constexpr unsigned int external_shift = 18;

void validate(MemoryAccess access) {
  if (access.address != 0 || access.size != 4) {
    throw std::runtime_error("SHARC PMCTL requires mapped offset0 and 32-bit word access");
  }
}

std::uint32_t remaining(std::uint32_t count, std::uint32_t elapsed) {
  return count > elapsed ? count - elapsed : 0;
}

SharcClockRatio ratio(std::uint32_t numerator, std::uint32_t denominator) {
  if (numerator == 0 || denominator == 0) { throw std::logic_error("Invalid SHARC clock ratio"); }
  const auto common = std::gcd(numerator, denominator);
  if (common == 0) { throw std::logic_error("Invalid SHARC clock divisor"); }
  return {.numerator = numerator / common, .denominator = denominator / common};
}

std::optional<SharcClockRatio> core(const SharcClockState &state) {
  if (!state.input_stable || !state.reset_settled) { return {}; }
  if ((state.control & bypass) != 0) { return ratio(1, 1); }
  if (!state.multiplier_known || state.pll_remaining != 0 || state.pending_dividers) { return {}; }
  const auto encoded = state.control & multiplier;
  constexpr std::uint32_t zero_multiplier = 128;
  const auto vco = encoded == 0 ? zero_multiplier : encoded * 2;
  const auto input = (state.control & input_divider) != 0 ? 2U : 1U;
  const auto output = 2U << ((state.effective_dividers & divider) >> 6);
  return ratio(vco, input * output);
}

void program(SharcClockState &state, std::uint32_t value) {
  constexpr auto supported = writable | divider_enable | hardware_ratio;
  if ((value & ~supported) != 0 || (value & clockout_enable) != 0) {
    throw std::runtime_error("SHARC PMCTL reserved/family/CLKOUT control not qualified");
  }
  const auto next = value & writable;
  const bool was_bypassed = (state.control & bypass) != 0;
  const bool will_bypass = (next & bypass) != 0;
  const bool trigger = (value & divider_enable) != 0;
  if (trigger && (will_bypass || was_bypassed != will_bypass)) {
    throw std::runtime_error("SHARC PMCTL DIVEN and PLLBP must be programmed separately");
  }
  if (trigger && state.pending_dividers) {
    throw std::runtime_error("SHARC PMCTL divider update already pending");
  }
  if (trigger && ((next & external_divider) >> external_shift) > 4) {
    throw std::runtime_error("Reserved SHARC external clock divider");
  }
  const bool changed =
      !state.multiplier_known || ((next ^ state.control) & (multiplier | input_divider)) != 0;
  if (was_bypassed && !will_bypass &&
      (changed || !state.input_stable || state.pll_remaining != 0)) {
    throw std::runtime_error("SHARC PLL exit before guaranteed CLKIN settling");
  }
  if (!was_bypassed && will_bypass && state.bypass_reentry_remaining != 0) {
    throw std::runtime_error("SHARC PLL bypass reentry requires six core clocks");
  }
  if (changed) {
    state.pll_remaining = pll_wait;
    state.bypass_required = !will_bypass;
  }
  if (will_bypass && state.bypass_required) {
    state.pll_remaining = pll_wait;
    state.bypass_required = false;
  }
  if (was_bypassed && !will_bypass) { state.bypass_reentry_remaining = bypass_reentry_wait; }
  if (trigger) {
    state.pending_dividers = next & divider_mask;
    state.divider_remaining = divider_wait;
  }
  state.control = next;
  state.multiplier_known = true;
}

}

SharcClock::SharcClock(SharcClockReference initial) {
  if ((initial.configuration && *initial.configuration > 2) ||
      (initial.reset_settled && !initial.input_stable)) {
    throw std::invalid_argument("Invalid SHARC clock reset reference");
  }
  current_.configuration = initial.configuration;
  current_.input_stable = initial.input_stable;
  current_.reset_settled = initial.reset_settled;
  current_.multiplier_known = initial.configuration.has_value();
  if (initial.configuration) {
    constexpr std::array<std::uint32_t, 3> reset_multipliers{8, 32, 16};
    current_.control = reset_multipliers[*initial.configuration];
  }
  if (!initial.reset_settled) { current_.pll_remaining = pll_wait; }
}

SharcClockState SharcClock::state() const {
  return working();
}

SharcClockRates SharcClock::rates() const {
  const auto &state = working();
  const auto core_clock = core(state);
  SharcClockRates rates;
  if (!core_clock) { return rates; }
  const auto clock = *core_clock;
  rates.core = clock;
  rates.peripheral = ratio(clock.numerator, clock.denominator * 2);
  if (!state.pending_dividers) {
    const auto code = (state.effective_dividers & external_divider) >> external_shift;
    rates.external = ratio(clock.numerator * 2, clock.denominator * (4 + code));
  }
  return rates;
}

void SharcClock::advance(SharcClockElapsed elapsed) {
  if (pending_) { throw std::logic_error("SHARC clock advance during active instruction"); }
  current_.divider_remaining = remaining(current_.divider_remaining, elapsed.core_cycles);
  current_.bypass_reentry_remaining =
      remaining(current_.bypass_reentry_remaining, elapsed.core_cycles);
  if (current_.input_stable && !current_.bypass_required) {
    current_.pll_remaining = remaining(current_.pll_remaining, elapsed.clkin_cycles);
    if (current_.pll_remaining == 0) { current_.reset_settled = true; }
  }
  if (current_.pending_dividers && current_.divider_remaining == 0) {
    current_.effective_dividers = *current_.pending_dividers;
    current_.pending_dividers.reset();
  }
}

void SharcClock::input_stable(bool stable) {
  if (pending_) { throw std::logic_error("SHARC input change during active instruction"); }
  current_.input_stable = stable;
  if (!stable) { current_.pll_remaining = pll_wait; }
}

std::uint64_t SharcClock::read(MemoryAccess access) {
  validate(access);
  const auto &state = working();
  if (!state.configuration || !state.multiplier_known) {
    throw std::runtime_error("Unknown SHARC CLK_CFG pins/PMCTL reset multiplier");
  }
  constexpr unsigned int pin_shift = 16;
  // DIVEN is a write-one command; the documented read value is zero.
  return state.control | (*state.configuration << pin_shift);
}

void SharcClock::write(MemoryAccess access, std::uint64_t value) {
  validate(access);
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized SHARC PMCTL write");
  }
  auto next = working();
  program(next, static_cast<std::uint32_t>(value));
  working() = next;
}

void SharcClock::begin_instruction() {
  if (pending_) { throw std::logic_error("Nested SHARC clock instruction"); }
  pending_ = current_;
}

void SharcClock::commit_instruction() noexcept {
  if (pending_) {
    current_ = *pending_;
    pending_.reset();
  }
}

void SharcClock::abort_instruction() noexcept {
  pending_.reset();
}

SharcClockState &SharcClock::working() {
  return pending_ ? *pending_ : current_;
}

const SharcClockState &SharcClock::working() const {
  return pending_ ? *pending_ : current_;
}

}
