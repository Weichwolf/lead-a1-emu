// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_dai_pin.hpp"

#include "common/memory.hpp"
#include "sharc/peripherals/sharc_dai_enable.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr unsigned int fields = 4;
constexpr unsigned int field_bits = 7;
constexpr unsigned int selector_mask = 127;
constexpr unsigned int low_source = 126;
constexpr unsigned int earliest_effect = 4;
constexpr unsigned int guaranteed_effect = 6;

bool dynamic_source(unsigned int encoding) {
  constexpr unsigned int first_gap = 0x32;
  constexpr unsigned int sport6_clock = 0x34;
  constexpr unsigned int source_gap = 0x3c;
  constexpr unsigned int src0_output = 0x3d;
  constexpr unsigned int last_sport_valid = 0x4d;
  constexpr unsigned int pcg_clock_c = 0x50;
  constexpr unsigned int pcg_frame_d = 0x53;
  return encoding < first_gap || (encoding >= sport6_clock && encoding < source_gap) ||
         (encoding >= src0_output && encoding <= last_sport_valid) ||
         (encoding >= pcg_clock_c && encoding <= pcg_frame_d);
}

unsigned int selector(std::uint32_t word, unsigned int pin) {
  return (word >> ((pin % fields) * field_bits)) & selector_mask;
}

void validate_word(std::uint32_t word) {
  for (unsigned int pin = 0; pin < fields; ++pin) {
    const auto encoding = selector(word, pin);
    if (!dynamic_source(encoding) && encoding < low_source) {
      throw std::invalid_argument("Reserved SHARC DAI pin source selection");
    }
  }
}

void validate(const SharcDaiPinState &state) {
  for (unsigned int index = 0; index < state.routes.size(); ++index) {
    validate_word(state.routes[index]);
    const auto pending = state.pending[index];
    if (pending) {
      validate_word(pending->target);
      if (pending->half_periods >= guaranteed_effect) {
        throw std::invalid_argument("Invalid SHARC DAI pin pending interval");
      }
    }
  }
  for (unsigned int source = 0; source < state.sources.size(); ++source) {
    if (state.sources[source].has_value() && !dynamic_source(source)) {
      throw std::invalid_argument("Reserved SHARC DAI pin source input state");
    }
  }
}

unsigned int register_index(MemoryAccess access) {
  constexpr unsigned int bytes = 4;
  constexpr unsigned int mapped_bytes = 20;
  if (access.size != bytes || access.address % bytes != 0 || access.address >= mapped_bytes) {
    throw std::runtime_error("SHARC DAI pin routing requires aligned32-bit mapped offsets0..16");
  }
  return access.address / bytes;
}

std::optional<bool> routed(const SharcDaiPinState &state, std::uint32_t word, unsigned int pin) {
  const auto encoding = selector(word, pin);
  auto value = encoding >= low_source ? std::optional<bool>{encoding != low_source}
                                      : state.sources[encoding];
  constexpr unsigned int first_inverted = 18;
  constexpr unsigned int inversion_offset = 10;
  if (pin >= first_inverted && (word & (1U << (pin + inversion_offset))) != 0 && value) {
    value = !*value;
  }
  return value;
}

}

SharcDaiPin::SharcDaiPin(SharcPower &power, SharcDaiPinState initial)
    : power_(power), current_(initial) {
  validate(current_);
}

SharcDaiPinState SharcDaiPin::state() const {
  return working();
}

SharcDaiPinState SharcDaiPin::snapshot() const {
  if (staged_) { throw std::logic_error("SHARC DAI pin snapshot during active instruction"); }
  return current_;
}

SharcDaiPinOutputs SharcDaiPin::outputs() const {
  require_clock();
  SharcDaiPinOutputs result{};
  for (unsigned int pin = 0; pin < result.size(); ++pin) {
    const auto index = pin / fields;
    auto value = routed(current_, current_.routes[index], pin);
    const auto pending = current_.pending[index];
    if (pending && pending->half_periods >= earliest_effect) {
      const auto target = routed(current_, pending->target, pin);
      if (value != target) { value.reset(); }
    }
    result[pin] = value;
  }
  return result;
}

SharcDaiPinDrive SharcDaiPin::drives(const SharcDaiEnable &enables) const {
  const auto data = outputs();
  const auto enabled = enables.outputs();
  SharcDaiPinDrive result;
  for (unsigned int pin = 0; pin < data.size(); ++pin) {
    const auto bit = 1U << pin;
    const auto enable = enabled[pin];
    const auto value = data[pin];
    if (enable == false) {
      result.high_impedance |= bit;
    } else if (!enable || !value) {
      result.unresolved |= bit;
    } else if (*value) {
      result.high |= bit;
    } else {
      result.low |= bit;
    }
  }
  return result;
}

void SharcDaiPin::source(unsigned int encoding, std::optional<bool> value) {
  if (staged_) { throw std::logic_error("SHARC DAI pin source event during active instruction"); }
  if (!dynamic_source(encoding)) {
    throw std::invalid_argument("Invalid SHARC DAI pin dynamic source encoding");
  }
  current_.sources[encoding] = value;
}

void SharcDaiPin::advance(std::uint64_t half_periods) {
  if (staged_) { throw std::logic_error("SHARC DAI pin clock event during active instruction"); }
  require_clock();
  for (unsigned int index = 0; index < current_.pending.size(); ++index) {
    auto &pending = current_.pending[index];
    if (!pending) { continue; }
    if (half_periods >= guaranteed_effect - pending->half_periods) {
      current_.routes[index] = pending->target;
      pending.reset();
    } else {
      pending->half_periods += static_cast<unsigned int>(half_periods);
    }
  }
}

void SharcDaiPin::advance_pclk(std::uint32_t clocks) {
  advance(std::uint64_t{clocks} * 2);
}

void SharcDaiPin::advance_half_pclk(std::uint32_t half_periods) {
  advance(half_periods);
}

std::uint64_t SharcDaiPin::read(MemoryAccess access) {
  const auto index = register_index(access);
  require_clock();
  const auto &state = working();
  if (state.pending[index]) {
    throw std::runtime_error("SHARC DAI pin unsettled readback timing not qualified");
  }
  return state.routes[index];
}

void SharcDaiPin::write(MemoryAccess access, std::uint64_t value) {
  const auto index = register_index(access);
  require_clock();
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized SHARC DAI pin write");
  }
  const auto target = static_cast<std::uint32_t>(value);
  validate_word(target);
  auto &state = working();
  if (state.pending[index]) {
    throw std::runtime_error("SHARC DAI pin overlapping write timing not qualified");
  }
  state.pending[index] = SharcDaiPinPending{.target = target};
}

void SharcDaiPin::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested SHARC DAI pin routing instruction"); }
  staged_ = current_;
}

void SharcDaiPin::commit_instruction() noexcept {
  if (staged_) { current_ = *staged_; }
  staged_.reset();
}

void SharcDaiPin::abort_instruction() noexcept {
  staged_.reset();
}

void SharcDaiPin::require_clock() const {
  if (!power_.enabled(SharcClockGate::dai)) {
    throw std::runtime_error("SHARC pin routing with stopped DAI clock not qualified");
  }
}

SharcDaiPinState &SharcDaiPin::working() {
  return staged_ ? *staged_ : current_;
}

const SharcDaiPinState &SharcDaiPin::working() const {
  return staged_ ? *staged_ : current_;
}

}
