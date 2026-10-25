// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_dpi_routing.hpp"

#include "common/memory.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr unsigned int fields = 5;
constexpr unsigned int field_bits = 6;
constexpr unsigned int field_mask = 63;
constexpr unsigned int first_dynamic = 2;
constexpr unsigned int earliest_effect = 4;
constexpr unsigned int guaranteed_effect = 6;

SharcDpiState reset(SharcDpiGroup group) {
  constexpr std::array<std::uint32_t, 3> pins{0x18017556, 0x004db699, 0x00450000};
  constexpr std::array<std::uint32_t, 3> enables{0x0d00c28b, 0x002103ce, 0x00185964};
  switch (group) {
    case SharcDpiGroup::pin: return {.routes = pins};
    case SharcDpiGroup::enable: return {.routes = enables};
    default: throw std::invalid_argument("Invalid SHARC DPI routing group");
  }
}

unsigned int selector(std::uint32_t value, unsigned int field) {
  return (value >> (field * field_bits)) & field_mask;
}

unsigned int register_index(MemoryAccess access) {
  constexpr unsigned int bytes = 4;
  constexpr unsigned int mapped_bytes = 12;
  if (access.size != bytes || access.address % bytes != 0 || access.address >= mapped_bytes) {
    throw std::runtime_error("SHARC DPI routing requires aligned32-bit mapped offsets0..8");
  }
  return access.address / bytes;
}

std::optional<bool> level(const SharcDpiState &state, unsigned int encoding) {
  if (encoding < first_dynamic) { return encoding == 1; }
  return state.sources[encoding - first_dynamic];
}

}

SharcDpiRouting::SharcDpiRouting(SharcDpiGroup group, std::optional<SharcDpiState> initial)
    : group_(group), current_(reset(group)) {
  if (initial) { current_ = *initial; }
  validate(current_);
}

bool SharcDpiRouting::valid_source(unsigned int encoding) const {
  constexpr unsigned int pin_last = 50;
  constexpr unsigned int pin_hole0 = 18;
  constexpr unsigned int pin_hole1 = 20;
  constexpr unsigned int enable_last = 43;
  constexpr unsigned int enable_hole0 = 7;
  constexpr unsigned int enable_hole1 = 9;
  if (group_ == SharcDpiGroup::pin) {
    return encoding <= pin_last && encoding != pin_hole0 && encoding != pin_hole1;
  }
  return encoding <= enable_last && encoding != enable_hole0 && encoding != enable_hole1;
}

void SharcDpiRouting::validate_word(RegisterValue word) const {
  for (unsigned int field = 0; field < fields && (word.index * fields) + field < sharc_dpi_pins;
       ++field) {
    if (!valid_source(selector(word.value, field))) {
      throw std::invalid_argument("Reserved SHARC DPI routing source selection");
    }
  }
}

void SharcDpiRouting::validate(const SharcDpiState &state) const {
  for (unsigned int index = 0; index < state.routes.size(); ++index) {
    validate_word({.index = index, .value = state.routes[index]});
    const auto pending = state.pending[index];
    if (pending) {
      validate_word({.index = index, .value = pending->target});
      if (pending->half_periods >= guaranteed_effect) {
        throw std::invalid_argument("Invalid SHARC DPI pending effect interval");
      }
    }
  }
  for (unsigned int index = 0; index < state.sources.size(); ++index) {
    if (state.sources[index].has_value() && !valid_source(index + first_dynamic)) {
      throw std::invalid_argument("Reserved SHARC DPI source input state");
    }
  }
}

SharcDpiState SharcDpiRouting::state() const {
  return working();
}

SharcDpiState SharcDpiRouting::snapshot() const {
  if (staged_) { throw std::logic_error("SHARC DPI snapshot during active instruction"); }
  return current_;
}

SharcDpiOutputs SharcDpiRouting::outputs() const {
  SharcDpiOutputs result{};
  for (unsigned int pin = 0; pin < result.size(); ++pin) {
    const auto index = pin / fields;
    const auto field = pin % fields;
    auto value = level(current_, selector(current_.routes[index], field));
    const auto pending = current_.pending[index];
    if (pending && pending->half_periods >= earliest_effect) {
      const auto target = level(current_, selector(pending->target, field));
      if (value != target) { value.reset(); }
    }
    result[pin] = value;
  }
  return result;
}

SharcDpiDrive SharcDpiRouting::drives(const SharcDpiRouting &enables) const {
  if (group_ != SharcDpiGroup::pin || enables.group_ != SharcDpiGroup::enable) {
    throw std::invalid_argument("SHARC DPI drives require pin then enable routers");
  }
  const auto data = outputs();
  const auto enabled = enables.outputs();
  SharcDpiDrive result;
  for (unsigned int pin = 0; pin < data.size(); ++pin) {
    const auto bit = static_cast<std::uint16_t>(1U << pin);
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

void SharcDpiRouting::source(unsigned int encoding, std::optional<bool> value) {
  if (staged_) { throw std::logic_error("SHARC DPI source event during active instruction"); }
  if (encoding < first_dynamic || !valid_source(encoding)) {
    throw std::invalid_argument("Invalid SHARC DPI dynamic source encoding");
  }
  current_.sources[encoding - first_dynamic] = value;
}

void SharcDpiRouting::advance(std::uint64_t half_periods) {
  if (staged_) { throw std::logic_error("SHARC DPI clock event during active instruction"); }
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

void SharcDpiRouting::advance_pclk(std::uint32_t clocks) {
  advance(std::uint64_t{clocks} * 2);
}

void SharcDpiRouting::advance_half_pclk(std::uint32_t half_periods) {
  advance(half_periods);
}

std::uint64_t SharcDpiRouting::read(MemoryAccess access) {
  const auto index = register_index(access);
  const auto &state = working();
  if (state.pending[index]) {
    throw std::runtime_error("SHARC DPI unsettled readback timing not qualified");
  }
  return state.routes[index];
}

void SharcDpiRouting::write(MemoryAccess access, std::uint64_t value) {
  const auto index = register_index(access);
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized SHARC DPI routing write");
  }
  const auto target = static_cast<std::uint32_t>(value);
  validate_word({.index = index, .value = target});
  auto &state = working();
  if (state.pending[index]) {
    throw std::runtime_error("SHARC DPI overlapping write timing not qualified");
  }
  state.pending[index] = SharcDpiPending{.target = target};
}

void SharcDpiRouting::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested SHARC DPI routing instruction"); }
  staged_ = current_;
}

void SharcDpiRouting::commit_instruction() noexcept {
  if (staged_) { current_ = *staged_; }
  staged_.reset();
}

void SharcDpiRouting::abort_instruction() noexcept {
  staged_.reset();
}

SharcDpiState &SharcDpiRouting::working() {
  return staged_ ? *staged_ : current_;
}

const SharcDpiState &SharcDpiRouting::working() const {
  return staged_ ? *staged_ : current_;
}

}
