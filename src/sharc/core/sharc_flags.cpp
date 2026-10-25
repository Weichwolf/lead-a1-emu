// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr unsigned int flag_count = 16;
constexpr std::uint32_t packed_flags = 0xffff;
constexpr std::uint32_t direction_bits = 0xaaaaaaaa;

}

namespace detail {

void sharc_validate_flag_inputs(SharcKnown32 samples) {
  if ((samples.known & ~packed_flags) != 0 || (samples.value & ~samples.known) != 0) {
    throw std::invalid_argument("Invalid SHARC FLAG input samples");
  }
}

SharcKnown32 sharc_flags_read(const SharcState &state) {
  const auto control = state.system[sharc_flags];
  SharcKnown32 result{.value = control.value & direction_bits,
                      .known = control.known & direction_bits};
  for (unsigned int pin = 0; pin < flag_count; ++pin) {
    const auto level = std::uint32_t{1} << (pin * 2);
    const auto sample = std::uint32_t{1} << pin;
    if ((state.flag_inputs.known & sample) != 0) {
      result.known |= level;
      if ((state.flag_inputs.value & sample) != 0) { result.value |= level; }
    }
  }
  return result;
}

void sharc_flags_write(SharcState &state, SharcKnown32 value, std::uint32_t mask) {
  const auto previous = state.system[sharc_flags];
  for (unsigned int pin = 0; pin < flag_count; ++pin) {
    const auto level = std::uint32_t{1} << (pin * 2);
    const auto direction = level << 1;
    if ((mask & level) == 0) { continue; }
    const bool same_direction = (previous.known & value.known & direction) != 0 &&
                                ((previous.value ^ value.value) & direction) == 0;
    if ((mask & direction) != 0 && !same_direction) {
      throw std::runtime_error("SHARC FLAGS direction and level require separate writes");
    }
    if ((previous.known & previous.value & direction) == 0) {
      throw std::runtime_error(
          "SHARC FLAGS level write without known output selection not qualified");
    }
  }
  state.system[sharc_flags] = {.value = (previous.value & ~mask) | (value.value & mask),
                               .known = (previous.known & ~mask) | (value.known & mask)};
  state.pending[sharc_flags] = state.system[sharc_flags];
}

}

void Sharc::flag_inputs(SharcKnown32 samples) {
  if (memory_.active()) {
    throw std::logic_error("SHARC FLAG sampling during a memory transaction");
  }
  detail::sharc_validate_flag_inputs(samples);
  state_.flag_inputs = samples;
}

SharcFlagDrive Sharc::flag_drives() const {
  const auto control = state_.effective[detail::sharc_flags];
  SharcFlagDrive result;
  for (unsigned int pin = 0; pin < flag_count; ++pin) {
    const auto level = std::uint32_t{1} << (pin * 2);
    const auto direction = level << 1;
    const auto packed = static_cast<std::uint16_t>(std::uint32_t{1} << pin);
    if ((control.known & direction) != 0 && (control.value & direction) == 0) {
      result.high_impedance |= packed;
    } else if ((control.known & (direction | level)) != (direction | level)) {
      result.unresolved |= packed;
    } else if ((control.value & level) != 0) {
      result.high |= packed;
    } else {
      result.low |= packed;
    }
  }
  return result;
}

}
