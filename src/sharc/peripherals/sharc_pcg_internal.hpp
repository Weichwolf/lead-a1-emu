// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "sharc/peripherals/sharc_pcg.hpp"

#include <cstdint>
#include <optional>

namespace lead_a1::pcg_detail {
struct Tap {
  unsigned int unit;
  unsigned int lane;
};

inline constexpr unsigned int units = 4;
inline constexpr unsigned int lanes = 2;
inline constexpr std::uint32_t divider_mask = 0xfffff;
[[nodiscard]] unsigned int control_word(unsigned int unit);
[[nodiscard]] std::uint32_t
divider(const SharcPcgState &state, unsigned int unit, unsigned int lane);
[[nodiscard]] unsigned int source(const SharcPcgState &state, Tap tap);
[[nodiscard]] bool enabled(const SharcPcgState &state, unsigned int unit, unsigned int lane);
[[nodiscard]] std::uint32_t phase(const SharcPcgState &state, unsigned int unit);
[[nodiscard]] std::uint32_t pulse(const SharcPcgState &state, unsigned int unit);
[[nodiscard]] std::uint32_t sync(const SharcPcgState &state, unsigned int unit);
[[nodiscard]] std::uint8_t mask(unsigned int unit, unsigned int lane);
void validate(const SharcPcgState &state);
void configure(SharcPcgState &next, const SharcPcgState &before);
void edge(SharcPcgState &state,
          unsigned int unit,
          unsigned int lane,
          std::optional<bool> before,
          std::optional<bool> after);
}
