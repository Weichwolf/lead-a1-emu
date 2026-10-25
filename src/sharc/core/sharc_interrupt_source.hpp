// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "sharc/core/sharc.hpp"

#include <cstdint>
#include <optional>

namespace lead_a1::detail {

inline constexpr unsigned int sharc_lirptl = 13;
inline constexpr std::uint32_t sharc_lirptl_latches = 0x3ff;
inline constexpr std::uint32_t sharc_lirptl_pointers = 0x3ff00000;

struct SharcInterruptSource {
  SharcInterruptBank bank{};
  std::uint32_t mask{};
  std::uint32_t offset{};
};

[[nodiscard]] SharcInterruptSource sharc_programmable_source(unsigned int priority);
[[nodiscard]] std::optional<SharcInterruptSource> sharc_pending_interrupt(const SharcState &state);
[[nodiscard]] bool sharc_programmable_source(SharcInterruptSource source);
[[nodiscard]] unsigned int sharc_latch_register(SharcInterruptBank bank);
void sharc_latch_source(SharcState &state, SharcInterruptBank bank, std::uint32_t mask, bool set);

}
