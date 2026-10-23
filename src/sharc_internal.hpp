// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/sharc.hpp"

#include <cstdint>

namespace lead_a1::detail {

inline constexpr std::uint32_t sharc_pc_mask = 0xffffff;
inline constexpr std::uint32_t sharc_frame_mask = 0x3ffffff;
inline constexpr std::uint32_t sharc_call_frame = 0x1000000;
inline constexpr std::uint32_t sharc_special_frame = 0x2000000;
inline constexpr unsigned int sharc_mode1 = 2;
inline constexpr unsigned int sharc_astat_x = 6;
inline constexpr unsigned int sharc_astat_y = 7;
inline constexpr unsigned int sharc_stky_x = 8;
inline constexpr unsigned int sharc_irptl = 10;
inline constexpr unsigned int sharc_imask = 11;
inline constexpr std::uint32_t sharc_simd = 0x200000;
inline constexpr std::uint32_t sharc_btf = 0x40000;

[[nodiscard]] std::uint32_t sharc_value(SharcKnown32 word);
[[nodiscard]] SharcKnown32 sharc_ureg(const SharcState &state, unsigned int ureg);
void sharc_ureg(SharcState &state, unsigned int ureg, SharcKnown32 value);
void sharc_bit_operation(SharcState &state, std::uint64_t opcode);
[[nodiscard]] bool sharc_condition(const SharcState &state, unsigned int code);
void sharc_interrupt_guard(const SharcState &state);
void sharc_stack_status(SharcState &state);

}
