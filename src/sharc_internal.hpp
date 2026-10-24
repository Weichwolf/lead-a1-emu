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
inline constexpr unsigned int sharc_flags = 5;
inline constexpr unsigned int sharc_astat_x = 6;
inline constexpr unsigned int sharc_astat_y = 7;
inline constexpr unsigned int sharc_stky_x = 8;
inline constexpr unsigned int sharc_irptl = 10;
inline constexpr unsigned int sharc_imask = 11;
inline constexpr std::uint32_t sharc_simd = 0x200000;
inline constexpr std::uint32_t sharc_btf = 0x40000;
inline constexpr std::uint64_t sharc_data_mask = 0xffffffffff;

[[nodiscard]] std::uint32_t sharc_value(SharcKnown32 word);
[[nodiscard]] SharcKnown32 sharc_ureg(const SharcState &state, unsigned int ureg);
void sharc_ureg(SharcState &state, unsigned int ureg, SharcKnown32 value);
[[nodiscard]] bool sharc_data_ureg(unsigned int ureg);
[[nodiscard]] SharcKnown32 sharc_register_read(const SharcState &state, unsigned int ureg);
void sharc_register_write(SharcState &state, unsigned int ureg, SharcKnown32 value);
void sharc_validate_registers(const SharcState &state);
[[nodiscard]] SharcKnown40 &sharc_data_register(SharcState &state, unsigned int reg);

struct SharcDataTransfer {
  SharcAccess access;
  unsigned int ureg{};
  bool write{};
};

void sharc_ram_transfer(SharcMemory &memory,
                        SharcState &state,
                        SharcDataTransfer transfer,
                        const SharcControl *control);
void sharc_px_ram_transfer(SharcMemory &memory, SharcState &state, SharcDataTransfer transfer);
[[nodiscard]] bool
sharc_px_register_move(SharcState &state, unsigned int source, unsigned int destination);
[[nodiscard]] bool sharc_px_half(unsigned int reg);
[[nodiscard]] SharcKnown32 sharc_px_half_read(const SharcState &state, unsigned int reg);
void sharc_px_half_write(SharcState &state, unsigned int reg, SharcKnown32 value);
void sharc_short_transfer(
    SharcMemory &memory, SharcKnown40 &data, SharcKnown32 mode, SharcAccess access, bool write);
[[nodiscard]] bool sharc_data_move_condition(const SharcState &state, std::uint64_t opcode);
void sharc_addressing_guard(const SharcState &state, unsigned int index);
void sharc_indexed_move(SharcMemory &memory,
                        SharcState &state,
                        std::uint64_t opcode,
                        const SharcControl *control);
[[nodiscard]] SharcKnown40 &
sharc_compute_register(SharcState &state, unsigned int reg, unsigned int element);
void sharc_compute(SharcState &state, std::uint64_t opcode);
[[nodiscard]] bool sharc_compare_encoding(std::uint64_t opcode);
void sharc_compare_lane(SharcState &state, std::uint64_t opcode, unsigned int element);
[[nodiscard]] bool sharc_float_multiply_encoding(std::uint64_t opcode);
void sharc_float_lane(SharcState &state, std::uint64_t opcode, unsigned int element);
void sharc_shift(SharcState &state, std::uint64_t opcode, bool immediate);
void sharc_shift_status(SharcState &state, SharcKnown32 flags, unsigned int element);
enum class SharcBitOperation : std::uint8_t {
  set = 0xc0,
  clear = 0xc4,
  toggle = 0xc8,
  test = 0xcc
};

struct SharcBitInstruction {
  std::uint32_t position{};
  SharcBitOperation operation{};
  unsigned int destination{};
};

void sharc_bit_shift(SharcState &state,
                     SharcKnown32 input,
                     SharcBitInstruction instruction,
                     unsigned int element);
void sharc_register_move(SharcState &state, std::uint64_t opcode);
void sharc_dual_move(SharcMemory &memory,
                     SharcState &state,
                     std::uint64_t opcode,
                     const SharcControl *control);
void sharc_data_move(SharcMemory &memory,
                     SharcState &state,
                     std::uint64_t opcode,
                     const SharcControl *control);
void sharc_bit_operation(SharcState &state, std::uint64_t opcode);
[[nodiscard]] SharcKnown32 sharc_flags_read(const SharcState &state);
void sharc_flags_write(SharcState &state, SharcKnown32 value, std::uint32_t mask);
void sharc_validate_flag_inputs(SharcKnown32 samples);
[[nodiscard]] bool sharc_condition(const SharcState &state, unsigned int code);
[[nodiscard]] bool
sharc_pe_condition(const SharcState &state, unsigned int code, unsigned int element);
void sharc_interrupt_guard(const SharcState &state);
void sharc_stack_status(SharcState &state);
void sharc_validate_count_loop(const SharcState &state);
void sharc_begin_count_loop(SharcState &state, std::uint64_t opcode);
void sharc_push_call(SharcState &state, std::uint32_t address);
[[nodiscard]] std::uint32_t sharc_pop_call(SharcState &state);
[[nodiscard]] bool sharc_four_loop(const SharcState &state);
void sharc_validate_four_loop(const SharcState &state);
[[nodiscard]] unsigned int sharc_four_loop_prepare(SharcState &state, std::uint64_t opcode);
void sharc_four_loop_finish(const SharcState &previous, SharcState &next, bool clocked);
void sharc_four_loop_clock_pop(SharcState &state);
[[nodiscard]] bool sharc_loop_body_supported(std::uint64_t opcode);
[[nodiscard]] unsigned int
sharc_count_loop_step(SharcState &state, std::uint64_t opcode, bool clocked = false);
void sharc_clocked_loop_pop(SharcState &state);
void sharc_long_loop_prepare(SharcState &state, std::uint64_t opcode);
void sharc_long_loop_finish(const SharcState &previous, SharcState &next);
void sharc_validate_arithmetic_loop(const SharcState &state);
void sharc_begin_arithmetic_loop(SharcState &state, std::uint64_t opcode);
void sharc_arithmetic_loop_prepare(SharcState &state, std::uint64_t opcode);
[[nodiscard]] unsigned int sharc_arithmetic_loop_finish(const SharcState &previous,
                                                        SharcState &next);
void sharc_arithmetic_stack_write(SharcState &state, SharcKnown32 value);
[[nodiscard]] SharcKnown32 sharc_loop_address(const SharcState &state);
void sharc_validate_cache(const SharcCacheState &cache);
void sharc_cache_flush(SharcState &state, std::uint64_t opcode);
void sharc_cache_guard(const SharcState &state, std::uint64_t opcode);
void sharc_cache_program_access(SharcState &state);
[[nodiscard]] bool sharc_cache_pm_request(const SharcState &state, std::uint64_t opcode);
[[nodiscard]] bool sharc_cache_pm_encoding(std::uint64_t opcode);
void sharc_cache_finish(const SharcState &previous, SharcState &next);

void sharc_interrupt_environment_guard(const SharcState &state);
[[nodiscard]] bool sharc_interrupt_deferred(const SharcState &state);
void sharc_interrupt_service_guard(const SharcState &state);
void sharc_validate_interrupt(const SharcState &state);
[[nodiscard]] std::optional<SharcInterruptStep>
sharc_service_interrupt(SharcState &state, SharcInterruptReference reference);
[[nodiscard]] std::optional<std::uint32_t> sharc_return_interrupt(
    SharcState &state, std::uint64_t opcode, std::optional<bool> captured = std::nullopt);
void sharc_interrupt_finish(const SharcState &previous, SharcState &next);

void sharc_validate_idle(const SharcState &state);
void sharc_begin_idle(SharcState &state, bool fetched);
[[nodiscard]] bool sharc_wake_idle(SharcState &state);

}
