// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t all_bits = std::numeric_limits<std::uint32_t>::max();
constexpr std::uint32_t sign_bit = 0x80000000;
constexpr unsigned int word_bits = 32;
constexpr unsigned int arithmetic_shift = 4;
constexpr unsigned int bit_set = 0xc0;
constexpr unsigned int bit_test = 0xcc;
constexpr std::uint64_t immediate_operation_mask = 63;
constexpr std::uint64_t byte_mask = 0xff;
constexpr std::uint32_t shift_zero = 0x1000;
constexpr std::uint32_t shift_overflow = 0x800;
constexpr std::uint32_t shift_flags = 0x3800; // SZ/SV/SS; no ALU/multiplier/FIFO flags.

struct ShiftInstruction {
  unsigned int source{};
  unsigned int destination{};
  unsigned int count_register{};
  std::uint32_t count{};
  bool immediate{};
  unsigned int operation{};
};

ShiftInstruction decode(std::uint64_t opcode, bool immediate) {
  ShiftInstruction fields{.source = static_cast<unsigned int>(opcode & 15), .immediate = immediate};
  unsigned int operation{};
  if (immediate) {
    constexpr auto native_class = std::uint64_t{2} << 40;
    constexpr auto condition_mask = std::uint64_t{31} << 33;
    constexpr auto extra_mask = std::uint64_t{15} << 27;
    constexpr auto immediate_mask = (std::uint64_t{1} << 22) - 1;
    constexpr auto allowed = native_class | condition_mask | extra_mask | immediate_mask;
    if ((opcode & ~allowed) != 0) {
      throw std::runtime_error("SHARC immediate shift memory/reserved/VISA fields not implemented");
    }
    operation = static_cast<unsigned int>((opcode >> 16) & immediate_operation_mask) << 2;
    fields.destination = static_cast<unsigned int>((opcode >> 4) & 15);
    fields.count = static_cast<std::uint32_t>((opcode >> 8) & byte_mask);
    constexpr std::uint32_t immediate_sign = 0x80;
    constexpr std::uint32_t sign_extension = 0xffffff00;
    if ((fields.count & immediate_sign) != 0) { fields.count |= sign_extension; }
    // DATAEX supplies only the extra bits of >8-bit field operations. Basic
    // shifts use signed DATA8, including the original kernel's DATAEX=15.
  } else {
    operation = static_cast<unsigned int>((opcode >> 12) & byte_mask);
    fields.source = static_cast<unsigned int>((opcode >> 4) & 15);
    fields.destination = static_cast<unsigned int>((opcode >> 8) & 15);
    fields.count_register = static_cast<unsigned int>(opcode & 15);
  }
  if (operation != 0 && operation != arithmetic_shift &&
      (operation < bit_set || operation > bit_test || (operation & 3) != 0)) {
    throw std::runtime_error("SHARC rotate/OR/bit-field/other shifter operation not implemented");
  }
  fields.operation = operation;
  return fields;
}

SharcKnown32 operand(SharcState &state, unsigned int reg, unsigned int element) {
  const auto data = sharc_compute_register(state, reg, element);
  return {.value = static_cast<std::uint32_t>(data.value >> 8),
          .known = static_cast<std::uint32_t>(data.known >> 8)};
}

SharcKnown32 sign_fill(SharcKnown32 input) {
  if ((input.known & sign_bit) == 0) { return {}; }
  return {.value = (input.value & sign_bit) != 0 ? all_bits : 0, .known = all_bits};
}

SharcKnown32 shifted(SharcKnown32 input, std::uint32_t count, bool arithmetic) {
  if (count == 0) { return input; }
  const bool negative = (count & sign_bit) != 0;
  const auto amount = negative ? (std::uint64_t{1} << word_bits) - count : count;
  if (amount >= word_bits) {
    return negative && arithmetic ? sign_fill(input) : SharcKnown32{.value = 0, .known = all_bits};
  }
  const auto distance = static_cast<unsigned int>(amount);
  if (!negative) {
    const auto zeros = static_cast<std::uint32_t>((std::uint64_t{1} << distance) - 1);
    return {.value = static_cast<std::uint32_t>(std::uint64_t{input.value} << distance),
            .known = static_cast<std::uint32_t>(std::uint64_t{input.known} << distance) | zeros};
  }
  const auto fill = all_bits ^ (all_bits >> distance);
  SharcKnown32 result{.value = input.value >> distance, .known = input.known >> distance};
  if (!arithmetic) {
    result.known |= fill;
  } else if ((input.known & sign_bit) != 0) {
    result.known |= fill;
    if ((input.value & sign_bit) != 0) { result.value |= fill; }
  }
  return result;
}

struct ShiftResult {
  SharcKnown32 word;
  bool left{};
};

void status(SharcState &state, ShiftResult result, unsigned int element) {
  SharcKnown32 flags{.value = result.left ? shift_overflow : 0, .known = shift_flags & ~shift_zero};
  if (result.word.value != 0 || result.word.known == all_bits) {
    flags.known |= shift_zero;
    if (result.word.value == 0) { flags.value |= shift_zero; }
  }
  sharc_shift_status(state, flags, element);
}

void lane(SharcState &state, ShiftInstruction fields, unsigned int element) {
  const auto input = operand(state, fields.source, element);
  const auto count =
      fields.immediate ? fields.count : sharc_value(operand(state, fields.count_register, element));
  if (fields.operation >= bit_set) {
    sharc_bit_shift(state,
                    input,
                    {.position = count,
                     .operation = static_cast<SharcBitOperation>(fields.operation),
                     .destination = fields.destination},
                    element);
    return;
  }
  const auto result = shifted(input, count, fields.operation == arithmetic_shift);
  constexpr std::uint64_t extension = 0xff;
  sharc_compute_register(state, fields.destination, element) = {
      .value = std::uint64_t{result.value} << 8,
      .known = (std::uint64_t{result.known} << 8) | extension};
  status(state, {.word = result, .left = count != 0 && (count & sign_bit) == 0}, element);
}

}

void sharc_shift_status(SharcState &state, SharcKnown32 flags, unsigned int element) {
  for (auto *view :
       {&state.system[sharc_astat_x + element], &state.effective[sharc_astat_x + element]}) {
    view->value = (view->value & ~shift_flags) | flags.value;
    view->known = (view->known & ~shift_flags) | flags.known;
  }
}

void sharc_shift(SharcState &state, std::uint64_t opcode, bool immediate) {
  const auto fields = decode(opcode, immediate);
  const auto mode = state.effective[sharc_mode1];
  if ((mode.known & sharc_simd) == 0) {
    throw std::runtime_error("Unknown SHARC shifter SIMD mode");
  }
  const auto condition = static_cast<unsigned int>((opcode >> 33) & 31);
  const bool execute_x = sharc_pe_condition(state, condition, 0);
  const bool execute_y = (mode.value & sharc_simd) != 0 && sharc_pe_condition(state, condition, 1);
  if (execute_x) { lane(state, fields, 0); }
  if (execute_y) { lane(state, fields, 1); }
}

}
