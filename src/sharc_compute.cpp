// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "sharc_internal.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t all_bits = std::numeric_limits<std::uint32_t>::max();
constexpr std::uint32_t sign_bit = 0x80000000;
constexpr std::uint32_t alu_flags = 0x43f; // AZ/AV/AN/AC/AS/AI/AF, PGR A-17..19.
constexpr std::uint64_t extension = 0xff;
constexpr unsigned int pass_operation = 0x21;
constexpr unsigned int and_operation = 0x40;
constexpr unsigned int or_operation = 0x41;
constexpr unsigned int xor_operation = 0x42;
constexpr unsigned int not_operation = 0x43;
constexpr unsigned int add_operation = 1;
constexpr unsigned int subtract_operation = 2;
constexpr unsigned int increment_operation = 0x29;
constexpr unsigned int decrement_operation = 0x2a;

struct AluResult {
  SharcKnown32 value;
  std::uint32_t flags{};
};

bool arithmetic_operation(unsigned int operation) {
  return operation == add_operation || operation == subtract_operation ||
         operation == increment_operation || operation == decrement_operation;
}

bool unary_operation(unsigned int operation) {
  return operation == pass_operation || operation == not_operation ||
         operation == increment_operation || operation == decrement_operation;
}

AluResult
arithmetic(const SharcState &state, unsigned int operation, std::array<SharcKnown32, 2> inputs) {
  const auto left = sharc_value(inputs[0]);
  const auto right = unary_operation(operation) ? 1U : sharc_value(inputs[1]);
  const bool subtract = operation == subtract_operation || operation == decrement_operation;
  const auto adder_right = subtract ? ~right : right;
  const auto sum = std::uint64_t{left} + adder_right + (subtract ? 1U : 0U);
  auto result = static_cast<std::uint32_t>(sum);
  const bool overflow = (~(left ^ adder_right) & (left ^ result) & sign_bit) != 0;
  constexpr std::uint32_t carry_flag = 8;
  constexpr std::uint32_t overflow_flag = 2;
  std::uint32_t flags = (sum >> 32) != 0 ? carry_flag : 0;
  if (overflow) {
    flags |= overflow_flag;
    constexpr std::uint32_t saturation = 0x2000;
    const auto mode = state.effective[sharc_mode1];
    if ((mode.known & saturation) == 0) {
      throw std::runtime_error("Unknown SHARC ALU saturation mode on overflow");
    }
    if ((mode.value & saturation) != 0) {
      result = (left & sign_bit) != 0 ? sign_bit : sign_bit - 1;
    }
  }
  return {.value = {.value = result, .known = all_bits}, .flags = flags};
}

SharcKnown32 operand(SharcState &state, unsigned int reg, unsigned int element) {
  const auto word = sharc_compute_register(state, reg, element);
  return {.value = static_cast<std::uint32_t>(word.value >> 8),
          .known = static_cast<std::uint32_t>(word.known >> 8)};
}

SharcKnown32 logical(SharcKnown32 left, SharcKnown32 right, unsigned int operation) {
  switch (operation) {
    case pass_operation: return left;
    case and_operation:
      return {.value = left.value & right.value,
              .known = (left.known & right.known) | (left.known & ~left.value) |
                       (right.known & ~right.value)};
    case or_operation:
      return {.value = left.value | right.value,
              .known = (left.known & right.known) | left.value | right.value};
    case xor_operation:
      return {.value = (left.value ^ right.value) & left.known & right.known,
              .known = left.known & right.known};
    case not_operation: return {.value = ~left.value & left.known, .known = left.known};
    default:
      throw std::runtime_error("SHARC arithmetic/float/compare ALU operation not implemented");
  }
}

void status(SharcState &state, AluResult result, unsigned int element) {
  SharcKnown32 flags{.value = result.flags, .known = alu_flags & ~std::uint32_t{5}};
  if (result.value.value != 0 || result.value.known == all_bits) {
    flags.known |= 1;
    if (result.value.value == 0) { flags.value |= 1; }
  }
  if ((result.value.known & sign_bit) != 0) {
    flags.known |= 4;
    if ((result.value.value & sign_bit) != 0) { flags.value |= 4; }
  }
  for (auto *view :
       {&state.system[sharc_astat_x + element], &state.effective[sharc_astat_x + element]}) {
    view->value = (view->value & ~alu_flags) | flags.value;
    view->known = (view->known & ~alu_flags) | flags.known;
  }
  constexpr std::uint32_t overflow_flag = 2;
  if ((result.flags & overflow_flag) != 0) {
    constexpr std::uint32_t sticky_overflow = 4;
    for (auto *view :
         {&state.system[sharc_stky_x + element], &state.effective[sharc_stky_x + element]}) {
      view->value |= sticky_overflow;
      view->known |= sticky_overflow;
    }
  }
}

void lane(SharcState &state, std::uint64_t opcode, unsigned int element) {
  constexpr unsigned int operation_shift = 12;
  const auto operation = static_cast<unsigned int>((opcode >> operation_shift) & 0xff);
  const auto destination = static_cast<unsigned int>((opcode >> 8) & 15);
  const auto left = operand(state, static_cast<unsigned int>((opcode >> 4) & 15), element);
  SharcKnown32 right;
  if (!unary_operation(operation)) {
    right = operand(state, static_cast<unsigned int>(opcode & 15), element);
  }
  const bool same_source = ((opcode >> 4) & 15) == (opcode & 15);
  AluResult result;
  if (operation == subtract_operation && same_source) {
    result = {.value = {.value = 0, .known = all_bits}, .flags = 8};
  } else if (arithmetic_operation(operation)) {
    result = arithmetic(state, operation, {left, right});
  } else {
    result.value = operation == xor_operation && same_source
                       ? SharcKnown32{.value = 0, .known = all_bits}
                       : logical(left, right, operation);
  }
  sharc_compute_register(state, destination, element) = {
      .value = std::uint64_t{result.value.value} << 8,
      .known = (std::uint64_t{result.value.known} << 8) | extension};
  status(state, result, element);
}

}

void sharc_compute(SharcState &state, std::uint64_t opcode) {
  constexpr std::uint64_t allowed = 0x013e007fffff;
  if ((opcode & ~allowed) != 0) {
    throw std::runtime_error("SHARC VISA/reserved compute-only fields not implemented");
  }
  constexpr std::uint64_t compute_mask = 0x7fffff;
  if ((opcode & compute_mask) == 0) {
    throw std::runtime_error("SHARC empty compute-only form not qualified");
  }
  const auto mode = state.effective[sharc_mode1];
  if ((mode.known & sharc_simd) == 0) {
    throw std::runtime_error("Unknown SHARC compute SIMD mode");
  }
  constexpr std::uint64_t unit_mask = 0x700000;
  constexpr std::uint64_t shifter_unit = 0x200000;
  if ((opcode & unit_mask) == shifter_unit) {
    sharc_shift(state, opcode, false);
    return;
  }
  if ((opcode & unit_mask) != 0) {
    throw std::runtime_error("SHARC multiplier/shifter/multifunction compute not implemented");
  }
  const auto condition = static_cast<unsigned int>((opcode >> 33) & 31);
  const bool execute_x = sharc_pe_condition(state, condition, 0);
  const bool execute_y = (mode.value & sharc_simd) != 0 && sharc_pe_condition(state, condition, 1);
  if (execute_x) { lane(state, opcode, 0); }
  if (execute_y) { lane(state, opcode, 1); }
}

}
