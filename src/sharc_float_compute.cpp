// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_float.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t round32 = 0x10000;
constexpr std::uint32_t truncate = 0x8000;
constexpr std::uint32_t alu_flags = 0x43f;
constexpr std::uint32_t floating_flag = 0x400;
constexpr std::uint32_t multiply_flags = 0x3c0;
constexpr std::uint32_t multiply_negative = 0x40;
constexpr std::uint64_t sign = 0x8000000000;
constexpr std::uint64_t magnitude = 0x7fffffffff;

std::uint64_t input(SharcState &state, unsigned int reg, unsigned int element, bool short_input) {
  const auto value = sharc_compute_register(state, reg, element);
  const auto required = short_input ? sharc_data_mask & ~std::uint64_t{0xff} : sharc_data_mask;
  if ((value.known & required) != required) {
    throw std::runtime_error("Unknown SHARC floating register operand");
  }
  return value.value & required;
}

}

bool sharc_float_multiply_encoding(std::uint64_t opcode) {
  constexpr std::uint64_t fields = 0x7ff000;
  constexpr std::uint64_t multiply = 0x130000;
  constexpr unsigned int class_shift = 40;
  constexpr std::uint64_t dual_mask = 0xe00000000000;
  constexpr std::uint64_t dual_move = 0x200000000000;
  constexpr std::uint64_t register_mask = 0xf00000000000;
  constexpr std::uint64_t register_move = 0x700000000000;
  const bool compute = (opcode >> class_shift) == 1 || (opcode & dual_mask) == dual_move ||
                       (opcode & register_mask) == register_move;
  return compute && (opcode & fields) == multiply;
}

void sharc_float_lane(SharcState &state, std::uint64_t opcode, unsigned int element) {
  const auto mode = state.effective[sharc_mode1];
  if ((mode.known & (round32 | truncate)) != (round32 | truncate)) {
    throw std::runtime_error("Unknown SHARC floating rounding/precision mode");
  }
  const bool narrow = (mode.value & round32) != 0;
  const auto left = input(state, static_cast<unsigned int>((opcode >> 4) & 15), element, narrow);
  const auto right = input(state, static_cast<unsigned int>(opcode & 15), element, narrow);
  constexpr unsigned int operation_shift = 12;
  constexpr unsigned int subtract = 0x82;
  const auto operation = static_cast<unsigned int>((opcode >> operation_shift) & 0xff);
  const auto format = narrow ? SharcFloatFormat::binary32 : SharcFloatFormat::extended40;
  const auto rounding = (mode.value & truncate) != 0 ? SharcFloatRounding::toward_zero
                                                     : SharcFloatRounding::nearest_even;
  const bool multiply = sharc_float_multiply_encoding(opcode);
  const auto result = multiply
                          ? sharc_float_multiply(left, right, format, rounding)
                          : sharc_float_add(left, right, format, rounding, operation == subtract);
  const bool zero = (result.value & magnitude) == 0;
  // PGR A-19: zero/denormal operands never cause a multiplier underflow.
  // Positive zero has an unambiguous cleared MN. Negative-zero MN is open.
  const bool guarded_zero = zero && (!multiply || result.value != 0);
  if (result.underflow || result.overflow || guarded_zero) {
    throw std::runtime_error("SHARC floating zero/exception status not qualified");
  }
  const auto destination = static_cast<unsigned int>((opcode >> 8) & 15);
  sharc_compute_register(state, destination, element) = {.value = result.value,
                                                         .known = sharc_data_mask};
  const auto mask = multiply ? multiply_flags : alu_flags;
  const auto negative = multiply ? multiply_negative : 4U;
  const auto flags = (multiply ? 0U : floating_flag) | ((result.value & sign) != 0 ? negative : 0U);
  for (auto *view :
       {&state.system[sharc_astat_x + element], &state.effective[sharc_astat_x + element]}) {
    view->value = (view->value & ~mask) | flags;
    view->known |= mask;
  }
  // Normal results and genuine positive-zero products change only unit flags.
}

}
