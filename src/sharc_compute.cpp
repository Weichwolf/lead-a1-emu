// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "sharc_internal.hpp"

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

void status(SharcState &state, SharcKnown32 result, unsigned int element) {
  SharcKnown32 flags{.known = alu_flags & ~std::uint32_t{5}};
  if (result.value != 0 || result.known == all_bits) {
    flags.known |= 1;
    if (result.value == 0) { flags.value |= 1; }
  }
  if ((result.known & sign_bit) != 0) {
    flags.known |= 4;
    if ((result.value & sign_bit) != 0) { flags.value |= 4; }
  }
  for (auto *view :
       {&state.system[sharc_astat_x + element], &state.effective[sharc_astat_x + element]}) {
    view->value = (view->value & ~alu_flags) | flags.value;
    view->known = (view->known & ~alu_flags) | flags.known;
  }
}

void lane(SharcState &state, std::uint64_t opcode, unsigned int element) {
  constexpr unsigned int operation_shift = 12;
  const auto operation = static_cast<unsigned int>((opcode >> operation_shift) & 0xff);
  const auto destination = static_cast<unsigned int>((opcode >> 8) & 15);
  const auto left = operand(state, static_cast<unsigned int>((opcode >> 4) & 15), element);
  SharcKnown32 right;
  if (operation != pass_operation && operation != not_operation) {
    right = operand(state, static_cast<unsigned int>(opcode & 15), element);
  }
  const bool same_source = ((opcode >> 4) & 15) == (opcode & 15);
  const auto result = operation == xor_operation && same_source
                          ? SharcKnown32{.value = 0, .known = all_bits}
                          : logical(left, right, operation);
  sharc_compute_register(state, destination, element) = {
      .value = std::uint64_t{result.value} << 8,
      .known = (std::uint64_t{result.known} << 8) | extension};
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
