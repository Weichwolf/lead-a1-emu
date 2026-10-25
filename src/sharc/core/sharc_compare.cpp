// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"

#include <cstdint>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t alu_flags = 0x43f;
constexpr std::uint32_t cacc = 0xff000000;
constexpr std::uint32_t newest = 0x80000000;

std::uint32_t input(SharcState &state, unsigned int reg, unsigned int element) {
  const auto word = sharc_compute_register(state, reg, element);
  return sharc_value({.value = static_cast<std::uint32_t>(word.value >> 8),
                      .known = static_cast<std::uint32_t>(word.known >> 8)});
}

void status(SharcKnown32 &view, std::uint32_t flags, bool greater) {
  const auto history = (view.value >> 1) & cacc;
  const auto history_known = (view.known >> 1) & cacc;
  constexpr std::uint32_t selected = alu_flags | cacc;
  view.value = (view.value & ~selected) | history | flags | (greater ? newest : 0);
  view.known = (view.known & ~selected) | history_known | alu_flags | newest;
}

}

bool sharc_compare_encoding(std::uint64_t opcode) {
  constexpr std::uint64_t fields = 0x7ff000;
  constexpr std::uint64_t signed_compare = 0x0a000;
  constexpr std::uint64_t unsigned_compare = 0x0b000;
  constexpr unsigned int class_shift = 40;
  constexpr std::uint64_t dual_mask = 0xe00000000000;
  constexpr std::uint64_t dual_move = 0x200000000000;
  constexpr std::uint64_t register_mask = 0xf00000000000;
  constexpr std::uint64_t register_move = 0x700000000000;
  const bool compute = (opcode >> class_shift) == 1 || (opcode & dual_mask) == dual_move ||
                       (opcode & register_mask) == register_move;
  return compute && ((opcode & fields) == signed_compare || (opcode & fields) == unsigned_compare);
}

void sharc_compare_lane(SharcState &state, std::uint64_t opcode, unsigned int element) {
  const auto left = input(state, static_cast<unsigned int>((opcode >> 4) & 15), element);
  const auto right = input(state, static_cast<unsigned int>(opcode & 15), element);
  constexpr unsigned int operation_shift = 12;
  constexpr unsigned int unsigned_operation = 11;
  const bool unsigned_compare = ((opcode >> operation_shift) & 255) == unsigned_operation;
  // Flipping the sign bit maps the signed32 ordering to an unsigned rank.
  const auto bias = unsigned_compare ? 0U : newest;
  const bool less = (left ^ bias) < (right ^ bias);
  const bool greater = (left ^ bias) > (right ^ bias);
  const auto flags = (left == right ? 1U : 0U) | (less ? 4U : 0U);
  for (auto *view :
       {&state.system[sharc_astat_x + element], &state.effective[sharc_astat_x + element]}) {
    status(*view, flags, greater);
  }
}

}
