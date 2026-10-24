// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_internal.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int class_shift = 40;
constexpr std::uint64_t dual_mask = 0xe00000000000;
constexpr std::uint64_t dual_move = 0x200000000000;
constexpr std::uint64_t register_mask = 0xf00000000000;
constexpr std::uint64_t register_move = 0x700000000000;
constexpr std::array fixed_alu{
    1U, 2U, 0x0aU, 0x0bU, 0x21U, 0x29U, 0x2aU, 0x40U, 0x41U, 0x42U, 0x43U};
constexpr std::array fixed_shifter{0U, 4U, 0xc0U, 0xc4U, 0xc8U, 0xccU};

bool fixed_inputs(std::uint64_t opcode) {
  const auto type = opcode >> class_shift;
  constexpr unsigned int immediate_shift = 2;
  if (type == immediate_shift) {
    constexpr std::uint64_t operation_mask = 63;
    const auto operation = static_cast<unsigned int>((opcode >> 16) & operation_mask) << 2;
    if (std::ranges::find(fixed_shifter, operation) != fixed_shifter.end()) { return true; }
    throw std::runtime_error("SHARC immediate shifter input-format interlock not qualified");
  }
  if (type != 1 && (opcode & dual_mask) != dual_move && (opcode & register_mask) != register_move) {
    return false;
  }
  constexpr std::uint64_t compute = 0x7fffff;
  if ((opcode & compute) == 0 || sharc_float_multiply_encoding(opcode)) { return false; }
  constexpr std::uint64_t unit_mask = 0x700000;
  constexpr std::uint64_t shifter = 0x200000;
  const auto operation = static_cast<unsigned int>((opcode >> 12) & 255);
  if ((opcode & unit_mask) == 0) {
    constexpr unsigned int floating_add = 0x81;
    constexpr unsigned int floating_subtract = 0x82;
    if (operation == floating_add || operation == floating_subtract) { return false; }
    if (std::ranges::find(fixed_alu, operation) != fixed_alu.end()) { return true; }
  }
  if ((opcode & unit_mask) == shifter &&
      std::ranges::find(fixed_shifter, operation) != fixed_shifter.end()) {
    return true;
  }
  throw std::runtime_error("SHARC compute input-format interlock not qualified");
}

}

bool sharc_fetch_fixed_interlock(const SharcState &cpu, const SharcFetchState &state) {
  if (!state.decode || !state.address || !sharc_float_multiply_encoding(state.address->opcode) ||
      !fixed_inputs(state.decode->opcode)) {
    return false;
  }
  constexpr unsigned int mode1 = 0x72;
  if (!state.flow || state.flow->loop || state.flow->discard_decode != 0 || cpu.branch ||
      cpu.pending[sharc_mode1] || sharc_fetch_destination(state.address->opcode) == mode1) {
    throw std::runtime_error(
        "SHARC floating-to-fixed interlock with mode/control/loop not qualified");
  }
  // PGR4-118: this input-format transition stalls Decode regardless of DREG identity.
  return true;
}

}
