// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_fetch.hpp"
#include "sharc_dag_internal.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <ios>
#include <sstream>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint64_t dual_mask = 0xe00000000000;
constexpr std::uint64_t dual = 0x200000000000;
constexpr std::uint64_t register_mask = 0xf00000000000;
constexpr std::uint64_t register_move = 0x700000000000;

std::uint32_t data_bit(unsigned int reg) {
  return reg < 16 ? std::uint32_t{1} << reg : 0;
}

std::uint32_t possible_writes(std::uint64_t opcode) {
  constexpr unsigned int class_shift = 40;
  constexpr std::uint64_t compute = 0x7fffff;
  const auto type = opcode >> class_shift;
  const bool dual_move = (opcode & dual_mask) == dual;
  const bool parallel_move = (opcode & register_mask) == register_move;
  std::uint32_t writes{};
  if ((type == 1 || dual_move || parallel_move) && (opcode & compute) != 0) {
    writes |= data_bit(static_cast<unsigned int>((opcode >> 8) & 15));
  }
  if (type == 2) { writes |= data_bit(static_cast<unsigned int>((opcode >> 4) & 15)); }
  const auto destination = sharc_fetch_destination(opcode);
  if (destination) { writes |= data_bit(*destination); }
  const auto transfer = sharc_dag_instruction(opcode);
  if (transfer && !transfer->write) { writes |= data_bit(transfer->reg); }
  if (dual_move) {
    constexpr unsigned int dm_write = 44;
    constexpr unsigned int pm_write = 37;
    constexpr unsigned int dm_register = 33;
    constexpr unsigned int pm_register = 23;
    if (((opcode >> dm_write) & 1) == 0) {
      writes |= data_bit(static_cast<unsigned int>((opcode >> dm_register) & 15));
    }
    if (((opcode >> pm_write) & 1) == 0) {
      writes |= data_bit(static_cast<unsigned int>((opcode >> pm_register) & 15));
    }
  }
  return writes;
}

}

void sharc_fetch_multiplier_guard(const SharcFetchState &state) {
  if (!state.decode || !state.address || !sharc_float_multiply_encoding(state.decode->opcode)) {
    return;
  }
  const auto consumer = state.decode->opcode;
  const auto left = static_cast<unsigned int>((consumer >> 4) & 15);
  const auto right = static_cast<unsigned int>(consumer & 15);
  const auto writes = possible_writes(state.address->opcode);
  if ((writes & data_bit(left)) == 0 || (writes & data_bit(right)) == 0) { return; }
  // PGR3-14/4-111 requires an extra cycle for dual ALU/MUL operand forwarding.
  // The combined pipeline phase contract is not qualified yet. Also guard
  // possible load/parallel/conditional producers rather than guess a bypass.
  std::ostringstream message;
  message << "SHARC dual multiplier operand forwarding timing not qualified: Decode pc=0x"
          << std::hex << state.decode->address << " opcode=0x" << consumer << " Address pc=0x"
          << state.address->address << " opcode=0x" << state.address->opcode;
  throw std::runtime_error(message.str());
}

}
