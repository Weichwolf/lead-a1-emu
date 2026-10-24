// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int class_shift = 40;
constexpr std::uint64_t compute_mask = 0x7fffff;
constexpr unsigned int ureg_mask = 127;
constexpr unsigned int move_destination_shift = 23;
constexpr unsigned int system_operation_shift = 37;
constexpr std::uint32_t shift_flags = 0x3800;

}

std::optional<unsigned int> sharc_fetch_destination(std::uint64_t opcode) {
  const auto type = opcode >> class_shift;
  constexpr unsigned int immediate = 15;
  constexpr unsigned int dm_read = 16;
  constexpr unsigned int pm_read = 18;
  if (type == immediate || type == dm_read || type == pm_read) {
    return static_cast<unsigned int>((opcode >> 32) & ureg_mask);
  }
  constexpr std::uint64_t register_mask = 0xf00000000000;
  constexpr std::uint64_t register_move = 0x700000000000;
  if ((opcode & register_mask) == register_move) {
    return static_cast<unsigned int>((opcode >> move_destination_shift) & ureg_mask);
  }
  constexpr unsigned int system_bit = 20;
  if (type == system_bit && ((opcode >> system_operation_shift) & 7) < 3) {
    constexpr unsigned int system_base = 0x70;
    return system_base + static_cast<unsigned int>((opcode >> 32) & 15);
  }
  return {};
}

namespace {

std::uint32_t compute_flags(std::uint64_t opcode) {
  constexpr std::uint64_t unit_mask = 0x700000;
  constexpr std::uint64_t shifter = 0x200000;
  constexpr std::uint32_t alu_flags = 0x43f;
  if ((opcode & compute_mask) == 0) { return 0; }
  if ((opcode & unit_mask) == 0) { return alu_flags; }
  if ((opcode & unit_mask) == shifter) { return shift_flags; }
  constexpr std::uint32_t multiply_flags = 0x3c0;
  if (sharc_float_multiply_encoding(opcode)) { return multiply_flags; }
  throw std::runtime_error("SHARC multiplier/multifunction branch hazard not qualified");
}

std::uint32_t produced(std::uint64_t opcode) {
  const auto type = opcode >> class_shift;
  constexpr unsigned int compute = 1;
  constexpr unsigned int immediate_shift = 2;
  constexpr unsigned int system_bit = 20;
  constexpr std::uint64_t dual_mask = 0xe00000000000;
  constexpr std::uint64_t dual_move = 0x200000000000;
  constexpr std::uint64_t register_mask = 0xf00000000000;
  constexpr std::uint64_t register_move = 0x700000000000;
  if (type == compute || (opcode & dual_mask) == dual_move ||
      (opcode & register_mask) == register_move) {
    return compute_flags(opcode);
  }
  if (type == immediate_shift) { return shift_flags; }
  if (type == system_bit) {
    const auto operation = (opcode >> system_operation_shift) & 7;
    if (operation == 4 || operation == 5) { return sharc_btf; }
  }
  return 0;
}

}

SharcFetchInterlock sharc_fetch_interlock(const SharcState &cpu,
                                          const SharcFetchState &previous) try {
  const auto branch = previous.decode;
  if (!branch || !sharc_fetch_branch_encoding(branch->opcode)) { return SharcFetchInterlock::none; }
  const auto condition = static_cast<unsigned int>((branch->opcode >> 33) & 31);
  const auto consumed = sharc_fetch_condition_mask(condition);
  const auto producer = previous.address;
  const auto reg = producer ? sharc_fetch_destination(producer->opcode) : std::nullopt;
  constexpr unsigned int mode1 = 0x72;
  constexpr unsigned int pc_stack = 0x64;
  if (consumed != 0 && (cpu.pending[sharc_mode1] || reg == mode1)) {
    throw std::runtime_error("SHARC conditional branch MODE1 Decode interlock not qualified");
  }
  if (previous.flow && previous.flow->multiplier_branch_wait) {
    return SharcFetchInterlock::condition;
  }
  if (producer && consumed != 0 && (produced(producer->opcode) & consumed) != 0) {
    return sharc_float_multiply_encoding(producer->opcode) ? SharcFetchInterlock::multiplier
                                                           : SharcFetchInterlock::condition;
  }
  constexpr unsigned int rts = 10;
  if ((branch->opcode >> class_shift) == rts && reg == pc_stack &&
      sharc_condition(cpu, condition)) {
    throw std::runtime_error("SHARC return after PCSTK write interlock not qualified");
  }
  return SharcFetchInterlock::none;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC interlock Decode=";
  if (previous.decode) {
    message << "0x" << std::hex << previous.decode->address << " opcode=0x"
            << previous.decode->opcode;
  } else {
    message << "bubble";
  }
  message << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
