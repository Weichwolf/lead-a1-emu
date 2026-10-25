// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc_immediate_indexed.hpp"

#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/memory/sharc_memory.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

unsigned int address_space(std::uint32_t address) {
  constexpr std::uint32_t normal = 0x80000;
  constexpr std::uint32_t short_word = 0x100000;
  constexpr std::uint32_t ram_end = 0x200000;
  if (address >= normal && address < short_word) { return 1; }
  if (address >= short_word && address < ram_end) { return 2; }
  return 0;
}

}

bool sharc_pre_modify_space(std::uint32_t before, std::uint32_t address) {
  return address_space(before) == address_space(address);
}

std::optional<SharcImmediateIndexed> sharc_immediate_indexed_instruction(std::uint64_t opcode) {
  constexpr std::uint64_t type_mask = 0xe00000000000;
  constexpr std::uint64_t immediate_indexed = 0xa00000000000;
  if ((opcode & type_mask) != immediate_indexed) { return {}; }
  constexpr unsigned int group_shift = 44;
  constexpr unsigned int index_shift = 41;
  constexpr unsigned int write_shift = 40;
  constexpr unsigned int long_shift = 39;
  constexpr unsigned int register_shift = 32;
  constexpr unsigned int register_mask = 127;
  const bool program = ((opcode >> group_shift) & 1) != 0;
  return SharcImmediateIndexed{
      .modifier = static_cast<std::uint32_t>(opcode),
      .index = static_cast<unsigned int>((opcode >> index_shift) & 7) + (program ? 8U : 0U),
      .reg = static_cast<unsigned int>((opcode >> register_shift) & register_mask),
      .program = program,
      .write = ((opcode >> write_shift) & 1) != 0,
      .long_word = ((opcode >> long_shift) & 1) != 0};
}

void sharc_dag_register_guard(unsigned int reg, unsigned int group) {
  constexpr unsigned int dag_begin = 16;
  constexpr unsigned int dag_end = 80;
  if (reg >= dag_begin && reg < dag_end && ((reg & 15) / 8) == group) {
    throw std::runtime_error("SHARC same-DAG UREG/index addressing prohibited");
  }
}

void sharc_immediate_indexed_move(SharcMemory &memory,
                                  SharcState &state,
                                  SharcImmediateIndexed instruction,
                                  const SharcControl *control) try {
  if (instruction.long_word) {
    throw std::runtime_error("SHARC forced-LW immediate-indexed transfer not implemented");
  }
  sharc_dag_register_guard(instruction.reg, instruction.program ? 1U : 0U);
  sharc_addressing_guard(state, instruction.index);
  const auto before = sharc_value(sharc_register_read(state, 16 + instruction.index));
  const auto address = static_cast<std::uint32_t>(std::uint64_t{before} + instruction.modifier);
  if (!sharc_pre_modify_space(before, address)) {
    throw std::runtime_error("SHARC pre-modify space crossing not qualified");
  }
  if (instruction.program) { sharc_cache_program_access(state); }
  sharc_ram_transfer(
      memory,
      state,
      {.access = {.address = address}, .ureg = instruction.reg, .write = instruction.write},
      control);
  // Type15a is pre-modify only: the architectural index is never written.
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC immediate-indexed " << (instruction.program ? "PM " : "DM ")
          << (instruction.write ? "write" : "read") << " I" << instruction.index << " modifier=0x"
          << std::hex << instruction.modifier << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
