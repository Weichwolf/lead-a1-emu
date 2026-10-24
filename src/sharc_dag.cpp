// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_dag.hpp"

#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "sharc_dag_internal.hpp"
#include "sharc_immediate_indexed.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
std::optional<SharcDagInstruction> sharc_dag_instruction(std::uint64_t opcode) {
  constexpr std::uint64_t immediate_mask = 0xf00000000000;
  constexpr std::uint64_t immediate_move = 0x600000000000;
  constexpr std::uint64_t indexed_mask = 0xe00000000000;
  constexpr std::uint64_t indexed_move = 0x400000000000;
  constexpr std::uint64_t compute_bits = 0x7fffff;
  constexpr unsigned int condition_shift = 33;
  constexpr unsigned int always = 31;
  constexpr unsigned int index_shift = 41;
  constexpr unsigned int data_shift = 23;
  SharcDagInstruction result;
  if (const auto instruction = sharc_immediate_indexed_instruction(opcode)) {
    constexpr unsigned int combined_px = 0x6b;
    constexpr unsigned int complement_begin = 0x50;
    constexpr unsigned int complement_end = 0x60;
    result.index = instruction->index;
    result.reg = instruction->reg;
    result.immediate = instruction->modifier;
    result.program = instruction->program;
    result.write = instruction->write;
    result.qualified = !instruction->long_word &&
                       (result.reg < 16 || result.reg == combined_px ||
                        (result.reg >= complement_begin && result.reg < complement_end));
    return result;
  }
  bool plain_width = true;
  if ((opcode & immediate_mask) == immediate_move) {
    constexpr unsigned int program_shift = 40;
    constexpr unsigned int update_shift = 38;
    constexpr unsigned int write_shift = 39;
    constexpr unsigned int modifier_shift = 27;
    constexpr unsigned int modify_bits = 63;
    constexpr unsigned int negative = 32;
    constexpr std::uint32_t negative_base = 0xffffffc0;
    const auto value = static_cast<std::uint32_t>((opcode >> modifier_shift) & modify_bits);
    result.immediate = value < negative ? value : negative_base + value;
    result.program = ((opcode >> program_shift) & 1) != 0;
    result.post = ((opcode >> update_shift) & 1) != 0;
    result.write = ((opcode >> write_shift) & 1) != 0;
    result.reg = static_cast<unsigned int>((opcode >> data_shift) & 15);
  } else if ((opcode & indexed_mask) == indexed_move) {
    constexpr unsigned int modifier_shift = 38;
    constexpr unsigned int update_shift = 44;
    constexpr unsigned int write_shift = 31;
    constexpr unsigned int ureg_mask = 127;
    constexpr std::uint64_t long_word = std::uint64_t{1} << 30;
    result.program = ((opcode >> 32) & 1) != 0;
    result.post = ((opcode >> update_shift) & 1) != 0;
    result.write = ((opcode >> write_shift) & 1) != 0;
    result.modifier_register =
        static_cast<unsigned int>((opcode >> modifier_shift) & 7) + (result.program ? 8U : 0U);
    result.reg = static_cast<unsigned int>((opcode >> data_shift) & ureg_mask);
    plain_width = (opcode & long_word) == 0;
  } else {
    return {};
  }
  result.index =
      static_cast<unsigned int>((opcode >> index_shift) & 7) + (result.program ? 8U : 0U);
  constexpr unsigned int complement_begin = 0x50;
  constexpr unsigned int complement_end = 0x60;
  const bool data =
      result.reg < 16 || (result.reg >= complement_begin && result.reg < complement_end);
  result.qualified = data && plain_width && (opcode & compute_bits) == 0 &&
                     ((opcode >> condition_shift) & always) == always;
  return result;
}

bool sharc_dag_loop_body(std::uint64_t opcode) {
  const auto instruction = sharc_dag_instruction(opcode);
  return instruction && instruction->qualified && !instruction->program;
}

void sharc_validate_dag_address(std::uint64_t opcode, const SharcDagAddress &address) {
  const auto instruction = sharc_dag_instruction(opcode);
  const auto sum = static_cast<std::uint32_t>(std::uint64_t{address.before} + address.modifier);
  if (!instruction || !instruction->qualified || address.index != instruction->index ||
      address.bank > 1 || address.post != instruction->post ||
      address.address != (address.post ? address.before : sum) ||
      address.after != (address.post ? sum : address.before) ||
      (!instruction->modifier_register && address.modifier != instruction->immediate)) {
    throw std::invalid_argument("Invalid SHARC captured DAG address");
  }
  constexpr std::uint32_t normal = 0x80000;
  constexpr std::uint32_t short_word = 0x100000;
  constexpr std::uint32_t ram_end = 0x200000;
  if (!address.post && !sharc_pre_modify_space(address.before, address.address)) {
    throw std::invalid_argument("Invalid SHARC captured DAG pre-modify space crossing");
  }
  constexpr unsigned int combined_px = 0x6b;
  if (instruction->reg == combined_px) {
    if (address.address < normal || address.address >= short_word ||
        address.width != SharcWidth::wide48) {
      throw std::invalid_argument("Invalid SHARC captured DAG PX width/domain");
    }
    return;
  }
  if (address.address >= normal && address.address < short_word) {
    if (address.width != SharcWidth::normal32 && address.width != SharcWidth::extended40) {
      throw std::invalid_argument("Invalid SHARC captured DAG normal width");
    }
  } else if (address.address < short_word || address.address >= ram_end ||
             address.width != SharcWidth::short16) {
    throw std::invalid_argument("Invalid SHARC captured DAG RAM domain");
  }
}

SharcWidth sharc_dag_width(const SharcControl *control,
                           std::uint32_t address,
                           const SharcDagInstruction &instruction) {
  constexpr unsigned int combined_px = 0x6b;
  if (instruction.reg == combined_px) {
    constexpr std::uint32_t normal = 0x80000;
    constexpr std::uint32_t normal_end = 0x100000;
    if (address < normal || address >= normal_end) {
      throw std::runtime_error("SHARC DAG PX transfer outside normal-word RAM not qualified");
    }
    return SharcWidth::wide48;
  }
  constexpr std::uint32_t short_word = 0x100000;
  constexpr std::uint32_t ram_end = 0x200000;
  if (address >= short_word && address < ram_end) { return SharcWidth::short16; }
  if (control == nullptr) {
    throw std::runtime_error("SHARC DAG Decode width requires a bound SYSCTL reference");
  }
  return control->data_width(address);
}

void sharc_execute_dag(SharcMemory &memory,
                       SharcState &state,
                       std::uint64_t opcode,
                       const SharcControl *control,
                       const SharcDagAddress &address) {
  sharc_validate_dag_address(opcode, address);
  const auto instruction = sharc_dag_instruction(opcode);
  if (!instruction) { throw std::runtime_error("SHARC captured DAG retirement form changed"); }
  if (sharc_immediate_indexed_instruction(opcode)) {
    sharc_addressing_guard(state, instruction->index);
  } else if (!sharc_data_move_condition(state, opcode)) {
    throw std::runtime_error("SHARC captured DAG retirement form changed");
  }
  if (sharc_dag_width(control, address.address, *instruction) != address.width) {
    throw std::runtime_error("SHARC captured DAG width changed before retirement");
  }
  if (instruction->program) { sharc_cache_program_access(state); }
  sharc_ram_transfer(memory,
                     state,
                     {.access = {.address = address.address},
                      .ureg = instruction->reg,
                      .write = instruction->write},
                     control);
  if (address.post) {
    constexpr std::uint32_t all_bits = 0xffffffff;
    state.dag[address.bank][0][address.index] = {.value = address.after, .known = all_bits};
  }
}

}
