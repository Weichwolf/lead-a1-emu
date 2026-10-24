// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "sharc_immediate_indexed.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <exception>
#include <sstream>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint64_t write_bit = std::uint64_t{1} << 31;

}

void sharc_indexed_move(SharcMemory &memory,
                        SharcState &state,
                        std::uint64_t opcode,
                        const SharcControl *control) try {
  constexpr std::uint64_t long_word = std::uint64_t{1} << 30;
  if ((opcode & long_word) != 0) {
    throw std::runtime_error("SHARC forced-LW indexed transfer not implemented");
  }
  if (!sharc_data_move_condition(state, opcode)) { return; }
  constexpr unsigned int index_shift = 41;
  constexpr unsigned int modifier_shift = 38;
  const auto bank = static_cast<unsigned int>((opcode >> 32) & 1);
  if (bank != 0) { sharc_cache_program_access(state); }
  const auto index = static_cast<unsigned int>((opcode >> index_shift) & 7) + (bank * 8);
  const auto modifier = static_cast<unsigned int>((opcode >> modifier_shift) & 7) + (bank * 8);
  const auto reg = static_cast<unsigned int>((opcode >> 23) & 127);
  sharc_dag_register_guard(reg, bank);
  sharc_addressing_guard(state, index);
  const auto current = sharc_value(sharc_register_read(state, 16 + index));
  const auto delta = sharc_value(sharc_register_read(state, 32 + modifier));
  const auto next = static_cast<std::uint32_t>(std::uint64_t{current} + delta);
  constexpr std::uint64_t update = std::uint64_t{1} << 44;
  const bool post = (opcode & update) != 0;
  sharc_ram_transfer(memory,
                     state,
                     {.access = {.address = post ? current : next},
                      .ureg = reg,
                      .write = (opcode & write_bit) != 0},
                     control);
  if (post) {
    constexpr std::uint32_t all_bits = 0xffffffff;
    sharc_register_write(state, 16 + index, {.value = next, .known = all_bits});
  }
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC register-indexed " << ((opcode & write_bit) != 0 ? "write" : "read") << ": "
          << error.what();
  throw std::runtime_error(message.str());
}

}
