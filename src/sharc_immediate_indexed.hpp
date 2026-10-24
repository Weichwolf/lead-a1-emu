// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>

namespace lead_a1 {
class SharcMemory;
class SharcControl;
struct SharcState;

namespace detail {

struct SharcImmediateIndexed {
  std::uint32_t modifier{};
  unsigned int index{};
  unsigned int reg{};
  bool program{};
  bool write{};
  bool long_word{};
};

[[nodiscard]] std::optional<SharcImmediateIndexed>
sharc_immediate_indexed_instruction(std::uint64_t opcode);
void sharc_dag_register_guard(unsigned int reg, unsigned int group);
[[nodiscard]] bool sharc_pre_modify_space(std::uint32_t before, std::uint32_t address);
void sharc_immediate_indexed_move(SharcMemory &memory,
                                  SharcState &state,
                                  SharcImmediateIndexed instruction,
                                  const SharcControl *control);

}
}
