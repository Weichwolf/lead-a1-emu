// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_dag.hpp"

#include <cstdint>
#include <optional>

namespace lead_a1::detail {

struct SharcDagInstruction {
  unsigned int index{};
  unsigned int reg{};
  std::optional<unsigned int> modifier_register;
  std::uint32_t immediate{};
  bool post{};
  bool program{};
  bool write{};
  bool qualified{};
};

[[nodiscard]] std::optional<SharcDagInstruction> sharc_dag_instruction(std::uint64_t opcode);
[[nodiscard]] bool sharc_dag_loop_body(std::uint64_t opcode);
void sharc_validate_dag_address(std::uint64_t opcode, const SharcDagAddress &address);
[[nodiscard]] SharcWidth sharc_dag_width(const SharcControl *control,
                                         std::uint32_t address,
                                         const SharcDagInstruction &instruction);
void sharc_execute_dag(SharcMemory &memory,
                       SharcState &state,
                       std::uint64_t opcode,
                       const SharcControl *control,
                       const SharcDagAddress &address);

}
