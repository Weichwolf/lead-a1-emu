// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/arm.hpp"

#include <cstddef>
#include <cstdint>

namespace lead_a1::detail {

[[nodiscard]] std::size_t arm_bank_index(std::uint32_t cpsr);
void capture_arm_banks(ArmState &state);
[[nodiscard]] bool execute_arm_status(std::uint32_t opcode, ArmState &state);

}
