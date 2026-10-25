// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "arm/core/arm.hpp"

#include <cstdint>
#include <optional>

namespace lead_a1::detail {

class ArmSystem;
[[nodiscard]] std::optional<ArmException> pending_arm_interrupt(const ArmState &state,
                                                                ArmInterruptLines lines);
[[nodiscard]] std::uint32_t arm_wrapped_address(std::uint64_t value);
void enter_arm_exception(ArmState &state, ArmException kind, std::uint32_t vector_base);
[[nodiscard]] bool
execute_arm_exception_return(std::uint32_t opcode, ArmState &state, ArmSystem &system);

}
