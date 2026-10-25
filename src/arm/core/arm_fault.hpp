// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "arm/core/arm.hpp"
#include "arm/core/arm_system.hpp"

#include <cstdint>

namespace lead_a1::detail {

class MmuFault;

struct FaultRegister {
  unsigned int crn;
  unsigned int selector;
};

void validate_fault_registers(const ArmFaultRegisters &state);
[[nodiscard]] std::uint32_t read_fault_register(const ArmFaultRegisters &state, FaultRegister reg);
void write_fault_register(ArmFaultRegisters &state, FaultRegister reg, std::uint32_t value);
void latch_mmu_fault(ArmFaultRegisters &state, const MmuFault &fault);
void restore_data_abort_registers(std::uint32_t opcode, const ArmState &before, ArmState &after);

}
