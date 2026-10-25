// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm_fault.hpp"

#include "arm/core/arm.hpp"
#include "arm/core/arm_mmu_internal.hpp"
#include "arm/core/arm_system.hpp"
#include "common/memory.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t nibble_mask = 15;
constexpr unsigned int source_shift = 16;
constexpr unsigned int destination_shift = 12;
constexpr std::uint32_t load_bit = 0x00100000;
constexpr unsigned int pc_index = 15;

bool valid(const std::optional<ArmFaultStatus> &value) {
  return !value ||
         (value->status <= nibble_mask && (!value->domain || *value->domain <= nibble_mask));
}

std::uint32_t status_value(const std::optional<ArmFaultStatus> &value) {
  if (!value) { throw std::runtime_error("Unknown ARM fault status register"); }
  if (!value->domain) {
    throw std::runtime_error("Unknown ARM fault domain (invalid for this fault)");
  }
  return static_cast<std::uint32_t>(value->status) |
         (static_cast<std::uint32_t>(*value->domain) << 4);
}

}

void validate_fault_registers(const ArmFaultRegisters &state) {
  if (!valid(state.data) || !valid(state.instruction)) {
    throw std::invalid_argument("Invalid ARM fault status/domain input");
  }
}

std::uint32_t read_fault_register(const ArmFaultRegisters &state, FaultRegister reg) {
  if (reg.crn == 5 && reg.selector < 2) {
    return status_value(reg.selector == 0 ? state.data : state.instruction);
  }
  if (reg.crn == 6 && reg.selector == 0) {
    if (!state.address) { throw std::runtime_error("Unknown ARM fault address register"); }
    return *state.address;
  }
  throw std::runtime_error("Unsupported ARM fault register selector");
}

void write_fault_register(ArmFaultRegisters &state, FaultRegister reg, std::uint32_t value) {
  if (reg.crn == 6 && reg.selector == 0) {
    state.address = value;
    return;
  }
  if (reg.crn != 5 || reg.selector > 1) {
    throw std::runtime_error("Unsupported ARM fault register selector");
  }
  constexpr std::uint32_t writable = 0x1ff;
  if ((value & ~writable) != 0) {
    throw std::runtime_error("Unsupported ARM fault status SBZP bits");
  }
  const ArmFaultStatus next{.status = static_cast<std::uint8_t>(value & nibble_mask),
                            .domain = static_cast<std::uint8_t>((value >> 4) & nibble_mask)};
  (reg.selector == 0 ? state.data : state.instruction) = next;
}

void latch_mmu_fault(ArmFaultRegisters &state, const MmuFault &fault) {
  const ArmFaultStatus next{.status = fault.status(), .domain = fault.domain()};
  if (fault.kind() == AccessKind::fetch) {
    state.instruction = next;
  } else {
    state.data = next;
    state.address = fault.modified_address();
  }
}

void restore_data_abort_registers(std::uint32_t opcode, const ArmState &before, ArmState &after) {
  constexpr std::uint32_t single_mask = 0x0c000000;
  constexpr std::uint32_t single_value = 0x04000000;
  constexpr std::uint32_t block_mask = 0x0e000000;
  constexpr std::uint32_t block_value = 0x08000000;
  constexpr std::uint32_t extra_mask = 0x0e000090;
  constexpr std::uint32_t extra_value = 0x90;
  constexpr std::uint32_t swap_mask = 0x0fb00ff0;
  constexpr std::uint32_t swap_value = 0x01000090;
  const bool single = (opcode & single_mask) == single_value;
  const bool block = (opcode & block_mask) == block_value;
  constexpr std::uint32_t extra_operation_mask = 0x60;
  const bool extra = (opcode & extra_mask) == extra_value && (opcode & extra_operation_mask) != 0;
  const bool swap = (opcode & swap_mask) == swap_value;
  if (!single && !block && !extra && !swap) {
    throw std::runtime_error("Unqualified ARM data-abort instruction class");
  }
  const auto base = (opcode >> source_shift) & nibble_mask;
  after.registers[base] = before.registers[base];
  if (swap || ((single || extra) && (opcode & load_bit) != 0)) {
    const auto destination = (opcode >> destination_shift) & nibble_mask;
    if (destination != pc_index) { after.registers[destination] = before.registers[destination]; }
  }
  after.cpsr = before.cpsr;
  after.registers[pc_index] = before.registers[pc_index];
}

}
