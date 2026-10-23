// SPDX-License-Identifier: GPL-3.0-only
#include "arm_status.hpp"

#include "lead_a1/arm.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t mode_mask = 0x1f;
constexpr std::uint32_t user_mode = 0x10;
constexpr std::uint32_t flags_mask = 0xf8000000;
constexpr std::uint32_t privileged_mask = 0x000000df;
constexpr std::uint32_t state_mask = 0x01000020;
constexpr std::uint32_t status_mask = flags_mask | privileged_mask | state_mask;
constexpr unsigned int register_mask = 15;
constexpr unsigned int pc_index = 15;
constexpr unsigned int field_shift = 16;
constexpr unsigned int saved_bit = 22;
constexpr unsigned int immediate_bit = 25;
constexpr unsigned int condition_shift = 28;
constexpr unsigned int byte_bits = 8;
constexpr std::uint32_t byte_mask = 0xff;

struct StatusOperation {
  bool read{};
  bool saved{};
  bool immediate{};
  unsigned int reg{};
  std::uint32_t fields{};
};

std::optional<StatusOperation> decode_status(std::uint32_t opcode) {
  constexpr std::uint32_t mrs_mask = 0x0fbf0fff;
  constexpr std::uint32_t mrs_value = 0x010f0000;
  constexpr std::uint32_t register_msr_mask = 0x0fb0fff0;
  constexpr std::uint32_t register_msr_value = 0x0120f000;
  constexpr std::uint32_t immediate_msr_mask = 0x0fb0f000;
  constexpr std::uint32_t immediate_msr_value = 0x0320f000;
  const bool read = (opcode & mrs_mask) == mrs_value;
  if (!read && (opcode & register_msr_mask) != register_msr_value &&
      (opcode & immediate_msr_mask) != immediate_msr_value) {
    return std::nullopt;
  }
  constexpr unsigned int destination_shift = 12;
  return StatusOperation{.read = read,
                         .saved = ((opcode >> saved_bit) & 1U) != 0,
                         .immediate = !read && ((opcode >> immediate_bit) & 1U) != 0,
                         .reg = read ? (opcode >> destination_shift) & register_mask
                                     : opcode & register_mask,
                         .fields = (opcode >> field_shift) & register_mask};
}

bool condition_passes(std::uint32_t opcode, const ArmState &state) {
  constexpr std::array<unsigned int, 4> bits{31, 30, 29, 28};
  const bool negative = ((state.cpsr >> bits[0]) & 1U) != 0;
  const bool zero = ((state.cpsr >> bits[1]) & 1U) != 0;
  const bool carry = ((state.cpsr >> bits[2]) & 1U) != 0;
  const bool overflow = ((state.cpsr >> bits[3]) & 1U) != 0;
  const std::array<bool, 15> outcomes{zero,
                                      !zero,
                                      carry,
                                      !carry,
                                      negative,
                                      !negative,
                                      overflow,
                                      !overflow,
                                      carry && !zero,
                                      !carry || zero,
                                      negative == overflow,
                                      negative != overflow,
                                      !zero && negative == overflow,
                                      zero || negative != overflow,
                                      true};
  const auto condition = opcode >> condition_shift;
  if (condition >= outcomes.size()) {
    throw std::runtime_error("Unsupported unconditional ARM status encoding");
  }
  return outcomes[condition];
}

void check_unimplemented_banked_form(std::uint32_t opcode, const ArmState &state) {
  constexpr std::uint32_t instruction_class_mask = 0x0e000000;
  constexpr std::uint32_t immediate_class = 0x02000000;
  constexpr std::uint32_t extra_register_mask = 0x90;
  constexpr std::uint32_t set_flags = 0x00100000;
  constexpr std::uint32_t destination_mask = 0x0000f000;
  constexpr std::uint32_t banked_block_mask = 0x0e400000;
  constexpr std::uint32_t banked_block_value = 0x08400000;
  const auto instruction_class = opcode & instruction_class_mask;
  const bool arithmetic =
      instruction_class == immediate_class ||
      (instruction_class == 0 && (opcode & extra_register_mask) != extra_register_mask);
  const bool exception_return =
      arithmetic && (opcode & set_flags) != 0 && (opcode & destination_mask) == destination_mask;
  const bool banked_block = (opcode & banked_block_mask) == banked_block_value;
  if ((exception_return || banked_block) && condition_passes(opcode, state)) {
    throw std::runtime_error("Unsupported ARM exception return or banked block transfer");
  }
}

std::uint32_t field_mask(std::uint32_t fields) {
  std::uint32_t mask{};
  for (unsigned int field = 0; field < 4; ++field) {
    if (((fields >> field) & 1U) != 0) { mask |= byte_mask << (field * byte_bits); }
  }
  return mask;
}

std::uint32_t
operand(std::uint32_t opcode, const ArmState &state, const StatusOperation &operation) {
  if (operation.immediate) {
    constexpr unsigned int rotation_shift = 8;
    const auto rotation = ((opcode >> rotation_shift) & register_mask) * 2;
    const auto value = opcode & byte_mask;
    if (rotation == 0) { return value; }
    constexpr unsigned int word_bits = 32;
    const auto left = static_cast<std::uint64_t>(value) << (word_bits - rotation);
    const auto rotated = (value >> rotation) | left;
    return static_cast<std::uint32_t>(rotated & std::numeric_limits<std::uint32_t>::max());
  }
  if (operation.reg == pc_index) { throw std::runtime_error("Unpredictable MSR operand PC"); }
  const auto value = state.registers[operation.reg];
  if (!value) { throw std::runtime_error("Uninitialized ARM status operand register"); }
  return *value;
}

void restore_banks(ArmState &state) {
  const auto index = arm_bank_index(state.cpsr);
  const auto high = index == 1 ? 1U : 0U;
  for (std::size_t reg = 0; reg < state.banks.high[high].size(); ++reg) {
    state.registers[8 + reg] = state.banks.high[high][reg];
  }
  state.registers[13] = state.banks.modes[index].sp;
  state.registers[14] = state.banks.modes[index].lr;
}

ArmSavedStatus &saved_status(ArmState &state) {
  const auto index = arm_bank_index(state.cpsr);
  if (index == 0) { throw std::runtime_error("Unpredictable SPSR access in User/System mode"); }
  return state.banks.modes[index].saved;
}

void transfer_saved(ArmState &state, const StatusOperation &operation, std::uint32_t value) {
  auto &saved = saved_status(state);
  if ((saved.bits & saved.known_mask & ~status_mask) != 0) {
    throw std::runtime_error("Unsupported reserved saved PSR bits");
  }
  if (operation.read) {
    if ((saved.known_mask & status_mask) != status_mask) {
      throw std::runtime_error("Uninitialized SPSR bits");
    }
    state.registers[operation.reg] = saved.bits & status_mask;
    return;
  }
  const auto mask = field_mask(operation.fields) & status_mask;
  if ((mask & mode_mask) == mode_mask) { static_cast<void>(arm_bank_index(value)); }
  saved.bits = (saved.bits & ~mask) | (value & mask);
  saved.known_mask |= mask;
}

void write_current(ArmState &state, const StatusOperation &operation, std::uint32_t value) {
  auto writable = flags_mask;
  if ((state.cpsr & mode_mask) != user_mode) {
    if ((value & state_mask) != 0) {
      throw std::runtime_error("Unpredictable MSR execution-state change");
    }
    writable |= privileged_mask;
  }
  const auto mask = field_mask(operation.fields) & writable;
  const auto updated = (state.cpsr & ~mask) | (value & mask);
  static_cast<void>(arm_bank_index(updated));
  capture_arm_banks(state);
  state.cpsr = updated;
  restore_banks(state);
}

}

std::size_t arm_bank_index(std::uint32_t cpsr) {
  if ((cpsr & ~status_mask) != 0) { throw std::runtime_error("Unsupported ARM status bits"); }
  constexpr std::array<std::uint32_t, 7> modes{0x10, 0x1f, 0x11, 0x12, 0x13, 0x17, 0x1b};
  const auto mode = cpsr & mode_mask;
  for (std::size_t index = 0; index < modes.size(); ++index) {
    if (modes[index] == mode) { return index == 0 ? 0 : index - 1; }
  }
  throw std::runtime_error("Unsupported ARM processor mode");
}

bool arm_condition_passes(std::uint32_t opcode, const ArmState &state) {
  return condition_passes(opcode, state);
}

void capture_arm_banks(ArmState &state) {
  const auto index = arm_bank_index(state.cpsr);
  const auto high = index == 1 ? 1U : 0U;
  for (std::size_t reg = 0; reg < state.banks.high[high].size(); ++reg) {
    state.banks.high[high][reg] = state.registers[8 + reg];
  }
  state.banks.modes[index].sp = state.registers[13];
  state.banks.modes[index].lr = state.registers[14];
}

bool execute_arm_status(std::uint32_t opcode, ArmState &state) {
  constexpr std::uint32_t instruction_mask = 0x0fffffff;
  constexpr std::uint32_t nop = 0x01a00000;
  if ((opcode & instruction_mask) == nop) {
    static_cast<void>(condition_passes(opcode, state));
    const auto pc = state.registers[pc_index];
    if (!pc) { throw std::runtime_error("Unknown ARM NOP PC"); }
    state.registers[pc_index] = static_cast<std::uint32_t>(
        (static_cast<std::uint64_t>(*pc) + 4) & std::numeric_limits<std::uint32_t>::max());
    return true;
  }
  const auto decoded = decode_status(opcode);
  if (!decoded) {
    check_unimplemented_banked_form(opcode, state);
    return false;
  }
  const auto &operation = *decoded;
  const auto pc = state.registers[pc_index];
  if (!pc) { throw std::runtime_error("Unknown ARM status instruction PC"); }
  if (condition_passes(opcode, state)) {
    if (operation.read && operation.reg == pc_index) {
      throw std::runtime_error("Unpredictable MRS destination PC");
    }
    const auto value = operation.read ? 0 : operand(opcode, state, operation);
    if ((value & ~status_mask) != 0) {
      throw std::runtime_error("Unpredictable reserved PSR bits");
    }
    if (operation.saved) {
      transfer_saved(state, operation, value);
    } else if (operation.read) {
      state.registers[operation.reg] = state.cpsr;
    } else {
      write_current(state, operation, value);
    }
  }
  constexpr auto address_mask = std::numeric_limits<std::uint32_t>::max();
  state.registers[pc_index] =
      static_cast<std::uint32_t>((static_cast<std::uint64_t>(*pc) + 4) & address_mask);
  return true;
}

}
