// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm_exception.hpp"

#include "arm/core/arm.hpp"
#include "arm/core/arm_status.hpp"
#include "arm/core/arm_system_internal.hpp"

#include <bit>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int pc_index = 15;
constexpr unsigned int lr_index = 14;
constexpr std::uint32_t mode_mask = 0x1f;
constexpr std::uint32_t irq_disable = 0x80;
constexpr std::uint32_t fiq_disable = 0x40;
constexpr std::uint32_t execution_state = 0x01000020;
constexpr std::uint32_t status_mask = 0xf90000ff;
constexpr std::uint64_t address_bytes = std::uint64_t{1} << 32;
constexpr std::uint32_t register_mask = 15;

struct Entry {
  std::uint32_t mode;
  std::uint32_t offset;
  std::uint32_t link_offset;
};

struct Arithmetic {
  unsigned int operation;
  std::uint32_t left;
  std::uint32_t right;
  std::uint32_t cpsr;
};

Entry entry(ArmException kind) {
  constexpr std::uint32_t undefined_mode = 0x1b;
  constexpr std::uint32_t supervisor_mode = 0x13;
  constexpr std::uint32_t abort_mode = 0x17;
  constexpr std::uint32_t irq_mode = 0x12;
  constexpr std::uint32_t fiq_mode = 0x11;
  constexpr std::uint32_t irq_vector = 0x18;
  constexpr std::uint32_t fiq_vector = 0x1c;
  switch (kind) {
    case ArmException::undefined_instruction:
      return {.mode = undefined_mode, .offset = 4, .link_offset = 4};
    case ArmException::software_interrupt:
      return {.mode = supervisor_mode, .offset = 8, .link_offset = 4};
    case ArmException::prefetch_abort: return {.mode = abort_mode, .offset = 12, .link_offset = 4};
    case ArmException::data_abort: return {.mode = abort_mode, .offset = 16, .link_offset = 8};
    case ArmException::irq: return {.mode = irq_mode, .offset = irq_vector, .link_offset = 4};
    case ArmException::fiq: return {.mode = fiq_mode, .offset = fiq_vector, .link_offset = 4};
    default: throw std::runtime_error("Invalid ARM exception kind");
  }
}

std::uint32_t reg(const ArmState &state, unsigned int index) {
  const auto value = state.registers.at(index);
  if (!value) { throw std::runtime_error("Uninitialized ARM exception operand register"); }
  return index == pc_index ? arm_wrapped_address(static_cast<std::uint64_t>(*value) + 8) : *value;
}

std::uint32_t return_status(const ArmState &state) {
  const auto bank = arm_bank_index(state.cpsr);
  if (bank == 0) {
    throw std::runtime_error("Unpredictable ARM exception return in User/System mode");
  }
  const auto &saved = state.banks.modes[bank].saved;
  if ((saved.known_mask & status_mask) != status_mask) {
    throw std::runtime_error("Unknown ARM exception-return SPSR bits");
  }
  if ((saved.bits & saved.known_mask & ~status_mask) != 0) {
    throw std::runtime_error("Invalid ARM exception-return reserved SPSR bits");
  }
  const auto value = saved.bits & status_mask;
  static_cast<void>(arm_bank_index(value));
  if ((value & execution_state) != 0) {
    throw std::runtime_error("Unsupported ARM exception return to Thumb/Jazelle state");
  }
  return value;
}

std::uint32_t rotate_right(std::uint32_t value, unsigned int rotation) {
  if (rotation == 0) { return value; }
  constexpr unsigned int word_bits = 32;
  return arm_wrapped_address((static_cast<std::uint64_t>(value) >> rotation) |
                             (static_cast<std::uint64_t>(value) << (word_bits - rotation)));
}

std::uint32_t shifter(std::uint32_t opcode, const ArmState &state) {
  constexpr std::uint32_t immediate = std::uint32_t{1} << 25;
  constexpr std::uint32_t byte_mask = 255;
  if ((opcode & immediate) != 0) {
    const auto rotation = ((opcode >> 8) & register_mask) * 2;
    return rotate_right(opcode & byte_mask, rotation);
  }
  if ((opcode & 16) != 0) {
    throw std::runtime_error("Unpredictable ARM register-shift exception return to PC");
  }
  const auto value = reg(state, opcode & register_mask);
  constexpr unsigned int shift_mask = 31;
  const auto shift = (opcode >> 7) & shift_mask;
  const auto type = (opcode >> 5) & 3;
  constexpr std::uint32_t sign = std::uint32_t{1} << 31;
  constexpr unsigned int word_bits = 32;
  switch (type) {
    case 0: return arm_wrapped_address(static_cast<std::uint64_t>(value) << shift);
    case 1: return shift == 0 ? 0 : value >> shift;
    case 2:
      if (shift == 0) {
        return (value & sign) != 0 ? std::numeric_limits<std::uint32_t>::max() : 0;
      }
      return arm_wrapped_address(
          static_cast<std::uint64_t>(value >> shift) |
          ((value & sign) != 0
               ? static_cast<std::uint64_t>(std::numeric_limits<std::uint32_t>::max())
                     << (word_bits - shift)
               : 0));
    case 3:
      if (shift == 0) {
        constexpr unsigned int carry_bit = 29;
        return (value >> 1) | (((state.cpsr >> carry_bit) & 1U) << (word_bits - 1));
      }
      return rotate_right(value, shift);
    default: throw std::runtime_error("Invalid ARM return shifter type");
  }
}

std::uint32_t calculate(const Arithmetic &operands) {
  const auto &[operation, left, right, cpsr] = operands;
  constexpr unsigned int carry_bit = 29;
  const auto carry = (cpsr >> carry_bit) & 1U;
  const auto extended = static_cast<std::uint64_t>(left);
  switch (operation) {
    case 0: return left & right;
    case 1: return left ^ right;
    case 2: return arm_wrapped_address(extended + address_bytes - right);
    case 3: return arm_wrapped_address(static_cast<std::uint64_t>(right) + address_bytes - left);
    case 4: return arm_wrapped_address(extended + right);
    case 5: return arm_wrapped_address(extended + right + carry);
    case 6: return arm_wrapped_address(extended + address_bytes - right - (1U - carry));
    case 7:
      return arm_wrapped_address(static_cast<std::uint64_t>(right) + address_bytes - left -
                                 (1U - carry));
    case 12: return left | right;
    case 13: return right;
    case 14: return left & ~right;
    case 15: return ~right;
    default: throw std::runtime_error("Unpredictable ARM test/compare destination PC");
  }
}

std::uint32_t arithmetic_target(std::uint32_t opcode, const ArmState &state) {
  constexpr unsigned int operation_shift = 21;
  const auto operation = (opcode >> operation_shift) & register_mask;
  constexpr unsigned int source_shift = 16;
  const auto source = (opcode >> source_shift) & register_mask;
  const auto left = operation == 13 || operation == 15 ? 0 : reg(state, source);
  return calculate(
      {.operation = operation, .left = left, .right = shifter(opcode, state), .cpsr = state.cpsr});
}

std::uint32_t block_target(std::uint32_t opcode, ArmState &state, ArmSystem &system) {
  constexpr std::uint32_t list_mask = 0xffff;
  constexpr unsigned int base_shift = 16;
  const auto base_register = (opcode >> base_shift) & register_mask;
  const auto list = opcode & list_mask;
  const bool writeback = ((opcode >> 21) & 1U) != 0;
  if (base_register == pc_index || (writeback && (list & (1U << base_register)) != 0)) {
    throw std::runtime_error("Unpredictable ARM exception-return LDM base/writeback");
  }
  const auto base = reg(state, base_register);
  if (base % 4 != 0) { throw std::runtime_error("Unsupported unaligned ARM exception-return LDM"); }
  const bool increment = ((opcode >> 23) & 1U) != 0;
  const bool before = ((opcode >> 24) & 1U) != 0;
  const auto count = static_cast<std::uint32_t>(std::popcount(list));
  const auto span = count * 4;
  const auto start =
      increment ? static_cast<std::uint64_t>(base) + (before ? 4 : 0)
                : static_cast<std::uint64_t>(base) + address_bytes - span + (before ? 0 : 4);
  const auto low = arm_wrapped_address(start);
  if (span > address_bytes - low) {
    throw std::runtime_error("Unsupported wrapping ARM exception-return LDM");
  }
  std::uint32_t offset{};
  for (unsigned int index = 0; index <= pc_index; ++index) {
    if ((list & (1U << index)) == 0) { continue; }
    state.registers[index] =
        static_cast<std::uint32_t>(system.read({.address = low + offset, .size = 4}));
    offset += 4;
  }
  if (writeback) {
    state.registers[base_register] =
        arm_wrapped_address(increment ? static_cast<std::uint64_t>(base) + span
                                      : static_cast<std::uint64_t>(base) + address_bytes - span);
  }
  const auto target = state.registers[pc_index];
  if (!target) { throw std::logic_error("Missing ARM LDM return target"); }
  return *target;
}

}

std::uint32_t arm_wrapped_address(std::uint64_t value) {
  return static_cast<std::uint32_t>(value & std::numeric_limits<std::uint32_t>::max());
}

std::optional<ArmException> pending_arm_interrupt(const ArmState &state, ArmInterruptLines lines) {
  if (lines.fiq && (state.cpsr & fiq_disable) == 0) { return ArmException::fiq; }
  if (lines.irq && (state.cpsr & irq_disable) == 0) { return ArmException::irq; }
  return std::nullopt;
}

void enter_arm_exception(ArmState &state, ArmException kind, std::uint32_t vector_base) {
  constexpr std::uint32_t high_vectors = 0xffff0000;
  if (vector_base != 0 && vector_base != high_vectors) {
    throw std::runtime_error("Invalid ARM exception vector base");
  }
  const auto pc = state.registers[pc_index];
  if (!pc || *pc % 4 != 0 || (state.cpsr & execution_state) != 0) {
    throw std::runtime_error("Unsupported ARM exception source PC/execution state");
  }
  const auto descriptor = entry(kind);
  const auto old = state.cpsr;
  auto changed = (old & ~(mode_mask | execution_state)) | descriptor.mode | irq_disable;
  if (kind == ArmException::fiq) { changed |= fiq_disable; }
  set_arm_status(state, changed);
  const auto bank = arm_bank_index(changed);
  state.banks.modes[bank].saved = {.bits = old, .known_mask = status_mask};
  state.registers[lr_index] =
      arm_wrapped_address(static_cast<std::uint64_t>(*pc) + descriptor.link_offset);
  state.registers[pc_index] = vector_base + descriptor.offset;
  capture_arm_banks(state);
}

bool execute_arm_exception_return(std::uint32_t opcode, ArmState &state, ArmSystem &system) {
  constexpr std::uint32_t arithmetic_mask = 0x0c10f000;
  constexpr std::uint32_t arithmetic_value = 0x0010f000;
  constexpr std::uint32_t block_mask = 0x0e508000;
  constexpr std::uint32_t block_value = 0x08508000;
  constexpr std::uint32_t extra_register_mask = 0x90;
  const bool block = (opcode & block_mask) == block_value;
  const bool arithmetic =
      (opcode & arithmetic_mask) == arithmetic_value &&
      (((opcode >> 25) & 1U) != 0 || (opcode & extra_register_mask) != extra_register_mask);
  if (!block && !arithmetic) { return false; }
  if (!arm_condition_passes(opcode, state)) {
    state.registers[pc_index] =
        arm_wrapped_address(static_cast<std::uint64_t>(reg(state, pc_index)) + address_bytes - 4);
    return true;
  }
  const auto saved = return_status(state);
  const auto target =
      block ? block_target(opcode, state, system) : arithmetic_target(opcode, state);
  set_arm_status(state, saved);
  constexpr std::uint32_t alignment_mask = 3;
  state.registers[pc_index] = target & ~alignment_mask;
  return true;
}

}
