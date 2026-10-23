// SPDX-License-Identifier: GPL-3.0-only
#include "arm_system.hpp"

#include "arm_status.hpp"
#include "lead_a1/arm.hpp"
#include "lead_a1/arm_system.hpp"
#include "lead_a1/memory.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <ios>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t instruction_enable = 0x1000;
constexpr std::uint32_t round_robin = 0x4000;
constexpr std::uint32_t control_sbo = 0x00050078;
constexpr std::uint32_t control_supported = control_sbo | 0x7306;
constexpr std::uint32_t mmu_enable = 1;
constexpr std::uint32_t endian_and_legacy = 0x8080;
constexpr unsigned int register_mask = 15;
constexpr unsigned int pc_index = 15;
constexpr unsigned int byte_bits = 8;
constexpr unsigned int word_bytes = 4;
constexpr std::uint32_t condition_flags = 0xf0000000;
constexpr std::uint32_t lockdown_sbo = 0xfff0;
constexpr std::uint32_t lockdown_reserved = 0xffff0000;

}

ArmSystem::ArmSystem(Memory &memory, const std::optional<ArmSystemInitial> &initial)
    : memory_(memory) {
  if (!initial) { return; }
  state_.configured = true;
  state_.control = initial->control;
  if (initial->control) { validate_control(*initial->control); }
  if (const auto capacity = initial->instruction_cache_bytes) {
    constexpr std::size_t maximum_cache_bytes = std::size_t{64} * 1024;
    if (*capacity < line_bytes * ways || *capacity > maximum_cache_bytes ||
        !std::has_single_bit(*capacity)) {
      throw std::invalid_argument("Invalid reference ARM instruction cache geometry");
    }
    lines_.resize(*capacity / line_bytes);
  }
  state_.instruction_cache_known = initial->instruction_cache_invalid;
  state_.data_cache_invalid = initial->data_cache_invalid;
  state_.instruction_lockdown = initial->instruction_lockdown;
  state_.data_lockdown = initial->data_lockdown;
  if (initial->instruction_lockdown) { validate_lockdown(*initial->instruction_lockdown); }
  if (initial->data_lockdown) { validate_lockdown(*initial->data_lockdown); }
  if (initial->write_buffer) {
    constexpr std::size_t maximum_buffer_entries = 16;
    if (initial->write_buffer->size() > maximum_buffer_entries) {
      throw std::invalid_argument("Reference ARM write-buffer input exceeds supported capacity");
    }
    write_buffer_ = *initial->write_buffer;
    state_.write_buffer_known = true;
    state_.buffered_writes = write_buffer_.size();
  }
}

void ArmSystem::validate_control(std::uint32_t value) {
  if ((value & control_sbo) != control_sbo) {
    throw std::runtime_error("ARM926 control register requires documented SBO bits");
  }
  if ((value & mmu_enable) != 0) { throw std::runtime_error("Unsupported ARM MMU enable"); }
  if ((value & endian_and_legacy) != 0) {
    throw std::runtime_error("Unsupported ARM endian/legacy-PC control");
  }
  if ((value & ~control_supported) != 0) {
    throw std::runtime_error("Unsupported or reserved ARM control bits");
  }
}

std::uint32_t ArmSystem::control() const {
  if (!pending_.configured) { throw std::runtime_error("ARM CP15 profile is not configured"); }
  const auto value = pending_.control;
  if (!value) { throw std::runtime_error("Unknown initial ARM control register"); }
  return *value;
}

void ArmSystem::begin_instruction() {
  if (active_) { throw std::logic_error("Nested ARM system instruction"); }
  active_ = true;
  pending_ = state_;
  fill_.reset();
  invalidate_instruction_ = false;
  clear_buffer_ = false;
}

void ArmSystem::abort_instruction() {
  fill_.reset();
  active_ = false;
}

void ArmSystem::commit_instruction() {
  if (!active_) { throw std::logic_error("No ARM system instruction to commit"); }
  if (invalidate_instruction_) {
    for (auto &line : lines_) { line.valid = false; }
  } else if (fill_) {
    lines_[fill_->slot] = fill_->line;
  }
  if (clear_buffer_) { write_buffer_.clear(); }
  state_ = pending_;
  fill_.reset();
  active_ = false;
}

std::size_t ArmSystem::set_index(std::uint32_t address) const {
  if (lines_.empty()) { throw std::runtime_error("Unknown ARM instruction cache geometry"); }
  return (address / line_bytes) % (lines_.size() / ways);
}

void ArmSystem::validate_lockdown(std::uint32_t value) {
  if ((value & lockdown_sbo) != lockdown_sbo || (value & lockdown_reserved) != 0) {
    throw std::runtime_error("Invalid ARM cache lockdown SBO/reserved bits");
  }
}

std::uint32_t ArmSystem::allocation_mask() const {
  const auto value = pending_.instruction_lockdown;
  if (!value) {
    throw std::runtime_error("Unknown initial ARM instruction-cache lockdown register");
  }
  constexpr std::uint32_t way_mask = 15;
  const auto available = ~*value & way_mask;
  // ARM926 c9 explicitly uses way 3 when all four L bits are set.
  constexpr std::uint32_t fallback_way = 8;
  return available == 0 ? fallback_way : available;
}

std::size_t ArmSystem::allocation_slot(std::size_t set) const {
  const auto available = allocation_mask();
  std::size_t last_slot{};
  for (std::size_t way = 0; way < ways; ++way) {
    if ((available & (std::uint32_t{1} << way)) == 0) { continue; }
    last_slot = (set * ways) + way;
    if (!lines_[last_slot].valid) { return last_slot; }
  }
  if (std::has_single_bit(available)) { return last_slot; }
  if ((control() & round_robin) == 0) {
    throw std::runtime_error("Unknown ARM random cache replacement state");
  }
  throw std::runtime_error("Unqualified ARM round-robin cache replacement state");
}

ArmSystem::Fill ArmSystem::fill(std::uint32_t address) {
  const auto slot = allocation_slot(set_index(address));
  Fill result{.slot = slot, .line = {.address = address, .bytes = {}, .valid = true}};
  for (std::size_t offset = 0; offset < line_bytes; offset += word_bytes) {
    const auto word =
        memory_.read({.address = address + static_cast<std::uint32_t>(offset), .size = word_bytes},
                     AccessKind::fetch);
    for (unsigned int byte = 0; byte < word_bytes; ++byte) {
      constexpr auto byte_mask = std::numeric_limits<std::uint8_t>::max();
      result.line.bytes[offset + byte] =
          static_cast<std::uint8_t>((word >> (byte * byte_bits)) & byte_mask);
    }
  }
  return result;
}

std::uint32_t ArmSystem::fetch(std::uint32_t pc) {
  try {
    return fetch_instruction(pc);
  } catch (const std::exception &error) {
    std::ostringstream text;
    text << "ARM instruction fetch address=0x" << std::hex << pc << " width=4: " << error.what();
    throw std::runtime_error(text.str());
  }
}

std::uint32_t ArmSystem::fetch_instruction(std::uint32_t pc) {
  if (!active_) { throw std::logic_error("ARM fetch outside an instruction"); }
  if (pc % word_bytes != 0) { throw std::runtime_error("Unaligned ARM system fetch"); }
  if (!pending_.configured || (control() & instruction_enable) == 0) {
    return static_cast<std::uint32_t>(
        memory_.read({.address = pc, .size = word_bytes}, AccessKind::fetch));
  }
  if (!pending_.instruction_cache_known) {
    throw std::runtime_error("Unknown initial ARM instruction cache validity");
  }
  constexpr auto offset_mask = static_cast<std::uint32_t>(line_bytes - 1);
  const auto address = pc & ~offset_mask;
  const auto set = set_index(address);
  const Line *selected = nullptr;
  for (auto slot = set * ways; slot < (set + 1) * ways; ++slot) {
    if (lines_[slot].valid && lines_[slot].address == address) {
      selected = &lines_[slot];
      break;
    }
  }
  if (selected != nullptr) {
    ++pending_.instruction_hits;
  } else {
    fill_ = fill(address);
    selected = &fill_->line;
    ++pending_.instruction_misses;
    if (lines_[fill_->slot].valid) {
      ++pending_.instruction_evictions;
    } else {
      ++pending_.cached_lines;
    }
  }
  std::uint32_t word{};
  const auto offset = pc & offset_mask;
  for (unsigned int byte = 0; byte < word_bytes; ++byte) {
    word |= static_cast<std::uint32_t>(selected->bytes[offset + byte]) << (byte * byte_bits);
  }
  return word;
}

void ArmSystem::write_control(std::uint32_t value) {
  validate_control(value);
  if ((value & instruction_enable) != 0) {
    if (!pending_.instruction_cache_known) {
      throw std::runtime_error("Cannot enable an unknown ARM instruction cache");
    }
    static_cast<void>(set_index(0));
  }
  pending_.control = value;
}

void ArmSystem::drain_buffer() {
  if (!pending_.write_buffer_known) {
    throw std::runtime_error("Unknown initial ARM write buffer");
  }
  for (const auto &write : write_buffer_) { memory_.write(write.access, write.value); }
  clear_buffer_ = true;
  pending_.buffered_writes = 0;
  ++pending_.write_buffer_drains;
}

void ArmSystem::maintenance(const Transfer &transfer, std::uint32_t value) {
  if (value != 0) { throw std::runtime_error("ARM cache operation requires a zero operand"); }
  if (transfer.selector == 0 && (transfer.crm == 5 || transfer.crm == 6 || transfer.crm == 7)) {
    if (transfer.crm != 6) {
      invalidate_instruction_ = true;
      pending_.instruction_cache_known = true;
      pending_.cached_lines = 0;
      ++pending_.instruction_invalidations;
    }
    if (transfer.crm != 5) {
      pending_.data_cache_invalid = true;
      ++pending_.data_invalidations;
    }
    return;
  }
  constexpr unsigned int drain_crm = 10;
  if (transfer.crm == drain_crm && transfer.selector == 4) {
    drain_buffer();
    return;
  }
  throw std::runtime_error("Unsupported ARM cache operation");
}

std::string ArmSystem::diagnostic(const Transfer &transfer) {
  std::ostringstream text;
  text << "CP15 " << (transfer.read ? "read" : "write") << " opc1=" << transfer.operation
       << " crn=" << transfer.crn << " crm=" << transfer.crm << " opc2=" << transfer.selector
       << " rt=" << transfer.reg;
  return text.str();
}

std::uint32_t ArmSystem::read_lockdown(const Transfer &transfer) const {
  if (transfer.crm != 0 || transfer.selector > 1) {
    throw std::runtime_error("Unsupported CP15 cache-lockdown selector");
  }
  const auto value =
      transfer.selector == 1 ? pending_.instruction_lockdown : pending_.data_lockdown;
  if (!value) { throw std::runtime_error("Unknown initial ARM cache-lockdown register"); }
  return *value;
}

void ArmSystem::write_lockdown(const Transfer &transfer, std::uint32_t value) {
  if (transfer.crm != 0 || transfer.selector > 1) {
    throw std::runtime_error("Unsupported CP15 cache-lockdown selector");
  }
  validate_lockdown(value);
  if (transfer.selector == 1) {
    pending_.instruction_lockdown = value;
  } else {
    pending_.data_lockdown = value;
  }
}

void ArmSystem::read_register(const Transfer &transfer, ArmState &cpu) const {
  const auto value = [&] {
    if (transfer.crn == 1 && transfer.crm == 0 && transfer.selector == 0) { return control(); }
    if (transfer.crn == 9) { return read_lockdown(transfer); }
    throw std::runtime_error("Unsupported CP15 register read");
  }();
  if (transfer.reg == pc_index) {
    cpu.cpsr = (cpu.cpsr & ~condition_flags) | (value & condition_flags);
  } else {
    cpu.registers[transfer.reg] = value;
  }
}

void ArmSystem::write_register(const Transfer &transfer, const ArmState &cpu) {
  if (transfer.reg == pc_index) { throw std::runtime_error("Unsupported CP15 PC operand"); }
  const auto value = cpu.registers[transfer.reg];
  if (!value) { throw std::runtime_error("Unknown CP15 operand register"); }
  if (transfer.crn == 1 && transfer.crm == 0 && transfer.selector == 0) {
    write_control(*value);
  } else if (transfer.crn == 7) {
    maintenance(transfer, *value);
  } else if (transfer.crn == 9) {
    write_lockdown(transfer, *value);
  } else {
    throw std::runtime_error("Unsupported CP15 register write");
  }
}

bool ArmSystem::execute(std::uint32_t opcode, ArmState &cpu) {
  constexpr std::uint32_t transfer_mask = 0x0f000010;
  constexpr std::uint32_t transfer_value = 0x0e000010;
  constexpr unsigned int coprocessor_shift = 8;
  if ((opcode & transfer_mask) != transfer_value ||
      ((opcode >> coprocessor_shift) & register_mask) != 15) {
    return false;
  }
  constexpr unsigned int read_bit = 20;
  constexpr unsigned int operation_shift = 21;
  constexpr unsigned int crn_shift = 16;
  constexpr unsigned int reg_shift = 12;
  constexpr unsigned int selector_shift = 5;
  constexpr unsigned int operation_mask = 7;
  const Transfer transfer{.read = ((opcode >> read_bit) & 1U) != 0,
                          .operation = (opcode >> operation_shift) & operation_mask,
                          .reg = (opcode >> reg_shift) & register_mask,
                          .crn = (opcode >> crn_shift) & register_mask,
                          .crm = opcode & register_mask,
                          .selector = (opcode >> selector_shift) & operation_mask};
  try {
    const auto pc = cpu.registers[pc_index];
    if (!pc) { throw std::runtime_error("Unknown CP15 instruction PC"); }
    if (arm_condition_passes(opcode, cpu)) {
      static_cast<void>(control());
      constexpr std::uint32_t mode_mask = 0x1f;
      constexpr std::uint32_t user_mode = 0x10;
      if ((cpu.cpsr & mode_mask) == user_mode) {
        throw std::runtime_error("Unsupported unprivileged ARM CP15 access");
      }
      if (transfer.operation != 0) { throw std::runtime_error("Unsupported CP15 opcode 1"); }
      if (transfer.read) {
        read_register(transfer, cpu);
      } else {
        write_register(transfer, cpu);
      }
    }
    const auto next = static_cast<std::uint64_t>(*pc) + word_bytes;
    cpu.registers[pc_index] =
        static_cast<std::uint32_t>(next & std::numeric_limits<std::uint32_t>::max());
  } catch (const std::exception &error) {
    throw std::runtime_error(diagnostic(transfer) + ": " + error.what());
  }
  return true;
}

}
