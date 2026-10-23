// SPDX-License-Identifier: GPL-3.0-only
#include "arm_system.hpp"

#include "arm_data.hpp"
#include "arm_fault.hpp"
#include "arm_mmu.hpp"
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
constexpr std::uint32_t control_supported = control_sbo | 0x7307;
constexpr std::uint32_t mmu_enable = 1;
constexpr std::uint32_t data_cache_enable = 4;
constexpr std::uint32_t endian_and_legacy = 0x8080;
constexpr unsigned int register_mask = 15;
constexpr unsigned int pc_index = 15;
constexpr unsigned int byte_bits = 8;
constexpr unsigned int word_bytes = 4;
constexpr std::uint32_t condition_flags = 0xf0000000;
constexpr std::uint32_t lockdown_sbo = 0xfff0;
constexpr std::uint32_t lockdown_reserved = 0xffff0000;

std::string data_error(MemoryAccess access, AccessKind kind, const std::exception &error) {
  std::ostringstream text;
  text << "ARM data " << (kind == AccessKind::write ? "write" : "read") << " va=0x" << std::hex
       << access.address << std::dec << " width=" << access.size << ": " << error.what();
  return text.str();
}

}

ArmSystem::ArmSystem(Memory &memory, const std::optional<ArmSystemInitial> &initial)
    : data_(memory, initial), mmu_(memory, initial ? initial->mmu : std::nullopt, &data_) {
  state_.mmu = mmu_.state();
  summarize_data(state_);
  if (!initial) { return; }
  state_.configured = true;
  validate_fault_registers(initial->faults);
  state_.faults = initial->faults;
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
  state_.instruction_lockdown = initial->instruction_lockdown;
  state_.data_lockdown = initial->data_lockdown;
  if (initial->instruction_lockdown) { validate_lockdown(*initial->instruction_lockdown); }
  if (initial->data_lockdown) { validate_lockdown(*initial->data_lockdown); }
}

void ArmSystem::validate_control(std::uint32_t value) {
  if ((value & control_sbo) != control_sbo) {
    throw std::runtime_error("ARM926 control register requires documented SBO bits");
  }
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

std::uint32_t ArmSystem::exception_vector_base() const {
  constexpr std::uint32_t high_vectors = 0x2000;
  constexpr std::uint32_t high_base = 0xffff0000;
  return (control() & high_vectors) != 0 ? high_base : 0;
}

void ArmSystem::latch_fault(const MmuFault &fault) {
  if (!active_) { throw std::logic_error("ARM fault latch outside transaction"); }
  latch_mmu_fault(pending_.faults, fault);
}

void ArmSystem::begin_instruction(std::uint32_t cpsr) {
  if (active_) { throw std::logic_error("Nested ARM system instruction"); }
  active_ = true;
  pending_ = state_;
  constexpr std::uint32_t mode_mask = 0x1f;
  constexpr std::uint32_t user_mode = 0x10;
  user_ = (cpsr & mode_mask) == user_mode;
  data_user_ = user_;
  data_.begin_instruction();
  mmu_.begin_instruction();
  instruction_changed_ = 0;
  invalidate_instruction_ = false;
}

void ArmSystem::abort_instruction() {
  instruction_changed_ = 0;
  mmu_.abort_instruction();
  data_.abort_instruction();
  active_ = false;
}

void ArmSystem::commit_instruction() {
  if (!active_) { throw std::logic_error("No ARM system instruction to commit"); }
  if (invalidate_instruction_) {
    for (auto &line : lines_) { line.valid = false; }
  }
  for (std::size_t index = 0; index < instruction_changed_; ++index) {
    const auto &change = instruction_changes_[index];
    lines_[change.slot] = change.line;
  }
  mmu_.commit_instruction();
  data_.commit_instruction();
  summarize_data(pending_);
  pending_.mmu = mmu_.state();
  state_ = pending_;
  instruction_changed_ = 0;
  active_ = false;
}

std::size_t ArmSystem::set_index(std::uint32_t address) const {
  if (lines_.empty()) { throw std::runtime_error("Unknown ARM instruction cache geometry"); }
  return (address / line_bytes) % (lines_.size() / ways);
}

const ArmSystem::Line &ArmSystem::instruction_line(std::size_t slot) const {
  for (std::size_t index = 0; index < instruction_changed_; ++index) {
    if (instruction_changes_[index].slot == slot) { return instruction_changes_[index].line; }
  }
  static constexpr Line invalid;
  return invalidate_instruction_ ? invalid : lines_.at(slot);
}

void ArmSystem::stage_instruction_line(std::size_t slot, const Line &value) {
  pending_.cached_lines -= static_cast<std::size_t>(instruction_line(slot).valid);
  pending_.cached_lines += static_cast<std::size_t>(value.valid);
  for (std::size_t index = 0; index < instruction_changed_; ++index) {
    if (instruction_changes_[index].slot == slot) {
      instruction_changes_[index].line = value;
      return;
    }
  }
  if (instruction_changed_ == instruction_changes_.size()) {
    throw std::runtime_error("ARM instruction-cache transaction journal exhausted");
  }
  instruction_changes_[instruction_changed_++] = {.slot = slot, .line = value};
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
    if (!instruction_line(last_slot).valid) { return last_slot; }
  }
  if (std::has_single_bit(available)) { return last_slot; }
  if ((control() & round_robin) == 0) {
    throw std::runtime_error("Unknown ARM random cache replacement state");
  }
  throw std::runtime_error("Unqualified ARM round-robin cache replacement state");
}

ArmSystem::Fill ArmSystem::fill(const MmuTranslation &translation) {
  constexpr auto offset_mask = static_cast<std::uint32_t>(line_bytes - 1);
  const auto address = translation.modified & ~offset_mask;
  const auto physical = translation.physical.address & ~offset_mask;
  const auto slot = allocation_slot(set_index(address));
  Fill result{.slot = slot, .line = {.address = address, .bytes = {}, .valid = true}};
  const auto critical = translation.physical.address & offset_mask;
  for (std::size_t progress = 0; progress < line_bytes; progress += word_bytes) {
    const auto offset = (critical + progress) & offset_mask;
    const auto word = data_.physical_read(
        {.address = physical + static_cast<std::uint32_t>(offset), .size = word_bytes},
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
    const auto opcode = fetch_instruction(pc);
    constexpr std::uint32_t unprivileged_mask = 0x0d200000;
    constexpr std::uint32_t unprivileged_transfer = 0x04200000;
    data_user_ = user_ || (opcode & unprivileged_mask) == unprivileged_transfer;
    return opcode;
  } catch (const MmuFault &) { throw; } catch (const std::exception &error) {
    std::ostringstream text;
    text << "ARM instruction fetch address=0x" << std::hex << pc << " width=4: " << error.what();
    throw std::runtime_error(text.str());
  }
}

std::uint32_t ArmSystem::fetch_instruction(std::uint32_t pc) {
  if (!active_) { throw std::logic_error("ARM fetch outside an instruction"); }
  if (pc % word_bytes != 0) { throw std::runtime_error("Unaligned ARM system fetch"); }
  const auto translation =
      mmu_.translate(request({.address = pc, .size = word_bytes}, AccessKind::fetch));
  if (!pending_.configured || (control() & instruction_enable) == 0 || !translation.cacheable) {
    return static_cast<std::uint32_t>(data_.physical_read(translation.physical, AccessKind::fetch));
  }
  if (!pending_.instruction_cache_known) {
    throw std::runtime_error("Unknown initial ARM instruction cache validity");
  }
  constexpr auto offset_mask = static_cast<std::uint32_t>(line_bytes - 1);
  const auto address = translation.modified & ~offset_mask;
  const auto set = set_index(address);
  const Line *selected = nullptr;
  for (auto slot = set * ways; slot < (set + 1) * ways; ++slot) {
    const auto &line = instruction_line(slot);
    if (line.valid && line.address == address) {
      selected = &line;
      break;
    }
  }
  if (selected != nullptr) {
    ++pending_.instruction_hits;
  } else {
    const auto fetched = fill(translation);
    const auto evicted = instruction_line(fetched.slot).valid;
    stage_instruction_line(fetched.slot, fetched.line);
    selected = &instruction_line(fetched.slot);
    ++pending_.instruction_misses;
    if (evicted) { ++pending_.instruction_evictions; }
  }
  std::uint32_t word{};
  const auto offset = pc & offset_mask;
  for (unsigned int byte = 0; byte < word_bytes; ++byte) {
    word |= static_cast<std::uint32_t>(selected->bytes[offset + byte]) << (byte * byte_bits);
  }
  return word;
}

MmuRequest ArmSystem::request(MemoryAccess access, AccessKind kind) const {
  const auto current_control = pending_.configured ? control() : 0;
  return {.access = access,
          .kind = kind,
          .control = current_control,
          .user = kind == AccessKind::fetch ? user_ : data_user_,
          .retain = true};
}

std::uint64_t ArmSystem::read(MemoryAccess access) try {
  const auto translation = mmu_.translate(request(access, AccessKind::read));
  return data_.read(translation, data_policy());
} catch (const MmuFault &) { throw; } catch (const std::exception &error) {

  throw std::runtime_error(data_error(access, AccessKind::read, error));
}

void ArmSystem::write(MemoryAccess access, std::uint64_t value) try {
  const auto translation = mmu_.translate(request(access, AccessKind::write));
  data_.write(translation, data_policy(), value);
} catch (const MmuFault &) { throw; } catch (const std::exception &error) {

  throw std::runtime_error(data_error(access, AccessKind::write, error));
}

void ArmSystem::verify_transition(std::uint32_t value, const ArmState &cpu) {
  const auto old = control();
  if (((old ^ value) & mmu_enable) == 0) { return; }
  if (((old ^ value) & instruction_enable) != 0) {
    throw std::runtime_error("Unsupported combined ARM MMU/I-cache transition");
  }
  if ((value & mmu_enable) != 0) { mmu_.require_ready(); }
  const auto pc = cpu.registers[pc_index];
  if (!pc) { throw std::runtime_error("Unknown ARM MMU transition PC"); }
  for (const auto offset : {word_bytes, word_bytes * 2}) {
    const auto next =
        (static_cast<std::uint64_t>(*pc) + offset) & std::numeric_limits<std::uint32_t>::max();
    auto access = request({.address = static_cast<std::uint32_t>(next), .size = word_bytes},
                          AccessKind::fetch);
    access.retain = false;
    access.architectural = false;
    const auto before = mmu_.translate(access);
    access.control = value;
    const auto after = mmu_.translate(access);
    const bool cached_change =
        (old & instruction_enable) != 0 &&
        (before.modified != after.modified || before.cacheable != after.cacheable);
    if (before.physical.address != after.physical.address || cached_change) {
      throw std::runtime_error("Unsupported nonidentity ARM MMU prefetch transition");
    }
  }
}

void ArmSystem::write_control(std::uint32_t value, const ArmState &cpu) {
  validate_control(value);
  verify_transition(value, cpu);
  if ((value & (mmu_enable | data_cache_enable)) == (mmu_enable | data_cache_enable)) {
    data_.require_cache();
  }
  if ((value & instruction_enable) != 0) {
    if (!pending_.instruction_cache_known) {
      throw std::runtime_error("Cannot enable an unknown ARM instruction cache");
    }
    static_cast<void>(set_index(0));
  }
  pending_.control = value;
}

void ArmSystem::drain_buffer() {
  data_.drain();
}

DataPolicy ArmSystem::data_policy() const {
  const auto current = pending_.configured ? control() : 0;
  return {.lockdown = pending_.data_lockdown,
          .enabled =
              (current & (mmu_enable | data_cache_enable)) == (mmu_enable | data_cache_enable),
          .round_robin = (current & round_robin) != 0};
}

void ArmSystem::summarize_data(ArmSystemState &summary) const {
  summary.data = data_.state();
  summary.data_cache_invalid = summary.data.cache_known && summary.data.cached_lines == 0;
  summary.data_invalidations = summary.data.invalidations;
  summary.write_buffer_known = summary.data.stores_known && summary.data.writeback_known;
  summary.buffered_writes = summary.data.buffered_stores;
  summary.write_buffer_drains = summary.data.explicit_drains;
}

void ArmSystem::maintenance(const Transfer &transfer, std::uint32_t value) {
  if (transfer.selector == 0 && (transfer.crm == 5 || transfer.crm == 6 || transfer.crm == 7)) {
    if (value != 0) { throw std::runtime_error("ARM cache operation requires a zero operand"); }
    if (transfer.crm != 6) {
      invalidate_instruction_ = true;
      instruction_changed_ = 0;
      pending_.instruction_cache_known = true;
      pending_.cached_lines = 0;
      ++pending_.instruction_invalidations;
    }
    if (transfer.crm != 5) { data_.invalidate_all(); }
    return;
  }
  if (transfer.crm == 5 && (transfer.selector == 1 || transfer.selector == 2)) {
    invalidate_instruction_entry(transfer, value);
    return;
  }
  constexpr unsigned int drain_crm = 10;
  if (transfer.crm == drain_crm && transfer.selector == 4) {
    if (value != 0) { throw std::runtime_error("ARM cache operation requires a zero operand"); }
    drain_buffer();
    return;
  }
  if (transfer.crm == 6 || transfer.crm == 10 || transfer.crm == 14) {
    data_.maintain({.operation = transfer.crm, .selector = transfer.selector, .operand = value});
    return;
  }
  throw std::runtime_error("Unsupported ARM cache operation");
}

void ArmSystem::invalidate_instruction_entry(const Transfer &transfer, std::uint32_t value) {
  if (!pending_.instruction_cache_known) {
    throw std::runtime_error("Unknown ARM instruction cache validity for invalidation");
  }
  constexpr std::uint32_t offset_mask = line_bytes - 1;
  if ((value & offset_mask) != 0) {
    throw std::runtime_error("Invalid ARM instruction-cache maintenance word/SBZ bits");
  }
  if (transfer.selector == 1) {
    const auto set = set_index(value);
    for (auto slot = set * ways; slot < (set + 1) * ways; ++slot) {
      const auto &line = instruction_line(slot);
      if (line.valid && line.address == value) {
        stage_instruction_line(slot, {});
        break;
      }
    }
  } else {
    static_cast<void>(set_index(0));
    const auto sets = lines_.size() / ways;
    const auto index_mask = static_cast<std::uint32_t>((sets - 1) * line_bytes);
    constexpr std::uint32_t way_mask = 0xc0000000;
    if ((value & ~(way_mask | index_mask)) != 0) {
      throw std::runtime_error("Invalid ARM instruction-cache maintenance Set/Way bits");
    }
    constexpr unsigned int way_shift = 30;
    const auto slot =
        (static_cast<std::size_t>((value & index_mask) / line_bytes) * ways) + (value >> way_shift);
    stage_instruction_line(slot, {});
  }
  ++pending_.instruction_invalidations;
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
    if ((transfer.crn == 5 || transfer.crn == 6) && transfer.crm == 0) {
      if (transfer.crn == 5 && transfer.selector < 2 && transfer.reg == pc_index) {
        return std::uint32_t{0};
      }
      return read_fault_register(pending_.faults,
                                 {.crn = transfer.crn, .selector = transfer.selector});
    }
    if (transfer.crn == 2 || transfer.crn == 3 || transfer.crn == 10 || transfer.crn == 13) {
      return mmu_.read({.crn = transfer.crn, .crm = transfer.crm, .selector = transfer.selector});
    }
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
    write_control(*value, cpu);
  } else if (transfer.crn == 7) {
    maintenance(transfer, *value);
  } else if (transfer.crn == 9) {
    write_lockdown(transfer, *value);
  } else if ((transfer.crn == 5 || transfer.crn == 6) && transfer.crm == 0) {
    write_fault_register(
        pending_.faults, {.crn = transfer.crn, .selector = transfer.selector}, *value);
  } else if (transfer.crn == 2 || transfer.crn == 3 || transfer.crn == 8 || transfer.crn == 10 ||
             transfer.crn == 13) {
    if (transfer.crn == 13 && transfer.selector == 0 && (control() & mmu_enable) != 0 &&
        mmu_.read({.crn = 13, .crm = 0, .selector = 0}) != *value) {
      throw std::runtime_error("Unsupported ARM FCSE change with prefetched instructions");
    }
    mmu_.write({.crn = transfer.crn, .crm = transfer.crm, .selector = transfer.selector}, *value);
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
