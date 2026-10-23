// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "arm_data.hpp"
#include "arm_mmu.hpp"
#include "lead_a1/arm.hpp"
#include "lead_a1/arm_system.hpp"
#include "lead_a1/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lead_a1::detail {

class ArmSystem {
public:
  ArmSystem(Memory &memory, const std::optional<ArmSystemInitial> &initial);

  [[nodiscard]] ArmSystemState state() const { return state_; }

  [[nodiscard]] std::uint32_t exception_vector_base() const;
  void latch_fault(const MmuFault &fault);

  void begin_instruction(std::uint32_t cpsr);
  void commit_instruction();
  void abort_instruction();
  [[nodiscard]] std::uint32_t fetch(std::uint32_t pc);
  [[nodiscard]] bool execute(std::uint32_t opcode, ArmState &cpu);
  [[nodiscard]] std::uint64_t read(MemoryAccess access);
  void write(MemoryAccess access, std::uint64_t value);

private:
  static constexpr std::size_t line_bytes = 32;
  static constexpr std::size_t ways = 4;

  struct Line {
    std::uint32_t address{};
    std::array<std::uint8_t, line_bytes> bytes{};
    bool valid{};
  };

  struct Fill {
    std::size_t slot{};
    Line line;
  };

  struct Transfer {
    bool read{};
    unsigned int operation{};
    unsigned int reg{};
    unsigned int crn{};
    unsigned int crm{};
    unsigned int selector{};
  };

  [[nodiscard]] std::uint32_t control() const;
  static void validate_control(std::uint32_t value);
  static void validate_lockdown(std::uint32_t value);
  [[nodiscard]] std::uint32_t allocation_mask() const;
  [[nodiscard]] std::size_t allocation_slot(std::size_t set) const;
  [[nodiscard]] std::uint32_t read_lockdown(const Transfer &transfer) const;
  void write_lockdown(const Transfer &transfer, std::uint32_t value);
  [[nodiscard]] std::size_t set_index(std::uint32_t address) const;
  [[nodiscard]] std::uint32_t fetch_instruction(std::uint32_t pc);
  [[nodiscard]] Fill fill(const MmuTranslation &translation);
  [[nodiscard]] const Line &instruction_line(std::size_t slot) const;
  void stage_instruction_line(std::size_t slot, const Line &value);
  void invalidate_instruction_entry(const Transfer &transfer, std::uint32_t value);
  [[nodiscard]] MmuRequest request(MemoryAccess access, AccessKind kind) const;
  void write_control(std::uint32_t value, const ArmState &cpu);
  void verify_transition(std::uint32_t value, const ArmState &cpu);
  void maintenance(const Transfer &transfer, std::uint32_t value);
  void drain_buffer();
  [[nodiscard]] DataPolicy data_policy() const;
  void summarize_data(ArmSystemState &summary) const;
  void read_register(const Transfer &transfer, ArmState &cpu) const;
  void write_register(const Transfer &transfer, const ArmState &cpu);
  [[nodiscard]] static std::string diagnostic(const Transfer &transfer);
  ArmDataPath data_;
  ArmMmu mmu_;
  ArmSystemState state_;
  ArmSystemState pending_;
  std::vector<Line> lines_;
  std::array<Fill, 2> instruction_changes_{};
  std::size_t instruction_changed_{};
  bool invalidate_instruction_{};
  bool active_{};
  bool user_{};
  bool data_user_{};
};

}
