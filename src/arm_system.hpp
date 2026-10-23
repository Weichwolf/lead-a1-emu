// SPDX-License-Identifier: GPL-3.0-only
#pragma once

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

  void begin_instruction();
  void commit_instruction();
  void abort_instruction();
  [[nodiscard]] std::uint32_t fetch(std::uint32_t pc);
  [[nodiscard]] bool execute(std::uint32_t opcode, ArmState &cpu);

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
  [[nodiscard]] Fill fill(std::uint32_t address);
  void write_control(std::uint32_t value);
  void maintenance(const Transfer &transfer, std::uint32_t value);
  void drain_buffer();
  void read_register(const Transfer &transfer, ArmState &cpu) const;
  void write_register(const Transfer &transfer, const ArmState &cpu);
  [[nodiscard]] static std::string diagnostic(const Transfer &transfer);
  Memory &memory_;
  ArmSystemState state_;
  ArmSystemState pending_;
  std::vector<Line> lines_;
  std::vector<ArmBufferedWrite> write_buffer_;
  std::optional<Fill> fill_;
  bool invalidate_instruction_{};
  bool clear_buffer_{};
  bool active_{};
};

}
