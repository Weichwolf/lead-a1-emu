// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "arm/core/arm_system.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace lead_a1 {

class Memory;

struct ArmSavedStatus {
  std::uint32_t bits{};
  std::uint32_t known_mask{};
  bool operator==(const ArmSavedStatus &) const = default;
};

struct ArmExceptionBank {
  std::optional<std::uint32_t> sp;
  std::optional<std::uint32_t> lr;
  ArmSavedStatus saved;
  bool operator==(const ArmExceptionBank &) const = default;
};

struct ArmBanks {
  // High registers: common r8-r12, then FIQ r8-r12.
  std::array<std::array<std::optional<std::uint32_t>, 5>, 2> high;
  // SP/LR: User+System, FIQ, IRQ, Supervisor, Abort, Undefined. No SPSR in bank 0.
  std::array<ArmExceptionBank, 6> modes;
  bool operator==(const ArmBanks &) const = default;
};

struct ArmState {
  std::array<std::optional<std::uint32_t>, 16> registers;
  std::uint32_t cpsr{};
  ArmBanks banks;
  bool operator==(const ArmState &) const = default;
};

enum class ArmException : std::uint8_t {
  undefined_instruction,
  software_interrupt,
  prefetch_abort,
  data_abort,
  irq,
  fiq
};

struct ArmInterruptLines {
  bool irq{};
  bool fiq{};
  bool operator==(const ArmInterruptLines &) const = default;
};

struct ArmStep {
  std::uint32_t before{};
  std::uint32_t after{};
  std::string assembly;
  std::optional<ArmException> exception;
};

struct ArmExecutionOptions {
  bool cache_translations{true};
  bool deliver_mmu_aborts{true};
};

struct ArmDecodeStatistics {
  std::size_t entries{};
  std::uint64_t translations{};
  std::uint64_t hits{};
};

// ARM-state instruction reference, not a reset/SoC/clock or complete ARM926 model.
class Arm {
public:
  Arm(Memory &memory,
      const std::filesystem::path &specification,
      const ArmState &initial,
      const std::optional<ArmSystemInitial> &system = std::nullopt,
      ArmExecutionOptions execution = {});
  ~Arm();
  Arm(const Arm &) = delete;
  Arm &operator=(const Arm &) = delete;
  [[nodiscard]] ArmState state() const;
  [[nodiscard]] ArmSystemState system_state() const;
  [[nodiscard]] ArmDecodeStatistics decode_statistics() const;
  void set_interrupts(ArmInterruptLines lines);
  [[nodiscard]] ArmInterruptLines interrupts() const;
  [[nodiscard]] ArmStep step();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}
