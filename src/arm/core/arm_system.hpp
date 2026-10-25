// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "arm/core/arm_data.hpp"
#include "arm/core/arm_mmu.hpp"
#include "common/memory.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace lead_a1 {

struct ArmBufferedWrite {
  MemoryAccess access;
  std::uint64_t value{};
};

struct ArmFaultStatus {
  std::uint8_t status{};
  std::optional<std::uint8_t> domain;
  bool operator==(const ArmFaultStatus &) const = default;
};

struct ArmFaultRegisters {
  std::optional<ArmFaultStatus> data;
  std::optional<ArmFaultStatus> instruction;
  std::optional<std::uint32_t> address;
  bool operator==(const ArmFaultRegisters &) const = default;
};

// Explicit software-entry inputs, not a detected board or physical-reset claim.
// TCM regions and cycle-level pipeline behavior remain outside this reference.
struct ArmSystemInitial {
  std::optional<std::uint32_t> control;
  std::optional<std::size_t> instruction_cache_bytes;
  std::optional<std::size_t> data_cache_bytes;
  bool instruction_cache_invalid{};
  bool data_cache_invalid{};
  std::optional<std::uint32_t> instruction_lockdown;
  std::optional<std::uint32_t> data_lockdown;
  std::optional<std::vector<ArmBufferedWrite>> write_buffer;
  bool writeback_buffer_empty{};
  std::optional<ArmMmuInitial> mmu;
  ArmFaultRegisters faults{};
};

struct ArmSystemState {
  bool configured{};
  std::optional<std::uint32_t> control;
  bool instruction_cache_known{};
  bool data_cache_invalid{};
  std::optional<std::uint32_t> instruction_lockdown;
  std::optional<std::uint32_t> data_lockdown;
  bool write_buffer_known{};
  std::size_t cached_lines{};
  std::size_t buffered_writes{};
  std::uint64_t instruction_hits{};
  std::uint64_t instruction_misses{};
  std::uint64_t instruction_evictions{};
  std::uint64_t instruction_invalidations{};
  std::uint64_t data_invalidations{};
  std::uint64_t write_buffer_drains{};
  ArmMmuState mmu;
  ArmDataState data;
  ArmFaultRegisters faults{};
  bool operator==(const ArmSystemState &) const = default;
};

}
