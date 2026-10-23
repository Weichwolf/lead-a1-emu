// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/arm_data.hpp"
#include "lead_a1/arm_mmu.hpp"
#include "lead_a1/memory.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace lead_a1 {

struct ArmBufferedWrite {
  MemoryAccess access;
  std::uint64_t value{};
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
  bool operator==(const ArmSystemState &) const = default;
};

}
