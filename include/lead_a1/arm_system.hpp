// SPDX-License-Identifier: GPL-3.0-only
#pragma once

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
// This bounded flat-memory reference assumes disabled MMU and TCM regions.
struct ArmSystemInitial {
  std::optional<std::uint32_t> control;
  std::optional<std::size_t> instruction_cache_bytes;
  bool instruction_cache_invalid{};
  bool data_cache_invalid{};
  std::optional<std::uint32_t> instruction_lockdown;
  std::optional<std::uint32_t> data_lockdown;
  std::optional<std::vector<ArmBufferedWrite>> write_buffer;
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
  bool operator==(const ArmSystemState &) const = default;
};

}
