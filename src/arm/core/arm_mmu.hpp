// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

namespace lead_a1 {

struct ArmMmuInitial {
  std::optional<std::uint32_t> translation_table;
  std::optional<std::uint32_t> domain_access;
  std::optional<std::uint32_t> fcse_pid;
  std::optional<std::uint32_t> context_id;
  std::optional<std::uint32_t> tlb_lockdown;
  bool tlb_invalid{};
  bool locked_tlb_invalid{};
};

struct ArmMmuState {
  std::optional<std::uint32_t> translation_table;
  std::optional<std::uint32_t> domain_access;
  std::optional<std::uint32_t> fcse_pid;
  std::optional<std::uint32_t> context_id;
  std::optional<std::uint32_t> tlb_lockdown;
  bool tlb_known{};
  bool locked_tlb_known{};
  std::size_t entries{};
  std::size_t locked_entries{};
  std::uint64_t hits{};
  std::uint64_t walks{};
  std::uint64_t invalidations{};
  bool operator==(const ArmMmuState &) const = default;
};

}
