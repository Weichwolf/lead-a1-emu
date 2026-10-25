// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>

namespace lead_a1 {

struct ArmDataState {
  bool cache_known{};
  bool stores_known{};
  bool writeback_known{};
  std::size_t cached_lines{};
  std::size_t dirty_halves{};
  std::size_t buffered_stores{};
  std::size_t buffered_words{};
  std::size_t writeback_words{};
  std::uint64_t read_hits{};
  std::uint64_t read_misses{};
  std::uint64_t write_hits{};
  std::uint64_t write_misses{};
  std::uint64_t evictions{};
  std::uint64_t writebacks{};
  std::uint64_t invalidations{};
  std::uint64_t explicit_drains{};
  std::uint64_t bus_drains{};
  bool operator==(const ArmDataState &) const = default;
};

}
