// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "sharc/memory/sharc_memory.hpp"

#include <cstdint>

namespace lead_a1 {

// Address geometry only: mapping, permissions and data knowledge remain separate.
struct SharcMemoryView {
  unsigned int bank{};
  std::uint32_t cell{};
  unsigned int cells{};
  unsigned int shift{};
  bool operator==(const SharcMemoryView &) const = default;
};

[[nodiscard]] SharcMemoryView sharc_memory_view(SharcAccess access);

}
