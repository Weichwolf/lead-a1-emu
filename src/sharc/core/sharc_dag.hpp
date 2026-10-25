// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "sharc/memory/sharc_memory.hpp"

#include <cstdint>

namespace lead_a1 {

// Resolved Decode address; architectural register writes still retire in order.
struct SharcDagAddress {
  std::uint32_t before{};
  std::uint32_t modifier{};
  std::uint32_t address{};
  std::uint32_t after{};
  unsigned int index{};
  unsigned int bank{};
  SharcWidth width{SharcWidth::normal32};
  bool post{};
  bool operator==(const SharcDagAddress &) const = default;
};

}
