// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_memory_view.hpp"

#include "lead_a1/sharc_memory.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1 {

SharcMemoryView sharc_memory_view(SharcAccess access) {
  switch (access.width) {
    case SharcWidth::short16:
    case SharcWidth::normal32:
    case SharcWidth::extended40:
    case SharcWidth::wide48:
    case SharcWidth::long64: break;
    default: throw std::invalid_argument("Invalid SHARC word width");
  }
  constexpr std::uint32_t short_base = 0x100000;
  constexpr std::uint32_t normal_base = 0x80000;
  constexpr std::uint32_t long_base = 0x40000;
  constexpr std::uint32_t memory_end = 0x200000;
  constexpr std::uint32_t bank_cells = 0x40000;
  const auto address = access.address;
  if (address >= short_base && address < memory_end && access.width == SharcWidth::short16) {
    return {.bank = (address - short_base) / bank_cells,
            .cell = (address - short_base) % bank_cells,
            .cells = 1};
  }
  if (address >= normal_base && address < short_base && access.width != SharcWidth::short16) {
    constexpr std::uint32_t span = 0x20000;
    const auto offset = (address - normal_base) % span;
    auto columns = access.width == SharcWidth::normal32 ? 2U : 3U;
    auto cell = offset * columns;
    if (access.width == SharcWidth::long64) {
      columns = 4;
      cell = (offset & ~1U) * 2;
    }
    return {.bank = (address - normal_base) / span,
            .cell = cell,
            .cells = columns,
            .shift = access.width == SharcWidth::extended40 ? 8U : 0U};
  }
  if (address >= long_base && address < normal_base && access.width == SharcWidth::long64) {
    constexpr std::uint32_t span = 0x10000;
    return {.bank = (address - long_base) / span,
            .cell = ((address - long_base) % span) * 4,
            .cells = 4};
  }
  throw std::runtime_error("Unsupported SHARC address/width view or external memory");
}

}
