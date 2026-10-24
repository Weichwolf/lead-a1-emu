// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {

void sharc_short_transfer(
    SharcMemory &memory, SharcKnown40 &data, SharcKnown32 mode, SharcAccess access, bool write) {
  constexpr std::uint64_t halfword = 0xffff;
  constexpr std::uint64_t stored_bits = halfword << 8;
  access.width = SharcWidth::short16;
  if (write) {
    if ((data.known & stored_bits) != stored_bits) {
      throw std::runtime_error("Unknown SHARC short-word store bits");
    }
    memory.write(access, (data.value >> 8) & halfword);
    return;
  }
  // PGR3-37/7-28 and Figure7-10: data23:8, extension39:24, zero7:0.
  const auto value = memory.read(access);
  SharcKnown40 result{.value = value << 8, .known = sharc_data_mask};
  constexpr std::uint64_t sign = 0x8000;
  constexpr std::uint64_t upper = halfword << 24;
  constexpr std::uint32_t sign_extend = 0x4000;
  if ((value & sign) != 0) {
    if ((mode.known & sign_extend) == 0) {
      result.known &= ~upper;
    } else if ((mode.value & sign_extend) != 0) {
      result.value |= upper;
    }
  }
  data = result;
}

}
