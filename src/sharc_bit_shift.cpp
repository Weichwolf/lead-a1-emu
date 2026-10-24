// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t all_bits = 0xffffffff;
constexpr std::uint32_t flags_mask = 0x3800;
constexpr std::uint32_t zero = 0x1000;
constexpr std::uint32_t overflow = 0x800;
constexpr unsigned int bits = 32;

SharcKnown32 modified(SharcKnown32 input, std::uint32_t mask, SharcBitOperation operation) {
  switch (operation) {
    case SharcBitOperation::set: return {.value = input.value | mask, .known = input.known | mask};
    case SharcBitOperation::clear:
      return {.value = input.value & ~mask, .known = input.known | mask};
    case SharcBitOperation::toggle:
      return {.value = input.value ^ (mask & input.known), .known = input.known};
    case SharcBitOperation::test:
    default: throw std::logic_error("Invalid SHARC bit-modification operation");
  }
}

SharcKnown32 result_flags(SharcKnown32 result, bool offscale) {
  SharcKnown32 flags{.value = offscale ? overflow : 0, .known = flags_mask & ~zero};
  if (result.value != 0 || result.known == all_bits) {
    flags.known |= zero;
    if (result.value == 0) { flags.value |= zero; }
  }
  return flags;
}

SharcKnown32 tested(SharcKnown32 input, std::uint32_t mask) {
  // A zero mask is an off-scale position: no bit, SZ=SV=1 (A-19/20).
  SharcKnown32 flags{.value = mask == 0 ? overflow : 0, .known = flags_mask & ~zero};
  if (mask == 0 || (input.known & mask) != 0) {
    flags.known |= zero;
    if ((input.value & mask) == 0) { flags.value |= zero; }
  }
  return flags;
}

}

void sharc_bit_shift(SharcState &state,
                     SharcKnown32 input,
                     SharcBitInstruction instruction,
                     unsigned int element) {
  const auto mask = instruction.position < bits ? std::uint32_t{1} << instruction.position : 0;
  if (instruction.operation == SharcBitOperation::test) {
    sharc_shift_status(state, tested(input, mask), element);
    return;
  }
  const auto result = modified(input, mask, instruction.operation);
  constexpr std::uint64_t extension = 0xff;
  sharc_compute_register(state, instruction.destination, element) = {
      .value = std::uint64_t{result.value} << 8,
      .known = (std::uint64_t{result.known} << 8) | extension};
  sharc_shift_status(state, result_flags(result, mask == 0), element);
}

}
