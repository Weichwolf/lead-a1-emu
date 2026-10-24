// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>

namespace lead_a1 {

enum class SharcFloatFormat : std::uint8_t { binary32, extended40 };
enum class SharcFloatRounding : std::uint8_t { nearest_even, toward_zero };

struct SharcFloatSum {
  // Packed SHARC40 register bits; binary32 results have eight zero low bits.
  std::uint64_t value{};
  bool underflow{};
  bool overflow{};
  bool operator==(const SharcFloatSum &) const = default;
};

// Finite inputs only. Denormals flush to signed zero; RND32 clears the low8
// input bits BEFORE arithmetic. No host floating-point arithmetic or status.
[[nodiscard]] SharcFloatSum sharc_float_add(std::uint64_t left,
                                            std::uint64_t right,
                                            SharcFloatFormat format,
                                            SharcFloatRounding rounding,
                                            bool subtract = false);

}
