// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>

namespace original_picr {
enum class Target : std::uint8_t {
  base0,
  length0,
  data0,
  data2,
  data3,
  modifier8,
  modifier1,
  mode,
  nop
};

struct Setup {
  unsigned int offset;
  unsigned int words;
  Target target;
  std::uint32_t value{};
};

// Independently scanned original fields; main and part1 use different stack bases.
inline constexpr std::array<Setup, 11> setup{
    {{.offset = 944, .words = 3, .target = Target::base0},
     {.offset = 947, .words = 2, .target = Target::length0, .value = 0x100},
     {.offset = 949, .words = 2, .target = Target::data2, .value = 0x100},
     {.offset = 951, .words = 3, .target = Target::data3, .value = 0x80000},
     {.offset = 954, .words = 2, .target = Target::data0, .value = 0},
     {.offset = 956, .words = 2, .target = Target::modifier8, .value = 0},
     {.offset = 958, .words = 2, .target = Target::data0, .value = 1},
     {.offset = 960, .words = 2, .target = Target::modifier1, .value = 1},
     {.offset = 962, .words = 2, .target = Target::mode},
     {.offset = 964, .words = 1, .target = Target::nop},
     {.offset = 965, .words = 1, .target = Target::nop}}};
}
