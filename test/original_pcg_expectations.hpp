// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "original_sport_expectations.hpp"

#include <array>
#include <cstdint>

namespace original_pcg {
using Operation = original_sport::Operation;
using Step = original_sport::Step;

inline constexpr std::array<std::uint32_t, 12> configured{
    0x100, 0x200004, 0x100, 6, 0x10080, 0x40000, 0, 0, 0, 0, 0, 0};

// Owned scans of hash-pinned OS1.36 fields, independently corroborated in the
// private factual handoff6400242285. Values are data, not original opcodes.
// First four PCG setup instructions already belong to controls_0.
inline constexpr std::array<Step, 15> setup{
    {{.offset = 818,
      .words = 3,
      .reg = 0,
      .value = 0x10080,
      .operation = Operation::write,
      .address = 0x24c4},
     {.offset = 821,
      .words = 3,
      .reg = 0x70,
      .value = 0,
      .operation = Operation::read,
      .address = 0x24c5},
     {.offset = 824, .words = 3, .reg = 0x70, .value = 0x40000},
     {.offset = 827,
      .words = 3,
      .reg = 0x70,
      .value = 0x40000,
      .operation = Operation::write,
      .address = 0x24c5},
     {.offset = 830, .words = 2, .reg = 0, .value = 4},
     {.offset = 832, .words = 3, .reg = 1, .value = 0x200000},
     {.offset = 835, .words = 1, .reg = 0, .value = 0x200004},
     {.offset = 836,
      .words = 3,
      .reg = 0,
      .value = 0x200004,
      .operation = Operation::write,
      .address = 0x24c1},
     {.offset = 839, .words = 2, .reg = 0, .value = 6},
     {.offset = 841,
      .words = 3,
      .reg = 0,
      .value = 6,
      .operation = Operation::write,
      .address = 0x24c3},
     {.offset = 844, .words = 2, .reg = 0, .value = 0x100},
     {.offset = 846, .words = 2, .reg = 0x70, .value = 0x100},
     {.offset = 848,
      .words = 3,
      .reg = 0x70,
      .value = 0x100,
      .operation = Operation::write,
      .address = 0x24c0},
     {.offset = 851, .words = 2, .reg = 0x70, .value = 0x100},
     {.offset = 853,
      .words = 3,
      .reg = 0x70,
      .value = 0x100,
      .operation = Operation::write,
      .address = 0x24c2}}};
}
