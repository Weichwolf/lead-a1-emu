// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t internal_begin = 0x80000;
constexpr std::uint32_t internal_end = 0x100000;

}

void sharc_validate_idle(const SharcState &state) {
  if (!state.idle_pc) { return; }
  if (*state.idle_pc < internal_begin || *state.idle_pc >= internal_end - 1 ||
      state.pc != *state.idle_pc + 1 || state.branch || state.count_loop || state.arithmetic_loop ||
      state.loops_empty != true || state.cache.pending_flush) {
    throw std::invalid_argument("Invalid SHARC ordinary IDLE/resume state");
  }
  for (const auto &pending : state.pending) {
    if (pending) { throw std::invalid_argument("Invalid SHARC pending effect after retired IDLE"); }
  }
}

void sharc_begin_idle(SharcState &state) {
  if (state.branch || state.count_loop || state.arithmetic_loop || state.loops_empty != true) {
    throw std::runtime_error("SHARC delayed-branch/loop IDLE not qualified");
  }
  if (state.pc < internal_begin || state.pc >= internal_end - 1) {
    throw std::runtime_error("SHARC IDLE native internal resume boundary not qualified");
  }
  state.idle_pc = state.pc;
}

}
