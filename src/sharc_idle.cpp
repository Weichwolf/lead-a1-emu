// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <sstream>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t internal_begin = 0x80000;
constexpr std::uint32_t internal_end = 0x100000;

bool bit(SharcKnown32 value, std::uint32_t mask) {
  if ((value.known & mask) != mask) { throw std::runtime_error("Unknown SHARC IDLE wake control"); }
  return (value.value & mask) != 0;
}

bool idle_context(const SharcState &state) {
  return (!state.count_loop && !state.arithmetic_loop && state.loops_empty == true) ||
         sharc_arithmetic_entry_delay(state);
}

}

void sharc_validate_idle(const SharcState &state) {
  if (state.idle_delay_word) {
    constexpr std::uint64_t native_bits = 0xffffffffffff;
    if ((*state.idle_delay_word & ~native_bits) != 0 || !state.branch || state.branch->slots != 1 ||
        state.pc < internal_begin || state.pc >= internal_end || !idle_context(state) ||
        state.cache.pending_flush) {
      throw std::invalid_argument("Invalid SHARC retained IDLE delay word");
    }
    for (const auto &pending : state.pending) {
      if (pending) {
        throw std::invalid_argument("Invalid SHARC effect before retained IDLE slot");
      }
    }
  }
  if (!state.idle_pc) { return; }
  if (*state.idle_pc < internal_begin || *state.idle_pc >= internal_end - 1 ||
      state.pc != *state.idle_pc + 1 || (state.branch && !state.idle_delay_word) ||
      !idle_context(state) || state.cache.pending_flush) {
    throw std::invalid_argument("Invalid SHARC ordinary IDLE/resume state");
  }
  for (const auto &pending : state.pending) {
    if (pending) { throw std::invalid_argument("Invalid SHARC pending effect after retired IDLE"); }
  }
}

void sharc_begin_idle(SharcState &state, bool fetched) {
  const bool delayed = state.branch && (!fetched || state.branch->slots != 2 ||
                                        state.branch->target < internal_begin ||
                                        state.branch->target >= internal_end);
  if (delayed || !idle_context(state) || (!fetched && state.arithmetic_loop)) {
    throw std::runtime_error("SHARC delayed-branch/loop IDLE not qualified");
  }
  if (state.pc < internal_begin || state.pc >= internal_end - 1) {
    throw std::runtime_error("SHARC IDLE native internal resume boundary not qualified");
  }
  state.idle_pc = state.pc;
}

bool sharc_wake_idle(SharcState &state) {
  if (!state.idle_pc) { return false; }
  sharc_interrupt_environment_guard(state);
  constexpr std::uint32_t all_bits = 0xffffffff;
  constexpr std::uint32_t p1i = 0x1000;
  constexpr std::uint32_t global_enable = 0x1000;
  constexpr std::uint32_t interrupt_bits = 0xffffff7c;
  const auto latch = state.effective[sharc_irptl];
  if (latch.known == all_bits && latch.value == 0) { return false; }
  if (!bit(state.effective[sharc_mode1], global_enable)) { return false; }
  const auto mask = state.effective[sharc_imask];
  const auto possible = (latch.value | ~latch.known) & (mask.value | ~mask.known) & interrupt_bits;
  if (possible == 0) { return false; }
  if (possible != p1i || !bit(latch, p1i) || !bit(mask, p1i) || state.interrupt) {
    throw std::runtime_error("SHARC other/nested IDLE wake source not qualified");
  }
  constexpr std::uint32_t nesting = 0x800;
  if (bit(state.effective[sharc_mode1], nesting)) {
    throw std::runtime_error("SHARC nested IDLE wake source not qualified");
  }
  state.idle_pc.reset();
  return true; // Wake does not clear IRPTL, push a frame or complete the pending branch.
}

}

namespace lead_a1 {

bool Sharc::wake_idle() try {
  if (memory_.active()) { throw std::logic_error("Active SHARC transaction at IDLE wake"); }
  auto next = state_;
  const bool woke = detail::sharc_wake_idle(next);
  detail::sharc_validate_idle(next);
  state_ = next;
  return woke;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC IDLE wake pc=0x" << std::hex << state_.pc << " IRPTL=0x"
          << state_.effective[10].value << "/0x" << state_.effective[10].known << " IMASK=0x"
          << state_.effective[11].value << "/0x" << state_.effective[11].known << ": "
          << error.what();
  throw std::runtime_error(message.str());
}

}
