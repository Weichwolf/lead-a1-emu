// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int class_shift = 40;
constexpr std::uint64_t call = 0x8000000000;
constexpr std::uint64_t delayed = 0x4000000;

bool body_context(const SharcState &state) {
  if (!state.count_loop) { throw std::logic_error("Missing SHARC count loop"); }
  const auto &loop = *state.count_loop;
  if (!state.depth) { throw std::runtime_error("Unknown SHARC loop PC stack depth"); }
  return *state.depth == loop.saved_depth + (loop.frame_live ? 1U : 0U);
}

bool body_address(const SharcCountLoop &loop, std::uint32_t address) {
  return address >= loop.body && address <= loop.end;
}

void require_call_frames(const SharcState &state) {
  if (!state.count_loop) { throw std::logic_error("Missing SHARC count loop"); }
  const auto &loop = *state.count_loop;
  const auto floor = loop.saved_depth + (loop.frame_live ? 1U : 0U);
  if (!state.depth || *state.depth < floor) {
    throw std::runtime_error("SHARC loop PC stack underflow");
  }
  for (auto index = floor; index < *state.depth; ++index) {
    const auto frame = state.pc_stack[index];
    if (!frame || (*frame & (sharc_call_frame | sharc_special_frame)) != sharc_call_frame) {
      throw std::runtime_error("SHARC unknown/special loop subroutine frame not qualified");
    }
  }
}

}

void sharc_long_loop_prepare(SharcState &state, std::uint64_t opcode) {
  if (!state.count_loop) { return; }
  require_call_frames(state);
  const auto type = opcode >> class_shift;
  const bool branch = type == 6 || type == 7;
  if (state.branch || ((branch || type == 10) && (opcode & delayed) != 0)) {
    throw std::runtime_error("SHARC delayed control flow inside a count loop not qualified");
  }
  if (!body_context(state)) {
    if (body_address(*state.count_loop, state.pc)) {
      throw std::runtime_error("SHARC loop subroutine overlaps its active body");
    }
    return;
  }
  auto &loop = *state.count_loop;
  if (!body_address(loop, state.pc)) {
    throw std::runtime_error("SHARC count-loop execution address mismatch");
  }
  // Calls from e-3 need PCSTK bit25 handling; the final three instruction
  // positions have additional re-entry restrictions (PGR4-55/74,A-10).
  // An immediate CALL at e-4 redirects Fetch1 before e is fetched (4-20).
  if (branch && ((opcode & call) == 0 || state.pc > loop.end - 4)) {
    throw std::runtime_error("SHARC loop-body JUMP/late CALL re-entry not qualified");
  }
  if (type == 10) { throw std::runtime_error("SHARC RTS from count-loop body not qualified"); }
  // PGR4-47/48: stacks pop in e-3's cycle, before its register operands.
  // For CALL at e-4 this occurs after RTS, when the final four instructions
  // resume. Subroutine instructions must not consume loop iterations or pop
  // a CALL frame. CURLCNTR/LADDR and speculative fetch readback stay guarded.
  if (loop.remaining == 1 && state.pc == loop.end - 3) {
    state.depth = loop.saved_depth;
    state.loops_empty = true;
    loop.frame_live = false;
    sharc_stack_status(state);
  }
}

void sharc_long_loop_finish(const SharcState &previous, SharcState &next) {
  if (!previous.count_loop) { return; }
  if (!next.count_loop) { throw std::logic_error("Lost active SHARC count loop"); }
  require_call_frames(next);
  auto &loop = *next.count_loop;
  if (body_context(previous) && previous.pc == loop.end) {
    if (loop.remaining == 1) {
      if (loop.frame_live) { throw std::logic_error("SHARC final loop frame was not popped"); }
      next.count_loop.reset();
      return;
    }
    --loop.remaining;
    next.pc = loop.body;
  }
  if (body_context(next) != body_address(loop, next.pc)) {
    throw std::runtime_error("SHARC loop branch/return escaped its execution context");
  }
}

}
