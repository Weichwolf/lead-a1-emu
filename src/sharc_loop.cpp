// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint64_t full_counter = std::uint64_t{1} << 32;
constexpr unsigned int before_almost_full = 28;

void pop(SharcState &state, SharcCountLoop &loop) {
  if (!loop.frame_live) { return; }
  state.depth = loop.saved_depth;
  state.loops_empty = true;
  loop.frame_live = false;
}

}

void sharc_validate_count_loop(const SharcState &state) {
  const auto loop = state.count_loop;
  if (!loop) { return; }
  if (state.branch || !state.depth || !state.loops_empty || loop->total == 0 ||
      loop->total > full_counter || loop->remaining == 0 || loop->remaining > loop->total ||
      loop->saved_depth >= before_almost_full || loop->end < loop->body ||
      loop->end > sharc_pc_mask || (loop->end != loop->body && loop->end - loop->body < 4) ||
      *state.loops_empty == loop->frame_live) {
    throw std::invalid_argument("Invalid SHARC count-loop state");
  }
  const auto floor = loop->saved_depth + (loop->frame_live ? 1U : 0U);
  const bool in_body = state.pc >= loop->body && state.pc <= loop->end;
  if (*state.depth < floor || (in_body != (*state.depth == floor)) ||
      (loop->end != loop->body && !loop->frame_live &&
       (loop->remaining != 1 || state.pc < loop->end - 3 || !in_body || *state.depth != floor)) ||
      (loop->end != loop->body && loop->frame_live && loop->remaining == 1 && in_body &&
       state.pc > loop->end - 3) ||
      (loop->end == loop->body && (state.pc != loop->body || *state.depth != floor))) {
    throw std::invalid_argument("Invalid SHARC count-loop execution context");
  }
  if (loop->frame_live && state.pc_stack[loop->saved_depth] != loop->body) {
    throw std::invalid_argument("Invalid SHARC loop PC frame");
  }
  for (const auto &pending : state.pending) {
    if (pending) {
      throw std::invalid_argument("Unqualified pending control in inherited SHARC count loop");
    }
  }
}

void sharc_begin_count_loop(SharcState &state, std::uint64_t opcode) {
  if (state.branch || state.count_loop || !state.loops_empty || !*state.loops_empty) {
    throw std::runtime_error("SHARC nested/delayed count-loop setup not implemented");
  }
  if (!state.depth) { throw std::runtime_error("Unknown SHARC PC stack depth"); }
  if (*state.depth >= before_almost_full) {
    throw std::runtime_error("SHARC loop stack-full interrupt boundary not implemented");
  }
  const auto end =
      (state.pc + (static_cast<std::uint32_t>(opcode) & sharc_pc_mask)) & sharc_pc_mask;
  const auto body = (state.pc + 1) & sharc_pc_mask;
  if (end < body || (end != body && end - body < 4)) {
    throw std::runtime_error("SHARC wrapped/two-to-four-instruction count loops not implemented");
  }
  constexpr unsigned int class_shift = 40;
  constexpr unsigned int immediate_class = 12;
  constexpr unsigned int counter_shift = 24;
  constexpr std::uint64_t immediate_mask = 0xffff;
  std::uint32_t count{};
  if ((opcode >> class_shift) == immediate_class) {
    count = static_cast<std::uint32_t>((opcode >> counter_shift) & immediate_mask);
  } else {
    constexpr std::uint64_t reserved = 0x80ff000000;
    if ((opcode & reserved) != 0) { throw std::runtime_error("Reserved SHARC UREG loop fields"); }
    constexpr unsigned int register_shift = 32;
    constexpr std::uint64_t register_mask = 127;
    count = sharc_value(
        sharc_ureg(state, static_cast<unsigned int>((opcode >> register_shift) & register_mask)));
  }
  const auto iterations = count == 0 ? full_counter : count;
  state.count_loop = SharcCountLoop{.body = body,
                                    .end = end,
                                    .total = iterations,
                                    .remaining = iterations,
                                    .saved_depth = *state.depth,
                                    .frame_live = true};
  state.pc_stack[*state.depth] = body; // Bit24=0: loop frame, not a CALL.
  ++*state.depth;
  state.loops_empty = false;
}

bool sharc_loop_body_supported(std::uint64_t opcode) {
  constexpr std::uint64_t indexed_mask = 0xf00000000000;
  constexpr std::uint64_t indexed_move = 0x600000000000;
  constexpr unsigned int class_shift = 40;
  return opcode == 0 || (opcode >> class_shift) == 1 || (opcode >> class_shift) == 2 ||
         (opcode & indexed_mask) == indexed_move;
}

unsigned int sharc_count_loop_step(SharcState &state, std::uint64_t opcode) {
  if (!sharc_loop_body_supported(opcode)) {
    throw std::runtime_error("SHARC count-loop body instruction/pipeline not implemented");
  }
  auto loop = state.count_loop;
  if (!loop || loop->body != state.pc) {
    throw std::runtime_error("SHARC count-loop execution address mismatch");
  }
  const bool first = loop->remaining == loop->total;
  const bool last = loop->remaining == 1;
  // PGR4-61..63: stack pre-pop occurs before the final executions have drained.
  // Reviewed RAM/compute operations cannot access loop resources.
  if ((loop->total > 4 && loop->remaining == 4) || (loop->total < 4 && loop->total > 1 && last)) {
    pop(state, *loop);
  }
  --loop->remaining;
  if ((loop->total == 4 && first) || last) { pop(state, *loop); }
  unsigned int minimum_cycles = first ? 2U : 1U; // Initial short-loop bubble.
  if (last && loop->total < 4) { minimum_cycles += 3; }
  if (last) {
    state.count_loop.reset();
    state.pc = (state.pc + 1) & sharc_pc_mask;
  } else {
    state.count_loop = *loop;
  }
  return minimum_cycles;
}

}
