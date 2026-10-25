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
constexpr std::uint64_t full_count = std::uint64_t{1} << 32;
constexpr unsigned int before_almost_full = 28;

const SharcCountLoop &loop(const SharcState &state) {
  if (!state.count_loop) { throw std::logic_error("Missing SHARC four-instruction loop"); }
  return *state.count_loop;
}

const auto &frames(const SharcState &state) {
  const auto &active = loop(state);
  return active.control ? active.control->frames : state.pc_stack;
}

unsigned int depth(const SharcState &state) {
  const auto &active = loop(state);
  const auto value = active.control ? active.control->depth : state.depth;
  if (!value) { throw std::runtime_error("Unknown SHARC loop control depth"); }
  return *value;
}

unsigned int floor(const SharcState &state) {
  const auto &active = loop(state);
  return active.saved_depth + (active.frame_live ? 1U : 0U);
}

bool body_address(const SharcState &state, std::uint32_t address) {
  const auto &active = loop(state);
  return address >= active.body && address <= active.end;
}

bool body_instruction(const SharcState &state) {
  return !state.branch && depth(state) == floor(state) && body_address(state, state.pc);
}

void pop(SharcState &state) {
  if (!state.count_loop) { throw std::logic_error("Missing SHARC loop to pop"); }
  auto &active = *state.count_loop;
  state.depth = active.saved_depth;
  state.loops_empty = true;
  active.frame_live = false;
  active.four_pop_pending = false;
  sharc_stack_status(state);
}

void unknown_startup(SharcState &state) {
  if (!state.count_loop) { throw std::logic_error("Missing SHARC loop startup"); }
  auto &active = *state.count_loop;
  active.control = SharcLoopControl{.frames = state.pc_stack, .depth = state.depth};
  state.depth.reset();
  state.loops_empty.reset();
  for (auto index = active.saved_depth; index < state.pc_stack.size(); ++index) {
    state.pc_stack[index].reset();
  }
  sharc_stack_status(state);
}

void context(const SharcState &state) {
  const auto &active = loop(state);
  const auto current = depth(state);
  const auto base = floor(state);
  if (current < base || current > before_almost_full) {
    throw std::runtime_error("Invalid SHARC four-loop control depth");
  }
  if (state.branch) {
    if (!body_address(state, state.pc) || state.pc == active.body || state.pc == active.end ||
        state.pc - active.body + state.branch->slots != 3 || current != base + 1 ||
        body_address(state, state.branch->target)) {
      throw std::runtime_error("Invalid SHARC four-loop delay-slot context");
    }
  } else if (body_address(state, state.pc) != (current == base)) {
    throw std::runtime_error("SHARC four-loop branch/return escaped its execution context");
  }
  const auto &stored = frames(state);
  if (active.frame_live && stored[active.saved_depth] != active.body) {
    throw std::runtime_error("Invalid SHARC four-loop start frame");
  }
  for (auto index = base; index < current; ++index) {
    const auto frame = stored[index];
    if (!frame || (*frame & sharc_call_frame) == 0) {
      throw std::runtime_error("Unknown SHARC four-loop subroutine frame");
    }
    if ((*frame & sharc_special_frame) != 0 &&
        (active.control || active.frame_live || active.remaining != 1 ||
         index != active.saved_depth || (*frame & sharc_pc_mask) != active.end)) {
      throw std::runtime_error("Unqualified SHARC four-loop special frame");
    }
  }
}

void validate_startup(const SharcState &state) {
  const auto &active = loop(state);
  if (!active.control) { throw std::logic_error("Missing SHARC startup control"); }
  if (active.total != 1 || active.remaining != 1 || !active.frame_live || active.four_pop_pending ||
      state.depth || state.loops_empty) {
    throw std::invalid_argument("Invalid SHARC unknown four-loop startup state");
  }
  for (unsigned int index = 0; index < state.pc_stack.size(); ++index) {
    const auto frame = active.control->frames[index];
    if (frame && (*frame & ~sharc_frame_mask) != 0) {
      throw std::invalid_argument("Invalid SHARC startup control frame");
    }
    if (index < active.saved_depth && frame != state.pc_stack[index]) {
      throw std::invalid_argument("Changed SHARC startup parent frame");
    }
    if (index >= active.saved_depth && state.pc_stack[index]) {
      throw std::invalid_argument("Fabricated SHARC startup PC frame");
    }
  }
}

}

bool sharc_four_loop(const SharcState &state) {
  return state.count_loop && state.count_loop->end >= state.count_loop->body &&
         state.count_loop->end - state.count_loop->body == 3;
}

void sharc_validate_four_loop(const SharcState &state) {
  const auto &active = loop(state);
  if (active.end > sharc_pc_mask || active.total == 0 || active.total > full_count ||
      active.remaining == 0 || active.remaining > active.total ||
      active.saved_depth >= before_almost_full) {
    throw std::invalid_argument("Invalid SHARC four-loop state");
  }
  if (active.four_pop_pending && (active.total != 1 || !active.frame_live || active.control ||
                                  state.branch || state.pc != active.body + 1)) {
    throw std::invalid_argument("Invalid SHARC four-loop pending pop");
  }
  if (active.control) {
    validate_startup(state);
  } else if (!state.depth || !state.loops_empty || *state.loops_empty == active.frame_live ||
             (!active.frame_live && active.remaining != 1) ||
             (active.frame_live && active.remaining == 1 && state.pc != active.body &&
              !active.four_pop_pending)) {
    throw std::invalid_argument("Invalid SHARC four-loop resource state");
  }
  context(state);
  for (const auto &pending : state.pending) {
    if (pending) {
      throw std::invalid_argument("Unqualified pending control in inherited SHARC four loop");
    }
  }
}

unsigned int sharc_four_loop_prepare(SharcState &state, std::uint64_t opcode) {
  if (loop(state).four_pop_pending) {
    throw std::runtime_error("SHARC four-loop pending pop requires its owner clock");
  }
  context(state);
  const auto &active = loop(state);
  const auto type = opcode >> class_shift;
  const bool branch = type == 6 || type == 7;
  const bool returning = type == 10;
  const bool body = body_instruction(state);
  if (state.branch && (branch || returning)) {
    throw std::runtime_error("SHARC branch in four-loop delay slot not qualified");
  }
  if (body &&
      ((branch && ((opcode & (call | delayed)) != (call | delayed) || state.pc != active.body)) ||
       returning)) {
    throw std::runtime_error("SHARC four-loop body JUMP/plain-CALL/RTS not qualified");
  }
  if (!body && !state.branch && (branch || returning) && (opcode & delayed) != 0) {
    throw std::runtime_error("SHARC delayed four-loop callee flow not qualified");
  }
  const bool first = body && state.pc == active.body;
  const bool initial = first && active.remaining == active.total;
  if (first && active.remaining == 1 && active.frame_live) {
    const auto condition = static_cast<unsigned int>((opcode >> 33) & 31);
    if (active.total == 1 && branch && sharc_condition(state, condition)) {
      // PGR4-67's startup bubble does not specify CALL/frame arbitration.
      // Preserve exact functional control, but expose no guessed stack state.
      unknown_startup(state);
    } else if (active.total > 1) {
      // PGR4-47/A-10: final steady CALL's Address stage coincides with LCE.
      // Pop the loop frame before pushing its bit25-marked CALL frame.
      pop(state);
    }
  }
  return initial ? 2U : 1U;
}

void sharc_four_loop_finish(const SharcState &previous, SharcState &next, bool clocked) {
  const auto &before = loop(previous);
  if (!next.count_loop) { throw std::logic_error("Lost SHARC four loop"); }
  auto &active = *next.count_loop;
  if (before.total == 1 && !active.control && previous.pc == before.body) {
    // PGR4-67: startup bubble after the first body instruction pops the stacks.
    if (clocked) {
      active.four_pop_pending = true;
    } else {
      pop(next);
    }
  }
  if (body_instruction(previous) && previous.pc == before.end) {
    if (active.remaining == 1) {
      if (active.control) {
        next.depth = active.saved_depth;
        next.loops_empty = true;
      } else if (active.frame_live) {
        throw std::logic_error("SHARC final four-loop frame was not popped");
      }
      next.count_loop.reset();
      return;
    }
    --active.remaining;
    next.pc = active.body;
  }
  context(next);
}

void sharc_four_loop_clock_pop(SharcState &state) {
  const auto &active = loop(state);
  if (!active.four_pop_pending || active.total != 1 || active.control || !active.frame_live ||
      state.branch || state.pc != active.body + 1 || state.depth != active.saved_depth + 1 ||
      state.pc_stack[active.saved_depth] != active.body || state.loops_empty != false) {
    throw std::runtime_error("SHARC four-loop clocked pop context mismatch");
  }
  pop(state);
}

}
