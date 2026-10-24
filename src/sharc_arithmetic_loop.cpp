// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t all_bits = 0xffffffff;
constexpr unsigned int before_almost_full = 28;
constexpr unsigned int condition_count = 32;
constexpr unsigned int counter_condition = 15;
constexpr unsigned int forever = 31;
constexpr unsigned int condition_shift = 24;

bool live(const SharcArithmeticLoop &loop) {
  return loop.phase != SharcArithmeticPhase::draining;
}

bool body_address(const SharcArithmeticLoop &loop, std::uint32_t pc) {
  return pc >= loop.body && pc <= loop.end;
}

bool register_or_compute(std::uint64_t opcode) {
  constexpr unsigned int class_shift = 40;
  const auto type = opcode >> class_shift;
  constexpr std::uint64_t register_mask = 0xf00000000000;
  constexpr std::uint64_t register_move = 0x700000000000;
  return opcode == 0 || type == 1 || type == 2 || type == 15 ||
         (opcode & register_mask) == register_move;
}

void deferred_guard(const SharcState &state, std::uint64_t opcode) {
  if (!state.arithmetic_loop) { throw std::logic_error("Missing deferred SHARC arithmetic loop"); }
  const auto &loop = *state.arithmetic_loop;
  constexpr std::uint64_t modulus = std::uint64_t{1} << 24;
  const auto until_end = (modulus + loop.end - state.pc) & sharc_pc_mask;
  constexpr unsigned int fetch_distance = 4;
  constexpr unsigned int class_shift = 40;
  const auto type = opcode >> class_shift;
  if (until_end <= fetch_distance || type == 6 || type == 7 || type == 10 || type == 11) {
    throw std::runtime_error("SHARC deferred arithmetic-loop fetch/branch not qualified");
  }
}

}

void sharc_validate_arithmetic_loop(const SharcState &state) {
  if (!state.arithmetic_loop) { return; }
  const auto &loop = *state.arithmetic_loop;
  if (state.count_loop || state.branch || !state.depth || !state.loops_empty ||
      loop.body > sharc_pc_mask || loop.end > sharc_pc_mask ||
      loop.termination >= condition_count || loop.termination == counter_condition ||
      loop.saved_depth >= before_almost_full || loop.kind != SharcArithmeticKind::multiple ||
      *state.depth != loop.saved_depth + (live(loop) ? 1U : 0U) ||
      *state.loops_empty == live(loop)) {
    throw std::invalid_argument("Invalid SHARC arithmetic-loop resources");
  }
  if (live(loop) && state.pc_stack[loop.saved_depth] != loop.body) {
    throw std::invalid_argument("Invalid SHARC arithmetic-loop PC frame");
  }
  switch (loop.phase) {
    case SharcArithmeticPhase::deferred: return;
    case SharcArithmeticPhase::active:
    case SharcArithmeticPhase::draining:
      if (loop.end < loop.body || loop.end - loop.body < 4 || !body_address(loop, state.pc) ||
          (!live(loop) && state.pc < loop.end - 1)) {
        throw std::invalid_argument("Invalid SHARC arithmetic-loop execution context");
      }
      return;
    default: throw std::invalid_argument("Invalid SHARC arithmetic-loop phase");
  }
}

void sharc_begin_arithmetic_loop(SharcState &state, std::uint64_t opcode) {
  constexpr std::uint64_t allowed = 0x0e3e00ffffff;
  if ((opcode & ~allowed) != 0) { throw std::runtime_error("Reserved SHARC arithmetic DO fields"); }
  if (state.branch || state.count_loop || state.arithmetic_loop || state.loops_empty != true) {
    throw std::runtime_error("SHARC nested/delayed arithmetic DO not implemented");
  }
  if (!state.depth) { throw std::runtime_error("Unknown SHARC PC stack depth"); }
  if (*state.depth >= before_almost_full) {
    throw std::runtime_error("SHARC loop stack-full interrupt boundary not implemented");
  }
  const auto body = (state.pc + 1) & sharc_pc_mask;
  const auto end = static_cast<std::uint32_t>((std::uint64_t{state.pc} + (opcode & sharc_pc_mask)) &
                                              sharc_pc_mask);
  const auto termination = static_cast<unsigned int>((opcode >> 33) & 31);
  if (termination == counter_condition) {
    throw std::runtime_error("SHARC arithmetic DO with counter condition not implemented");
  }
  if (end >= body && end - body < 4) {
    throw std::runtime_error("SHARC short arithmetic DO pipeline not implemented");
  }
  state.arithmetic_loop = SharcArithmeticLoop{.body = body,
                                              .end = end,
                                              .termination = termination,
                                              .saved_depth = *state.depth,
                                              .kind = SharcArithmeticKind::multiple,
                                              .phase = end < body ? SharcArithmeticPhase::deferred
                                                                  : SharcArithmeticPhase::active};
  state.pc_stack[*state.depth] = body;
  ++*state.depth;
  state.loops_empty = false;
}

void sharc_arithmetic_loop_prepare(SharcState &state, std::uint64_t opcode) {
  if (!state.arithmetic_loop) { return; }
  auto &loop = *state.arithmetic_loop;
  if (loop.phase == SharcArithmeticPhase::deferred) {
    deferred_guard(state, opcode);
    return;
  }
  if (!body_address(loop, state.pc) || !register_or_compute(opcode)) {
    throw std::runtime_error("SHARC arithmetic-loop body memory/control flow not qualified");
  }
  // PGR4-53/Table4-15: Fetch1 contains b+1 while Execute contains e-2.
  // Test old flags before e-2 computes or reads the just-popped resources.
  if (live(loop) && state.pc == loop.end - 2 && loop.termination != forever &&
      sharc_condition(state, loop.termination)) {
    state.depth = loop.saved_depth;
    state.loops_empty = true;
    loop.phase = SharcArithmeticPhase::draining;
    sharc_stack_status(state);
  }
}

unsigned int sharc_arithmetic_loop_finish(const SharcState &previous, SharcState &next) {
  if (!previous.arithmetic_loop) { return 0; }
  if (!next.arithmetic_loop) { throw std::logic_error("Lost SHARC arithmetic loop"); }
  const auto loop = *next.arithmetic_loop;
  if (loop.phase == SharcArithmeticPhase::deferred || previous.pc != loop.end) { return 0; }
  if (live(loop)) {
    next.pc = loop.body;
    return 0;
  }
  next.arithmetic_loop.reset();
  return 2; // Two flushed instructions, not two retired guest NOPs (4-53/54).
}

void sharc_arithmetic_stack_write(SharcState &state, SharcKnown32 value) {
  if (!state.arithmetic_loop) { throw std::logic_error("Missing SHARC arithmetic-loop stack"); }
  auto &loop = *state.arithmetic_loop;
  if (loop.phase != SharcArithmeticPhase::deferred || !state.depth ||
      *state.depth != loop.saved_depth + 1) {
    throw std::runtime_error("SHARC active arithmetic-loop PCSTK write not qualified");
  }
  const auto address = sharc_value(value);
  if ((address & ~sharc_pc_mask) != 0) {
    throw std::runtime_error("SHARC arithmetic-loop PCSTK metadata write not qualified");
  }
  loop.body = address;
  state.pc_stack[loop.saved_depth] = address;
  // Do not infer a new short-loop type or a new fetch phase from this write.
}

SharcKnown32 sharc_loop_address(const SharcState &state) {
  if (state.arithmetic_loop && live(*state.arithmetic_loop)) {
    const auto &loop = *state.arithmetic_loop;
    return {.value = loop.end | (loop.termination << condition_shift), .known = all_bits};
  }
  if (state.loops_empty == true) { return {.value = all_bits, .known = all_bits}; }
  throw std::runtime_error("SHARC active/unknown counter-loop LADDR read not implemented");
}

}
