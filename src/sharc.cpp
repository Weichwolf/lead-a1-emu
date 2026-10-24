// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "sharc_dag_internal.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint64_t opcode_byte = 0xff0000000000;
constexpr std::uint64_t condition_field = 0x3e00000000;
constexpr std::uint64_t delayed = 0x4000000;
constexpr std::uint64_t call_flag = 0x8000000000;
constexpr std::uint64_t loop_abort = 0x4000000000;
constexpr std::uint64_t clear_interrupt = 0x1000000;
constexpr unsigned int ureg_shift = 32;
constexpr unsigned int class_shift = 40;
constexpr unsigned int ureg_mask = 127;
constexpr unsigned int dm_read_class = 16;
constexpr unsigned int dm_write_class = 17;
constexpr unsigned int pm_read_class = 18;
constexpr unsigned int pm_write_class = 19;
constexpr unsigned int system_bit_class = 20;
constexpr unsigned int cache_stack_class = 23;
constexpr std::uint32_t all_bits = std::numeric_limits<std::uint32_t>::max();

void validate(const SharcState &state) {
  detail::sharc_validate_registers(state);
  detail::sharc_validate_cache(state.cache);
  detail::sharc_validate_flag_inputs(state.flag_inputs);
  if (state.pc > detail::sharc_pc_mask || (state.depth && *state.depth > state.pc_stack.size())) {
    throw std::invalid_argument("Invalid SHARC PC/stack state");
  }
  for (unsigned int index = 0; index < state.system.size(); ++index) {
    if (state.pending[index] && index != detail::sharc_mode1 && index != detail::sharc_irptl &&
        index != detail::sharc_imask && index != detail::sharc_flags) {
      throw std::invalid_argument("Unsupported inherited SHARC system effect");
    }
    for (const auto word : {state.system[index],
                            state.effective[index],
                            state.pending[index].value_or(SharcKnown32{})}) {
      if ((word.value & ~word.known) != 0) {
        throw std::invalid_argument("Invalid SHARC bit knowledge");
      }
    }
  }
  for (const auto frame : state.pc_stack) {
    if (frame && (*frame & ~detail::sharc_frame_mask) != 0) {
      throw std::invalid_argument("Invalid SHARC PC stack frame");
    }
  }
  if (state.branch && (state.branch->target > detail::sharc_pc_mask || state.branch->slots == 0 ||
                       state.branch->slots > 2)) {
    throw std::invalid_argument("Invalid SHARC delayed branch state");
  }
}

std::optional<std::uint32_t> schedule(SharcState &state, std::uint32_t target, bool db) {
  if (db) {
    state.branch = SharcBranch{.target = target, .slots = 2};
  } else {
    return target;
  }
  return {};
}

bool taken(const SharcState &state, std::uint64_t opcode, std::optional<bool> captured) {
  if (captured) { return *captured; }
  const auto condition = static_cast<unsigned int>((opcode >> 33) & 31);
  return detail::sharc_condition(state, condition);
}

std::optional<std::uint32_t>
direct_branch(SharcState &state, std::uint64_t opcode, std::optional<bool> captured) {
  constexpr auto allowed = opcode_byte | condition_field | delayed | call_flag | loop_abort |
                           clear_interrupt | detail::sharc_pc_mask;
  if ((opcode & ~allowed) != 0) { throw std::runtime_error("Reserved SHARC direct-branch fields"); }
  if ((opcode & (loop_abort | clear_interrupt)) != 0) {
    throw std::runtime_error("SHARC loop-abort/clear-interrupt branch not implemented");
  }
  if (!taken(state, opcode, captured)) { return {}; }
  if (state.branch) {
    throw std::runtime_error("SHARC taken branch in delay slot not implemented");
  }
  const bool db = (opcode & delayed) != 0;
  if ((opcode & call_flag) != 0) {
    detail::sharc_push_call(state, (state.pc + (db ? 3U : 1U)) & detail::sharc_pc_mask);
  }
  auto target = static_cast<std::uint32_t>(opcode) & detail::sharc_pc_mask;
  constexpr unsigned int relative = 7;
  if ((opcode >> class_shift) == relative) { target = (state.pc + target) & detail::sharc_pc_mask; }
  return schedule(state, target, db);
}

std::optional<std::uint32_t>
return_branch(SharcState &state, std::uint64_t opcode, std::optional<bool> captured) {
  constexpr auto allowed = opcode_byte | condition_field | delayed;
  if ((opcode & ~allowed) != 0) {
    throw std::runtime_error("SHARC RTS compute/loop-return/reserved fields not implemented");
  }
  if (!taken(state, opcode, captured)) { return {}; }
  if (state.branch) {
    throw std::runtime_error("SHARC taken return in delay slot not implemented");
  }
  return schedule(state, detail::sharc_pop_call(state), (opcode & delayed) != 0);
}

void direct_move(SharcMemory &memory,
                 SharcState &state,
                 std::uint64_t opcode,
                 const SharcControl *control) try {
  constexpr std::uint64_t long_word = 0x8000000000;
  if ((opcode & long_word) != 0) {
    throw std::runtime_error("SHARC forced-LW register transfer not implemented");
  }
  constexpr std::uint64_t program = std::uint64_t{1} << 41;
  if ((opcode & program) != 0) { detail::sharc_cache_program_access(state); }
  const auto ureg = static_cast<unsigned int>((opcode >> ureg_shift) & ureg_mask);
  const auto address = static_cast<std::uint32_t>(opcode);
  constexpr std::uint32_t iop_end = 0x40000;
  if (address >= iop_end) {
    detail::sharc_ram_transfer(memory,
                               state,
                               {.access = {.address = address},
                                .ureg = ureg,
                                .write = (opcode & (std::uint64_t{1} << class_shift)) != 0},
                               control);
    return;
  }
  const auto mode1 = state.effective[detail::sharc_mode1];
  if ((mode1.known & detail::sharc_simd) == 0 || (mode1.value & detail::sharc_simd) != 0) {
    throw std::runtime_error("SHARC SIMD IOP transfer not implemented");
  }
  constexpr std::uint32_t monitor = 0x100000;
  const auto mode2 = state.effective[4];
  if ((mode2.known & monitor) == 0 || (mode2.value & monitor) != 0) {
    throw std::runtime_error("SHARC IOP access monitor/interrupt not implemented");
  }
  if ((opcode & (std::uint64_t{1} << class_shift)) != 0) {
    memory.write({.address = address}, detail::sharc_value(detail::sharc_ureg(state, ureg)));
  } else {
    const auto value = static_cast<std::uint32_t>(memory.read({.address = address}));
    detail::sharc_ureg(state, ureg, {.value = value, .known = all_bits});
  }
} catch (const std::exception &error) {

  std::ostringstream message;
  const auto *operation = (opcode & (std::uint64_t{1} << class_shift)) != 0 ? "write" : "read";
  message << "SHARC direct " << operation << " word-address=0x" << std::hex
          << static_cast<std::uint32_t>(opcode) << ": " << error.what();
  throw std::runtime_error(message.str());
}

std::optional<std::uint32_t> execute(SharcMemory &memory,
                                     SharcState &state,
                                     std::uint64_t opcode,
                                     const SharcControl *control,
                                     std::optional<bool> branch_taken = std::nullopt,
                                     const SharcDagAddress *dag = nullptr,
                                     bool fetched = false) {
  if (dag != nullptr) {
    detail::sharc_execute_dag(memory, state, opcode, control, *dag);
    return {};
  }
  constexpr std::uint64_t dual_mask = 0xe00000000000;
  constexpr std::uint64_t dual_class = 0x200000000000;
  if ((opcode & dual_mask) == dual_class) {
    detail::sharc_dual_move(memory, state, opcode, control);
    return {};
  }
  constexpr std::uint64_t register_indexed_mask = 0xe00000000000;
  constexpr std::uint64_t register_indexed = 0x400000000000;
  if ((opcode & register_indexed_mask) == register_indexed) {
    detail::sharc_indexed_move(memory, state, opcode, control);
    return {};
  }
  constexpr std::uint64_t register_class_mask = 0xf00000000000;
  constexpr std::uint64_t register_class = 0x700000000000;
  if ((opcode & register_class_mask) == register_class) {
    detail::sharc_register_move(state, opcode);
    return {};
  }
  constexpr std::uint64_t indexed_mask = 0xf00000000000;
  constexpr std::uint64_t indexed_move = 0x600000000000;
  if ((opcode & indexed_mask) == indexed_move) {
    detail::sharc_data_move(memory, state, opcode, control);
    return {};
  }
  const auto type = opcode >> class_shift;
  switch (type) {
    case 0: {
      constexpr std::uint64_t idle = 0x008000000000;
      if (opcode == idle) {
        detail::sharc_begin_idle(state, fetched);
      } else if (opcode != 0) {
        throw std::runtime_error("SHARC emuidle/VISA/reserved instruction not implemented");
      }
      return {};
    }
    case 1: detail::sharc_compute(state, opcode); return {};
    case 2: detail::sharc_shift(state, opcode, true); return {};
    case 6:
    case 7: return direct_branch(state, opcode, branch_taken);
    case 10: return return_branch(state, opcode, branch_taken);
    case 11: return detail::sharc_return_interrupt(state, opcode, branch_taken);
    case 12:
    case 13: detail::sharc_begin_count_loop(state, opcode); return {};
    case 14: detail::sharc_begin_arithmetic_loop(state, opcode); return {};
    case 15: {
      constexpr std::uint64_t visa_form = 0x8000000000;
      if ((opcode & visa_form) != 0) {
        throw std::runtime_error("SHARC VISA immediate form in native stream");
      }
      detail::sharc_ureg(state,
                         static_cast<unsigned int>((opcode >> ureg_shift) & ureg_mask),
                         {.value = static_cast<std::uint32_t>(opcode), .known = all_bits});
      return {};
    }
    case dm_read_class:
    case dm_write_class:
    case pm_read_class:
    case pm_write_class: direct_move(memory, state, opcode, control); return {};
    case system_bit_class: {
      constexpr std::uint64_t reserved_sreg_bit = 0x1000000000;
      if ((opcode & reserved_sreg_bit) != 0) {
        throw std::runtime_error("Reserved SHARC system-register field");
      }
      detail::sharc_bit_operation(state, opcode);
      return {};
    }
    case cache_stack_class: detail::sharc_cache_flush(state, opcode); return {};
    default: throw std::runtime_error("SHARC instruction class not implemented");
  }
}

unsigned int execute_loop(SharcMemory &memory,
                          SharcState &state,
                          std::uint64_t opcode,
                          const SharcControl *control,
                          bool clocked,
                          const SharcDagAddress *dag) {
  if (!detail::sharc_loop_body_supported(opcode)) {
    throw std::runtime_error("SHARC count-loop body instruction/pipeline not implemented");
  }
  static_cast<void>(execute(memory, state, opcode, control, std::nullopt, dag));
  return detail::sharc_count_loop_step(state, opcode, clocked);
}

void finish_control(const SharcState &previous, SharcState &next, bool clocked) {
  for (unsigned int index = 0; index < previous.pending.size(); ++index) {
    const auto effect = previous.pending[index];
    if (effect) { next.effective[index] = *effect; }
  }
  if (previous.branch) {
    if (!next.branch) { throw std::logic_error("Lost SHARC delayed branch"); }
    --next.branch->slots;
    if (next.branch->slots == 0) {
      next.pc = next.branch->target;
      next.branch.reset();
    }
  }
  if (detail::sharc_four_loop(previous)) {
    detail::sharc_four_loop_finish(previous, next, clocked);
  } else if (previous.count_loop && previous.count_loop->body != previous.count_loop->end) {
    detail::sharc_long_loop_finish(previous, next);
  }
  detail::sharc_stack_status(next);
}

void execution_guard(const SharcState &state) {
  if (state.idle_pc) {
    throw std::runtime_error("SHARC IDLE core requires an owner event, no instruction retires");
  }
  constexpr unsigned int almost_full = 29;
  if (state.depth && *state.depth >= almost_full) {
    throw std::runtime_error("SHARC stack-full interrupt state not implemented");
  }
  if ((!state.loops_empty || !*state.loops_empty) && !state.count_loop && !state.arithmetic_loop) {
    throw std::runtime_error("SHARC active/unknown loop sequencing not implemented");
  }
  detail::sharc_interrupt_guard(state);
  constexpr std::uint32_t visa_start = 0x100000;
  constexpr std::uint32_t visa_end = 0x200000;
  if (state.pc >= visa_start && state.pc < visa_end) {
    throw std::runtime_error("SHARC VISA execution not implemented");
  }
}

}

Sharc::Sharc(SharcMemory &memory, SharcState initial, SharcControl *control)
    : memory_(memory), state_(initial), control_(control) {
  validate(state_);
  detail::sharc_validate_count_loop(state_);
  detail::sharc_validate_arithmetic_loop(state_);
  detail::sharc_validate_interrupt(state_);
  detail::sharc_validate_idle(state_);
  detail::sharc_stack_status(state_);
}

SharcState Sharc::reset_reference_214xx(std::uint32_t entry) {
  SharcState state;
  state.pc = entry;
  for (auto &reg : state.system) { reg = {.value = 0, .known = all_bits}; }
  constexpr std::uint32_t mode2_reset = 0x42000000;
  constexpr std::uint32_t stky_reset = 0x05400000;
  constexpr std::uint32_t core_flag_directions = 0xaa;
  state.system[3].value = detail::sharc_simd; // MMASK reset has PEYEN set.
  state.system[4].value = mode2_reset;
  // HWR24-28 confirms core FLAG0..3 reset as inputs; PGR A-5 defines odd bits
  // as direction. Other directions/latches remain unknown despite A-18's typo.
  state.system[detail::sharc_flags] = {.known = core_flag_directions};
  state.system[8].value = stky_reset;
  state.system[9].value = stky_reset;
  state.system[detail::sharc_imask].value = 3; // EMUI/RSTI are nonmaskable/read-only.
  state.effective = state.system;
  state.depth = 0;
  state.loops_empty = true;
  state.cache.valid.known = all_bits;
  constexpr std::uint32_t lru_bits = 0xffff;
  state.cache.lru.known = lru_bits;
  return state;
}

SharcStep Sharc::step() {
  return step_opcode({});
}

SharcStep Sharc::step_opcode(std::optional<std::uint64_t> captured,
                             bool preserve_cache,
                             std::optional<bool> branch_taken,
                             SharcLoopClock loop_clock,
                             const SharcDagAddress *dag) {
  auto opcode = captured;
  bool transaction{};
  bool control_transaction{};
  try {
    if (memory_.active()) {
      throw std::logic_error("SHARC core step during another memory transaction");
    }
    execution_guard(state_);
    if (state_.idle_delay_word && captured != state_.idle_delay_word) {
      throw std::runtime_error("SHARC retained IDLE delay word required at resumed retirement");
    }
    memory_.set_pc(state_.pc);
    memory_.begin_instruction();
    transaction = true;
    if (control_ != nullptr) {
      control_->begin_core_instruction();
      control_transaction = true;
    }
    if (!opcode) {
      opcode = memory_.read({.address = state_.pc, .width = SharcWidth::wide48}, AccessKind::fetch);
    }
    detail::sharc_cache_guard(state_, *opcode);
    auto next = state_;
    next.idle_delay_word.reset();
    next.pending.fill({});
    next.cache.pending_flush = false;
    if (loop_clock == SharcLoopClock::clocked_pop) { detail::sharc_clocked_loop_pop(next); }
    detail::sharc_arithmetic_loop_prepare(next, *opcode);
    unsigned int minimum_cycles = 1;
    if (state_.count_loop && state_.count_loop->body == state_.count_loop->end) {
      minimum_cycles = execute_loop(
          memory_, next, *opcode, control_, loop_clock != SharcLoopClock::functional, dag);
    } else {
      if (detail::sharc_four_loop(next)) {
        minimum_cycles = detail::sharc_four_loop_prepare(next, *opcode);
        // The bound transport already emits the actual four-loop startup bubble.
        if (loop_clock != SharcLoopClock::functional) { minimum_cycles = 1; }
      } else {
        detail::sharc_long_loop_prepare(next, *opcode);
      }
      const auto target =
          execute(memory_, next, *opcode, control_, branch_taken, dag, captured.has_value());
      next.pc = target.value_or((state_.pc + 1) & detail::sharc_pc_mask);
    }
    // A bound fetch clock already accounted for the actual PM cache request.
    // The instruction-only helper must not erase that staged fetch knowledge.
    if (preserve_cache) { next.cache = state_.cache; }
    finish_control(state_, next, loop_clock != SharcLoopClock::functional);
    minimum_cycles += detail::sharc_arithmetic_loop_finish(state_, next);
    detail::sharc_cache_finish(state_, next);
    detail::sharc_interrupt_finish(state_, next);
    memory_.commit_instruction();
    transaction = false;
    if (control_transaction) {
      control_->commit_core_instruction();
      control_transaction = false;
    }
    const SharcStep result{.pc = state_.pc,
                           .opcode = *opcode,
                           .next_pc = next.pc,
                           .minimum_core_cycles = minimum_cycles};
    state_ = next;
    return result;
  } catch (const std::exception &error) {
    if (transaction) { memory_.abort_instruction(); }
    if (control_transaction) { control_->abort_core_instruction(); }
    std::ostringstream message;
    message << "SHARC native48 pc=0x" << std::hex << state_.pc << " opcode=";
    if (opcode) {
      message << "0x" << *opcode;
    } else {
      message << "unfetched";
    }
    message << ": " << error.what();
    throw std::runtime_error(message.str());
  }
}

}
