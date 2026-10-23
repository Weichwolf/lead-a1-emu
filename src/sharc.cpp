// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_memory.hpp"
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
constexpr std::uint32_t all_bits = std::numeric_limits<std::uint32_t>::max();

void validate(const SharcState &state) {
  detail::sharc_validate_registers(state);
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

bool taken(const SharcState &state, std::uint64_t opcode) {
  const auto condition = static_cast<unsigned int>((opcode >> 33) & 31);
  return detail::sharc_condition(state, condition);
}

std::optional<std::uint32_t> direct_branch(SharcState &state, std::uint64_t opcode) {
  constexpr auto allowed = opcode_byte | condition_field | delayed | call_flag | loop_abort |
                           clear_interrupt | detail::sharc_pc_mask;
  if ((opcode & ~allowed) != 0) { throw std::runtime_error("Reserved SHARC direct-branch fields"); }
  if ((opcode & (loop_abort | clear_interrupt)) != 0) {
    throw std::runtime_error("SHARC loop-abort/clear-interrupt branch not implemented");
  }
  if (!taken(state, opcode)) { return {}; }
  if (state.branch) {
    throw std::runtime_error("SHARC taken branch in delay slot not implemented");
  }
  const bool db = (opcode & delayed) != 0;
  if ((opcode & call_flag) != 0) {
    if (!state.depth) { throw std::runtime_error("Unknown SHARC PC stack depth"); }
    constexpr unsigned int before_almost_full = 28;
    if (*state.depth >= before_almost_full) {
      throw std::runtime_error("SHARC stack-full interrupt boundary not implemented");
    }
    state.pc_stack[*state.depth] =
        detail::sharc_call_frame | ((state.pc + (db ? 3U : 1U)) & detail::sharc_pc_mask);
    ++*state.depth;
  }
  auto target = static_cast<std::uint32_t>(opcode) & detail::sharc_pc_mask;
  constexpr unsigned int relative = 7;
  if ((opcode >> class_shift) == relative) { target = (state.pc + target) & detail::sharc_pc_mask; }
  return schedule(state, target, db);
}

std::optional<std::uint32_t> return_branch(SharcState &state, std::uint64_t opcode) {
  constexpr auto allowed = opcode_byte | condition_field | delayed;
  if ((opcode & ~allowed) != 0) {
    throw std::runtime_error("SHARC RTS compute/loop-return/reserved fields not implemented");
  }
  if (!taken(state, opcode)) { return {}; }
  if (state.branch) {
    throw std::runtime_error("SHARC taken return in delay slot not implemented");
  }
  if (!state.depth) { throw std::runtime_error("Unknown SHARC PC stack depth"); }
  if (*state.depth == 0) { throw std::runtime_error("SHARC empty-stack RTS not qualified"); }
  const auto frame = state.pc_stack[*state.depth - 1];
  if (!frame) { throw std::runtime_error("Unknown SHARC return frame"); }
  if ((*frame & detail::sharc_special_frame) != 0 || (*frame & detail::sharc_call_frame) == 0) {
    throw std::runtime_error("SHARC loop/interrupt return frame not implemented");
  }
  --*state.depth;
  return schedule(state, *frame & detail::sharc_pc_mask, (opcode & delayed) != 0);
}

void direct_move(SharcMemory &memory,
                 SharcState &state,
                 std::uint64_t opcode,
                 const SharcControl *control) try {
  constexpr std::uint64_t long_word = 0x8000000000;
  if ((opcode & long_word) != 0) {
    throw std::runtime_error("SHARC forced-LW register transfer not implemented");
  }
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

std::optional<std::uint32_t>
execute(SharcMemory &memory, SharcState &state, std::uint64_t opcode, const SharcControl *control) {
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
    case 0:
      if (opcode != 0) {
        throw std::runtime_error("SHARC idle/VISA/reserved instruction not implemented");
      }
      return {};
    case 1: detail::sharc_compute(state, opcode); return {};
    case 6:
    case 7: return direct_branch(state, opcode);
    case 10: return return_branch(state, opcode);
    case 12:
    case 13: detail::sharc_begin_count_loop(state, opcode); return {};
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
    default: throw std::runtime_error("SHARC instruction class not implemented");
  }
}

unsigned int execute_loop(SharcMemory &memory,
                          SharcState &state,
                          std::uint64_t opcode,
                          const SharcControl *control) {
  if (opcode != 0) {
    if (!detail::sharc_loop_body_supported(opcode)) {
      throw std::runtime_error("SHARC count-loop body instruction/pipeline not implemented");
    }
    if ((opcode >> class_shift) == 1) {
      detail::sharc_compute(state, opcode);
    } else {
      detail::sharc_data_move(memory, state, opcode, control);
    }
  }
  return detail::sharc_count_loop_step(state, opcode);
}

void execution_guard(const SharcState &state) {
  constexpr unsigned int almost_full = 29;
  if (state.depth && *state.depth >= almost_full) {
    throw std::runtime_error("SHARC stack-full interrupt state not implemented");
  }
  if ((!state.loops_empty || !*state.loops_empty) && !state.count_loop) {
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
  return state;
}

SharcStep Sharc::step() {
  std::optional<std::uint64_t> opcode;
  bool transaction{};
  bool control_transaction{};
  try {
    if (memory_.active()) {
      throw std::logic_error("SHARC core step during another memory transaction");
    }
    execution_guard(state_);
    memory_.set_pc(state_.pc);
    memory_.begin_instruction();
    transaction = true;
    if (control_ != nullptr) {
      control_->begin_core_instruction();
      control_transaction = true;
    }
    opcode = memory_.read({.address = state_.pc, .width = SharcWidth::wide48}, AccessKind::fetch);
    auto next = state_;
    next.pending.fill({});
    unsigned int minimum_cycles = 1;
    if (state_.count_loop) {
      minimum_cycles = execute_loop(memory_, next, *opcode, control_);
    } else {
      const auto target = execute(memory_, next, *opcode, control_);
      next.pc = target.value_or((state_.pc + 1) & detail::sharc_pc_mask);
    }
    for (unsigned int index = 0; index < state_.pending.size(); ++index) {
      const auto effect = state_.pending[index];
      if (effect) { next.effective[index] = *effect; }
    }
    detail::sharc_stack_status(next);
    if (state_.branch) {
      --next.branch->slots;
      if (next.branch->slots == 0) {
        next.pc = next.branch->target;
        next.branch.reset();
      }
    }
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
