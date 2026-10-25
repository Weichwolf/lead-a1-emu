// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/core/sharc_interrupt_source.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_control.hpp"

#include <bit>
#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t all_bits = 0xffffffff;
constexpr std::uint32_t nesting = 0x800;
constexpr std::uint32_t native_begin = 0x80000;
constexpr std::uint32_t native_end = 0x100000;
constexpr unsigned int imaskp = 12;
constexpr unsigned int before_almost_full = 28;

bool selected(SharcKnown32 value, std::uint32_t mask) {
  if ((value.known & mask) != mask) { throw std::runtime_error("Unknown SHARC interrupt control"); }
  return (value.value & mask) != 0;
}

bool internal(std::uint32_t address) {
  return address >= native_begin && address < native_end;
}

void nonnesting(const SharcState &state) {
  if (selected(state.effective[sharc_mode1], nesting)) {
    throw std::runtime_error("SHARC nested interrupt recognition not qualified");
  }
}

void clear_source(SharcState &state, const SharcInterruptFrame &frame) {
  sharc_latch_source(state, frame.bank, frame.source, false);
}

std::uint32_t pointer(SharcInterruptSource source) {
  constexpr unsigned int pointer_shift = 20;
  return source.bank == SharcInterruptBank::lirptl ? source.mask << pointer_shift : 0;
}

void secondary_pointer(SharcState &state, std::uint32_t value) {
  auto &word = state.system[sharc_lirptl];
  word.value = (word.value & ~sharc_lirptl_pointers) | value;
  word.known |= sharc_lirptl_pointers;
}

bool return_pointer_delay(const SharcState &state) {
  const auto raw = state.system[sharc_lirptl];
  const auto effective = state.effective[sharc_lirptl];
  const auto pending = state.pending[sharc_lirptl];
  const auto changed = (raw.value ^ effective.value) | (raw.known ^ effective.known);
  return pending && *pending == raw && (changed & ~sharc_lirptl_pointers) == 0 &&
         std::has_single_bit(effective.value & sharc_lirptl_pointers) &&
         (effective.known & sharc_lirptl_pointers) == sharc_lirptl_pointers;
}

void settle_entry_effects(SharcState &state) {
  // PGR A-32: effect latency counts processor clocks, including IRQ entry.
  // Four internal branch clocks exceed all supported pending one-cycle effects.
  for (unsigned int index = 0; index < state.pending.size(); ++index) {
    const auto pending = state.pending[index];
    if (pending) { state.effective[index] = *pending; }
  }
  state.pending.fill({});
  if (state.cache.pending_flush) {
    if (!state.cache.flush_effect_applied) { state.cache.valid = {.known = all_bits}; }
    state.cache.pending_flush = false;
    state.cache.flush_effect_applied = false;
  }
  nonnesting(state); // A pending MODE1 update must not sneak in unqualified nesting.
}

void loop_boundary(const SharcState &state, bool fetched) {
  if (state.count_loop || (!state.arithmetic_loop && state.loops_empty != true)) {
    throw std::runtime_error("SHARC delayed-branch/loop interrupt entry not qualified");
  }
  if (!state.arithmetic_loop) { return; }
  if (!fetched) { throw std::runtime_error("SHARC loop interrupt requires bound fetch"); }
  const auto loop = *state.arithmetic_loop;
  if (loop.phase != SharcArithmeticPhase::fetched || state.pc != loop.body) {
    throw std::runtime_error(
        "SHARC arithmetic interrupt outside fetched start boundary not qualified");
  }
  sharc_validate_arithmetic_loop(state);
}

unsigned int
recognition_boundary(const SharcState &state, SharcInterruptReference reference, bool fetched) {
  constexpr std::uint32_t vector_alignment = 255;
  if (!internal(reference.vector_base) || (reference.vector_base & vector_alignment) != 0 ||
      !internal(state.pc)) {
    throw std::runtime_error("SHARC interrupt requires an explicit internal native IVT/PC");
  }
  if (state.branch) {
    throw std::runtime_error("SHARC delayed-branch/loop interrupt entry not qualified");
  }
  loop_boundary(state, fetched);
  if (!state.depth || *state.depth >= before_almost_full) {
    throw std::runtime_error("SHARC unknown/full interrupt PC stack not qualified");
  }
  if (sharc_value(state.system[imaskp]) != 0 || state.system[imaskp] != state.effective[imaskp]) {
    throw std::runtime_error("SHARC existing/unknown interrupt mask pointer not qualified");
  }
  const bool pointer_read_pending = return_pointer_delay(state);
  for (const auto word :
       {state.system[sharc_lirptl],
        pointer_read_pending ? state.system[sharc_lirptl] : state.effective[sharc_lirptl]}) {
    if ((word.known & sharc_lirptl_pointers) != sharc_lirptl_pointers ||
        (word.value & sharc_lirptl_pointers) != 0) {
      throw std::runtime_error("SHARC existing/unknown LIRPTL mask pointer not qualified");
    }
  }
  if (state.pending[sharc_irptl] || (state.pending[sharc_lirptl] && !pointer_read_pending)) {
    throw std::runtime_error("SHARC pending IRPTL/LIRPTL write at recognition not qualified");
  }
  return *state.depth;
}

}

bool sharc_interrupt_deferred(const SharcState &state) {
  // PGR4-22/31: both slots complete before maskable interrupt processing.
  // Loop reentry and non-native pipeline boundaries remain unqualified.
  return state.branch && internal(state.pc) && internal(state.branch->target) &&
         ((!state.count_loop && !state.arithmetic_loop && state.loops_empty == true) ||
          sharc_arithmetic_entry_delay(state));
}

void sharc_validate_interrupt(const SharcState &state) {
  if (!state.interrupt) { return; }
  const auto frame = *state.interrupt;
  constexpr std::uint32_t vector_low = 255;
  const SharcInterruptSource source{
      .bank = frame.bank, .mask = frame.source, .offset = frame.vector & vector_low};
  const auto primary_pointer = frame.bank == SharcInterruptBank::irptl ? frame.source : 0;
  if (!sharc_programmable_source(source) || !internal(frame.vector) || !internal(frame.return_pc) ||
      frame.saved_depth >= before_almost_full || !state.depth ||
      *state.depth <= frame.saved_depth ||
      state.pc_stack[frame.saved_depth] != (sharc_special_frame | frame.return_pc) ||
      state.system[imaskp] != (SharcKnown32{.value = primary_pointer, .known = all_bits}) ||
      state.effective[imaskp] != state.system[imaskp]) {
    throw std::invalid_argument("Invalid SHARC programmable service/frame state");
  }
  const auto index = sharc_latch_register(frame.bank);
  for (const auto word : {state.system[index],
                          state.effective[index],
                          state.pending[index].value_or(state.system[index])}) {
    if ((word.known & frame.source) == 0 || (word.value & frame.source) != 0) {
      throw std::invalid_argument("Invalid SHARC programmable pending service latch");
    }
  }
  for (const auto word : {state.system[sharc_lirptl],
                          state.effective[sharc_lirptl],
                          state.pending[sharc_lirptl].value_or(state.system[sharc_lirptl])}) {
    if ((word.known & sharc_lirptl_pointers) != sharc_lirptl_pointers ||
        (word.value & sharc_lirptl_pointers) != pointer(source)) {
      throw std::invalid_argument("Invalid SHARC programmable service mask pointer");
    }
  }
}

void sharc_interrupt_service_guard(const SharcState &state) {
  if (!state.interrupt) { throw std::logic_error("Missing SHARC interrupt service context"); }
  nonnesting(state);
}

std::optional<SharcInterruptStep>
sharc_service_interrupt(SharcState &state, SharcInterruptReference reference, bool fetched) {
  sharc_interrupt_environment_guard(state);
  if (state.interrupt) {
    nonnesting(state);
    return {}; // NESTM0 masks all sources; other requests remain latched.
  }
  if (sharc_interrupt_deferred(state)) { return {}; }
  const auto source = sharc_pending_interrupt(state);
  if (!source) { return {}; }
  if (!sharc_programmable_source(*source)) {
    throw std::runtime_error("SHARC unknown/other interrupt source not qualified");
  }
  nonnesting(state);
  const auto saved_depth = recognition_boundary(state, reference, fetched);
  const auto vector = reference.vector_base + source->offset;
  const auto return_pc = state.pc;
  // PGR A-10: interrupt push sets bit25, with CALL bit24 clear.
  state.pc_stack[saved_depth] = sharc_special_frame | return_pc;
  state.depth = saved_depth + 1;
  state.interrupt = SharcInterruptFrame{.source = source->mask,
                                        .vector = vector,
                                        .return_pc = return_pc,
                                        .saved_depth = saved_depth,
                                        .bank = source->bank};
  if (state.arithmetic_loop) {
    state.arithmetic_loop->phase = SharcArithmeticPhase::fetched_suspended;
  }
  settle_entry_effects(state);
  state.system[imaskp] = {.value = source->bank == SharcInterruptBank::irptl ? source->mask : 0,
                          .known = all_bits};
  state.effective[imaskp] = state.system[imaskp];
  secondary_pointer(state, pointer(*source));
  state.effective[sharc_lirptl] = state.system[sharc_lirptl];
  clear_source(state, *state.interrupt);
  state.pc = vector;
  state.idle_pc.reset();
  sharc_stack_status(state);
  // PGR4-30/31 branch phase, after explicit latching/recognition boundary.
  return SharcInterruptStep{.source = source->mask,
                            .return_pc = return_pc,
                            .vector = vector,
                            .minimum_core_cycles = 4,
                            .bank = source->bank};
}

std::optional<std::uint32_t>
sharc_return_interrupt(SharcState &state, std::uint64_t opcode, std::optional<bool> captured) {
  constexpr std::uint64_t allowed = 0x0b3e00000000;
  if ((opcode & ~allowed) != 0) {
    throw std::runtime_error("SHARC delayed/parallel/reserved RTI not implemented");
  }
  const auto condition = static_cast<unsigned int>((opcode >> 33) & 31);
  if (!(captured ? *captured : sharc_condition(state, condition))) { return {}; }
  const bool suspended = state.arithmetic_loop &&
                         state.arithmetic_loop->phase == SharcArithmeticPhase::fetched_suspended;
  if (!state.interrupt || state.branch || state.count_loop ||
      (state.arithmetic_loop && !suspended)) {
    throw std::runtime_error(
        "SHARC RTI requires a qualified programmable frame or suspended arithmetic frame");
  }
  sharc_validate_arithmetic_loop(state);
  const auto frame = *state.interrupt;
  if (state.depth != frame.saved_depth + 1 ||
      state.pc_stack[frame.saved_depth] != (sharc_special_frame | frame.return_pc)) {
    throw std::runtime_error("SHARC RTI top frame changed/unqualified");
  }
  state.depth = frame.saved_depth;
  state.system[imaskp] = {.known = all_bits};
  state.effective[imaskp] = state.system[imaskp];
  clear_source(state, frame);
  if (frame.bank == SharcInterruptBank::lirptl) {
    secondary_pointer(state, 0);
    // A-33: controller mask-pointer readback has one extra clock of latency.
    state.pending[sharc_lirptl] = state.system[sharc_lirptl];
  }
  state.interrupt.reset();
  if (state.arithmetic_loop) { state.arithmetic_loop->phase = SharcArithmeticPhase::fetched; }
  // Programmable IRQs do not push/pop ASTATx/y or MODE1 (PGR4-28,9-45).
  return frame.return_pc;
}

void sharc_interrupt_finish(const SharcState &previous, SharcState &next) {
  // PGR4-34/36: keep the active source latch clear on every service cycle.
  if (previous.interrupt) { clear_source(next, *previous.interrupt); }
}

}

namespace lead_a1 {

std::optional<SharcInterruptStep> Sharc::service_interrupt(SharcInterruptReference reference) try {
  if (memory_.active()) { throw std::logic_error("Active SHARC transaction at recognition"); }
  auto next = state_;
  const auto result = detail::sharc_service_interrupt(next, reference);
  detail::sharc_validate_arithmetic_loop(next);
  detail::sharc_validate_interrupt(next);
  detail::sharc_validate_idle(next);
  if (result && control_ != nullptr) {
    control_->advance_internal_clocks(result->minimum_core_cycles);
  }
  state_ = next;
  return result;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC interrupt recognition pc=0x" << std::hex << state_.pc << " IVT=0x"
          << reference.vector_base << " IRPTL=0x" << state_.effective[10].value << "/0x"
          << state_.effective[10].known << " IMASK=0x" << state_.effective[11].value << "/0x"
          << state_.effective[11].known << " MODE1=0x" << state_.effective[2].value << "/0x"
          << state_.effective[2].known << " MODE2=0x" << state_.effective[4].value << "/0x"
          << state_.effective[4].known << " IMASKP=0x" << state_.effective[12].value << "/0x"
          << state_.effective[12].known << " LIRPTL=0x" << state_.effective[13].value << "/0x"
          << state_.effective[13].known << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
