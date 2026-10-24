// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t all_bits = 0xffffffff;
constexpr std::uint32_t p1i = 0x1000;
constexpr std::uint32_t global_enable = 0x1000;
constexpr std::uint32_t nesting = 0x800;
constexpr std::uint32_t interrupt_bits = 0xffffff7c;
constexpr std::uint32_t native_begin = 0x80000;
constexpr std::uint32_t native_end = 0x100000;
constexpr std::uint32_t vector_offset = 0x30;
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

void clear_source(SharcState &state, std::uint32_t mask) {
  for (auto *latch : {&state.system[sharc_irptl], &state.effective[sharc_irptl]}) {
    latch->value &= ~mask;
    latch->known |= mask;
  }
  auto &pending = state.pending[sharc_irptl];
  if (pending) {
    pending->value &= ~mask;
    pending->known |= mask;
  }
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
    state.cache.valid = {.known = all_bits};
    state.cache.pending_flush = false;
  }
  nonnesting(state); // A pending MODE1 update must not sneak in unqualified nesting.
}

unsigned int recognition_boundary(const SharcState &state, SharcInterruptReference reference) {
  constexpr std::uint32_t vector_alignment = 255;
  if (!internal(reference.vector_base) || (reference.vector_base & vector_alignment) != 0 ||
      !internal(state.pc)) {
    throw std::runtime_error("SHARC interrupt requires an explicit internal native IVT/PC");
  }
  if (state.branch || state.count_loop || state.arithmetic_loop || state.loops_empty != true) {
    throw std::runtime_error("SHARC delayed-branch/loop interrupt entry not qualified");
  }
  if (!state.depth || *state.depth >= before_almost_full) {
    throw std::runtime_error("SHARC unknown/full interrupt PC stack not qualified");
  }
  if (sharc_value(state.system[imaskp]) != 0 || state.system[imaskp] != state.effective[imaskp]) {
    throw std::runtime_error("SHARC existing/unknown interrupt mask pointer not qualified");
  }
  if (state.pending[sharc_irptl]) {
    throw std::runtime_error("SHARC pending IRPTL write at recognition not qualified");
  }
  return *state.depth;
}

}

void sharc_validate_interrupt(const SharcState &state) {
  if (!state.interrupt) { return; }
  const auto frame = *state.interrupt;
  constexpr std::uint32_t vector_low = 255;
  if (frame.source != p1i || !internal(frame.vector) ||
      (frame.vector & vector_low) != vector_offset || !internal(frame.return_pc) ||
      frame.saved_depth >= before_almost_full || !state.depth ||
      *state.depth <= frame.saved_depth ||
      state.pc_stack[frame.saved_depth] != (sharc_special_frame | frame.return_pc) ||
      state.system[imaskp] != (SharcKnown32{.value = p1i, .known = all_bits}) ||
      state.effective[imaskp] != state.system[imaskp] ||
      (state.system[sharc_irptl].known & p1i) == 0 ||
      (state.system[sharc_irptl].value & p1i) != 0 ||
      (state.effective[sharc_irptl].known & p1i) == 0 ||
      (state.effective[sharc_irptl].value & p1i) != 0) {
    throw std::invalid_argument("Invalid SHARC P1I service/frame state");
  }
  const auto pending = state.pending[sharc_irptl];
  if (pending && ((pending->known & p1i) == 0 || (pending->value & p1i) != 0)) {
    throw std::invalid_argument("Invalid SHARC P1I pending service latch");
  }
}

void sharc_interrupt_service_guard(const SharcState &state) {
  if (!state.interrupt) { throw std::logic_error("Missing SHARC interrupt service context"); }
  nonnesting(state);
}

std::optional<SharcInterruptStep> sharc_service_interrupt(SharcState &state,
                                                          SharcInterruptReference reference) {
  sharc_interrupt_environment_guard(state);
  if (state.interrupt) {
    nonnesting(state);
    return {}; // NESTM0 masks all sources; other requests remain latched.
  }
  const auto latch = state.effective[sharc_irptl];
  if (latch.known == all_bits && latch.value == 0) { return {}; }
  if (!selected(state.effective[sharc_mode1], global_enable)) { return {}; }
  const auto mask = state.effective[sharc_imask];
  const auto possible = (latch.value | ~latch.known) & (mask.value | ~mask.known) & interrupt_bits;
  if (possible == 0) { return {}; }
  if (possible != p1i || !selected(latch, p1i) || !selected(mask, p1i)) {
    throw std::runtime_error("SHARC unknown/other interrupt source not qualified");
  }
  nonnesting(state);
  const auto saved_depth = recognition_boundary(state, reference);
  const auto vector = reference.vector_base + vector_offset;
  const auto return_pc = state.pc;
  // PGR A-10: interrupt push sets bit25, with CALL bit24 clear.
  state.pc_stack[saved_depth] = sharc_special_frame | return_pc;
  state.depth = saved_depth + 1;
  state.interrupt = SharcInterruptFrame{
      .source = p1i, .vector = vector, .return_pc = return_pc, .saved_depth = saved_depth};
  state.system[imaskp] = {.value = p1i, .known = all_bits};
  state.effective[imaskp] = state.system[imaskp];
  clear_source(state, p1i);
  state.pc = vector;
  state.idle_pc.reset();
  settle_entry_effects(state);
  sharc_stack_status(state);
  // PGR4-30/31 branch phase, after explicit latching/recognition boundary.
  return SharcInterruptStep{
      .source = p1i, .return_pc = return_pc, .vector = vector, .minimum_core_cycles = 4};
}

std::optional<std::uint32_t> sharc_return_interrupt(SharcState &state, std::uint64_t opcode) {
  constexpr std::uint64_t allowed = 0x0b3e00000000;
  if ((opcode & ~allowed) != 0) {
    throw std::runtime_error("SHARC delayed/parallel/reserved RTI not implemented");
  }
  const auto condition = static_cast<unsigned int>((opcode >> 33) & 31);
  if (!sharc_condition(state, condition)) { return {}; }
  if (!state.interrupt || state.branch || state.count_loop || state.arithmetic_loop) {
    throw std::runtime_error("SHARC RTI requires a qualified non-loop P1I frame");
  }
  const auto frame = *state.interrupt;
  if (state.depth != frame.saved_depth + 1 ||
      state.pc_stack[frame.saved_depth] != (sharc_special_frame | frame.return_pc)) {
    throw std::runtime_error("SHARC RTI top frame changed/unqualified");
  }
  state.depth = frame.saved_depth;
  state.system[imaskp] = {.known = all_bits};
  state.effective[imaskp] = state.system[imaskp];
  clear_source(state, frame.source);
  state.interrupt.reset();
  // P1I does not push/pop ASTATx/y or MODE1 status (PGR4-28,9-45).
  return frame.return_pc;
}

void sharc_interrupt_finish(const SharcState &previous, SharcState &next) {
  // PGR4-34/36: keep the active source latch clear on every service cycle.
  if (previous.interrupt) { clear_source(next, previous.interrupt->source); }
}

}

namespace lead_a1 {

std::optional<SharcInterruptStep> Sharc::service_interrupt(SharcInterruptReference reference) try {
  if (memory_.active()) { throw std::logic_error("Active SHARC transaction at recognition"); }
  auto next = state_;
  const auto result = detail::sharc_service_interrupt(next, reference);
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
          << state_.effective[12].known << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
