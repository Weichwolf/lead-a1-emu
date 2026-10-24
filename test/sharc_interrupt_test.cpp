// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t vector = base + 0x30;
constexpr std::uint32_t foreground = base + 96;
constexpr std::uint32_t parent = 0x10c0070;
constexpr std::uint32_t irq_frame = 0x20c0060;
constexpr std::uint32_t p1i = 0x1000;
constexpr std::uint32_t global = 0x1000;
constexpr std::uint32_t nesting = 0x800;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t rti = 0x0b3e00000000;
constexpr std::uint64_t rti_eq = 0x0b0000000000;
constexpr std::uint64_t pass_r3_r4 = 0x013e00021430;
constexpr std::uint64_t read_imaskp_r5 = 0x77fe02800000;
constexpr std::uint64_t increment_r0 = 0x013e00029000;
constexpr std::uint64_t negative = 0x8000000100;
constexpr std::uint64_t initial_counter = 123;
constexpr std::uint64_t resumed_counter = 124;
constexpr std::uint32_t branch_target = base + 128;
constexpr std::uint64_t delayed_jump = 0x063e04000000;
constexpr std::uint64_t delayed_call = 0x06be04000000;
constexpr std::uint64_t delayed_return = 0x0a3e04000000;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC interrupt expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) == std::string::npos) {
      throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
    }
    return;
  }
  throw std::runtime_error("Missing SHARC interrupt rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t pc, std::uint64_t instruction) {
  memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, instruction);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 1024;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned P1I ISR/foreground"});
  code(memory, vector, pass_r3_r4);
  code(memory, vector + 1, read_imaskp_r5);
  code(memory, vector + 2, rti);
  code(memory, foreground, increment_r0);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(foreground);
  state.system[2].value |= global;
  state.system[11].value = p1i | 3;
  state.effective = state.system;
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.data[0][0] = {.value = initial_counter * 256, .known = full40};
  state.data[0][3] = {.value = negative, .known = full40};
  return state;
}

lead_a1::SharcInterruptStep accept(lead_a1::Sharc &cpu) {
  cpu.latch_interrupt(p1i);
  const auto event = cpu.service_interrupt({.vector_base = base});
  if (!event) { throw std::runtime_error("Missing owned P1I recognition"); }
  return *event;
}

void frame_and_real_handler() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  const auto before = cpu.state();
  const auto event = accept(cpu);
  require(event ==
          (lead_a1::SharcInterruptStep{
              .source = p1i, .return_pc = foreground, .vector = vector, .minimum_core_cycles = 4}));
  require(cpu.state().pc == vector && cpu.state().depth == 2 && cpu.state().interrupt);
  require(cpu.state().pc_stack[0] == parent && cpu.state().pc_stack[1] == irq_frame);
  require(cpu.state().system[12] == (lead_a1::SharcKnown32{.value = p1i, .known = full32}));
  require(cpu.state().effective[12] == cpu.state().system[12]);
  require(cpu.state().data == before.data && cpu.state().dag == before.dag);
  require(cpu.state().system[2] == before.system[2] && cpu.state().system[11] == before.system[11]);
  require((cpu.state().system[10].value & p1i) == 0);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][4].value == negative);
  lead_a1::Sharc restored(memory, cpu.state());
  for (unsigned int instruction = 0; instruction < 2; ++instruction) {
    static_cast<void>(cpu.step());
    static_cast<void>(restored.step());
    require(cpu.state() == restored.state());
  }
  require(cpu.state().data[0][5].value == std::uint64_t{p1i} * 256);
  require(cpu.state().pc == foreground && cpu.state().depth == 1 && !cpu.state().interrupt);
  require(cpu.state().system[12].value == 0 && cpu.state().pc_stack[1] == irq_frame);
  constexpr std::uint32_t alu_mask = 0x43f;
  require((cpu.state().system[6].value & alu_mask) == 4); // No SPI status-stack pop.
  require(cpu.state().data[0][0] == before.data[0][0]); // Recognition retired no guest instruction.
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][0].value == resumed_counter * 256);
}

void masks_and_delayed_controls() {
  for (const bool enabled : {false, true}) {
    for (const bool unmasked : {false, true}) {
      lead_a1::SharcMemory memory;
      program(memory);
      auto state = initial();
      state.system[2].value = enabled ? global : 0;
      state.system[11].value = unmasked ? p1i : 0;
      state.effective = state.system;
      lead_a1::Sharc cpu(memory, state);
      cpu.latch_interrupt(p1i);
      const auto before = cpu.state();
      require(cpu.service_interrupt({.vector_base = base}).has_value() == (enabled && unmasked));
      if (!enabled || !unmasked) { require(cpu.state() == before); }
    }
  }
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, vector, rti);
  auto state = initial();
  state.system[11] = {.value = 3, .known = full32};
  state.pending[11] = state.system[11]; // Effective old IMASK still accepts P1I.
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(accept(cpu));
  require(!cpu.state().pending[11] && cpu.state().effective[11].value == 3);
  static_cast<void>(cpu.step());
  require(cpu.state().effective[11].value == 3 && !cpu.state().pending[11]);
}

void pending_mode_clocks_before_handler() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t select_secondary = 0x0f7200001400;
  code(memory, foreground, select_secondary);
  auto state = initial();
  constexpr std::uint64_t secondary_value = 0x1234567800;
  state.data[1][3] = {.value = secondary_value, .known = full40};
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step()); // Real guest MODE1 write immediately before IRQ recognition.
  require(cpu.state().pending[2].has_value() && cpu.state().effective[2].value == global);
  static_cast<void>(accept(cpu));
  require(!cpu.state().pending[2]); // PGR A-32: branch processor clocks satisfy the effect latency.
  constexpr std::uint32_t secondary_mode = 0x1400;
  require(cpu.state().effective[2].value == secondary_mode);
  static_cast<void>(cpu.step());
  require(cpu.state().data[1][4].value == secondary_value);
  require(cpu.state().data[0][4] == state.data[0][4]);
}

void pending_sysctl_and_cache_clocks() {
  lead_a1::SharcControl control(true);
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint32_t sysctl = 0x30024;
  constexpr std::uint32_t data = base + 160;
  constexpr std::uint32_t extended_bank2 = 0x804;
  constexpr std::uint64_t write_sysctl_r1 = 0x110100030024;
  constexpr std::uint64_t read_pm_r6 = 0x120600000000 | data;
  constexpr std::uint64_t physical = 0x9182736455ee;
  constexpr std::uint64_t expected = 0x9182736455;
  memory.map_io({.first_word = sysctl, .words = 1}, control);
  code(memory, foreground, write_sysctl_r1);
  code(memory, vector, read_pm_r6);
  memory.write({.address = data, .width = lead_a1::SharcWidth::wide48}, physical);
  auto state = initial();
  state.data[0][1] = {.value = std::uint64_t{extended_bank2} * 256, .known = full40};
  lead_a1::Sharc cpu(memory, state, &control);
  static_cast<void>(cpu.step());
  require(control.state().effective == 4 && control.state().delayed == extended_bank2);
  static_cast<void>(accept(cpu));
  require(control.state().effective == extended_bank2 && !control.state().delayed);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][6].value == expected);

  constexpr std::uint64_t flush = 0x170200000000;
  code(memory, foreground, flush);
  state = initial();
  state.cache.valid = {.value = full32, .known = full32};
  lead_a1::Sharc flushed(memory, state);
  static_cast<void>(flushed.step());
  require(flushed.state().cache.pending_flush && flushed.state().cache.valid.value == full32);
  static_cast<void>(accept(flushed));
  require(!flushed.state().cache.pending_flush &&
          flushed.state().cache.valid == (lead_a1::SharcKnown32{.known = full32}));
}

void failed_clock_owner_preserves_cpu() {
  lead_a1::SharcControl control(true);
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial(), &control);
  cpu.latch_interrupt(p1i);
  const auto before = cpu.state();
  control.begin_instruction();
  const auto owned = control.state();
  rejected([&] { static_cast<void>(cpu.service_interrupt({.vector_base = base})); },
           "clock advance during a transaction");
  require(cpu.state() == before && control.state() == owned);
  control.abort_instruction();
  require(cpu.service_interrupt({.vector_base = base}).has_value());
}

void pending_nesting_is_rejected_atomically() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t enable_nesting = 0x0f7200001800;
  code(memory, foreground, enable_nesting);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  require(cpu.state().pending[2].has_value() && cpu.state().effective[2].value == global);
  cpu.latch_interrupt(p1i);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.service_interrupt({.vector_base = base})); },
           "nested interrupt recognition not qualified");
  require(cpu.state() == before && !memory.active());
}

void atomic_delayed_branches() {
  for (const auto branch :
       {delayed_jump | branch_target, delayed_call | branch_target, delayed_return}) {
    for (const unsigned int arrival : {0U, 1U}) {
      lead_a1::SharcMemory memory;
      program(memory);
      code(memory, foreground, branch);
      code(memory, foreground + 1, increment_r0);
      code(memory, foreground + 2, pass_r3_r4);
      auto state = initial();
      constexpr std::uint32_t call_frame = 0x1000000;
      state.pc_stack[0] = call_frame | branch_target;
      lead_a1::Sharc cpu(memory, state);
      static_cast<void>(cpu.step());
      if (arrival == 1) { static_cast<void>(cpu.step()); }
      cpu.latch_interrupt(p1i);
      for (unsigned int slot = arrival; slot < 2; ++slot) {
        const auto branch_state = cpu.state().branch;
        require(branch_state && branch_state->slots == 2 - slot);
        const auto before = cpu.state();
        require(!cpu.service_interrupt({.vector_base = base}) && cpu.state() == before);
        static_cast<void>(cpu.step());
      }
      require(!cpu.state().branch && cpu.state().pc == branch_target);
      require(cpu.state().data[0][4].value == negative);
      require(cpu.state().data[0][0].value == resumed_counter * 256);
      require((cpu.state().effective[10].value & p1i) != 0 && !cpu.state().interrupt);
      const auto completed = cpu.state();
      const auto event = cpu.service_interrupt({.vector_base = base});
      require(event && event->return_pc == branch_target);
      const auto service = cpu.state().interrupt;
      require(service && service->saved_depth == completed.depth);
      for (unsigned int instruction = 0; instruction < 3; ++instruction) {
        static_cast<void>(cpu.step());
      }
      require(!cpu.state().interrupt && cpu.state().pc == branch_target);
      require(cpu.state().depth == completed.depth);
      require(cpu.state().data[0][0].value == resumed_counter * 256);
    }
  }
}

void delay_slot_mask_write_and_checkpoint() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, foreground, delayed_jump | branch_target);
  code(memory, foreground + 1, 0);
  constexpr std::uint64_t clear_imask = 0x0f7b00000000;
  code(memory, foreground + 2, clear_imask);
  code(memory, branch_target, increment_r0);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  cpu.latch_interrupt(p1i);
  require(!cpu.service_interrupt({.vector_base = base}));
  static_cast<void>(cpu.step());
  lead_a1::Sharc restored(memory, cpu.state());
  require(!restored.service_interrupt({.vector_base = base}));
  static_cast<void>(cpu.step());
  static_cast<void>(restored.step());
  require(cpu.state() == restored.state() && !cpu.state().branch);
  require(cpu.state().system[11].value == 3 && cpu.state().effective[11].value == (p1i | 3));
  require(cpu.state().pending[11].has_value());
  const auto event = cpu.service_interrupt({.vector_base = base});
  require(event && event->return_pc == branch_target);
  require(cpu.state().effective[11].value == 3 && !cpu.state().pending[11]);
  for (unsigned int instruction = 0; instruction < 3; ++instruction) {
    static_cast<void>(cpu.step());
  }
  require(cpu.state().pc == branch_target && !cpu.state().interrupt);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][0].value == resumed_counter * 256);
}

void failed_delay_slot_keeps_pending_request() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, foreground, delayed_jump | branch_target);
  constexpr std::uint64_t idle = 0x008000000000;
  code(memory, foreground + 1, idle);
  code(memory, foreground + 2, pass_r3_r4);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  cpu.latch_interrupt(p1i);
  const auto before = cpu.state();
  require(!cpu.service_interrupt({.vector_base = base}) && cpu.state() == before);
  rejected([&] { static_cast<void>(cpu.step()); }, "delayed-branch/loop IDLE not qualified");
  require(cpu.state() == before && !memory.active());
  code(memory, foreground + 1, increment_r0); // Repair the owned fixture and retry both real slots.
  for (unsigned int slot = 0; slot < 2; ++slot) {
    require(!cpu.service_interrupt({.vector_base = base}));
    static_cast<void>(cpu.step());
  }
  require(cpu.state().pc == branch_target && !cpu.state().idle_pc);
  const auto event = cpu.service_interrupt({.vector_base = base});
  require(event && event->return_pc == branch_target && cpu.state().interrupt);
}

void in_service_latch_clearing() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t set_p1i = 0x140a00001000;
  code(memory, vector, set_p1i);
  code(memory, vector + 1, rti);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(accept(cpu));
  constexpr std::uint32_t other = 4;
  cpu.latch_interrupt(p1i | other);
  require(cpu.state().system[10].value == other && cpu.state().effective[10].value == other);
  const auto before = cpu.state();
  require(!cpu.service_interrupt({.vector_base = base}) && cpu.state() == before);
  static_cast<void>(cpu.step());
  require(cpu.state().system[10].value == other && cpu.state().effective[10].value == other);
  const auto pending = cpu.state().pending[10];
  require(pending && pending->value == other);
  static_cast<void>(cpu.step());
  require(!cpu.state().interrupt && cpu.state().system[10].value == other);
  cpu.latch_interrupt(p1i); // The same source can latch again after RTI.
  require(cpu.state().system[10].value == (p1i | other));
}

enum class InvalidBoundary : std::uint8_t {
  source,
  unknown_mask,
  nesting,
  depth,
  loops,
  loop_branch,
  latch_write
};

void rejected_boundaries() {
  for (const auto invalid : {InvalidBoundary::source,
                             InvalidBoundary::unknown_mask,
                             InvalidBoundary::nesting,
                             InvalidBoundary::depth,
                             InvalidBoundary::loops,
                             InvalidBoundary::loop_branch,
                             InvalidBoundary::latch_write}) {
    lead_a1::SharcMemory memory;
    program(memory);
    auto state = initial();
    state.system[10].value = p1i;
    switch (invalid) {
      case InvalidBoundary::source:
        state.system[11].value |= 4;
        state.system[10].value |= 4;
        break;
      case InvalidBoundary::unknown_mask:
        state.system[11].known &= ~p1i;
        state.system[11].value &= ~p1i;
        break;
      case InvalidBoundary::nesting: state.system[2].value |= nesting; break;
      case InvalidBoundary::depth: state.depth.reset(); break;
      case InvalidBoundary::loops: state.loops_empty.reset(); break;
      case InvalidBoundary::loop_branch:
        state.branch = lead_a1::SharcBranch{.target = base, .slots = 2};
        state.loops_empty.reset();
        break;
      case InvalidBoundary::latch_write: state.pending[10] = state.system[10]; break;
      default: throw std::logic_error("Invalid owned boundary case");
    }
    state.effective = state.system;
    lead_a1::Sharc cpu(memory, state);
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(cpu.service_interrupt({.vector_base = base})); },
             "interrupt recognition pc=");
    require(cpu.state() == before && !memory.active());
  }
}

void bad_vector_and_failed_handler() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  cpu.latch_interrupt(p1i);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.service_interrupt({.vector_base = base + 1})); },
           "explicit internal native IVT/PC");
  require(cpu.state() == before);
  constexpr std::uint64_t unmapped_iop = 0x100000002004;
  code(memory, vector, unmapped_iop);
  require(cpu.service_interrupt({.vector_base = base}).has_value());
  const auto entered = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Unmapped SHARC IOP");
  require(cpu.state() == entered && !memory.active());
  code(memory, vector, rti);
  static_cast<void>(cpu.step());
  require(cpu.state().pc == foreground && !cpu.state().interrupt);
}

void rti_predicate_and_stack_guards() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, vector, rti_eq);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(accept(cpu));
  static_cast<void>(cpu.step()); // False condition leaves the frame/controller active.
  require(cpu.state().pc == vector + 1 && cpu.state().interrupt && cpu.state().depth == 2);
  constexpr std::uint64_t rts = 0x0a3e00000000;
  code(memory, vector + 1, rts);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "loop/interrupt return frame");
  require(cpu.state() == before);
  constexpr std::uint64_t delayed = 0x4000000;
  code(memory, vector + 1, rti | delayed);
  rejected([&] { static_cast<void>(cpu.step()); }, "delayed/parallel/reserved RTI");
  require(cpu.state() == before);
  auto state = before;
  state.pc_stack[1] = foreground; // Missing hardware bit25.
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "P1I service/frame state");
  state = before;
  state.system[12].value = 0;
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "P1I service/frame state");
  state = before;
  state.pending[10] = lead_a1::SharcKnown32{.value = p1i, .known = full32};
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "P1I pending service latch");
}

void spi_preserves_simd_and_ands_return_condition() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, vector, rti_eq);
  code(memory, vector + 1, rti_eq);
  auto state = initial();
  constexpr std::uint32_t simd = 0x200000;
  state.system[2].value |= simd;
  state.system[6].value = 1;
  state.system[7].value = 0;
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(accept(cpu));
  require(cpu.state().system[2] == state.system[2]); // No MMASK operation for P1I.
  static_cast<void>(cpu.step());
  require(cpu.state().interrupt && cpu.state().pc == vector + 1);
  auto both = cpu.state();
  both.system[7].value = 1;
  both.effective[7] = both.system[7];
  lead_a1::Sharc restored(memory, both);
  static_cast<void>(restored.step());
  require(!restored.state().interrupt && restored.state().pc == foreground);
  require(restored.state().system[2] == state.system[2] && restored.state().system[6].value == 1 &&
          restored.state().system[7].value == 1);
}

}

int main() {
  try {
    frame_and_real_handler();
    pending_mode_clocks_before_handler();
    pending_sysctl_and_cache_clocks();
    failed_clock_owner_preserves_cpu();
    pending_nesting_is_rejected_atomically();
    atomic_delayed_branches();
    delay_slot_mask_write_and_checkpoint();
    failed_delay_slot_keeps_pending_request();
    masks_and_delayed_controls();
    in_service_latch_clearing();
    rejected_boundaries();
    bad_vector_and_failed_handler();
    rti_predicate_and_stack_guards();
    spi_preserves_simd_and_ands_return_condition();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
