// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t subroutine = base + 32;
constexpr std::uint32_t nested = base + 48;
constexpr std::uint32_t parent = 0x10c0060;
constexpr std::uint64_t immediate_do = 0x0c0000000000;
constexpr std::uint64_t call = 0x06be00000000;
constexpr std::uint64_t rts = 0x0a3e00000000;
constexpr std::uint64_t read_stack_r0 = 0x767e00000000;
constexpr std::uint64_t read_stack_r1 = 0x767e00800000;
constexpr std::uint64_t read_depth_r2 = 0x767e81000000;
constexpr std::uint64_t read_stack_r4 = 0x767e02000000;
constexpr std::uint32_t loop_empty = 0x4000000;
constexpr std::uint64_t full40 = 0xffffffffff;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC long-loop expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC long-loop rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, unsigned int count, unsigned int length = 5) {
  constexpr unsigned int cells = 256;
  constexpr unsigned int count_shift = 24;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned native loop and subroutines"});
  code(memory, base, immediate_do | (std::uint64_t{count} << count_shift) | length);
  for (unsigned int offset = 1; offset <= length + 1; ++offset) { code(memory, base + offset, 0); }
  code(memory, base + 1, read_stack_r0);
  code(memory, base + length - 3, read_stack_r1);
  code(memory, base + length - 2, read_depth_r2);
  code(memory, base + length, read_stack_r4);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.depth = 1;
  state.pc_stack[0] = parent;
  return state;
}

std::uint32_t data(const lead_a1::Sharc &cpu, unsigned int reg) {
  require(cpu.state().data[0][reg].known == full40);
  return static_cast<std::uint32_t>(cpu.state().data[0][reg].value >> 8);
}

void straight_iteration(lead_a1::Sharc &cpu, unsigned int length, bool last) {
  for (unsigned int offset = 1; offset <= length; ++offset) {
    require(cpu.state().pc == base + offset);
    require(cpu.step().minimum_core_cycles == 1);
    const bool popped = last && offset >= length - 3;
    require(cpu.state().depth == (popped ? 1U : 2U));
    require(cpu.state().loops_empty == popped);
    require(((cpu.state().system[8].value & loop_empty) != 0) == popped);
    if (offset == 1) { require(data(cpu, 0) == base + 1); }
    if (offset == length - 3) { require(data(cpu, 1) == (popped ? parent : base + 1)); }
    if (offset == length - 2) { require(data(cpu, 2) == (popped ? 1U : 2U)); }
  }
  require(cpu.state().pc == (last ? base + length + 1 : base + 1));
  require(data(cpu, 4) == (last ? parent : base + 1));
}

void straight_case(unsigned int length, unsigned int count) {
  lead_a1::SharcMemory memory;
  program(memory, count, length);
  lead_a1::Sharc cpu(memory, initial());
  require(cpu.step().next_pc == base + 1);
  for (unsigned int iteration = 1; iteration <= count; ++iteration) {
    straight_iteration(cpu, length, iteration == count);
  }
  require(!cpu.state().count_loop && cpu.state().pc_stack[0] == parent);
  require(cpu.state().pc_stack[1] == base + 1);
}

void straight_loops() {
  for (const unsigned int length : {5U, 6U, 9U}) {
    for (const unsigned int count : {1U, 2U, 5U}) { straight_case(length, count); }
  }
}

const lead_a1::SharcCountLoop &loop_state(const lead_a1::SharcState &state) {
  if (!state.count_loop) { throw std::runtime_error("Missing expected SHARC count loop"); }
  return *state.count_loop;
}

void install_calls(lead_a1::SharcMemory &memory) {
  code(memory, base + 1, call | subroutine);                    // e-4: last safe ordinary CALL.
  constexpr std::array<std::uint64_t, 9> helper{0x767e02800000, // R5=PCSTK.
                                                0x767e83000000, // R6=PCSTKP.
                                                0x0f0700000003, // R7=3.
                                                0x013e0002a770, // DEC R7.
                                                0x062000000000 |
                                                    (subroutine + 3), // IF NE JUMP to DEC.
                                                call | nested,
                                                0x767e03800000, // R7=PCSTK after nested return.
                                                0,
                                                rts};
  for (unsigned int index = 0; index < helper.size(); ++index) {
    code(memory, subroutine + index, helper[index]);
  }
  code(memory, nested, 0);
  code(memory, nested + 1, rts);
}

void calls_and_polling() {
  for (const unsigned int count : {1U, 2U, 5U}) {
    lead_a1::SharcMemory memory;
    program(memory, count);
    install_calls(memory);
    lead_a1::Sharc cpu(memory, initial());
    static_cast<void>(cpu.step());
    for (unsigned int iteration = 1; iteration <= count; ++iteration) {
      require(cpu.step().next_pc == subroutine);
      const auto remaining = count - iteration + 1;
      require(cpu.state().count_loop->remaining == remaining && cpu.state().depth == 3);
      require(cpu.state().pc_stack[2] == (0x1000000 | (base + 2)));
      require(cpu.state().count_loop->frame_live && cpu.state().loops_empty == false);
      lead_a1::Sharc restored(memory, cpu.state());
      static_cast<void>(restored.step());
      static_cast<void>(cpu.step());
      require(cpu.state() == restored.state());
      require(data(cpu, 5) == (0x1000000 | (base + 2)));
      static_cast<void>(cpu.step());
      require(data(cpu, 6) == 3);
      constexpr unsigned int bound = 32;
      unsigned int steps{};
      while (cpu.state().pc != base + 2 && steps++ < bound) {
        static_cast<void>(cpu.step());
        require(cpu.state().count_loop->remaining == remaining);
        require(cpu.state().count_loop->frame_live && cpu.state().loops_empty == false);
      }
      require(cpu.state().pc == base + 2 && cpu.state().depth == 2);
      require(data(cpu, 7) == (0x1000000 | (base + 2)));
      static_cast<void>(cpu.step());
      require(data(cpu, 1) == (iteration == count ? parent : base + 1));
      require(cpu.state().depth == (iteration == count ? 1U : 2U));
      for (unsigned int index = 0; index < 3; ++index) { static_cast<void>(cpu.step()); }
    }
    require(!cpu.state().count_loop && cpu.state().pc == base + 6);
    require(cpu.state().pc_stack[0] == parent && cpu.state().depth == 1);
  }
}

void failed_pop_and_pending_effect() {
  lead_a1::SharcMemory memory;
  program(memory, 1);
  constexpr std::uint64_t mode_write = 0x0f7200000000;
  constexpr std::uint64_t unmapped_read = 0x107000001080;
  code(memory, base + 1, mode_write);
  code(memory, base + 2, unmapped_read);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  const auto before = cpu.state();
  require(before.pending[2] && loop_state(before).frame_live);
  rejected([&] { static_cast<void>(cpu.step()); }, "word-address=0x1080");
  require(cpu.state() == before && !memory.active());
  code(memory, base + 2, read_stack_r1);
  static_cast<void>(cpu.step());
  require(data(cpu, 1) == parent && !cpu.state().pending[2]);
  require(!loop_state(cpu.state()).frame_live && cpu.state().depth == 1);
}

void guarded_flow() {
  struct InvalidFlow {
    std::uint64_t opcode;
    const char *reason;
  };

  constexpr std::array<InvalidFlow, 6> invalid{
      {{.opcode = 0x063e00000000 | subroutine, .reason = "loop-body JUMP"},
       {.opcode = call | 0x4000000 | subroutine, .reason = "delayed control flow"},
       {.opcode = call | (base + 2), .reason = "escaped its execution context"},
       {.opcode = rts, .reason = "RTS from count-loop body"},
       {.opcode = 0x703e32000000, .reason = "PCSTK modification"},
       {.opcode = 0x0c0002000001, .reason = "nested/delayed count-loop setup"}}};
  for (const auto [opcode, reason] : invalid) {
    lead_a1::SharcMemory memory;
    program(memory, 2);
    code(memory, base + 1, opcode);
    auto state = initial();
    state.data[0][0] = {.known = full40};
    lead_a1::Sharc cpu(memory, state);
    static_cast<void>(cpu.step());
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(cpu.step()); }, reason);
    require(cpu.state() == before && !memory.active());
  }
  lead_a1::SharcMemory memory;
  program(memory, 2);
  code(memory, base + 2, call | subroutine);
  lead_a1::Sharc late(memory, initial());
  static_cast<void>(late.step());
  static_cast<void>(late.step());
  const auto before = late.state();
  rejected([&] { static_cast<void>(late.step()); }, "late CALL");
  require(late.state() == before);
}

void empty_parent_and_false_call() {
  lead_a1::SharcMemory memory;
  program(memory, 1);
  constexpr std::uint64_t if_eq_call = 0x068000000000;
  code(memory, base + 1, if_eq_call | subroutine);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(cpu.step());
  require(cpu.step().next_pc == base + 2); // Reset AZ=0; target stays unfetched.
  require(cpu.state().depth == 1 && loop_state(cpu.state()).remaining == 1);
  static_cast<void>(cpu.step());
  constexpr std::uint32_t empty_frame = 0x3ffffff;
  require(data(cpu, 1) == empty_frame && cpu.state().depth == 0);
  for (unsigned int index = 0; index < 3; ++index) { static_cast<void>(cpu.step()); }
  require(data(cpu, 4) == empty_frame && !cpu.state().count_loop);
  require(cpu.state().pc == base + 6 && cpu.state().loops_empty == true);
}

void status_operand_and_restored_tail() {
  lead_a1::SharcMemory memory;
  program(memory, 1);
  constexpr std::uint64_t read_stky_r1 = 0x77be00800000;
  code(memory, base + 2, read_stky_r1);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  for (unsigned int index = 0; index < 3; ++index) { static_cast<void>(cpu.step()); }
  constexpr std::uint32_t both_empty = 0x4400000;
  require((data(cpu, 1) & both_empty) == both_empty);
  require(cpu.state().pc == base + 3 && cpu.state().depth == 0);
  require(!loop_state(cpu.state()).frame_live);
  lead_a1::Sharc restored(memory, cpu.state());
  for (unsigned int index = 0; index < 3; ++index) { static_cast<void>(restored.step()); }
  require(!restored.state().count_loop && restored.state().pc == base + 6);
}

void inconsistent_restoration() {
  lead_a1::SharcMemory memory;
  program(memory, 1);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  auto state = cpu.state();
  state.pc = base + 3; // e-2 must already see popped stacks in the final pass.
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "execution context");
  if (!state.count_loop) { throw std::runtime_error("Missing inherited loop"); }
  state.count_loop->frame_live = false;
  state.loops_empty = true;
  state.pc = subroutine; // A callee cannot outlive a popped loop frame.
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "execution context");
}

void zero_count_and_inherited_state() {
  lead_a1::SharcMemory memory;
  program(memory, 0);
  constexpr std::uint64_t ustat_loop = 0x0d7000000005;
  code(memory, base, ustat_loop); // LCNTR=USTAT1, DO PC+5.
  auto state = initial();         // USTAT1=0 represents 2^32 iterations.
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  constexpr unsigned int prefix = 10;
  for (unsigned int index = 0; index < prefix; ++index) { static_cast<void>(cpu.step()); }
  constexpr std::uint64_t full_count = std::uint64_t{1} << 32;
  require(loop_state(cpu.state()).remaining == full_count - 2);
  state = cpu.state();
  lead_a1::Sharc restored(memory, state);
  require(restored.step().next_pc == base + 2);
  if (!state.count_loop) { throw std::runtime_error("Missing inherited loop"); }
  state.count_loop->end = base + 3;
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "count-loop state");
}

}

int main() {
  try {
    straight_loops();
    calls_and_polling();
    failed_pop_and_pending_effect();
    guarded_flow();
    empty_parent_and_false_call();
    status_operand_and_restored_tail();
    inconsistent_restoration();
    zero_count_and_inherited_state();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
