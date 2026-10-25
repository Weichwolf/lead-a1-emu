// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_control.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t parent = 0x10c0060;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t arithmetic_do = 0x0e0000000000;
constexpr std::uint64_t compute = 0x013e00000000;
constexpr std::uint64_t nop = 0;
constexpr std::uint64_t read_pcstk_r0 = 0x767e00000000;
constexpr std::uint64_t read_laddr_r1 = 0x767f00800000;
constexpr std::uint64_t read_depth_r2 = 0x767e81000000;
constexpr std::uint32_t empty_loops = 0x4000000;
constexpr unsigned int termination_shift = 33;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC arithmetic-loop expectation at line " +
                             std::to_string(where.line()));
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
  throw std::runtime_error("Missing SHARC arithmetic-loop rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, unsigned int length, unsigned int condition = 0) {
  constexpr unsigned int cells = 384;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned arithmetic loop"});
  code(memory, base, arithmetic_do | (std::uint64_t{condition} << termination_shift) | length);
  for (unsigned int index = 1; index < 32; ++index) { code(memory, base + index, nop); }
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.depth = 1;
  state.pc_stack[0] = parent;
  return state;
}

void data(lead_a1::SharcState &state, unsigned int reg, std::uint32_t value) {
  state.data[0][reg] = {.value = std::uint64_t{value} * 256, .known = full40};
}

std::uint32_t data(const lead_a1::Sharc &cpu, unsigned int reg) {
  require(cpu.state().data[0][reg].known == full40);
  return static_cast<std::uint32_t>(cpu.state().data[0][reg].value >> 8);
}

const lead_a1::SharcArithmeticLoop &loop(const lead_a1::Sharc &cpu) {
  const auto &state = cpu.state();
  if (!state.arithmetic_loop) { throw std::runtime_error("Missing arithmetic loop"); }
  return *state.arithmetic_loop;
}

lead_a1::SharcArithmeticLoop &loop(lead_a1::SharcState &state) {
  if (!state.arithmetic_loop) { throw std::runtime_error("Missing arithmetic loop state"); }
  return *state.arithmetic_loop;
}

void manufacturer_example() {
  lead_a1::SharcMemory memory;
  constexpr unsigned int length = 6;
  program(memory, length);
  constexpr std::array<std::uint64_t, 3> decrements{0x2a660, 0x2a770, 0x2a550};
  for (unsigned int index = 0; index < decrements.size(); ++index) {
    code(memory, base + index + 1, compute | decrements[index]);
  }
  auto state = initial();
  data(state, 6, 10);
  data(state, 7, 14);
  data(state, 5, 6);
  lead_a1::Sharc cpu(memory, state);
  unsigned int cycles = cpu.step().minimum_core_cycles;
  for (unsigned int pass = 0; pass < 6; ++pass) {
    for (unsigned int index = 1; index <= length; ++index) {
      require(cpu.state().pc == base + index);
      cycles += cpu.step().minimum_core_cycles;
      const bool popped = pass == 5 && index >= 4;
      require(cpu.state().depth == (popped ? 1U : 2U));
      require(cpu.state().loops_empty == popped);
      require(((cpu.state().system[8].value & empty_loops) != 0) == popped);
    }
  }
  require(data(cpu, 6) == 4 && data(cpu, 7) == 8 && data(cpu, 5) == 0);
  require(cpu.state().pc == base + 7 && !cpu.state().arithmetic_loop);
  constexpr unsigned int expected_cycles = 39; // DO + 6*6 actual instructions + 2 flush clocks.
  require(cycles == expected_cycles && cpu.state().pc_stack[0] == parent);
}

void sampled_flags_and_popped_operands() {
  lead_a1::SharcMemory memory;
  program(memory, 6);
  code(memory, base + 1, read_pcstk_r0);
  code(memory, base + 2, read_laddr_r1);
  constexpr std::uint64_t xor_r5 = 0x013e00042555;
  code(memory, base + 3, xor_r5); // e-3 produces AZ=1.
  // e-2: old AZ terminates, read popped PCSTK, then PASS R3 clears AZ.
  constexpr std::uint64_t parallel_parent = 0x767e01021430;
  code(memory, base + 4, parallel_parent);
  code(memory, base + 5, read_depth_r2);
  code(memory, base + 6, read_laddr_r1);
  auto state = initial();
  data(state, 3, 7);
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(data(cpu, 0) == base + 1);
  static_cast<void>(cpu.step());
  require(data(cpu, 1) == base + 6);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(data(cpu, 2) == parent && data(cpu, 4) == 7);
  require((cpu.state().system[6].value & 1) == 0);
  require(loop(cpu).phase == lead_a1::SharcArithmeticPhase::draining);
  lead_a1::Sharc restored(memory, cpu.state());
  static_cast<void>(restored.step());
  require(data(restored, 2) == 1);
  require(restored.step().minimum_core_cycles == 3);
  require(data(restored, 1) == full32 && !restored.state().arithmetic_loop);
}

void forever_and_simd_conditions() {
  constexpr std::uint32_t simd = 0x200000;
  for (const bool both : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory, 5);
    auto state = initial();
    state.system[2].value = simd;
    state.system[6].value = 1;
    state.system[7].value = both ? 1U : 0U;
    state.effective = state.system;
    lead_a1::Sharc cpu(memory, state);
    static_cast<void>(cpu.step());
    for (unsigned int index = 0; index < 5; ++index) { static_cast<void>(cpu.step()); }
    require(cpu.state().pc == (both ? base + 6 : base + 1));
    require(cpu.state().arithmetic_loop.has_value() != both);
  }
  lead_a1::SharcMemory memory;
  constexpr unsigned int forever = 31;
  program(memory, 5, forever);
  auto state = initial();
  state.system[6].value = 1;
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  constexpr unsigned int prefix = 20;
  for (unsigned int index = 0; index < prefix; ++index) { static_cast<void>(cpu.step()); }
  require(cpu.state().pc == base + 1 && cpu.state().depth == 2 && cpu.state().loops_empty == false);
  require(loop(cpu).termination == forever &&
          loop(cpu).kind == lead_a1::SharcArithmeticKind::multiple);
}

void failed_test_and_pop_rollback() {
  lead_a1::SharcMemory memory;
  program(memory, 5);
  auto state = initial();
  state.system[6] = {.known = full32 & ~1U};
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  for (unsigned int index = 0; index < 3; ++index) { static_cast<void>(cpu.step()); }
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Unknown SHARC control/status bits");
  require(cpu.state() == before && !memory.active());

  state = before;
  state.system[6] = {.value = 1, .known = full32};
  state.effective = state.system;
  constexpr std::uint32_t alternate_banks = 0x480;
  state.pending[2] = {.value = alternate_banks, .known = full32};
  // Tentative termination pops resources; subsequent unknown ADD must roll it all back.
  constexpr std::uint64_t unknown_add = 0x013e00001430;
  code(memory, base + 3, unknown_add);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc failing(memory, state, &control);
  const auto old_control = control.state();
  rejected([&] { static_cast<void>(failing.step()); }, "Unknown SHARC register operand bits");
  require(failing.state() == state && control.state() == old_control && !memory.active());
}

void deferred_stack_and_checkpoint(unsigned int distance) {
  lead_a1::SharcMemory memory;
  program(memory, 5);
  constexpr std::uint32_t setup = base + 20;
  constexpr std::uint32_t end = base + 4;
  const auto new_start = end - distance;
  constexpr std::uint32_t relative = 0xfffff0;
  code(memory, setup, arithmetic_do | relative);
  constexpr std::uint64_t pcstk_immediate = 0x0f6400000000;
  code(memory, setup + 1, pcstk_immediate | new_start);
  code(memory, setup + 2, read_laddr_r1);
  code(memory, setup + 3, read_pcstk_r0);
  auto state = initial();
  state.pc = setup;
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  require(loop(cpu).body == setup + 1 && loop(cpu).end == end);
  static_cast<void>(cpu.step());
  require(loop(cpu).body == new_start && loop(cpu).kind == lead_a1::SharcArithmeticKind::multiple);
  require(loop(cpu).phase == lead_a1::SharcArithmeticPhase::deferred && cpu.state().depth == 2);
  lead_a1::Sharc restored(memory, cpu.state());
  static_cast<void>(restored.step());
  require(data(restored, 1) == end); // Type0 retained even when the new start equals the end.
  static_cast<void>(restored.step());
  require(data(restored, 0) == new_start && restored.state().pc_stack[0] == parent);
  constexpr std::uint64_t jump = 0x063e00000000;
  code(memory, setup + 4, jump | new_start);
  const auto before = restored.state();
  rejected([&] { static_cast<void>(restored.step()); }, "deferred arithmetic-loop fetch/branch");
  require(restored.state() == before && !memory.active());
}

void setup_and_body_guards() {
  for (const unsigned int length : {1U, 2U, 3U, 4U}) {
    lead_a1::SharcMemory memory;
    program(memory, length);
    const auto state = initial();
    lead_a1::Sharc cpu(memory, state);
    const auto before = cpu.state(); // Constructor derives stack flags from the supplied depth.
    rejected([&] { static_cast<void>(cpu.step()); }, "short arithmetic DO");
    require(cpu.state() == before && !memory.active());
  }
  lead_a1::SharcMemory memory;
  program(memory, 5);
  constexpr std::uint64_t reserved = 0x1000000;
  code(memory, base, arithmetic_do | reserved | 5);
  lead_a1::Sharc bad(memory, initial());
  rejected([&] { static_cast<void>(bad.step()); }, "Reserved SHARC arithmetic DO");
  code(memory, base, arithmetic_do | 5);
  constexpr std::uint64_t store = 0x1100000c0040;
  code(memory, base + 1, store);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "arithmetic-loop body memory/control");
  require(cpu.state() == before && !memory.active());
}

void invalid_restoration() {
  lead_a1::SharcMemory memory;
  program(memory, 5);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  auto state = cpu.state();
  state.pc_stack[1] = base + 2;
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "arithmetic-loop PC frame");
  state = cpu.state();
  loop(state).phase = lead_a1::SharcArithmeticPhase::draining;
  state.depth = 1;
  state.loops_empty = true;
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "execution context");
  state = cpu.state();
  loop(state).kind = lead_a1::SharcArithmeticKind::single;
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "arithmetic-loop resources");
}

}

int main() {
  try {
    manufacturer_example();
    sampled_flags_and_popped_operands();
    forever_and_simd_conditions();
    failed_test_and_pop_rollback();
    deferred_stack_and_checkpoint(0);
    deferred_stack_and_checkpoint(1);
    setup_and_body_guards();
    invalid_restoration();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
