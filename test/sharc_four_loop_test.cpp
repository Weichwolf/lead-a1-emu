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
constexpr std::uint32_t helper = base + 32;
constexpr std::uint32_t inner = base + 48;
constexpr std::uint32_t data_base = 0x80000;
constexpr std::uint32_t parent = 0x10c0060;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t do4 = 0x0c0000000004;
constexpr std::uint64_t delayed_call = 0x06be04000000;
constexpr std::uint64_t rts = 0x0a3e00000000;
constexpr std::uint64_t increment_r1 = 0x013e00029110;
constexpr std::uint64_t r2_from_r1 = 0x013e00029210;
constexpr std::uint64_t read_stack_r10 = 0x767e05000000;
constexpr std::uint64_t read_depth_r11 = 0x767e85800000;
constexpr std::uint32_t resource_flags = 0x4600000;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC four-loop expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC four-loop rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, unsigned int count, bool calls) {
  constexpr unsigned int cells = 256;
  constexpr unsigned int count_shift = 24;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned four-loop code"});
  memory.map({.bank = 0, .halfwords = cells, .name = "owned four-loop data"});
  code(memory, base, do4 | (std::uint64_t{count} << count_shift));
  code(memory, base + 1, calls ? delayed_call | helper : increment_r1);
  code(memory, base + 2, calls ? increment_r1 : read_stack_r10);
  code(memory, base + 3, calls ? r2_from_r1 : read_depth_r11);
  constexpr std::uint64_t store_r8 = 0x51be84000000;
  code(memory, base + 4, calls ? store_r8 : 0);
  code(memory, base + 5, 0);
  constexpr std::array<std::uint64_t, 7> receive{
      0x013e00021820,                // R8=PASS R2, observing BOTH completed delay slots.
      0x0f0700000003,                // R7=3.
      0x013e0002a770,                // DEC R7.
      0x062000000000 | (helper + 2), // IF NE JUMP to DEC.
      0x06be00000000 | inner,
      0,
      rts};
  for (unsigned int index = 0; index < receive.size(); ++index) {
    code(memory, helper + index, receive[index]);
  }
  constexpr std::uint64_t increment_r4 = 0x013e00029440;
  code(memory, inner, increment_r4);
  code(memory, inner + 1, rts);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.data[0][1] = {.known = full40};
  state.data[0][2] = {.known = full40};
  state.data[0][4] = {.known = full40};
  state.dag[0][0][0] = {.value = data_base, .known = full32};
  state.dag[0][1][6] = {.value = 1, .known = full32};
  return state;
}

const lead_a1::SharcCountLoop &active(const lead_a1::Sharc &cpu) {
  const auto &state = cpu.state();
  if (!state.count_loop) { throw std::runtime_error("Expected four-loop state"); }
  return *state.count_loop;
}

void straight_case(unsigned int count) {
  lead_a1::SharcMemory memory;
  program(memory, count, false);
  lead_a1::Sharc cpu(memory, initial());
  unsigned int cycles = cpu.step().minimum_core_cycles;
  for (unsigned int iteration = 1; iteration <= count; ++iteration) {
    cycles += cpu.step().minimum_core_cycles;
    require(cpu.state().depth == (iteration == count ? 1U : 2U));
    require(cpu.state().loops_empty == (iteration == count));
    cycles += cpu.step().minimum_core_cycles;
    const auto expected = iteration == count ? parent : base + 1;
    require(cpu.state().data[0][10].value == std::uint64_t{expected} * 256);
    cycles += cpu.step().minimum_core_cycles;
    require(cpu.state().data[0][11].value == (iteration == count ? 256U : 512U));
    cycles += cpu.step().minimum_core_cycles;
    require(cpu.state().pc == (iteration == count ? base + 5 : base + 1));
  }
  require(cycles == 2 + (4 * count)); // PGR4-67 initial bubble, no drain.
  require(!cpu.state().count_loop && cpu.state().pc_stack[0] == parent);
}

void one_call(lead_a1::Sharc &cpu,
              lead_a1::SharcMemory &memory,
              unsigned int count,
              unsigned int iteration) {
  require(cpu.step().next_pc == base + 2);
  const auto branch = cpu.state().branch;
  require(branch && branch->slots == 2);
  if (count == 1) {
    require(!cpu.state().depth && !cpu.state().loops_empty && active(cpu).control);
    require(!cpu.state().pc_stack[1] && !cpu.state().pc_stack[2]);
    require((cpu.state().system[8].known & resource_flags) == 0);
  } else {
    const bool last = iteration == count;
    require(cpu.state().depth == (last ? 2U : 3U));
    require(cpu.state().loops_empty == last);
    const auto frame = (last ? 0x3000000U : 0x1000000U) | (base + 4);
    require(cpu.state().pc_stack[last ? 1U : 2U] == frame);
  }
  lead_a1::Sharc restored(memory, cpu.state());
  static_cast<void>(restored.step());
  require(cpu.step().next_pc == base + 3);
  require(cpu.state() == restored.state());
  require(cpu.state().data[0][1].value == std::uint64_t{iteration} * 256);
  require(cpu.step().next_pc == helper);
  require(cpu.state().data[0][2].value == std::uint64_t{iteration + 1} * 256);
  constexpr unsigned int limit = 32;
  unsigned int steps{};
  while (cpu.state().pc != base + 4 && steps++ < limit) {
    static_cast<void>(cpu.step());
    require(active(cpu).remaining == count - iteration + 1);
  }
  require(cpu.state().pc == base + 4 && !cpu.state().branch);
}

void delayed_calls(unsigned int count) {
  lead_a1::SharcMemory memory;
  program(memory, count, true);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  static_cast<void>(cpu.step());
  for (unsigned int iteration = 1; iteration <= count; ++iteration) {
    one_call(cpu, memory, count, iteration);
    static_cast<void>(cpu.step());
    require(memory.read({.address = data_base + iteration - 1}) == iteration + 1);
  }
  require(cpu.state().pc == base + 5 && !cpu.state().count_loop && cpu.state().depth == 1);
  require(cpu.state().pc_stack[0] == parent && cpu.state().loops_empty == true);
  require((cpu.state().system[8].known & resource_flags) == resource_flags);
  require(cpu.state().data[0][4].value == std::uint64_t{count} * 256);
}

void unknown_startup_read_and_repair() {
  lead_a1::SharcMemory memory;
  program(memory, 1, true);
  code(memory, base + 2, read_stack_r10);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Unknown SHARC PC stack depth");
  require(cpu.state() == before && !memory.active());
  code(memory, base + 2, increment_r1);
  constexpr unsigned int limit = 40;
  unsigned int steps{};
  while (cpu.state().pc != base + 5 && steps++ < limit) { static_cast<void>(cpu.step()); }
  require(cpu.state().pc == base + 5 && memory.read({.address = data_base}) == 2);
}

void delay_slot_failure() {
  lead_a1::SharcMemory memory;
  program(memory, 2, true);
  constexpr std::uint64_t bad_read = 0x107000001080;
  code(memory, base + 2, bad_read);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "word-address=0x1080");
  require(cpu.state() == before && !memory.active());
}

void final_frame_operand_reads() {
  lead_a1::SharcMemory memory;
  program(memory, 2, true);
  code(memory, base + 2, read_stack_r10);
  constexpr std::uint64_t read_stky_r11 = 0x77be05800000;
  code(memory, base + 3, read_stky_r11);
  auto state = initial();
  constexpr std::uint64_t payload = 7;
  state.data[0][2] = {.value = payload * 256, .known = full40};
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  static_cast<void>(cpu.step());
  for (unsigned int iteration = 1; iteration <= 2; ++iteration) {
    static_cast<void>(cpu.step());
    static_cast<void>(cpu.step());
    const auto frame = ((iteration == 2 ? 0x3000000U : 0x1000000U) | (base + 4));
    require(cpu.state().data[0][10].value == std::uint64_t{frame} * 256);
    require(cpu.state().data[0][10].known == full40);
    static_cast<void>(cpu.step());
    constexpr std::uint64_t lsem = std::uint64_t{1} << 34; // STKY bit26, left justified8.
    require(((cpu.state().data[0][11].value & lsem) != 0) == (iteration == 2));
    require((cpu.state().data[0][11].known & lsem) == lsem);
    constexpr unsigned int limit = 32;
    unsigned int steps{};
    while (cpu.state().pc != base + 4 && steps++ < limit) { static_cast<void>(cpu.step()); }
    require(cpu.state().pc == base + 4);
    static_cast<void>(cpu.step());
    require(memory.read({.address = data_base + iteration - 1}) == payload);
  }
}

void false_startup_call() {
  lead_a1::SharcMemory memory;
  program(memory, 1, true);
  constexpr std::uint64_t if_eq_delayed_call = 0x068004000000;
  code(memory, base + 1, if_eq_delayed_call | helper);
  auto state = initial();
  constexpr std::uint64_t payload = 7;
  state.data[0][8] = {.value = payload * 256, .known = full40};
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(cpu.state().depth == 1 && cpu.state().loops_empty == true);
  require(!active(cpu).control && !cpu.state().branch);
  for (unsigned int index = 0; index < 3; ++index) { static_cast<void>(cpu.step()); }
  require(!cpu.state().count_loop && memory.read({.address = data_base}) == payload);
  require(cpu.state().data[0][4].value == 0); // No callee/inner function ran.
}

void malformed_startup_restore() {
  lead_a1::SharcMemory memory;
  program(memory, 1, true);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  auto state = cpu.state();
  state.depth = 1; // Architectural startup depth cannot be fabricated.
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "unknown four-loop startup");
  state = cpu.state();
  state.pc_stack[1] = base + 1;
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "Fabricated SHARC startup");
}

void zero_count_prefix() {
  lead_a1::SharcMemory memory;
  program(memory, 0, false);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  for (unsigned int index = 0; index < 8; ++index) { static_cast<void>(cpu.step()); }
  constexpr std::uint64_t count = std::uint64_t{1} << 32;
  require(active(cpu).remaining == count - 2 && cpu.state().pc == base + 1);
}

}

int main() {
  try {
    for (const unsigned int count : {1U, 2U, 3U, 4U, 27U}) {
      straight_case(count);
      delayed_calls(count);
    }
    unknown_startup_read_and_repair();
    delay_slot_failure();
    final_frame_operand_reads();
    false_startup_call();
    malformed_startup_restore();
    zero_count_prefix();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
