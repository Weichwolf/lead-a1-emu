// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t parent_frame = 0x10c0030;
constexpr std::uint32_t loop_empty = 0x4000000;
constexpr std::uint64_t immediate_do = 0x0c0000000001;
constexpr std::uint64_t ustat_do = 0x0d7000000001;
constexpr unsigned int count_shift = 24;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC loop expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC loop rejection: " + reason);
}

void setup(lead_a1::SharcMemory &memory, std::uint64_t loop) {
  memory.map({.bank = 2, .halfwords = 16, .name = "owned short loop"});
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, loop);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, 0);
  memory.write({.address = base + 2, .width = lead_a1::SharcWidth::wide48}, 0);
}

struct ShortLoopCase {
  unsigned int count;
  unsigned int pop_iteration;
  unsigned int expected_cycles;
};

void short_case(ShortLoopCase expectation) {
  const auto [count, pop_iteration, expected_cycles] = expectation;
  lead_a1::SharcMemory memory;
  setup(memory, immediate_do | (static_cast<std::uint64_t>(count) << count_shift));
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.depth = 1;
  initial.pc_stack[0] = parent_frame;
  lead_a1::Sharc cpu(memory, initial);
  unsigned int cycles = cpu.step().minimum_core_cycles;
  require(cpu.state().pc == base + 1 && cpu.state().depth == 2 && cpu.state().loops_empty == false);
  require(cpu.state().pc_stack[1] == base + 1 && (cpu.state().system[8].value & loop_empty) == 0);
  for (unsigned int iteration = 1; iteration <= count; ++iteration) {
    cycles += cpu.step().minimum_core_cycles;
    require(cpu.state().depth == (iteration >= pop_iteration ? 1U : 2U));
    require(cpu.state().loops_empty == (iteration >= pop_iteration));
    require(cpu.state().pc == (iteration == count ? base + 2 : base + 1));
  }
  require(cycles == expected_cycles && !cpu.state().count_loop);
  require(cpu.state().pc_stack[0] == parent_frame && cpu.state().pc_stack[1] == base + 1);
  require((cpu.state().system[8].value & loop_empty) != 0);
  require(cpu.step().next_pc == base + 3);
}

void literal_short_loops() {
  // PGR4-61..63 execution charts; cycles include DO, initial bubble and drain.
  constexpr std::array<ShortLoopCase, 6> cases{
      {{.count = 1, .pop_iteration = 1, .expected_cycles = 6},
       {.count = 2, .pop_iteration = 2, .expected_cycles = 7},
       {.count = 3, .pop_iteration = 3, .expected_cycles = 8},
       {.count = 4, .pop_iteration = 1, .expected_cycles = 6},
       {.count = 5, .pop_iteration = 2, .expected_cycles = 7},
       {.count = 16, .pop_iteration = 13, .expected_cycles = 18}}};
  for (const auto &expectation : cases) { short_case(expectation); }
}

void register_count_and_zero_prefix() {
  lead_a1::SharcMemory memory;
  setup(memory, ustat_do);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[0].value = 5;
  initial.effective = initial.system;
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  const auto loop = cpu.state().count_loop;
  require(loop && loop->total == 5);
  for (unsigned int iteration = 0; iteration < 5; ++iteration) { static_cast<void>(cpu.step()); }
  require(cpu.state().pc == base + 2 && cpu.state().depth == 0 && !cpu.state().count_loop);
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, immediate_do);
  lead_a1::Sharc zero(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(zero.step());
  constexpr unsigned int prefix = 6;
  for (unsigned int index = 0; index < prefix; ++index) { static_cast<void>(zero.step()); }
  const auto large = zero.state().count_loop;
  constexpr std::uint64_t iterations = std::uint64_t{1} << 32;
  require(large && large->total == iterations && large->remaining == iterations - prefix);
  require(zero.state().pc == base + 1 && zero.state().depth == 1);
}

void failure_and_repair() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t loop5 = 0x0c0005000001;
  constexpr std::uint64_t immediate_status = 0x0f7012345678;
  setup(memory, loop5);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(cpu.step());
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, immediate_status);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "body instruction/pipeline");
  require(cpu.state() == before && !memory.active());
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, 0);
  for (unsigned int count = 0; count < 5; ++count) { static_cast<void>(cpu.step()); }
  require(!cpu.state().count_loop && cpu.state().pc == base + 2 && cpu.state().depth == 0);
  constexpr std::uint64_t multiple_instructions = 0x0c0005000002;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, multiple_instructions);
  lead_a1::Sharc multi(memory, lead_a1::Sharc::reset_reference_214xx(base));
  const auto untouched = multi.state();
  rejected([&] { static_cast<void>(multi.step()); }, "two-to-three-instruction count loops");
  require(multi.state() == untouched);
  constexpr std::uint64_t reserved = 0x0df000000001;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, reserved);
  lead_a1::Sharc invalid(memory, lead_a1::Sharc::reset_reference_214xx(base));
  rejected([&] { static_cast<void>(invalid.step()); }, "Reserved SHARC UREG loop fields");
}

}

int main() {
  try {
    literal_short_loops();
    register_count_and_zero_prefix();
    failure_and_repair();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
