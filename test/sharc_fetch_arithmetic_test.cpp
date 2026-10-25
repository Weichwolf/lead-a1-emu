// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t body = base + 1;
constexpr std::uint32_t parent = 0x10c0070;
constexpr std::uint64_t full40 = 0xffffffffff;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC fetched arithmetic expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing arithmetic fetch rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 384;
  constexpr unsigned int instructions = 64;
  constexpr std::array<std::uint64_t, 3> decrements{0x013e0002a660, 0x013e0002a770, 0x013e0002a550};
  memory.map({.bank = 2, .halfwords = cells, .name = "owned fetched arithmetic"});
  for (unsigned int index = 0; index < instructions; ++index) { code(memory, base + index, 0); }
  constexpr std::uint64_t setup = 0x0e0000000006;
  code(memory, base, setup);
  for (unsigned int index = 0; index < decrements.size(); ++index) {
    code(memory, body + index, decrements[index]);
  }
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.data[0][5] = {.value = std::uint64_t{6} * 256, .known = full40};
  state.data[0][6] = {.value = std::uint64_t{10} * 256, .known = full40};
  state.data[0][7] = {.value = std::uint64_t{14} * 256, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

std::optional<std::uint32_t> address(const std::optional<lead_a1::SharcFetchWord> &word) {
  return word ? std::optional{word->address} : std::nullopt;
}

void advance(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu) {
  const auto bus = fetch.address_pm_request(cpu) ? lead_a1::SharcFetchBus::pm_data
                                                 : lead_a1::SharcFetchBus::available;
  const auto event = fetch.advance_sequenced(cpu, bus);
  require(event.address_accepted && !event.stalled);
}

void retire(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu) {
  if (fetch.state().execute) { require(cpu.step(fetch).minimum_core_cycles == 1); }
}

void manufacturer_pipeline_columns() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int grants{};
  unsigned int retired{};
  constexpr unsigned int limit = 100;
  while (grants < limit) {
    advance(fetch, cpu);
    ++grants;
    if (address(fetch.state().execute) == body && cpu.state().data[0][5].value == 256) { break; }
    if (fetch.state().execute) { ++retired; }
    retire(fetch, cpu);
  }
  require(grants < limit);
  // PGRrev2.4 Table4-15, literal rows E/A/D/F2/F1; -1 denotes a real bubble.
  constexpr std::array<std::array<int, 9>, 5> columns{{
      {0, 1, 2, 3, 4, 5, -1, -1, 6},
      {1, 2, 3, 4, 5, -1, -1, 6, 7},
      {2, 3, 4, 5, -1, -1, 6, 7, 8},
      {3, 4, 5, 0, 1, 6, 7, 8, 9},
      {4, 5, 0, 1, 6, 7, 8, 9, 10},
  }};
  for (unsigned int cycle = 0; cycle < 9; ++cycle) {
    if (cycle != 0) {
      advance(fetch, cpu);
      ++grants;
    }
    const auto &state = fetch.state();
    const std::array stages{address(state.execute),
                            address(state.address),
                            address(state.decode),
                            address(state.fetch2),
                            state.fetch1};
    for (unsigned int stage = 0; stage < stages.size(); ++stage) {
      const int offset = columns[stage][cycle];
      const auto expected =
          offset < 0 ? std::nullopt : std::optional{body + static_cast<unsigned int>(offset)};
      require(stages[stage] == expected);
    }
    require(cpu.state().depth == (cycle < 3 ? 2U : 1U));
    require(cpu.state().loops_empty == (cycle >= 3));
    if (state.execute) { ++retired; }
    retire(fetch, cpu);
  }
  constexpr unsigned int expected_grants = 44;
  constexpr unsigned int expected_retirements = 38;
  require(grants == expected_grants && retired == expected_retirements &&
          cpu.state().pc == base + 8);
  require(cpu.state().data[0][5] == (lead_a1::SharcKnown40{.known = full40}));
  require(cpu.state().data[0][6].value == std::uint64_t{4} * 256 &&
          cpu.state().data[0][7].value == std::uint64_t{8} * 256);
  require(!cpu.state().arithmetic_loop && cpu.state().pc_stack[0] == parent);
}

void popped_operands_and_latched_flags() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t parent_and_pass = 0x767e01021430;
  code(memory, body + 3, parent_and_pass);
  auto state = initial();
  state.data[0][3] = {.value = std::uint64_t{7} * 256, .known = full40};
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int limit = 100;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != body + 6; ++clock) {
    advance(fetch, cpu);
    retire(fetch, cpu);
  }
  require(cpu.state().pc == body + 6 && !cpu.state().arithmetic_loop && cpu.state().depth == 1);
  require(cpu.state().data[0][2] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{parent} << 8, .known = full40}));
  require(cpu.state().data[0][4].value == std::uint64_t{7} * 256);
  require((cpu.state().system[6].value & 1) == 0 && cpu.state().data[0][5].value == 0);
}

void unknown_sampling_and_missing_marker() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int limit = 100;
  for (unsigned int clock = 0; clock < limit && cpu.state().data[0][5].value != 0; ++clock) {
    advance(fetch, cpu);
    retire(fetch, cpu);
  }
  require(cpu.state().pc == body + 3 && fetch.state().fetch1 == body);
  auto unknown = cpu.state();
  unknown.system[6].known &= ~1U;
  unknown.system[6].value &= unknown.system[6].known;
  unknown.effective[6] = unknown.system[6];
  lead_a1::Sharc unresolved(memory, unknown, &control);
  lead_a1::SharcFetch pending(memory, fetch.state());
  const auto before = pending.state();
  const auto device = control.snapshot();
  rejected([&] { static_cast<void>(pending.advance_sequenced(unresolved)); },
           "Unknown SHARC control/status bits");
  require(unresolved.state() == unknown && pending.state() == before &&
          control.snapshot() == device && !memory.active());
  advance(fetch, cpu);
  retire(fetch, cpu);
  auto corrupt = fetch.state();
  if (!corrupt.flow) { throw std::logic_error("Missing owned arithmetic cursor"); }
  corrupt.flow->discard_decode = 0;
  lead_a1::SharcFetch bad_columns(memory, corrupt);
  rejected([&] { static_cast<void>(bad_columns.address_pm_request(cpu)); },
           "drain pipeline columns");
  require(bad_columns.state() == corrupt);
  auto missing = fetch.state();
  if (!missing.decode) { throw std::logic_error("Missing owned arithmetic end"); }
  require(missing.decode->arithmetic_exit);
  missing.decode->arithmetic_exit = false;
  lead_a1::SharcFetch bad(memory, missing);
  rejected([&] { static_cast<void>(bad.address_pm_request(cpu)); }, "actual retained end word");
  require(bad.state() == missing);
}

void orphan_marker() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  auto state = sequenced();
  state.decode = lead_a1::SharcFetchWord{.address = body + 5, .opcode = 0, .arithmetic_exit = true};
  lead_a1::SharcFetch fetch(memory, state);
  const auto native = cpu.state();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "without its native frame");
  require(cpu.state() == native && fetch.state() == state && !memory.active());
}

void guarded_mixed_timing(std::uint64_t opcode, const std::string &reason) {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, body + 3, opcode);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int limit = 16;
  for (unsigned int clock = 0; clock < limit; ++clock) {
    const auto before = fetch.state();
    const auto native = cpu.state();
    const auto device = control.snapshot();
    try {
      advance(fetch, cpu);
    } catch (const std::exception &error) {
      require(std::string(error.what()).find(reason) != std::string::npos);
      require(fetch.state() == before && cpu.state() == native && control.snapshot() == device &&
              !memory.active());
      constexpr std::uint32_t target = base + 40;
      require(memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) == 0);
      return;
    }
    retire(fetch, cpu);
  }
  throw std::runtime_error("Missing arithmetic mixed-timing rejection: " + reason);
}

void simd_conditions_and_forever() {
  constexpr unsigned int forever_case = 2;
  for (unsigned int example = 0; example < 3; ++example) {
    lead_a1::SharcMemory memory;
    program(memory);
    for (unsigned int index = 0; index < 3; ++index) { code(memory, body + index, 0); }
    auto state = initial();
    constexpr std::uint32_t simd = 0x200000;
    state.system[2].value = simd;
    state.system[6].value = 1;
    state.system[7].value = example == 1 ? 1U : 0U;
    if (example == forever_case) {
      constexpr std::uint64_t forever_do = 0x0e3e00000006;
      code(memory, base, forever_do);
      state.system[6] = {};
      state.system[7] = {};
    }
    state.effective = state.system;
    lead_a1::Sharc cpu(memory, state);
    lead_a1::SharcFetch fetch(memory, sequenced());
    unsigned int ends{};
    constexpr unsigned int limit = 64;
    for (unsigned int clock = 0; clock < limit && cpu.state().pc != body + 6; ++clock) {
      advance(fetch, cpu);
      if (address(fetch.state().execute) == body + 5) { ++ends; }
      retire(fetch, cpu);
    }
    const bool terminated = example == 1;
    require((cpu.state().pc == body + 6) == terminated && cpu.state().loops_empty == terminated &&
            cpu.state().depth == (terminated ? 1U : 2U));
    require(cpu.state().arithmetic_loop.has_value() != terminated);
    require(terminated ? ends == 1 : ends > 1);
  }
}

}

int main() try {
  manufacturer_pipeline_columns();
  popped_operands_and_latched_flags();
  unknown_sampling_and_missing_marker();
  orphan_marker();
  constexpr std::uint64_t multiply = 0x013e00130312;
  guarded_mixed_timing(multiply, "multiplier arithmetic-loop condition latency");
  simd_conditions_and_forever();
  std::cout << "Fetched arithmetic Table4-15 columns and native effects pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
