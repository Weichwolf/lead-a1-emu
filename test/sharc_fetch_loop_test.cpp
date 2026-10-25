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
constexpr std::uint64_t immediate_do = 0x0c0000000001;
constexpr std::uint64_t increment = 0x013e00029000;
constexpr std::uint64_t decrement = 0x013e0002a000;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t parent = 0x10c0030;
using Address = std::optional<std::uint32_t>;
using Stages = std::array<Address, 5>;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC fetch loop expectation at line " +
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
  throw std::runtime_error("Missing SHARC fetch loop rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, unsigned int count) {
  constexpr unsigned int cells = 256;
  constexpr unsigned int words = 64;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned single-loop phases"});
  for (unsigned int index = 0; index < words; ++index) { code(memory, base + index, 0); }
  constexpr unsigned int count_shift = 24;
  code(memory, base, immediate_do | (static_cast<std::uint64_t>(count) << count_shift));
  code(memory, base + 1, increment);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.data[0][0] = {.value = std::uint64_t{10} * 256, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

lead_a1::SharcFetchFlow flow(const lead_a1::SharcFetch &fetch) {
  const auto value = fetch.state().flow;
  if (!value) { throw std::runtime_error("Missing test flow"); }
  return *value;
}

lead_a1::SharcFetchLoop loop(const lead_a1::SharcFetch &fetch) {
  const auto value = flow(fetch).loop;
  if (!value) { throw std::runtime_error("Missing test loop"); }
  return *value;
}

Address address(const std::optional<lead_a1::SharcFetchWord> &word) {
  return word ? Address{word->address - base} : Address{};
}

Stages stages(const lead_a1::SharcFetch &fetch) {
  const auto &state = fetch.state();
  return {state.fetch1 ? Address{*state.fetch1 - base} : Address{},
          address(state.fetch2),
          address(state.decode),
          address(state.address),
          address(state.execute)};
}

void warm(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  for (unsigned int clock = 0; clock < 3; ++clock) {
    const auto event = fetch.advance_sequenced(cpu);
    require(event.address_accepted && !event.stalled && !fetch.state().execute);
  }
}

struct Table {
  unsigned int count;
  unsigned int pop_clock;
  unsigned int end_clock;
  std::array<Stages, 10> clocks;
};

void pipeline_tables() {
  const Address gap;
  // PGR4-61/62 Tables4-17..20, F1,F2,D,A,X; n=0, body=1.
  // Held Decode arrows remain captured words; their issue is suppressed at c3.
  const std::array<Table, 4> tables{{
      {.count = 5,
       .pop_clock = 5,
       .end_clock = 9,
       .clocks = {{{3, 2, 1, 0, gap},
                   {1, 3, 1, 1, 0},
                   {1, 3, 1, gap, 1},
                   {1, 1, 1, 1, gap},
                   {2, 1, 1, 1, 1},
                   {3, 2, 1, 1, 1},
                   {4, 3, 2, 1, 1},
                   {5, 4, 3, 2, 1},
                   {6, 5, 4, 3, 2}}}},
      {.count = 4,
       .pop_clock = 4,
       .end_clock = 8,
       .clocks = {{{3, 2, 1, 0, gap},
                   {1, 3, 1, 1, 0},
                   {1, 3, 1, gap, 1},
                   {2, 1, 1, 1, gap},
                   {3, 2, 1, 1, 1},
                   {4, 3, 2, 1, 1},
                   {5, 4, 3, 2, 1},
                   {6, 5, 4, 3, 2}}}},
      {.count = 3,
       .pop_clock = 6,
       .end_clock = 10,
       .clocks = {{{3, 2, 1, 0, gap},
                   {1, 3, 1, 1, 0},
                   {1, 3, 1, gap, 1},
                   {1, 1, 1, 1, gap},
                   {1, 1, gap, 1, 1},
                   {2, 1, gap, gap, 1},
                   {3, 2, gap, gap, gap},
                   {4, 3, 2, gap, gap},
                   {5, 4, 3, 2, gap},
                   {6, 5, 4, 3, 2}}}},
      {.count = 2,
       .pop_clock = 5,
       .end_clock = 9,
       .clocks = {{{3, 2, 1, 0, gap},
                   {1, 3, 1, 1, 0},
                   {1, 3, 1, gap, 1},
                   {1, 1, gap, 1, gap},
                   {2, 1, gap, gap, 1},
                   {3, 2, gap, gap, gap},
                   {4, 3, 2, gap, gap},
                   {5, 4, 3, 2, gap},
                   {6, 5, 4, 3, 2}}}},
  }};
  for (const auto &table : tables) {
    lead_a1::SharcMemory memory;
    program(memory, table.count);
    lead_a1::Sharc cpu(memory, initial());
    lead_a1::SharcFetch fetch(memory, sequenced());
    warm(cpu, fetch);
    unsigned int iterations{};
    for (unsigned int clock = 1; clock <= table.end_clock; ++clock) {
      const auto event = fetch.advance_sequenced(cpu);
      require(stages(fetch) == table.clocks.at(clock - 1));
      require(event.loop_stall == (clock == 3) && event.stalled == (clock == 3));
      if (fetch.state().execute) {
        const auto step = cpu.step(fetch);
        require(step.minimum_core_cycles == 1);
        if (step.pc == base + 1) {
          require(step.opcode == increment);
          ++iterations;
        }
      }
      if (clock >= 2) {
        require(cpu.state().depth == (clock < table.pop_clock ? 2U : 1U));
        require(cpu.state().loops_empty == (clock >= table.pop_clock));
        require(cpu.state().pc_stack[1] == base + 1 && cpu.state().pc_stack[0] == parent);
      }
      require(cpu.state().data[0][0].value == (std::uint64_t{10} + iterations) * 256);
    }
    require(iterations == table.count && !cpu.state().count_loop && !flow(fetch).loop);
    require(cpu.state().pc == base + 3);
  }
}

void retained_decode_and_real_refetch() {
  lead_a1::SharcMemory memory;
  program(memory, 5);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  warm(cpu, fetch);
  unsigned int iterations{};
  constexpr std::array<std::uint64_t, 5> expected{
      increment, increment, increment, decrement, decrement};
  for (unsigned int clock = 1; clock <= 9; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (clock == 1) { code(memory, base + 1, decrement); }
    if (fetch.state().execute) {
      const auto step = cpu.step(fetch);
      if (step.pc == base + 1) { require(step.opcode == expected.at(iterations++)); }
    }
  }
  require(iterations == 5 && cpu.state().data[0][0].value == std::uint64_t{11} * 256);
}

void checkpoints() {
  for (const unsigned int count : {2U, 3U, 4U, 5U}) {
    lead_a1::SharcMemory memory;
    program(memory, count);
    lead_a1::Sharc cpu(memory, initial());
    lead_a1::SharcFetch fetch(memory, sequenced());
    constexpr unsigned int clocks = 14;
    for (unsigned int clock = 0; clock < clocks; ++clock) {
      static_cast<void>(fetch.advance_sequenced(cpu));
      // Independent RAM and owners, including occupied Execute/pre-pop states.
      lead_a1::SharcMemory restored_memory;
      program(restored_memory, count);
      lead_a1::Sharc restored(restored_memory, cpu.state());
      lead_a1::SharcFetch transport(restored_memory, fetch.state());
      if (fetch.state().execute) {
        const auto actual = cpu.step(fetch);
        const auto replay = restored.step(transport);
        require(actual.pc == replay.pc && actual.opcode == replay.opcode &&
                actual.next_pc == replay.next_pc &&
                actual.minimum_core_cycles == replay.minimum_core_cycles);
      }
      require(restored.state() == cpu.state() && transport.state() == fetch.state());
      for (unsigned int next = clock + 1; next < clocks; ++next) {
        lead_a1::SharcMemory expected_memory;
        program(expected_memory, count);
        lead_a1::Sharc expected_cpu(expected_memory, cpu.state());
        lead_a1::SharcFetch expected_fetch(expected_memory, fetch.state());
        for (unsigned int step = clock + 1; step <= next; ++step) {
          static_cast<void>(expected_fetch.advance_sequenced(expected_cpu));
          if (expected_fetch.state().execute) {
            static_cast<void>(expected_cpu.step(expected_fetch));
          }
        }
        static_cast<void>(transport.advance_sequenced(restored));
        if (transport.state().execute) { static_cast<void>(restored.step(transport)); }
        require(restored.state() == expected_cpu.state() &&
                transport.state() == expected_fetch.state());
      }
    }
  }
}

void zero_and_long_prefixes() {
  for (const unsigned int count : {0U, 16U, 65535U}) {
    lead_a1::SharcMemory memory;
    program(memory, count);
    code(memory, base + 1, 0);
    lead_a1::Sharc cpu(memory, initial());
    lead_a1::SharcFetch fetch(memory, sequenced());
    const std::uint64_t total = count == 0 ? std::uint64_t{1} << 32 : count;
    constexpr unsigned int limit = 40;
    unsigned int iterations{};
    for (unsigned int clock = 0; clock < limit; ++clock) {
      static_cast<void>(fetch.advance_sequenced(cpu));
      if (fetch.state().execute) {
        const auto step = cpu.step(fetch);
        if (step.pc == base + 1) { ++iterations; }
      }
      const auto active = cpu.state().count_loop;
      if (active) { require(active->total == total && active->remaining == total - iterations); }
    }
    require(iterations == (count == 16 ? 16U : limit - 6));
  }
}

void failing_pop_is_transactional() {
  lead_a1::SharcMemory memory;
  program(memory, 5);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  warm(cpu, fetch);
  for (unsigned int clock = 1; clock <= 5; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (clock < 5 && fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
  }
  const auto executing = fetch.state().execute;
  require(executing && executing->loop_pop);
  auto broken = cpu.state();
  broken.data[0][0] = {};
  lead_a1::Sharc unknown(memory, broken);
  const auto before = fetch.state();
  rejected([&] { static_cast<void>(unknown.step(fetch)); }, "Unknown SHARC");
  require(unknown.state() == broken && fetch.state() == before && !memory.active());
  // The real word and pending pop remain available to the repaired CPU.
  static_cast<void>(cpu.step(fetch));
  const auto active = cpu.state().count_loop;
  require(cpu.state().depth == 1 && active && !active->frame_live);
  require(loop(fetch).retired == 2);
}

void guarded_cases() {
  constexpr std::array<std::uint64_t, 4> unsupported{
      0x0c0001000001, 0x0c0005000002, 0x0d7000000001, 0x0c0005000001};
  for (const auto opcode : unsupported) {
    lead_a1::SharcMemory memory;
    program(memory, 5);
    code(memory, base, opcode);
    constexpr std::uint64_t ram_load = 0x106000080000;
    if (opcode == unsupported.back()) { code(memory, base + 1, ram_load); }
    lead_a1::Sharc cpu(memory, initial());
    lead_a1::SharcFetch fetch(memory, sequenced());
    warm(cpu, fetch);
    const auto before = fetch.state();
    const auto state = cpu.state();
    rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "not qualified");
    require(cpu.state() == state && fetch.state() == before && !memory.active());
  }
}

void quiet_pop_failure_and_repair() {
  lead_a1::SharcMemory memory;
  program(memory, 4);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  warm(cpu, fetch);
  for (unsigned int clock = 1; clock <= 3; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
  }
  require(loop(fetch).pop == lead_a1::SharcFetchLoopPop::due);
  lead_a1::SharcMemory repaired_memory;
  constexpr unsigned int cells = 256;
  repaired_memory.map({.bank = 2, .halfwords = cells, .name = "owned late missing F2 data"});
  lead_a1::SharcControl controller(true);
  lead_a1::Sharc restored(repaired_memory, cpu.state(), &controller);
  lead_a1::SharcFetch transport(repaired_memory, fetch.state());
  const auto before = transport.state();
  const auto old_cpu = restored.state();
  rejected([&] { static_cast<void>(transport.advance_sequenced(restored)); },
           "Unknown SHARC memory bits");
  require(restored.state() == old_cpu && transport.state() == before && !repaired_memory.active());
  code(repaired_memory, base + 1, increment);
  controller.begin_instruction();
  const auto old_controller = controller.state();
  rejected([&] { static_cast<void>(transport.advance_sequenced(restored)); },
           "during a transaction");
  require(restored.state() == old_cpu && transport.state() == before);
  require(controller.state() == old_controller && !repaired_memory.active());
  rejected([&] { controller.advance_internal_clocks(1); }, "during a transaction");
  controller.abort_instruction();
  const auto event = transport.advance_sequenced(restored);
  require(event.address_accepted && !event.stalled && !transport.state().execute);
  const auto active = restored.state().count_loop;
  require(restored.state().depth == 1 && active && !active->frame_live && active->remaining == 3);
  require(restored.state().data == old_cpu.data);
  require(loop(transport).pop == lead_a1::SharcFetchLoopPop::done);
}

template <typename Mutate>
void bad_snapshot(lead_a1::SharcMemory &memory,
                  lead_a1::SharcFetchState state,
                  const Mutate &mutate,
                  const std::string &reason = "Invalid SHARC") {
  if (!state.flow || !state.flow->loop) { throw std::runtime_error("Missing corruptible loop"); }
  mutate(*state.flow->loop);
  rejected([&] { const lead_a1::SharcFetch invalid(memory, state); }, reason);
}

void malformed_snapshots() {
  lead_a1::SharcMemory memory;
  program(memory, 5);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  warm(cpu, fetch);
  static_cast<void>(fetch.advance_sequenced(cpu));
  const auto snapshot = fetch.state();
  bad_snapshot(memory, snapshot, [](auto &item) { item.total = 1; });
  bad_snapshot(memory, snapshot, [](auto &item) { ++item.body; });
  bad_snapshot(
      memory,
      snapshot,
      [](auto &item) {
        constexpr std::uint32_t wrapped_address = 0xffffffff;
        item.body = wrapped_address;
      },
      "internal native address");
  bad_snapshot(memory, snapshot, [](auto &item) { item.fetch_remaining = 0; });
  bad_snapshot(memory, snapshot, [](auto &item) { item.issued = item.total + 1; });
  bad_snapshot(memory, snapshot, [](auto &item) { item.retired = 1; });
  bad_snapshot(memory, snapshot, [](auto &item) { item.issued = 1; });
  bad_snapshot(memory, snapshot, [](auto &item) { item.terminal = true; });
  bad_snapshot(memory, snapshot, [](auto &item) { item.pop = lead_a1::SharcFetchLoopPop::done; });
  bad_snapshot(
      memory, snapshot, [](auto &item) { item.phase = lead_a1::SharcFetchLoopPhase::stream; });
  bad_snapshot(memory, snapshot, [](auto &item) { item.held.reset(); });
  bad_snapshot(memory, snapshot, [](auto &item) {
    constexpr unsigned int invalid_depth = 28;
    item.saved_depth = invalid_depth;
  });
  bad_snapshot(memory, snapshot, [](auto &item) {
    item.held =
        lead_a1::SharcFetchWord{.address = item.body, .opcode = increment, .loop_body = true};
  });
  static_cast<void>(fetch.advance_sequenced(cpu));
  static_cast<void>(cpu.step(fetch));
  auto mismatched = cpu.state();
  if (!mismatched.count_loop) { throw std::runtime_error("Missing native loop"); }
  --mismatched.count_loop->remaining;
  lead_a1::Sharc restored(memory, mismatched);
  const auto before = fetch.state();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(restored)); }, "native state mismatch");
  require(restored.state() == mismatched && fetch.state() == before && !memory.active());
}

}

int main() {
  try {
    pipeline_tables();
    retained_decode_and_real_refetch();
    checkpoints();
    zero_and_long_prefixes();
    failing_pop_is_transactional();
    guarded_cases();
    quiet_pop_failure_and_repair();
    malformed_snapshots();
    std::cout << "SHARC single-loop fetch phases and transactional retirement passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
