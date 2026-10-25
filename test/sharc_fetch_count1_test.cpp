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
constexpr std::uint32_t helper = base + 32;
constexpr std::uint32_t inner = base + 64;
constexpr std::uint32_t parent = 0x10c0060;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t increment = 0x013e00029770;
constexpr std::uint64_t delayed_call = 0x06be04000000;
constexpr std::uint64_t rts = 0x0a3e00000000;
constexpr std::uint32_t resource_flags = 0x4600000;
using Address = std::optional<std::uint32_t>;
using Stages = std::array<Address, 5>;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC fetched count1 expectation at line " +
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
  throw std::runtime_error("Missing SHARC count1 rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, bool calls) {
  constexpr unsigned int cells = 512;
  constexpr unsigned int words = 96;
  constexpr std::uint64_t do_r2 = 0x0d0200000004;
  constexpr std::uint64_t write_r6 = 0x0f0600000003;
  constexpr std::uint64_t decrement_r6 = 0x013e0002a660;
  constexpr std::uint64_t jump_ne = 0x062000000000;
  constexpr std::uint64_t call = 0x06be00000000;
  constexpr std::uint64_t increment_r5 = 0x013e00029550;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned fetched count1 code"});
  for (unsigned int index = 0; index < words; ++index) { code(memory, base + index, 0); }
  code(memory, base, do_r2);
  for (unsigned int index = 1; index <= 4; ++index) { code(memory, base + index, increment); }
  if (!calls) { return; }
  code(memory, base + 1, delayed_call | helper);
  code(memory, helper, write_r6);
  code(memory, helper + 1, decrement_r6);
  code(memory, helper + 2, jump_ne | (helper + 1));
  code(memory, helper + 3, call | inner);
  code(memory, helper + 4, increment);
  code(memory, helper + 5, rts);
  code(memory, inner, increment_r5);
  code(memory, inner + 1, rts);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.data[0][2] = {.value = 256, .known = full40};
  state.data[0][5] = {.known = full40};
  state.data[0][7] = {.known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

void tick(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu) {
  static_cast<void>(fetch.advance_sequenced(cpu));
  if (fetch.state().execute) { require(cpu.step(fetch).minimum_core_cycles == 1); }
}

void finish(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu, unsigned int sum = 4) {
  constexpr unsigned int limit = 256;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != base + 5; ++clock) {
    tick(fetch, cpu);
  }
  require(cpu.state().pc == base + 5 && !cpu.state().count_loop);
  require(cpu.state().depth == 1 && cpu.state().loops_empty == true &&
          cpu.state().pc_stack[0] == parent);
  require(cpu.state().data[0][7] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{sum} << 8, .known = full40}));
}

Address address(const std::optional<lead_a1::SharcFetchWord> &word) {
  return word ? Address{word->address - base} : Address{};
}

Stages stages(const lead_a1::SharcFetch &fetch) {
  const auto &s = fetch.state();
  return {s.fetch1 ? Address{*s.fetch1 - base} : Address{},
          address(s.fetch2),
          address(s.decode),
          address(s.address),
          address(s.execute)};
}

void manual_columns() {
  const Address bubble;
  const std::array<Stages, 8> expected{{{3, 2, 1, 0, bubble},
                                        {4, 3, bubble, 1, 0},
                                        {4, 3, 2, bubble, 1},
                                        {5, 4, 3, 2, bubble},
                                        {6, 5, 4, 3, 2},
                                        {7, 6, 5, 4, 3},
                                        {8, 7, 6, 5, 4},
                                        {9, 8, 7, 6, 5}}};
  lead_a1::SharcMemory memory;
  program(memory, false);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int warmup = 3;
  for (unsigned int clock = 1; clock <= warmup + expected.size(); ++clock) {
    const auto event = fetch.advance_sequenced(cpu);
    require(event.loop_stall == (clock == 6));
    if (clock > warmup) { require(stages(fetch) == expected[clock - warmup - 1]); }
    if (fetch.state().execute) { require(cpu.step(fetch).minimum_core_cycles == 1); }
    if (clock >= 5) { require(cpu.state().depth == (clock < 7 ? 2U : 1U)); }
    const auto native = cpu.state().count_loop;
    if (clock == 6) { require(native && native->four_pop_pending && native->frame_live); }
    if (clock == 7) { require(native && !native->four_pop_pending && !native->frame_live); }
  }
  require(cpu.state().pc == base + 6 && cpu.state().data[0][7].value == 4ULL * 256);
}

void popped_operands_and_owner_repair() {
  lead_a1::SharcMemory memory;
  program(memory, false);
  constexpr std::uint64_t stack_r0 = 0x767e00000000;
  constexpr std::uint64_t depth_r1 = 0x767e80800000;
  code(memory, base + 2, stack_r0);
  code(memory, base + 3, depth_r1);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int pending_clock = 6;
  for (unsigned int clock = 0; clock < pending_clock; ++clock) { tick(fetch, cpu); }
  const auto before = cpu.state();
  const auto transport = fetch.state();
  const auto device = control.state();
  control.begin_instruction();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "transaction");
  require(cpu.state() == before && fetch.state() == transport && control.state() == device);
  require(!memory.active());
  control.abort_instruction();
  finish(fetch, cpu, 2);
  require(cpu.state().data[0][0] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{parent} << 8, .known = full40}));
  require(cpu.state().data[0][1] == (lead_a1::SharcKnown40{.value = 256, .known = full40}));
}

void independently_rebound_phases() {
  constexpr unsigned int checkpoints = 64;
  for (const bool calls : {false, true}) {
    for (unsigned int checkpoint = 0; checkpoint < checkpoints; ++checkpoint) {
      lead_a1::SharcMemory memory;
      program(memory, calls);
      lead_a1::SharcControl control(true);
      lead_a1::Sharc cpu(memory, initial(), &control);
      lead_a1::SharcFetch fetch(memory, sequenced());
      for (unsigned int clock = 0; clock < checkpoint && cpu.state().pc != base + 5; ++clock) {
        tick(fetch, cpu);
      }
      lead_a1::SharcMemory other_memory;
      program(other_memory, calls);
      auto other_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
      lead_a1::Sharc other(other_memory, cpu.state(), &other_control);
      lead_a1::SharcFetch transport(other_memory, fetch.state());
      finish(transport, other);
      finish(fetch, cpu);
      require(cpu.state() == other.state() && fetch.state() == transport.state());
      const auto expected_inner = calls ? 256U : 0U;
      require(cpu.state().data[0][5] ==
              (lead_a1::SharcKnown40{.value = expected_inner, .known = full40}));
      if (calls) { require(!cpu.state().pc_stack[1]); }
    }
  }
}

void immediate_and_false_calls() {
  constexpr std::uint64_t immediate_one = 0x0c0001000004;
  constexpr std::uint64_t false_call = 0x068004000000;
  constexpr std::uint64_t stack_r0 = 0x767e00000000;
  constexpr std::uint64_t depth_r1 = 0x767e80800000;
  for (const bool taken : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory, true);
    code(memory, base, immediate_one);
    if (!taken) {
      code(memory, base + 1, false_call | helper);
      code(memory, base + 2, stack_r0);
      code(memory, base + 3, depth_r1);
    }
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, initial(), &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    finish(fetch, cpu, taken ? 4U : 1U);
    require(cpu.state().data[0][5].value == (taken ? 256U : 0U));
    if (!taken) {
      require(cpu.state().data[0][0] ==
              (lead_a1::SharcKnown40{.value = std::uint64_t{parent} << 8, .known = full40}));
      require(cpu.state().data[0][1] == (lead_a1::SharcKnown40{.value = 256, .known = full40}));
    }
  }
}

void unknown_stack_observer_fails() {
  lead_a1::SharcMemory memory;
  program(memory, true);
  constexpr std::uint64_t read_stack = 0x767e00000000;
  code(memory, base + 2, read_stack);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int before_observer = 7;
  for (unsigned int clock = 0; clock < before_observer; ++clock) { tick(fetch, cpu); }
  static_cast<void>(fetch.advance_sequenced(cpu));
  require(cpu.state().pc == base + 2 && !cpu.state().depth && !cpu.state().loops_empty);
  require(cpu.state().pc_stack[0] == parent && !cpu.state().pc_stack[1]);
  require((cpu.state().system[8].known & resource_flags) == 0);
  const auto native = cpu.state();
  const auto transport = fetch.state();
  const auto device = control.state();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "Unknown SHARC PC stack depth");
  require(cpu.state() == native && fetch.state() == transport && control.state() == device &&
          !memory.active());
  code(memory, base + 2, 0); // Replacing RAM cannot replace the actual retained Execute word.
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "Unknown SHARC PC stack depth");
  require(cpu.state() == native && fetch.state() == transport && !memory.active());
}

void malformed_pending_pop() {
  lead_a1::SharcMemory memory;
  program(memory, false);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int pending_clock = 6;
  for (unsigned int clock = 0; clock < pending_clock; ++clock) { tick(fetch, cpu); }
  for (unsigned int variant = 0; variant < 3; ++variant) {
    auto invalid = cpu.state();
    if (!invalid.count_loop) { throw std::logic_error("Missing owned pending pop"); }
    if (variant == 0) {
      invalid.count_loop->total = 2;
    } else if (variant == 1) {
      ++invalid.pc;
    } else {
      invalid.count_loop->frame_live = false;
    }
    rejected([&] { const lead_a1::Sharc broken(memory, invalid, &control); }, "four-loop");
  }
  rejected([&] { static_cast<void>(cpu.step()); }, "requires its owner clock");
  auto missing = fetch.state();
  missing.decode.reset();
  lead_a1::SharcFetch broken(memory, missing);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(broken.advance_sequenced(cpu)); }, "pending-pop pipeline");
  require(cpu.state() == before && broken.state() == missing && !memory.active());
  finish(fetch, cpu);
}

}

int main() try {
  manual_columns();
  popped_operands_and_owner_repair();
  independently_rebound_phases();
  immediate_and_false_calls();
  unknown_stack_observer_fails();
  malformed_pending_pop();
  std::cout << "Fetched count1 four-body loops: timed pop, actual calls and explicit unknown stack "
               "pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
