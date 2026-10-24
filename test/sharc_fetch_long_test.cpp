// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"

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
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t increment = 0x013e00029770;
constexpr std::uint64_t do_r2 = 0x0d0200000005;
constexpr std::uint32_t parent = 0x10c0030;
constexpr std::uint64_t immediate_do = 0x0c0000000000;
constexpr std::uint64_t call = 0x06be00000000;
constexpr std::uint64_t rts = 0x0a3e00000000;
constexpr std::uint32_t subroutine = base + 32;
using Address = std::optional<std::uint32_t>;
using Stages = std::array<Address, 5>;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC long fetch expectation at line " +
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
  throw std::runtime_error("Missing SHARC fetched long-loop rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t pc, std::uint64_t opcode) {
  memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 256;
  constexpr unsigned int instructions = 64;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned long-loop code"});
  for (unsigned int index = 0; index < instructions; ++index) { code(memory, base + index, 0); }
  code(memory, base, do_r2);
  for (unsigned int index = 1; index <= 5; ++index) { code(memory, base + index, increment); }
}

lead_a1::SharcState initial(unsigned int count) {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.data[0][2] = {.value = std::uint64_t{count} << 8, .known = full40};
  state.data[0][7] = {.known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
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

void manual_iteration_and_termination() {
  // Tables4-13/14, e=5,b=1. Shared column is final iteration's e-4.
  const std::array<Stages, 11> expected{{{5, 4, 3, 2, 1},
                                         {1, 5, 4, 3, 2},
                                         {2, 1, 5, 4, 3},
                                         {3, 2, 1, 5, 4},
                                         {4, 3, 2, 1, 5},
                                         {5, 4, 3, 2, 1},
                                         {6, 5, 4, 3, 2},
                                         {7, 6, 5, 4, 3},
                                         {8, 7, 6, 5, 4},
                                         {9, 8, 7, 6, 5},
                                         {10, 9, 8, 7, 6}}};
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(2), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int retired{};
  constexpr unsigned int warmup = 5;
  for (unsigned int time = 1; time <= warmup + expected.size(); ++time) {
    const auto event = fetch.advance_sequenced(cpu);
    require(event.address_accepted && !event.stalled);
    if (time > warmup) { require(stages(fetch) == expected[time - warmup - 1]); }
    if (fetch.state().execute) {
      static_cast<void>(cpu.step(fetch));
      ++retired;
    }
    if (time >= 5) { require(cpu.state().depth == (time < 12 ? 2U : 1U)); }
  }
  require(retired == 12 && cpu.state().pc == base + 7 && !cpu.state().count_loop);
  require(cpu.state().data[0][7].value == 10ULL * 256 && cpu.state().pc_stack[0] == parent);
}

void tick(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu) {
  static_cast<void>(fetch.advance_sequenced(cpu));
  if (fetch.state().execute) { require(cpu.step(fetch).minimum_core_cycles == 1); }
}

struct Expected {
  unsigned int length;
  unsigned int sum;
};

void finish(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu, Expected expected) {
  constexpr unsigned int limit = 4096;
  unsigned int clocks{};
  while (cpu.state().pc != base + expected.length + 1 && clocks < limit) {
    tick(fetch, cpu);
    ++clocks;
  }
  require(cpu.state().pc == base + expected.length + 1 && !cpu.state().count_loop);
  require(cpu.state().depth == 1 && cpu.state().pc_stack[0] == parent);
  require(cpu.state().data[0][7] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{expected.sum} << 8, .known = full40}));
  const auto flow = fetch.state().flow;
  require(flow && !flow->long_loop);
}

void counts_and_lengths() {
  constexpr unsigned int count_shift = 24;
  for (const bool reg : {false, true}) {
    for (const unsigned int length : {5U, 6U, 9U}) {
      constexpr unsigned int larger = 17;
      for (const unsigned int count : {1U, 2U, 3U, 5U, larger}) {
        lead_a1::SharcMemory memory;
        program(memory);
        const auto setup = reg ? (do_r2 & ~std::uint64_t{0xffffff})
                               : immediate_do | (std::uint64_t{count} << count_shift);
        code(memory, base, setup | length);
        for (unsigned int index = 1; index <= length; ++index) {
          code(memory, base + index, increment);
        }
        lead_a1::SharcControl control(true);
        lead_a1::Sharc cpu(memory, initial(count), &control);
        lead_a1::SharcFetch fetch(memory, sequenced());
        finish(fetch, cpu, {.length = length, .sum = length * count});
      }
    }
  }
}

void late_count_producer() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t write_r2_three = 0x0f0200000003;
  code(memory, base, write_r2_three);
  code(memory, base + 1, do_r2);
  code(memory, base + 6, increment);
  auto state = initial(0);
  state.data[0][2] = {}; // The real producer has not executed at DO's Address stage.
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int before_producer = 4;
  for (unsigned int time = 0; time < before_producer; ++time) { tick(fetch, cpu); }
  static_cast<void>(fetch.advance_sequenced(cpu)); // DO Address, producer Execute.
  const auto flow = fetch.state().flow;
  require(flow && flow->long_loop && !flow->long_loop->counter);
  require(cpu.state().data[0][2].known == 0 && !cpu.state().count_loop);
  require(cpu.step(fetch).minimum_core_cycles == 1);
  finish(fetch, cpu, {.length = 6, .sum = 15});
}

void calls(lead_a1::SharcMemory &memory) {
  program(memory);
  code(memory, base + 1, call | subroutine);
  constexpr std::uint64_t write_r6_three = 0x0f0600000003;
  constexpr std::uint64_t decrement_r6 = 0x013e0002a660;
  constexpr std::uint64_t jump_ne = 0x062000000000;
  code(memory, subroutine, write_r6_three);
  code(memory, subroutine + 1, decrement_r6);
  code(memory, subroutine + 2, jump_ne | (subroutine + 1));
  code(memory, subroutine + 3, increment);
  code(memory, subroutine + 4, rts);
}

void calls_and_snapshots() {
  // Independent continuation in fresh memory/controller, including a callee
  // and the last iteration's popped tail. Every result has the literal sum15.
  constexpr unsigned int checkpoints = 128;
  for (unsigned int checkpoint = 0; checkpoint < checkpoints; ++checkpoint) {
    lead_a1::SharcMemory memory;
    calls(memory);
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, initial(3), &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    for (unsigned int time = 0; time < checkpoint && cpu.state().pc != base + 6; ++time) {
      tick(fetch, cpu);
    }
    lead_a1::SharcMemory other_memory;
    calls(other_memory);
    auto other_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
    lead_a1::Sharc other(other_memory, cpu.state(), &other_control);
    lead_a1::SharcFetch transport(other_memory, fetch.state());
    finish(transport, other, {.length = 5, .sum = 15});
    finish(fetch, cpu, {.length = 5, .sum = 15});
    require(cpu.state() == other.state() && fetch.state() == transport.state());
  }
}

void zero_and_full_register_counts() {
  constexpr std::uint64_t full_count = std::uint64_t{1} << 32;
  constexpr unsigned int large = 65536;
  for (const bool reg : {false, true}) {
    for (const unsigned int count : {0U, large}) {
      if (!reg && count != 0) { continue; } // Immediate field is only16 bits.
      lead_a1::SharcMemory memory;
      program(memory);
      if (!reg) { code(memory, base, immediate_do | 5); }
      lead_a1::SharcControl control(true);
      lead_a1::Sharc cpu(memory, initial(count), &control);
      lead_a1::SharcFetch fetch(memory, sequenced());
      constexpr unsigned int prefix = 20; // Four warmup clocks, DO, three whole iterations.
      for (unsigned int clock = 0; clock < prefix; ++clock) { tick(fetch, cpu); }
      const auto native = cpu.state().count_loop;
      const auto flow = fetch.state().flow;
      require(native && native->total == (count == 0 ? full_count : count));
      require(native && native->remaining == native->total - 3);
      require(flow && flow->long_loop && flow->long_loop->counter);
      require(cpu.state().data[0][7].value == 15ULL * 256 && cpu.state().depth == 2);
    }
  }
}

void malformed_counter_and_ownership() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(2), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int setup_executed = 5;
  for (unsigned int time = 0; time < setup_executed; ++time) { tick(fetch, cpu); }
  const auto before = fetch.state();
  require(before.flow && before.flow->long_loop && before.flow->long_loop->counter);
  for (unsigned int variant = 0; variant < 5; ++variant) {
    auto invalid = before;
    if (!invalid.flow || !invalid.flow->long_loop || !invalid.flow->long_loop->counter) {
      throw std::runtime_error("Missing test counter");
    }
    auto &loop = *invalid.flow->long_loop;
    switch (variant) {
      case 0: loop.counter->total = 0; break;
      case 1: loop.counter->remaining = 0; break;
      case 2: loop.counter->retired = 3; break;
      case 3: loop.counter->terminal = true; break;
      case 4: loop.end = loop.setup + 4; break;
      default: throw std::logic_error("Invalid test variant");
    }
    rejected([&] { const lead_a1::SharcFetch invalid_fetch(memory, invalid); }, "long-loop");
  }
  finish(fetch, cpu, {.length = 5, .sum = 10});
}

void quiet_ownership() {
  lead_a1::SharcMemory memory;
  calls(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(3), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int call_executed = 6;
  for (unsigned int time = 0; time < call_executed; ++time) { tick(fetch, cpu); }
  require(!fetch.state().address);
  const auto native = cpu.state();
  const auto before = fetch.state();
  control.begin_instruction();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "transaction");
  require(cpu.state() == native && fetch.state() == before && !memory.active());
  control.abort_instruction();
  finish(fetch, cpu, {.length = 5, .sum = 15});
}

void captured_word_and_changed_ram() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(2), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int first_body_f2 = 3;
  for (unsigned int time = 0; time < first_body_f2; ++time) { tick(fetch, cpu); }
  code(memory, base + 1, 0);
  lead_a1::SharcMemory other_memory;
  program(other_memory);
  code(other_memory, base + 1, 0);
  auto other_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
  lead_a1::Sharc other(other_memory, cpu.state(), &other_control);
  lead_a1::SharcFetch transport(other_memory, fetch.state());
  finish(fetch, cpu, {.length = 5, .sum = 9});
  finish(transport, other, {.length = 5, .sum = 9});
  require(cpu.state() == other.state() && fetch.state() == transport.state());
}

void failed_store_before_final_pop() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint32_t data = 0x80000;
  constexpr std::uint32_t payload = 0x12345678;
  constexpr std::uint64_t store_r1_i2 = 0x64fe08800000;
  constexpr std::uint32_t full32 = 0xffffffff;
  memory.map({.bank = 0, .halfwords = 2, .name = "owned first store"});
  code(memory, base + 2, store_r1_i2);
  auto state = initial(2);
  state.data[0][1] = {.value = std::uint64_t{payload} << 8, .known = full40};
  state.dag[0][0][2] = {.value = data, .known = full32};
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int before_second_store = 11;
  for (unsigned int time = 0; time < before_second_store; ++time) { tick(fetch, cpu); }
  static_cast<void>(fetch.advance_sequenced(cpu));
  const auto executing = fetch.state().execute;
  require(executing && executing->address == base + 2);
  const auto native = cpu.state();
  const auto before = fetch.state();
  const auto controller = control.snapshot();
  require(native.count_loop && native.count_loop->remaining == 1 && native.count_loop->frame_live);
  require(native.depth == 2 && native.dag[0][0][2].value == data + 1);
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "Unmapped SHARC physical");
  require(cpu.state() == native && fetch.state() == before && !memory.active());
  require(control.snapshot() == controller && memory.read({.address = data}) == payload);
  memory.map({.bank = 0, .first_halfword = 2, .halfwords = 2, .name = "repaired second store"});
  require(cpu.step(fetch).minimum_core_cycles == 1);
  const auto popped = cpu.state().count_loop;
  require(popped && !popped->frame_live && cpu.state().depth == 1);
  finish(fetch, cpu, {.length = 5, .sum = 8});
  require(cpu.state().dag[0][0][2].value == data + 2);
  require(memory.read({.address = data}) == payload &&
          memory.read({.address = data + 1}) == payload);
}

void unsupported_body_control() {
  struct Invalid {
    unsigned int position;
    std::uint64_t opcode;
    const char *reason;
  };

  constexpr std::uint64_t jump = 0x063e00000000;
  constexpr std::uint64_t delayed = 0x4000000;
  constexpr std::array<Invalid, 6> cases{{
      {.position = 1, .opcode = jump | subroutine, .reason = "body branch"},
      {.position = 2, .opcode = call | subroutine, .reason = "body branch"},
      {.position = 1, .opcode = call | delayed | subroutine, .reason = "body branch"},
      {.position = 1, .opcode = do_r2, .reason = "nested fetched counter"},
      {.position = 1, .opcode = call | (base + 3), .reason = "body branch"},
      {.position = 1, .opcode = rts, .reason = "body branch"},
  }};
  for (const auto entry : cases) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, base + entry.position, entry.opcode);
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, initial(2), &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    for (unsigned int clock = 0; clock < entry.position + 3; ++clock) { tick(fetch, cpu); }
    const auto native = cpu.state();
    const auto before = fetch.state();
    rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, entry.reason);
    require(cpu.state() == native && fetch.state() == before && !memory.active());
  }
}

void four_program(lead_a1::SharcMemory &memory, bool with_call) {
  if (with_call) {
    calls(memory);
  } else {
    program(memory);
  }
  constexpr std::uint64_t do_four = 0x0d0200000004;
  constexpr std::uint64_t delayed = 0x4000000;
  code(memory, base, do_four);
  if (with_call) { code(memory, base + 1, call | delayed | subroutine); }
}

void four_startup_and_termination() {
  // PGR4-67 Table4-27 startup and4-47 termination, two iterations.
  const Address bubble;
  const std::array<Stages, 11> expected{{{3, 2, 1, 0, bubble},
                                         {4, 3, bubble, 1, 0},
                                         {4, 3, 2, bubble, 1},
                                         {1, 4, 3, 2, bubble},
                                         {2, 1, 4, 3, 2},
                                         {3, 2, 1, 4, 3},
                                         {4, 3, 2, 1, 4},
                                         {5, 4, 3, 2, 1},
                                         {6, 5, 4, 3, 2},
                                         {7, 6, 5, 4, 3},
                                         {8, 7, 6, 5, 4}}};
  lead_a1::SharcMemory memory;
  four_program(memory, false);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(2), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int warmup = 3;
  for (unsigned int clock = 1; clock <= warmup + expected.size(); ++clock) {
    const auto event = fetch.advance_sequenced(cpu);
    require(event.loop_stall == (clock == 6) && event.stalled == event.loop_stall);
    if (clock > warmup) { require(stages(fetch) == expected[clock - warmup - 1]); }
    if (fetch.state().execute) { require(cpu.step(fetch).minimum_core_cycles == 1); }
    if (clock >= 5) { require(cpu.state().depth == (clock < 11 ? 2U : 1U)); }
  }
  finish(fetch, cpu, {.length = 4, .sum = 8});
}

void four_calls_and_reentry() {
  constexpr unsigned int checkpoints = 96;
  for (unsigned int checkpoint = 0; checkpoint < checkpoints; ++checkpoint) {
    lead_a1::SharcMemory memory;
    four_program(memory, true);
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, initial(2), &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    for (unsigned int clock = 0; clock < checkpoint && cpu.state().pc != base + 5; ++clock) {
      tick(fetch, cpu);
    }
    lead_a1::SharcMemory other_memory;
    four_program(other_memory, true);
    auto other_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
    lead_a1::Sharc other(other_memory, cpu.state(), &other_control);
    lead_a1::SharcFetch transport(other_memory, fetch.state());
    finish(transport, other, {.length = 4, .sum = 8});
    finish(fetch, cpu, {.length = 4, .sum = 8});
    constexpr std::uint32_t final_frame = 0x30c0004;
    require(cpu.state().pc_stack[1] == final_frame);
    require(cpu.state() == other.state() && fetch.state() == transport.state());
  }
}

void four_count_matrix() {
  constexpr unsigned int larger = 27;
  constexpr unsigned int count_shift = 24;
  for (const bool reg : {false, true}) {
    for (const bool with_call : {false, true}) {
      for (const unsigned int count : {2U, 3U, 4U, larger}) {
        lead_a1::SharcMemory memory;
        four_program(memory, with_call);
        if (!reg) { code(memory, base, immediate_do | (std::uint64_t{count} << count_shift) | 4); }
        lead_a1::SharcControl control(true);
        lead_a1::Sharc cpu(memory, initial(count), &control);
        lead_a1::SharcFetch fetch(memory, sequenced());
        finish(fetch, cpu, {.length = 4, .sum = 4 * count});
      }
    }
  }
}

void four_phase_failures() {
  lead_a1::SharcMemory memory;
  four_program(memory, true);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(2), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int hold_clock = 5;
  for (unsigned int clock = 0; clock < hold_clock; ++clock) { tick(fetch, cpu); }
  const auto before = fetch.state();
  for (unsigned int variant = 0; variant < 5; ++variant) {
    auto invalid = before;
    if (!invalid.flow || !invalid.flow->long_loop || !invalid.flow->long_loop->counter ||
        !invalid.flow->long_loop->held) {
      throw std::logic_error("Missing four-loop hold");
    }
    auto &loop = *invalid.flow->long_loop;
    switch (variant) {
      case 0: loop.held.reset(); break;
      case 1: loop.held->address = base + 3; break;
      case 2: loop.four = lead_a1::SharcFetchFourPhase::none; break;
      case 3: loop.refetch_end = true; break;
      case 4: loop.counter->remaining = 1; break;
      default: throw std::logic_error("Invalid four-loop test");
    }
    rejected([&] { const lead_a1::SharcFetch broken(memory, invalid); }, "four-loop");
  }
  finish(fetch, cpu, {.length = 4, .sum = 8});
}

void four_failed_end_store() {
  lead_a1::SharcMemory memory;
  four_program(memory, true);
  constexpr std::uint32_t data = 0x80000;
  constexpr std::uint32_t payload = 0x12345678;
  constexpr std::uint64_t store = 0x64fe08800000;
  constexpr std::uint32_t full32 = 0xffffffff;
  memory.map({.bank = 0, .halfwords = 2, .name = "owned four-loop first store"});
  code(memory, base + 4, store);
  auto state = initial(2);
  state.data[0][1] = {.value = std::uint64_t{payload} << 8, .known = full40};
  state.dag[0][0][2] = {.value = data, .known = full32};
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int limit = 256;
  bool reached{};
  for (unsigned int clock = 0; clock < limit && !reached; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    const auto executing = fetch.state().execute;
    const auto loop = cpu.state().count_loop;
    reached = executing && executing->address == base + 4 && loop && loop->remaining == 1;
    if (executing && !reached) { require(cpu.step(fetch).minimum_core_cycles == 1); }
  }
  const auto native = cpu.state();
  const auto before = fetch.state();
  const auto controller = control.snapshot();
  require(reached && native.count_loop && !native.count_loop->frame_live && native.depth == 1);
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "Unmapped SHARC physical");
  require(cpu.state() == native && fetch.state() == before && !memory.active());
  require(control.snapshot() == controller && memory.read({.address = data}) == payload);
  memory.map({.bank = 0, .first_halfword = 2, .halfwords = 2, .name = "repaired four-loop end"});
  require(cpu.step(fetch).minimum_core_cycles == 1);
  finish(fetch, cpu, {.length = 4, .sum = 6});
  require(memory.read({.address = data}) == payload &&
          memory.read({.address = data + 1}) == payload);
  require(cpu.state().dag[0][0][2].value == data + 2);
}

void four_full_counter_prefix() {
  constexpr unsigned int large = 65536;
  constexpr std::uint64_t full_count = std::uint64_t{1} << 32;
  for (const bool reg : {false, true}) {
    for (const unsigned int count : {0U, large}) {
      if (!reg && count != 0) { continue; }
      lead_a1::SharcMemory memory;
      four_program(memory, false);
      if (!reg) { code(memory, base, immediate_do | 4); }
      lead_a1::SharcControl control(true);
      lead_a1::Sharc cpu(memory, initial(count), &control);
      lead_a1::SharcFetch fetch(memory, sequenced());
      constexpr unsigned int clocks = 14;
      for (unsigned int clock = 0; clock < clocks; ++clock) { tick(fetch, cpu); }
      const auto native = cpu.state().count_loop;
      require(native && native->total == (count == 0 ? full_count : count));
      require(native && native->remaining == native->total - 2 && native->frame_live);
      require(cpu.state().pc == base + 1 && cpu.state().data[0][7].value == 8ULL * 256);
    }
  }
}

void four_deferred_dag_capture() {
  lead_a1::SharcMemory memory;
  four_program(memory, false);
  constexpr std::uint64_t store = 0x64fe08800000;
  constexpr std::uint32_t data = 0x80000;
  constexpr std::uint32_t payload = 0x12345678;
  constexpr std::uint32_t full32 = 0xffffffff;
  code(memory, base + 2, store);
  memory.map({.bank = 0, .halfwords = 4, .name = "owned released Decode stores"});
  auto state = initial(2);
  state.data[0][1] = {.value = std::uint64_t{payload} << 8, .known = full40};
  state.dag[0][0][2] = {}; // No address may be read by the aborted Decode.
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int hold_clock = 5;
  for (unsigned int clock = 0; clock < hold_clock; ++clock) { tick(fetch, cpu); }
  require(!fetch.state().decode && cpu.state().dag[0][0][2].known == 0);
  // Rebind with an independently supplied architectural address before release.
  // The raw held word must acquire this value, not a cached/fabricated address.
  auto restored_state = cpu.state();
  restored_state.dag[0][0][2] = {.value = data, .known = full32};
  lead_a1::Sharc restored(memory, restored_state, &control);
  lead_a1::SharcFetch transport(memory, fetch.state());
  require(transport.advance_sequenced(restored).loop_stall);
  const auto decoded = transport.state().decode;
  require(decoded && decoded->dag && decoded->dag->address == data);
  require(restored.step(transport).minimum_core_cycles == 1);
  finish(transport, restored, {.length = 4, .sum = 6});
  require(memory.read({.address = data}) == payload &&
          memory.read({.address = data + 1}) == payload);
  require(restored.state().dag[0][0][2].value == data + 2);
}

void four_return_stall_columns() {
  lead_a1::SharcMemory memory;
  four_program(memory, true);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(2), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int limit = 256;
  bool reached{};
  for (unsigned int clock = 0; clock < limit && !reached; ++clock) {
    tick(fetch, cpu);
    const auto word = fetch.state().address;
    reached = word && word->address == subroutine + 4;
  }
  require(reached && cpu.state().depth == 3 && cpu.state().pc == subroutine + 4);
  const auto previous = fetch.state();
  const auto before = stages(fetch);
  require(fetch.advance_sequenced(cpu).loop_stall);
  const auto held = stages(fetch);
  require(held[0] == before[0] && held[1] == before[1] && held[2] == before[2]);
  require(!held[3] && held[4] == before[3]); // Actual RTS advances; F1/F2/Decode hold.
  require(cpu.step(fetch).minimum_core_cycles == 1);
  require(cpu.state().depth == 2 && cpu.state().pc == base + 4);
  const auto event = fetch.advance_sequenced(cpu);
  require(event.address_accepted && !event.stalled && fetch.state().fetch1 == base + 4);
  const auto flow = fetch.state().flow;
  require(flow && flow->long_loop && !flow->long_loop->refetch_end);
  require(previous.flow && previous.flow->long_loop && previous.flow->long_loop->counter);
  require(flow && flow->long_loop && flow->long_loop->counter && previous.flow &&
          previous.flow->long_loop && previous.flow->long_loop->counter &&
          flow->long_loop->counter->remaining == previous.flow->long_loop->counter->remaining);
  finish(fetch, cpu, {.length = 4, .sum = 8});
}

}

int main() try {
  manual_iteration_and_termination();
  counts_and_lengths();
  late_count_producer();
  calls_and_snapshots();
  zero_and_full_register_counts();
  malformed_counter_and_ownership();
  quiet_ownership();
  captured_word_and_changed_ram();
  failed_store_before_final_pop();
  unsupported_body_control();
  four_startup_and_termination();
  four_calls_and_reentry();
  four_count_matrix();
  four_phase_failures();
  four_failed_end_store();
  four_full_counter_prefix();
  four_deferred_dag_capture();
  four_return_stall_columns();
  std::cout << "SHARC fetched long counter loops passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
