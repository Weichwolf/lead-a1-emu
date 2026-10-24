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
constexpr std::uint32_t data = 0x80000;
constexpr std::uint32_t target = data + 8;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t payload = 0x12345678;
constexpr std::uint32_t marker = 0xa5a5a5a5;
constexpr std::uint32_t parent = 0x10c0030;
constexpr std::uint64_t store = 0x543e80800000;
constexpr std::uint64_t do_count = 0x0c0000000001;
constexpr unsigned int cells = 256;
using Address = std::optional<std::uint32_t>;
using Stages = std::array<Address, 5>;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC DAG/DO expectation at line " + std::to_string(where.line()));
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
  throw std::runtime_error("Missing DAG/DO rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t pc, std::uint64_t opcode) {
  memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, opcode);
}

struct DataExtent {
  unsigned int halfwords{cells};
};

void program(lead_a1::SharcMemory &memory, unsigned int count, DataExtent extent = {}) {
  constexpr unsigned int instructions = 32;
  constexpr unsigned int counter_shift = 24;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned DAG/DO program"});
  memory.map({.bank = 0, .halfwords = extent.halfwords, .name = "owned DAG/DO data"});
  for (unsigned int index = 0; index < instructions; ++index) { code(memory, base + index, 0); }
  for (unsigned int index = 0; index < extent.halfwords / 2; ++index) {
    memory.write({.address = data + index}, marker);
  }
  constexpr std::uint64_t load_i2 = 0x0f1200000000;
  code(memory, base, load_i2 | target);
  code(memory, base + 1, do_count | (std::uint64_t{count} << counter_shift));
  code(memory, base + 2, store);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.dag[0][0][2] = {.value = data, .known = full32};
  state.dag[0][1][0] = {.value = 1, .known = full32};
  state.data[0][1] = {.value = std::uint64_t{payload} << 8, .known = full40};
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

void retire(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
}

void clock(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  static_cast<void>(fetch.advance_sequenced(cpu));
  retire(cpu, fetch);
}

void result(const lead_a1::SharcMemory &memory, const lead_a1::Sharc &cpu, unsigned int count) {
  require(cpu.state().pc == base + 4 && cpu.state().depth == 1);
  require(cpu.state().pc_stack[0] == parent && !cpu.state().count_loop);
  require(cpu.state().loops_empty == true);
  require(memory.read({.address = target - 1}) == marker);
  require(memory.read({.address = target + count}) == marker);
}

void composed_stage_reference() {
  const Address empty;
  // Compose Table4-52's advancing unrelated DO with4-17's separate startup hold.
  // The body cannot enter Decode until its real DAG producer has retired.
  const std::array<Stages, 13> expected{{{0, empty, empty, empty, empty},
                                         {1, 0, empty, empty, empty},
                                         {2, 1, 0, empty, empty},
                                         {3, 2, 1, 0, empty},
                                         {3, 2, empty, 1, 0},
                                         {2, 3, 2, empty, 1},
                                         {2, 3, 2, empty, empty},
                                         {2, 2, 2, 2, empty},
                                         {2, 2, 2, 2, 2},
                                         {3, 2, 2, 2, 2},
                                         {4, 3, 2, 2, 2},
                                         {5, 4, 3, 2, 2},
                                         {6, 5, 4, 3, 2}}};
  lead_a1::SharcMemory memory;
  constexpr unsigned int count = 5;
  program(memory, count);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int retired{};
  unsigned int writes{};
  for (unsigned int column = 0; column < expected.size(); ++column) {
    const auto event = fetch.advance_sequenced(cpu);
    require(stages(fetch) == expected[column]);
    require(event.dag_stall == (column == 4) && event.loop_stall == (column == 6));
    require(event.stalled == (column == 4 || column == 6));
    require(event.address_accepted != event.stalled);
    const auto executing = fetch.state().execute;
    if (!executing) { continue; }
    if (executing->address == base + 2) {
      require(executing->dag && executing->dag->address == target + writes);
      ++writes;
    }
    static_cast<void>(cpu.step(fetch));
    ++retired;
  }
  require(retired == count + 2 && writes == count && cpu.state().pc == base + 3);
  require(cpu.state().depth == 1 && cpu.state().pc_stack[0] == parent);
  require(!cpu.state().count_loop && cpu.state().loops_empty == true);
  require(cpu.state().dag[0][0][2].value == target + count);
  for (unsigned int index = 0; index < count; ++index) {
    require(memory.read({.address = target + index}) == payload);
  }
  require(memory.read({.address = target - 1}) == marker);
  require(memory.read({.address = target + count}) == marker);
}

struct DagLoad {
  unsigned int reg;
  std::uint32_t value;
  std::uint32_t old_index;
  std::uint32_t old_modifier;
};

void dependency_case(DagLoad load, unsigned int count, bool secondary, bool immediate) {
  lead_a1::SharcMemory memory;
  program(memory, count);
  constexpr unsigned int register_shift = 32;
  constexpr std::uint64_t load_register = 0x0f0000000000;
  constexpr std::uint64_t immediate_store = 0x64fe08800000;
  code(memory, base, load_register | (std::uint64_t{load.reg} << register_shift) | load.value);
  if (immediate) { code(memory, base + 2, immediate_store); }
  auto state = initial();
  const auto bank = secondary ? 1U : 0U;
  state.system[2].value = secondary ? 16U : 0U;
  state.effective = state.system;
  state.dag[bank] = state.dag[0];
  state.dag[bank][0][2] = {.value = load.old_index, .known = full32};
  state.dag[bank][1][0] = {.value = load.old_modifier, .known = full32};
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int dag_holds{};
  unsigned int loop_holds{};
  unsigned int retired{};
  unsigned int writes{};
  unsigned int clocks{};
  const auto limit = count + 24;
  while (cpu.state().pc != base + 4 && clocks < limit) {
    const auto event = fetch.advance_sequenced(cpu);
    ++clocks;
    dag_holds += event.dag_stall ? 1U : 0U;
    loop_holds += event.loop_stall ? 1U : 0U;
    require(event.stalled == (event.dag_stall || event.loop_stall));
    require(!event.control_stall && !event.compute_stall);
    const auto executing = fetch.state().execute;
    if (!executing) { continue; }
    if (executing->address == base + 2) {
      require(executing->dag && executing->dag->address == target + writes);
      require(executing->dag->bank == bank);
      ++writes;
    }
    retire(cpu, fetch);
    ++retired;
  }
  constexpr unsigned int modifier0 = 0x20;
  const bool dependent = load.reg != modifier0 || !immediate;
  constexpr unsigned int normal_overhead = 8;
  constexpr unsigned int short_drain = 3;
  require(clocks ==
          count + normal_overhead + (dependent ? 1U : 0U) + (count < 4 ? short_drain : 0U));
  require(dag_holds == (dependent ? 1U : 0U) && loop_holds == 1);
  const auto flow = fetch.state().flow;
  require(retired == count + 3 && writes == count && flow && !flow->loop);
  result(memory, cpu, count);
  require(cpu.state().dag[bank][0][2].value == target + count);
  for (unsigned int index = 0; index < count; ++index) {
    require(memory.read({.address = target + index}) == payload);
  }
}

void load_count_bank_matrix() {
  constexpr std::array<DagLoad, 4> loads{
      {{.reg = 0x12, .value = target, .old_index = data, .old_modifier = 1},
       {.reg = 0x42, .value = target, .old_index = data, .old_modifier = 1},
       {.reg = 0x32, .value = 0, .old_index = target, .old_modifier = 1},
       {.reg = 0x20, .value = 1, .old_index = target, .old_modifier = 2}}};
  for (const auto load : loads) {
    for (const unsigned int count : {2U, 3U, 4U, 5U, 16U}) {
      for (const bool secondary : {false, true}) {
        for (const bool immediate : {false, true}) {
          dependency_case(load, count, secondary, immediate);
        }
      }
    }
  }
}

void rebound_continuation(unsigned int checkpoint, bool after_execute) {
  constexpr unsigned int count = 5;
  constexpr std::uint32_t replacement = 0x87654321;
  constexpr std::uint64_t replacement_store = 0x543e81000000; // R2, same address update.
  lead_a1::SharcMemory memory;
  lead_a1::SharcMemory other_memory;
  program(memory, count);
  program(other_memory, count);
  auto state = initial();
  state.data[0][2] = {.value = std::uint64_t{replacement} << 8, .known = full40};
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  for (unsigned int time = 1; time < checkpoint; ++time) { clock(cpu, fetch); }
  static_cast<void>(fetch.advance_sequenced(cpu));
  if (after_execute) { retire(cpu, fetch); }
  for (unsigned int index = 0; index < cells / 2; ++index) {
    other_memory.write({.address = data + index}, memory.read({.address = data + index}));
  }
  auto other_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
  lead_a1::Sharc restored(other_memory, cpu.state(), &other_control);
  lead_a1::SharcFetch other(other_memory, fetch.state());
  code(memory, base + 2, replacement_store);
  code(other_memory, base + 2, replacement_store);
  constexpr unsigned int limit = 20;
  for (unsigned int time = 0; cpu.state().pc != base + 4 && time < limit; ++time) {
    retire(cpu, fetch);
    retire(restored, other);
    require(cpu.state() == restored.state() && fetch.state() == other.state());
    require(control.snapshot() == other_control.snapshot());
    if (cpu.state().pc != base + 4) {
      require(fetch.advance_sequenced(cpu) == other.advance_sequenced(restored));
    }
  }
  result(memory, cpu, count);
  result(other_memory, restored, count);
  constexpr unsigned int first_refetch = 8;
  const auto originals = checkpoint < first_refetch ? 2U : 3U;
  for (unsigned int index = 0; index < count; ++index) {
    const auto expected = index < originals ? payload : replacement;
    require(memory.read({.address = target + index}) == expected);
    require(other_memory.read({.address = target + index}) == expected);
  }
}

void restored_delayed_setup() {
  for (const unsigned int checkpoint : {5U, 6U, 7U, 8U}) {
    for (const bool after_execute : {false, true}) {
      rebound_continuation(checkpoint, after_execute);
    }
  }
}

void delayed_setup_ownership_and_restore_guard() {
  constexpr unsigned int count = 5;
  lead_a1::SharcMemory memory;
  program(memory, count);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  for (unsigned int time = 0; time < 6; ++time) { clock(cpu, fetch); }
  const auto before = cpu.state();
  const auto before_fetch = fetch.state();
  const auto before_control = control.snapshot();
  auto invalid = before_fetch;
  invalid.decode.reset();
  rejected([&] { const lead_a1::SharcFetch broken(memory, invalid); },
           "DAG-delayed loop Decode phase");
  invalid = before_fetch;
  if (!invalid.decode) { throw std::logic_error("Missing owned Decode"); }
  constexpr std::uint64_t other_store = 0x543e81000000;
  invalid.decode->opcode = other_store;
  rejected([&] { const lead_a1::SharcFetch broken(memory, invalid); },
           "DAG-delayed loop Decode phase");
  control.begin_instruction();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "during a transaction");
  require(cpu.state() == before && fetch.state() == before_fetch);
  require(control.state() == before_control && !memory.active());
  control.abort_instruction();
  require(fetch.advance_sequenced(cpu).loop_stall);
  constexpr unsigned int limit = 20;
  for (unsigned int time = 0; cpu.state().pc != base + 4 && time < limit; ++time) {
    clock(cpu, fetch);
  }
  result(memory, cpu, count);
  for (unsigned int index = 0; index < count; ++index) {
    require(memory.read({.address = target + index}) == payload);
  }
}

void failed_store_with_pending_pop() {
  constexpr unsigned int count = 5;
  constexpr unsigned int mapped_cells = 18;
  lead_a1::SharcMemory memory;
  program(memory, count, {.halfwords = mapped_cells});
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  for (unsigned int time = 0; time < 9; ++time) { clock(cpu, fetch); }
  require(memory.read({.address = target}) == payload && cpu.state().depth == 2);
  static_cast<void>(fetch.advance_sequenced(cpu));
  const auto executing = fetch.state().execute;
  require(executing && executing->address == base + 2 && executing->loop_pop);
  const auto before = cpu.state();
  const auto before_fetch = fetch.state();
  const auto before_control = control.snapshot();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "Unmapped SHARC physical");
  require(cpu.state() == before && fetch.state() == before_fetch && !memory.active());
  require(control.snapshot() == before_control && memory.read({.address = target}) == payload);
  memory.map({.bank = 0,
              .first_halfword = mapped_cells,
              .halfwords = cells - mapped_cells,
              .name = "repaired owned DAG/DO data"});
  for (unsigned int index = mapped_cells / 2; index < cells / 2; ++index) {
    memory.write({.address = data + index}, marker);
  }
  retire(cpu, fetch);
  require(cpu.state().depth == 1 && cpu.state().loops_empty == true);
  constexpr unsigned int limit = 20;
  for (unsigned int time = 0; cpu.state().pc != base + 4 && time < limit; ++time) {
    clock(cpu, fetch);
  }
  result(memory, cpu, count);
  require(cpu.state().dag[0][0][2].value == target + count);
  for (unsigned int index = 0; index < count; ++index) {
    require(memory.read({.address = target + index}) == payload);
  }
}

}

int main() try {
  composed_stage_reference();
  load_count_bank_matrix();
  restored_delayed_setup();
  delayed_setup_ownership_and_restore_guard();
  failed_store_with_pending_pop();
  std::cout << "SHARC DAG-load/DO pipeline composition passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
