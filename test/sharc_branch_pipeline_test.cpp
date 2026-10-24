// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
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
constexpr std::uint32_t target = base + 16;
constexpr std::uint32_t frame = 0x1000000 | target;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t simd = 0x200000;
constexpr std::uint64_t jump = 0x060000000000;
constexpr std::uint64_t call = 0x068000000000;
constexpr std::uint64_t rts = 0x0a0000000000;
constexpr std::uint64_t always = 0x3e00000000;
constexpr std::uint64_t delayed = 0x4000000;
constexpr std::uint64_t not_zero = 0x2000000000;
constexpr std::uint64_t decrement = 0x013e0002a000;
constexpr std::uint64_t increment = 0x013e00029000;
constexpr std::uint64_t increment_r1 = 0x013e00029110;
constexpr std::uint64_t increment_r2 = 0x013e00029220;
constexpr std::uint64_t increment_r3 = 0x013e00029330;
constexpr std::uint64_t increment_r4 = 0x013e00029440;
constexpr std::uint64_t increment_r5 = 0x013e00029550;
constexpr std::uint64_t invalid = 0xffffffffffff;
using Address = std::optional<std::uint32_t>;
using Stages = std::array<Address, 5>;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC branch pipeline expectation at line " +
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
  throw std::runtime_error("Missing SHARC branch pipeline rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t pc, std::uint64_t opcode) {
  memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 256;
  constexpr unsigned int words = 64;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned conditional branch program"});
  for (unsigned int index = 0; index < words; ++index) { code(memory, base + index, 0); }
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.depth = 1;
  state.pc_stack[0] = frame;
  for (unsigned int reg = 0; reg < 6; ++reg) {
    state.data[0][reg] = {.value = std::uint64_t{10} * 256, .known = full40};
  }
  state.system[0] = {.value = 1, .known = full32};
  state.effective = state.system;
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

std::optional<lead_a1::SharcStep> retire(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu) {
  if (fetch.state().execute) { return cpu.step(fetch); }
  return {};
}

lead_a1::SharcFetchBranch decision(const lead_a1::SharcFetch &fetch) {
  const auto word = fetch.state().address;
  if (!word || !word->branch) { throw std::runtime_error("Missing owned Address branch decision"); }
  return *word->branch;
}

void warmup(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu, unsigned int clocks = 4) {
  for (unsigned int clock = 0; clock < clocks; ++clock) {
    require(fetch.advance_sequenced(cpu).address_accepted);
    require(!retire(fetch, cpu));
  }
}

void condition_case(std::uint64_t kind,
                    unsigned int condition,
                    std::array<std::uint32_t, 2> flags,
                    bool both,
                    bool taken) {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr unsigned int condition_shift = 33;
  const auto opcode =
      kind | (std::uint64_t{condition} << condition_shift) | (kind == rts ? 0 : target);
  code(memory, base, opcode);
  auto state = initial();
  state.system[2].value = both ? simd : 0;
  state.system[6].value = flags[0];
  state.system[7].value = flags[1];
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  warmup(fetch, cpu);
  require(decision(fetch).taken == taken);
  require(decision(fetch).target == (taken ? Address{target} : Address{}));
  require(fetch.advance_sequenced(cpu).address_accepted);
  const auto step = cpu.step(fetch);
  require(step.opcode == opcode && step.pc == base);
  require(cpu.state().pc == (taken ? target : base + 1));
  unsigned int depth = 1;
  if (taken && kind == rts) { depth = 0; }
  if (taken && kind == call) { depth = 2; }
  require(cpu.state().depth == depth);
  if (taken && kind == call) { require(cpu.state().pc_stack[1] == (0x1000000 | (base + 1))); }
}

struct Condition {
  unsigned int code;
  std::uint32_t bit;
};

void condition_pattern(Condition item, unsigned int pattern) {
  const bool complement = (pattern & 1) != 0;
  const bool both = (pattern & 2) != 0;
  const bool x = (pattern & 4) != 0;
  const bool y = (pattern & 8) != 0;
  const bool taken = (x != complement) && (!both || (y != complement));
  for (const auto kind : {jump, call, rts}) {
    condition_case(kind,
                   item.code + (complement ? 16U : 0U),
                   {x ? item.bit : 0, y ? item.bit : 0},
                   both,
                   taken);
  }
}

void condition_matrix() {
  constexpr std::array<Condition, 9> conditions{{
      {.code = 0, .bit = 1},
      {.code = 3, .bit = 8},
      {.code = 4, .bit = 2},
      {.code = 5, .bit = 0x80},
      {.code = 6, .bit = 0x40},
      {.code = 7, .bit = 0x800},
      {.code = 8, .bit = 0x1000},
      {.code = 13, .bit = 0x40000},
      {.code = 14, .bit = 0x4000},
  }};
  for (const auto item : conditions) {
    for (unsigned int pattern = 0; pattern < 16; ++pattern) { condition_pattern(item, pattern); }
  }
}

void retained_condition_decisions() {
  for (const auto kind : {jump, call, rts}) {
    for (const bool taken : {false, true}) {
      lead_a1::SharcMemory memory;
      program(memory);
      const auto opcode = kind | not_zero | (kind == rts ? 0 : target);
      code(memory, base, opcode);
      auto state = initial();
      state.system[6].value = taken ? 0U : 1U;
      state.effective = state.system;
      lead_a1::Sharc cpu(memory, state);
      lead_a1::SharcFetch fetch(memory, sequenced());
      warmup(fetch, cpu);
      require(decision(fetch).taken == taken);
      auto changed = cpu.state();
      changed.system[6] = {};
      changed.system[7] = {};
      changed.effective = changed.system;
      lead_a1::Sharc restored_cpu(memory, changed);
      lead_a1::SharcFetch restored(memory, fetch.state());
      require(restored.advance_sequenced(restored_cpu).address_accepted);
      const auto step = restored_cpu.step(restored);
      require(step.opcode == opcode && step.next_pc == (taken ? target : base + 1));
      require(restored_cpu.state().effective[6].known == 0);
    }
  }
}

Address address(const std::optional<lead_a1::SharcFetchWord> &word) {
  return word ? Address{word->address} : Address{};
}

Stages stages(const lead_a1::SharcFetch &fetch) {
  const auto &state = fetch.state();
  return {state.fetch1,
          address(state.fetch2),
          address(state.decode),
          address(state.address),
          address(state.execute)};
}

std::array<Stages, 8> stall_oracle() {
  const Address bubble;
  return {{{base + 3, base + 2, base + 1, base, bubble},
           {base + 3, base + 2, base + 1, bubble, base},
           {base + 4, base + 3, bubble, base + 1, bubble},
           {target, base + 4, bubble, bubble, base + 1},
           {target + 1, target, bubble, bubble, bubble},
           {target + 2, target + 1, target, bubble, bubble},
           {target + 3, target + 2, target + 1, target, bubble},
           {target + 4, target + 3, target + 2, target + 1, target}}};
}

void continue_return_checkpoint(lead_a1::SharcMemory &memory,
                                const lead_a1::SharcState &cpu_state,
                                const lead_a1::SharcFetchState &fetch_state) {
  lead_a1::Sharc cpu(memory, cpu_state);
  lead_a1::SharcFetch fetch(memory, fetch_state);
  static_cast<void>(retire(fetch, cpu));
  constexpr unsigned int limit = 12;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != target + 1; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    static_cast<void>(retire(fetch, cpu));
  }
  require(cpu.state().pc == target + 1 && cpu.state().depth == 0);
  require(cpu.state().data[0][0].value == 256);
  require(cpu.state().data[0][1].value == std::uint64_t{11} * 256);
}

void literal_return_stall() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, decrement);
  code(memory, base + 1, rts | not_zero);
  for (unsigned int index = 2; index < 5; ++index) { code(memory, base + index, invalid); }
  code(memory, target, increment_r1);
  auto state = initial();
  state.data[0][0].value = std::uint64_t{2} * 256;
  state.system[6].value = 1; // The old AZ would incorrectly reject NE.
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  warmup(fetch, cpu, 3);
  const auto oracle = stall_oracle();
  unsigned int retired = 0;
  for (unsigned int cycle = 0; cycle < oracle.size(); ++cycle) {
    const auto event = fetch.advance_sequenced(cpu);
    require(event.control_stall == (cycle == 1) && event.stalled == (cycle == 1));
    require(event.address_accepted == (cycle != 1));
    require(stages(fetch) == oracle[cycle]);
    continue_return_checkpoint(memory, cpu.state(), fetch.state());
    if (retire(fetch, cpu)) { ++retired; }
  }
  require(retired == 3 && cpu.state().depth == 0 && cpu.state().pc == target + 1);
  require(cpu.state().data[0][0].value == 256);
  require(cpu.state().data[0][1].value == std::uint64_t{11} * 256);
}

void real_countdown_loop() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, decrement);
  code(memory, base + 1, jump | not_zero | base);
  code(memory, base + 2, increment_r1);
  auto state = initial();
  state.data[0][0].value = std::uint64_t{3} * 256;
  state.system[6].value = 1;
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr std::array<std::uint32_t, 7> pcs{
      base, base + 1, base, base + 1, base, base + 1, base + 2};
  unsigned int retired = 0;
  unsigned int stalls = 0;
  constexpr unsigned int limit = 48;
  for (unsigned int clock = 0; clock < limit && retired < pcs.size(); ++clock) {
    const auto event = fetch.advance_sequenced(cpu);
    if (event.control_stall) { ++stalls; }
    const auto step = retire(fetch, cpu);
    if (step) {
      require(step->pc == pcs[retired]);
      ++retired;
    }
  }
  require(retired == pcs.size() && stalls == 3 && cpu.state().pc == base + 3);
  require(cpu.state().data[0][0].value == 0);
  require(cpu.state().data[0][1].value == std::uint64_t{11} * 256);
}

void nested_real_calls_and_returns() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint32_t nested = target + 16;
  code(memory, base, call | always | delayed | target);
  code(memory, base + 1, increment_r1);
  code(memory, base + 2, increment_r2);
  code(memory, base + 3, increment_r3);
  code(memory, target, call | always | nested);
  code(memory, target + 1, rts | always);
  code(memory, nested, increment);
  code(memory, nested + 1, rts | always | delayed);
  code(memory, nested + 2, increment_r4);
  code(memory, nested + 3, increment_r5);
  code(memory, nested + 4, invalid);
  auto state = initial();
  state.depth = 0;
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int retired = 0;
  constexpr unsigned int limit = 64;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != base + 4; ++clock) {
    require(fetch.advance_sequenced(cpu).address_accepted);
    if (retire(fetch, cpu)) { ++retired; }
  }
  require(retired == 10 && cpu.state().pc == base + 4 && cpu.state().depth == 0);
  require(!cpu.state().branch);
  for (unsigned int reg = 0; reg < 6; ++reg) {
    require(cpu.state().data[0][reg].value == std::uint64_t{11} * 256);
  }
}

void precise_flag_dependencies() {
  struct Dependency {
    std::uint64_t producer;
    unsigned int condition;
    bool stall;
  };

  constexpr std::uint64_t immediate_shift = 0x023e00000031;
  constexpr std::uint64_t bit_test = 0x148000000001;
  constexpr std::uint64_t parallel_pass = 0x707e01021430;
  constexpr std::array<Dependency, 8> cases{{
      {.producer = decrement, .condition = 0, .stall = true},
      {.producer = decrement, .condition = 13, .stall = false},
      {.producer = immediate_shift, .condition = 8, .stall = true},
      {.producer = immediate_shift, .condition = 0, .stall = false},
      {.producer = bit_test, .condition = 13, .stall = true},
      {.producer = bit_test, .condition = 0, .stall = false},
      {.producer = parallel_pass, .condition = 0, .stall = true},
      {.producer = decrement, .condition = 31, .stall = false},
  }};
  for (const auto item : cases) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, base, item.producer);
    constexpr unsigned int condition_shift = 33;
    code(memory, base + 1, jump | (std::uint64_t{item.condition} << condition_shift) | target);
    lead_a1::Sharc cpu(memory, initial());
    lead_a1::SharcFetch fetch(memory, sequenced());
    warmup(fetch, cpu);
    const auto event = fetch.advance_sequenced(cpu);
    require(event.control_stall == item.stall && event.stalled == item.stall);
    const auto step = cpu.step(fetch);
    require(step.pc == base && step.opcode == item.producer);
    if (item.stall) { require(fetch.advance_sequenced(cpu).address_accepted); }
    const auto addressed = fetch.state().address;
    require(addressed && addressed->address == base + 1);
  }
}

void false_delay_branch_and_unknown_return() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, jump | not_zero | delayed | target);
  code(memory, base + 1, jump | delayed | (target + 16)); // EQ is false in this slot.
  code(memory, base + 2, increment_r1);
  code(memory, base + 3, invalid);
  code(memory, target, increment_r2);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int retired = 0;
  constexpr unsigned int limit = 24;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != target + 1; ++clock) {
    require(fetch.advance_sequenced(cpu).address_accepted);
    if (retire(fetch, cpu)) { ++retired; }
  }
  require(retired == 4 && cpu.state().pc == target + 1 && !cpu.state().branch);
  require(cpu.state().data[0][1].value == std::uint64_t{11} * 256);
  require(cpu.state().data[0][2].value == std::uint64_t{11} * 256);
  code(memory, base, rts | not_zero);
  auto unknown = initial();
  unknown.depth.reset();
  unknown.pc_stack[0].reset();
  unknown.system[6].value = 1;
  unknown.effective = unknown.system;
  lead_a1::Sharc false_return(memory, unknown);
  lead_a1::SharcFetch return_fetch(memory, sequenced());
  warmup(return_fetch, false_return);
  require(!decision(return_fetch).taken && !decision(return_fetch).target);
  require(return_fetch.advance_sequenced(false_return).address_accepted);
  static_cast<void>(false_return.step(return_fetch));
  require(false_return.state().pc == base + 1 && !false_return.state().depth);
}

void changed_return_frame_and_retry() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, rts | always);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  warmup(fetch, cpu);
  auto altered = cpu.state();
  altered.pc_stack[0] = frame + 1;
  lead_a1::Sharc changed(memory, altered);
  require(fetch.advance_sequenced(changed).address_accepted);
  const auto before = fetch.state();
  const auto old_cpu = changed.state();
  rejected([&] { static_cast<void>(changed.step(fetch)); }, "return frame changed before Execute");
  require(fetch.state() == before && changed.state() == old_cpu && !memory.active());
  lead_a1::Sharc repaired(memory, cpu.state());
  const auto step = repaired.step(fetch);
  require(step.opcode == (rts | always) && step.next_pc == target);
  require(repaired.state().depth == 0 && !fetch.state().execute);
}

void malformed_decision_snapshots() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, jump | always | target);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  warmup(fetch, cpu);
  const auto original = fetch.state();
  auto malformed = original;
  const auto word = original.address;
  if (!word || !word->branch) { throw std::runtime_error("Missing owned branch restore fixture"); }
  auto changed = *word;
  changed.branch = lead_a1::SharcFetchBranch{.taken = true, .target = target + 1};
  malformed.address = changed;
  rejected([&] { const lead_a1::SharcFetch invalid_fetch(memory, malformed); },
           "inherited direct target");
  changed.branch = lead_a1::SharcFetchBranch{.taken = false};
  malformed.address = changed;
  rejected([&] { const lead_a1::SharcFetch invalid_fetch(memory, malformed); },
           "False SHARC unconditional");
  changed.branch.reset();
  malformed.address = changed;
  rejected([&] { const lead_a1::SharcFetch invalid_fetch(memory, malformed); },
           "Missing SHARC Address-stage");
  malformed = original;
  malformed.flow.reset();
  rejected([&] { const lead_a1::SharcFetch invalid_fetch(memory, malformed); },
           "inherited branch decision");
  malformed = original;
  changed = *word;
  changed.pm_data.reset();
  malformed.decode = changed;
  rejected([&] { const lead_a1::SharcFetch invalid_fetch(memory, malformed); },
           "decision before Address");
}

void guarded_interlock_boundaries() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t set_mode = 0x0f7200000000;
  constexpr std::uint64_t write_stack = 0x0f6400000000;
  constexpr std::array<std::uint64_t, 2> producers{set_mode, write_stack | frame};
  for (const auto producer : producers) {
    auto state = initial();
    auto transport = sequenced();
    transport.address = lead_a1::SharcFetchWord{.address = base, .opcode = producer};
    const bool stack = producer != set_mode;
    transport.decode = lead_a1::SharcFetchWord{
        .address = base + 1, .opcode = stack ? rts | always : jump | not_zero | target};
    const char *reason = stack ? "PCSTK write interlock" : "MODE1 Decode interlock";
    lead_a1::Sharc cpu(memory, state);
    lead_a1::SharcFetch fetch(memory, transport);
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, reason);
    require(cpu.state() == before && fetch.state() == transport && !memory.active());
  }
}

void actual_pm_and_control_collision() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t dual_move_pass = 0x200000021000;
  code(memory, base, dual_move_pass);
  code(memory, base + 1, jump | not_zero | target);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  warmup(fetch, cpu, 3);
  const auto request = fetch.advance_sequenced(cpu, lead_a1::SharcFetchBus::pm_data);
  require(request.address_accepted && !request.stalled);
  const auto addressed = fetch.state().address;
  const auto lookup = fetch.state().lookup;
  require(addressed && addressed->opcode == dual_move_pass && addressed->pm_data == true);
  require(lookup && lookup->wait);
  const auto before = fetch.state();
  const auto old_cpu = cpu.state();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); },
           "simultaneous cache/control stalls");
  require(fetch.state() == before && cpu.state() == old_cpu && !memory.active());
}

void failed_producer_retains_branch() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, decrement);
  code(memory, base + 1, rts | not_zero);
  auto state = initial();
  state.data[0][0] = {};
  state.system[6].value = 1;
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  warmup(fetch, cpu);
  require(fetch.advance_sequenced(cpu).control_stall);
  const auto before = fetch.state();
  const auto old_cpu = cpu.state();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "Unknown SHARC register operand bits");
  require(cpu.state() == old_cpu && fetch.state() == before && !memory.active());
  auto repaired_state = old_cpu;
  repaired_state.data[0][0] = {.value = std::uint64_t{2} * 256, .known = full40};
  lead_a1::Sharc repaired(memory, repaired_state);
  require(repaired.step(fetch).opcode == decrement);
  require(fetch.advance_sequenced(repaired).address_accepted);
  require(decision(fetch).taken && decision(fetch).target == target);
  require(repaired.state().data[0][0].value == 256);
}

}

int main() {
  try {
    condition_matrix();
    retained_condition_decisions();
    literal_return_stall();
    real_countdown_loop();
    nested_real_calls_and_returns();
    precise_flag_dependencies();
    false_delay_branch_and_unknown_return();
    changed_return_frame_and_retry();
    malformed_decision_snapshots();
    guarded_interlock_boundaries();
    actual_pm_and_control_collision();
    failed_producer_retains_branch();
    std::cout << "SHARC Address decisions, condition interlocks and native returns passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
