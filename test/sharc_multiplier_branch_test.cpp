// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

using lead_a1::Sharc;
using lead_a1::SharcFetch;
using lead_a1::SharcFetchState;
using lead_a1::SharcMemory;
constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t target = base + 32;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t simd = 0x200000;
constexpr std::uint32_t old_positive = 0x81;
constexpr std::uint32_t old_negative = 0xc1;
constexpr std::uint32_t cache_controls = 0x80010;
constexpr unsigned int condition_shift = 33;
constexpr std::uint64_t sign = 0x8000000000;
constexpr std::uint64_t one = 0x3f80000000;
constexpr std::uint64_t one_half = 0x3fc0000000;
constexpr std::uint64_t two_half = 0x4020000000;
constexpr std::uint64_t product = 0x4070000000;
constexpr std::uint64_t multiply = 0x013e00130312;
constexpr std::uint64_t parallel = 0x707e01130312; // F3=F1*F2, R2=old R4.
constexpr std::uint64_t jump = 0x060000000000;
constexpr std::uint64_t relative = 0x070000000000;
constexpr std::uint64_t call = 0x068000000000;
constexpr std::uint64_t rts = 0x0a0000000000;
constexpr std::uint64_t delayed = 0x4000000;
constexpr std::uint64_t increment_r6 = 0x013e00029660;
constexpr std::uint64_t increment_r7 = 0x013e00029770;
constexpr std::uint64_t invalid = 0xffffffffffff;
constexpr unsigned int limit = 64;
constexpr std::array<unsigned int, 4> conditions{5, 6, 21, 22}; // MV, MS, NOT MV, NOT MS.
using Stages = std::array<std::optional<std::uint32_t>, 5>;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC multiplier branch expectation at line " +
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
  throw std::runtime_error("Missing SHARC multiplier branch rejection: " + reason);
}

void code(SharcMemory &memory, std::uint32_t pc, std::uint64_t opcode) {
  memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(SharcMemory &memory) {
  constexpr unsigned int cells = 256;
  constexpr unsigned int words = 64;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned multiplier branch program"});
  for (unsigned int index = 0; index < words; ++index) { code(memory, base + index, 0); }
  code(memory, base, multiply);
  constexpr std::uint64_t if_ms = 0x060c00000000;
  code(memory, base + 1, if_ms | target);
}

lead_a1::SharcState initial() {
  auto state = Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t disabled_cache = 0x10;
  state.system[4].value |= disabled_cache;
  state.system[6].value =
      old_positive; // Old MV=1, MN=0, AZ=1; result changes only multiplier flags.
  state.effective = state.system;
  state.depth = 1;
  state.pc_stack[0] = 0x1000000 | target;
  state.data[0][1] = {.value = sign | one_half, .known = full40};
  state.data[0][2] = {.value = two_half, .known = full40};
  state.data[0][4] = {.value = one, .known = full40};
  return state;
}

SharcFetchState sequenced() {
  SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

void retire(Sharc &cpu, SharcFetch &fetch) {
  if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
}

Stages stages(const SharcFetch &fetch) {
  const auto address = [](const auto &word) {
    return word ? std::optional{word->address} : std::nullopt;
  };
  const auto &state = fetch.state();
  return {state.fetch1,
          address(state.fetch2),
          address(state.decode),
          address(state.address),
          address(state.execute)};
}

constexpr std::array<Stages, 9> oracle() {
  constexpr auto bubble = std::nullopt;
  // PGR4-116 case2: Table4-55's ordinary condition hold plus one further hold.
  return {{{base + 3, base + 2, base + 1, base, bubble},
           {base + 3, base + 2, base + 1, bubble, base},
           {base + 3, base + 2, base + 1, bubble, bubble},
           {base + 4, base + 3, bubble, base + 1, bubble},
           {target, base + 4, bubble, bubble, base + 1},
           {target + 1, target, bubble, bubble, bubble},
           {target + 2, target + 1, target, bubble, bubble},
           {target + 3, target + 2, target + 1, target, bubble},
           {target + 4, target + 3, target + 2, target + 1, target}}};
}

void check_clock(Sharc &cpu, SharcFetch &fetch, unsigned int cycle) {
  const auto event = fetch.advance_sequenced(cpu);
  const bool held = cycle == 1 || cycle == 2;
  require(event.control_stall == held && event.stalled == held);
  require(event.address_accepted != held);
  require(stages(fetch) == oracle()[cycle]);
  const auto flow = fetch.state().flow;
  require(flow && flow->multiplier_branch_wait == (cycle == 1));
}

void continuation(const lead_a1::SharcState &cpu_state,
                  const SharcFetchState &fetch_state,
                  unsigned int next_cycle) {
  SharcMemory memory;
  program(memory);
  // Both words were already captured: continuation must not reread either.
  code(memory, base, invalid);
  code(memory, base + 1, invalid);
  Sharc cpu(memory, cpu_state);
  SharcFetch fetch(memory, fetch_state);
  retire(cpu, fetch);
  for (unsigned int cycle = next_cycle; cycle < oracle().size(); ++cycle) {
    check_clock(cpu, fetch, cycle);
    retire(cpu, fetch);
  }
  require(cpu.state().pc == target + 1 && cpu.state().depth == 1);
  require(cpu.state().data[0][3] ==
          (lead_a1::SharcKnown40{.value = sign | product, .known = full40}));
}

void warmup(Sharc &cpu, SharcFetch &fetch) {
  for (unsigned int clock = 0; clock < 4; ++clock) {
    require(fetch.advance_sequenced(cpu).address_accepted);
    require(!fetch.state().execute);
  }
}

void literal_phases_and_checkpoints() {
  SharcMemory memory;
  program(memory);
  Sharc cpu(memory, initial());
  SharcFetch fetch(memory, sequenced());
  for (unsigned int clock = 0; clock < 3; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
  }
  for (unsigned int cycle = 0; cycle < oracle().size(); ++cycle) {
    check_clock(cpu, fetch, cycle);
    continuation(cpu.state(), fetch.state(), cycle + 1);
    retire(cpu, fetch);
    continuation(cpu.state(), fetch.state(), cycle + 1);
  }
  require(cpu.state().pc == target + 1 && cpu.state().data[0][3].value == (sign | product));
}

enum class Producer : std::uint8_t { scalar, simd_mode, parallel_move };
enum class BranchKind : std::uint8_t { absolute, relative, call, returning };

struct BranchCase {
  Producer producer{};
  BranchKind kind{};
  unsigned int gap{};
  unsigned int condition{};
  unsigned int signs{};
  bool db{};
};

lead_a1::SharcState case_state(BranchCase test) {
  auto state = initial();
  state.system[2].value = test.producer == Producer::simd_mode ? simd : 0;
  for (unsigned int element = 0; element < 2; ++element) {
    const bool negative = (test.signs & (1U << element)) != 0;
    state.system[6 + element].value = negative ? old_positive : old_negative;
    const auto bank = element * 2;
    state.data[bank][1] = {.value = one_half | (negative ? sign : 0), .known = full40};
    state.data[bank][2] = {.value = two_half, .known = full40};
    state.data[bank][6] = {.known = full40};
    state.data[bank][7] = {.known = full40};
  }
  state.effective = state.system;
  return state;
}

bool expected(BranchCase test, unsigned int element) {
  constexpr std::array<unsigned int, 2> truth{12, 6}; // Positive/negative: MV,MS,NOT MV,NOT MS.
  const auto negative = (test.signs >> element) & 1;
  return ((truth[negative] >> test.condition) & 1) != 0;
}

void case_program(SharcMemory &memory, BranchCase test) {
  program(memory);
  code(memory, base, test.producer == Producer::parallel_move ? parallel : multiply);
  code(memory, base + 1, 0);
  const auto branch_at = base + test.gap + 1;
  constexpr std::array<std::uint64_t, 4> kinds{jump, relative, call, rts};
  auto instruction = kinds[static_cast<unsigned int>(test.kind)];
  instruction |= std::uint64_t{conditions[test.condition]} << condition_shift;
  if (test.kind != BranchKind::returning) {
    instruction |= test.kind == BranchKind::relative ? target - branch_at : target;
  }
  if (test.db) { instruction |= delayed; }
  code(memory, branch_at, instruction);
  code(memory, branch_at + 1, increment_r6);
  code(memory, branch_at + 2, increment_r7);
}

void check_case_result(const Sharc &cpu, BranchCase test, bool taken) {
  const bool slots = !taken || test.db;
  const auto elements = test.producer == Producer::simd_mode ? 2U : 1U;
  for (unsigned int element = 0; element < elements; ++element) {
    const auto bank = element * 2;
    const auto sign_bit = ((test.signs >> element) & 1) != 0 ? sign : 0;
    require(cpu.state().data[bank][3] ==
            (lead_a1::SharcKnown40{.value = product | sign_bit, .known = full40}));
    require(cpu.state().data[bank][6].value == (slots ? 256U : 0U));
    require(cpu.state().data[bank][7].value == (slots ? 256U : 0U));
  }
  if (test.producer == Producer::parallel_move) { require(cpu.state().data[0][2].value == one); }
  unsigned int depth = 1;
  if (taken && test.kind == BranchKind::call) { depth = 2; }
  if (taken && test.kind == BranchKind::returning) { depth = 0; }
  require(cpu.state().depth == depth);
  if (depth == 2) {
    const auto return_at = base + test.gap + 1 + (test.db ? 3U : 1U);
    require(cpu.state().pc_stack[1] == (0x1000000 | return_at));
  }
}

void matrix_case(BranchCase test) {
  SharcMemory memory;
  case_program(memory, test);
  Sharc cpu(memory, case_state(test));
  SharcFetch fetch(memory, sequenced());
  const bool taken =
      expected(test, 0) && (test.producer != Producer::simd_mode || expected(test, 1));
  const auto stop = taken ? target + 1 : base + test.gap + 4;
  unsigned int holds{};
  for (unsigned int clock = 0; clock < limit; ++clock) {
    const auto event = fetch.advance_sequenced(cpu);
    if (event.control_stall) { ++holds; }
    retire(cpu, fetch);
    if (cpu.state().pc != stop) { continue; }
    require(holds == (test.gap == 0 ? 2U : 0U));
    check_case_result(cpu, test, taken);
    return;
  }
  throw std::runtime_error("Missing owned multiplier branch matrix frontier");
}

void kind_matrix(BranchCase test) {
  for (const auto kind :
       {BranchKind::absolute, BranchKind::relative, BranchKind::call, BranchKind::returning}) {
    test.kind = kind;
    for (const bool db : {false, true}) {
      test.db = db;
      for (unsigned int signs = 0; signs < 4; ++signs) {
        test.signs = signs;
        for (unsigned int condition = 0; condition < conditions.size(); ++condition) {
          test.condition = condition;
          matrix_case(test);
        }
      }
    }
  }
}

void condition_matrix() {
  for (const auto producer : {Producer::scalar, Producer::simd_mode, Producer::parallel_move}) {
    for (unsigned int gap = 0; gap < 3; ++gap) { kind_matrix({.producer = producer, .gap = gap}); }
  }
}

void zero_case(unsigned int condition, bool both) {
  constexpr std::array<unsigned int, 6> codes{5, 6, 21, 22, 31, 0};
  constexpr unsigned int truth = 60; // MV/MS false; complements/TRUE/retained EQ true.
  SharcMemory memory;
  program(memory);
  code(memory, base + 1, jump | (std::uint64_t{codes[condition]} << condition_shift) | target);
  auto state = initial();
  state.system[2].value = both ? simd : 0;
  for (std::size_t element = 0; element < 2; ++element) {
    state.system[6 + element].value = old_negative;
    state.data[element * 2][1] = {.known = full40};
    state.data[element * 2][2] = {.value = two_half, .known = full40};
  }
  state.effective = state.system;
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, sequenced());
  const bool taken = ((truth >> condition) & 1) != 0;
  const auto stop = taken ? target + 1 : base + 2;
  unsigned int holds{};
  for (unsigned int clock = 0; clock < limit; ++clock) {
    if (fetch.advance_sequenced(cpu).control_stall) { ++holds; }
    retire(cpu, fetch);
    if (cpu.state().pc != stop) { continue; }
    require(holds == (condition < 4 ? 2U : 0U));
    constexpr std::uint32_t flags = 0x3c0;
    for (std::size_t element = 0; element < (both ? 2U : 1U); ++element) {
      require(cpu.state().data[element * 2][3] == (lead_a1::SharcKnown40{.known = full40}));
      require((cpu.state().system[6 + element].value & flags) == 0);
    }
    return;
  }
  throw std::runtime_error("Missing owned positive-zero branch frontier");
}

void positive_zero_and_unrelated_conditions() {
  constexpr unsigned int cases = 6;
  for (unsigned int condition = 0; condition < cases; ++condition) {
    zero_case(condition, false);
    zero_case(condition, true);
  }
}

void failed_producer_and_retry() {
  SharcMemory memory;
  program(memory);
  auto state = initial();
  state.data[0][1] = {};
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, sequenced());
  warmup(cpu, fetch);
  require(fetch.advance_sequenced(cpu).control_stall);
  const auto before_cpu = cpu.state();
  const auto before_fetch = fetch.state();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "Unknown SHARC floating register operand");
  require(cpu.state() == before_cpu && fetch.state() == before_fetch && !memory.active());
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "has not retired");
  require(cpu.state() == before_cpu && fetch.state() == before_fetch);
  state = before_cpu;
  state.data[0][1] = {.value = sign | one_half, .known = full40};
  Sharc repaired(memory, state);
  code(memory, base, invalid);
  require(repaired.step(fetch).opcode == multiply);
  check_clock(repaired, fetch, 2);
  require(repaired.state().pc == base + 1);
  check_clock(repaired, fetch, 3);
  const auto addressed = fetch.state().address;
  require(addressed && addressed->branch && addressed->branch->taken);
}

void quiet_clock_ownership_and_bus() {
  SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  Sharc cpu(memory, initial(), &control);
  SharcFetch fetch(memory, sequenced());
  warmup(cpu, fetch);
  require(fetch.advance_sequenced(cpu).control_stall);
  retire(cpu, fetch);
  const auto before_cpu = cpu.state();
  const auto before_fetch = fetch.state();
  const auto before_control = control.snapshot();
  rejected(
      [&] { static_cast<void>(fetch.advance_sequenced(cpu, lead_a1::SharcFetchBus::pm_data)); },
      "control stall with PM/cache");
  require(cpu.state() == before_cpu && fetch.state() == before_fetch);
  control.begin_instruction();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "during a transaction");
  require(cpu.state() == before_cpu && fetch.state() == before_fetch);
  require(control.state() == before_control);
  control.abort_instruction();
  check_clock(cpu, fetch, 2);
  require(cpu.state() == before_cpu && control.snapshot() == before_control);
  check_clock(cpu, fetch, 3);
}

void retained_wait_needs_no_flags() {
  SharcMemory memory;
  program(memory);
  auto state = initial();
  state.system[6] = {};
  state.effective = state.system;
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, sequenced());
  warmup(cpu, fetch);
  require(fetch.advance_sequenced(cpu).control_stall);
  retire(cpu, fetch);
  state = cpu.state();
  state.system[6] = {};
  state.effective[6] = {};
  Sharc unknown(memory, state);
  check_clock(unknown, fetch, 2); // The second hold does not resolve the branch.
  const auto before_fetch = fetch.state();
  const auto before_cpu = unknown.state();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(unknown)); },
           "Unknown SHARC control/status");
  require(fetch.state() == before_fetch && unknown.state() == before_cpu);
  constexpr std::uint32_t multiplier_flags = 0x3c0;
  constexpr std::uint32_t negative = 0x40;
  state.system[6] = {.value = negative, .known = multiplier_flags};
  state.effective[6] = state.system[6];
  Sharc repaired(memory, state);
  check_clock(repaired, fetch, 3);
}

void false_return_with_unknown_stack() {
  SharcMemory memory;
  program(memory);
  constexpr std::uint64_t if_mv_return = 0x0a0a00000000;
  code(memory, base + 1, if_mv_return);
  auto state = initial();
  state.depth.reset();
  state.pc_stack[0].reset();
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, sequenced());
  unsigned int holds{};
  for (unsigned int clock = 0; clock < limit; ++clock) {
    if (fetch.advance_sequenced(cpu).control_stall) { ++holds; }
    retire(cpu, fetch);
    if (cpu.state().pc != base + 2) { continue; }
    require(holds == 2 && !cpu.state().depth && !cpu.state().pc_stack[0]);
    return;
  }
  throw std::runtime_error("Missing owned false multiplier return frontier");
}

void simultaneous_cache_wait_rejects() {
  SharcMemory memory;
  program(memory);
  constexpr std::uint64_t dual_multiply = 0x200000130312;
  code(memory, base, dual_multiply);
  Sharc cpu(memory, initial());
  SharcFetch fetch(memory, sequenced());
  for (unsigned int clock = 0; clock < 3; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
  }
  require(fetch.advance_sequenced(cpu, lead_a1::SharcFetchBus::pm_data).address_accepted);
  const auto before_cpu = cpu.state();
  const auto before_fetch = fetch.state();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); },
           "simultaneous cache/control stalls");
  require(cpu.state() == before_cpu && fetch.state() == before_fetch && !memory.active());
}

void malformed_wait_states() {
  SharcMemory memory;
  program(memory);
  Sharc cpu(memory, initial());
  SharcFetch fetch(memory, sequenced());
  warmup(cpu, fetch);
  require(fetch.advance_sequenced(cpu).control_stall);
  const auto original = fetch.state();
  const auto bad = [&](SharcFetchState state) {
    rejected([&] { const SharcFetch restored(memory, state); }, "multiplier branch wait");
  };
  auto state = original;
  state.decode.reset();
  bad(state);
  state = original;
  // EQ is not a multiplier condition.
  state.decode = lead_a1::SharcFetchWord{.address = base + 1, .opcode = jump | target};
  bad(state);
  state = original;
  // The retained producer must actually be a multiplier.
  state.execute = lead_a1::SharcFetchWord{.address = base};
  bad(state);
  state = original;
  state.address = lead_a1::SharcFetchWord{.address = base + 2};
  bad(state);
  state = original;
  const auto original_flow = original.flow;
  if (!original_flow) { throw std::runtime_error("Missing owned waiting flow"); }
  auto changed_flow = *original_flow;
  changed_flow.discard_decode = 1;
  state.flow = changed_flow;
  bad(state);
  state = original;
  changed_flow = *original_flow;
  changed_flow.loop = lead_a1::SharcFetchLoop{};
  state.flow = changed_flow;
  bad(state);
  state = original;
  state.lookup = lead_a1::SharcFetchLookup{.mode = {.known = cache_controls}, .wait = true};
  bad(state);
}

}

int main() try {
  literal_phases_and_checkpoints();
  condition_matrix();
  positive_zero_and_unrelated_conditions();
  failed_producer_and_retry();
  quiet_clock_ownership_and_bus();
  retained_wait_needs_no_flags();
  false_return_with_unknown_stack();
  simultaneous_cache_wait_rejects();
  malformed_wait_states();
  std::cout << "SHARC two-cycle multiplier flag branches and retained waits passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
