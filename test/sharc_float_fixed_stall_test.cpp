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
using lead_a1::SharcMemory;
constexpr std::uint32_t base = 0xc0000;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t one = 0x3f80000000;
constexpr std::uint64_t two = 0x4000000000;
constexpr std::uint64_t three = 0x4040000000;
constexpr std::uint64_t sign = 0x8000000000;
constexpr std::uint64_t one_half = 0x3fc0000000;
constexpr std::uint64_t two_half = 0x4020000000;
constexpr std::uint64_t product = 0x4070000000;
constexpr std::uint64_t multiply = 0x013e00130312;
constexpr std::uint64_t decrement = 0x013e0002a550;
constexpr std::uint64_t ten = 2560;
constexpr std::uint64_t nine = 2304;
constexpr std::uint32_t simd = 0x200000;
constexpr std::uint32_t history = 0xa5000000;
constexpr std::uint32_t greater_history = 0xd2000000;
constexpr std::uint32_t cacc = 0xff000000;
constexpr std::uint32_t multiplier_flags = 0x3c0;
constexpr std::uint32_t negative_flag = 0x40;
constexpr std::uint64_t sentinel = 0x12345678b7;
constexpr std::uint64_t invalid = 0xffffffffffff;
constexpr std::uint64_t parallel_multiply = 0x707e01130312;
constexpr std::uint64_t parallel_copy = 0x707e01000000;
constexpr unsigned int limit = 64;
constexpr unsigned int class_shift = 40;
using Stages = std::array<std::optional<std::uint32_t>, 5>;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC float/fixed stall expectation at line " +
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
  throw std::runtime_error("Missing SHARC float/fixed rejection: " + reason);
}

lead_a1::SharcKnown40 word(std::uint32_t value) {
  constexpr std::uint64_t extension = 0xb7;
  return {.value = (std::uint64_t{value} << 8) | extension, .known = full40};
}

void code(SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(SharcMemory &memory, std::uint64_t consumer = decrement) {
  constexpr unsigned int cells = 192;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned floating-to-fixed program"});
  for (unsigned int index = 0; index < cells / 3; ++index) { code(memory, base + index, 0); }
  code(memory, base, multiply);
  code(memory, base + 1, consumer);
}

lead_a1::SharcState initial() {
  auto state = Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t disabled_cache = 0x10;
  state.system[4].value |= disabled_cache;
  constexpr std::uint32_t old_flags = 0x3fff;
  state.system[6] = {.value = history | old_flags, .known = full32};
  state.system[7] = state.system[6];
  state.effective = state.system;
  for (auto &bank : state.data) {
    for (auto &reg : bank) { reg = {.value = sentinel, .known = full40}; }
    bank[1] = {.value = one_half, .known = full40};
    bank[2] = {.value = two_half, .known = full40};
    bank[4] = {.value = one, .known = full40};
    bank[5] = {.value = ten, .known = full40};
    bank[6] = word(3);
  }
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
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

constexpr std::array<Stages, 4> oracle() {
  constexpr auto bubble = std::nullopt;
  // PGR4-118: fixed consumer holds Decode for one clock while MUL advances.
  return {{{base + 3, base + 2, base + 1, base, bubble},
           {base + 3, base + 2, base + 1, bubble, base},
           {base + 4, base + 3, base + 2, base + 1, bubble},
           {base + 5, base + 4, base + 3, base + 2, base + 1}}};
}

void check_clock(Sharc &cpu, SharcFetch &fetch, unsigned int cycle) {
  const auto event = fetch.advance_sequenced(cpu);
  require(event.stalled == (cycle == 1) && !event.control_stall);
  require(event.compute_stall == (cycle == 1));
  require(event.address_accepted == (cycle != 1));
  require(stages(fetch) == oracle()[cycle]);
}

void continuation(const lead_a1::SharcState &cpu_state,
                  const lead_a1::SharcFetchState &fetch_state,
                  unsigned int next_cycle) {
  SharcMemory memory;
  program(memory);
  code(memory, base, invalid);
  code(memory, base + 1, invalid);
  Sharc cpu(memory, cpu_state);
  SharcFetch fetch(memory, fetch_state);
  retire(cpu, fetch);
  for (unsigned int cycle = next_cycle; cycle < oracle().size(); ++cycle) {
    check_clock(cpu, fetch, cycle);
    retire(cpu, fetch);
  }
  require(cpu.state().pc == base + 2);
  require(cpu.state().data[0][3] == (lead_a1::SharcKnown40{.value = product, .known = full40}));
  require(cpu.state().data[0][5] == (lead_a1::SharcKnown40{.value = nine, .known = full40}));
}

void literal_phases() {
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
  require(cpu.state().pc == base + 2);
  require(cpu.state().data[0][3] == (lead_a1::SharcKnown40{.value = product, .known = full40}));
  require(cpu.state().data[0][5] == (lead_a1::SharcKnown40{.value = nine, .known = full40}));
}

struct Counts {
  unsigned int compute{};
  unsigned int control{};
};

Counts run_until(Sharc &cpu, SharcFetch &fetch, std::uint32_t stop) {
  Counts counts;
  for (unsigned int clock = 0; clock < limit; ++clock) {
    const auto event = fetch.advance_sequenced(cpu);
    if (event.compute_stall) { ++counts.compute; }
    if (event.control_stall) { ++counts.control; }
    require(!(event.compute_stall && event.control_stall));
    retire(cpu, fetch);
    if (cpu.state().pc == stop) { return counts; }
  }
  throw std::runtime_error("Missing owned float/fixed frontier");
}

struct FixedCase {
  std::uint64_t opcode{};
  std::uint32_t result{};
  bool writes{true};
  bool compare{};
};

constexpr std::array fixed_cases{
    FixedCase{.opcode = 0x013e00001756, .result = 13},
    FixedCase{.opcode = 0x013e00002756, .result = 7},
    FixedCase{.opcode = 0x013e00021756, .result = 10},
    FixedCase{.opcode = 0x013e00029756, .result = 11},
    FixedCase{.opcode = 0x013e0002a756, .result = 9},
    FixedCase{.opcode = 0x013e00040756, .result = 2},
    FixedCase{.opcode = 0x013e00041756, .result = 11},
    FixedCase{.opcode = 0x013e00042756, .result = 9},
    FixedCase{.opcode = 0x013e00043756, .result = 0xfffffff5},
    FixedCase{.opcode = 0x013e0000a756, .writes = false, .compare = true},
    FixedCase{.opcode = 0x013e0000b756, .writes = false, .compare = true},
    FixedCase{.opcode = 0x013e00200756, .result = 80},
    FixedCase{.opcode = 0x013e00204756, .result = 80},
    FixedCase{.opcode = 0x013e002c0756, .result = 10},
    FixedCase{.opcode = 0x013e002c4756, .result = 2},
    FixedCase{.opcode = 0x013e002c8756, .result = 2},
    FixedCase{.opcode = 0x013e002cc756, .writes = false},
    FixedCase{.opcode = 0x023e00000175, .result = 20},
    FixedCase{.opcode = 0x023e00010175, .result = 20},
    FixedCase{.opcode = 0x023e00300175, .result = 10},
    FixedCase{.opcode = 0x023e00310175, .result = 8},
    FixedCase{.opcode = 0x023e00320175, .result = 8},
    FixedCase{.opcode = 0x023e00330175, .writes = false}};

enum class Profile : std::uint8_t { scalar, both, producer_copy, consumer_copy };

struct MatrixCase {
  FixedCase fixed;
  Profile profile{};
  unsigned int gap{};
  bool negative{};
};

void matrix_case(MatrixCase test) {
  SharcMemory memory;
  program(memory, 0);
  auto consumer = test.fixed.opcode;
  if (test.profile == Profile::consumer_copy) {
    constexpr std::uint64_t compute = 0x7fffff;
    consumer = parallel_copy | (consumer & compute);
  }
  code(memory, base, test.profile == Profile::producer_copy ? parallel_multiply : multiply);
  code(memory, base + test.gap + 1, consumer);
  auto state = initial();
  state.system[2].value = test.profile == Profile::both ? simd : 0;
  state.effective = state.system;
  for (std::size_t element = 0; element < 2; ++element) {
    const bool negative = element == 0 ? test.negative : !test.negative;
    state.data[element * 2][1].value = one_half | (negative ? sign : 0);
  }
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, sequenced());
  const auto counts = run_until(cpu, fetch, base + test.gap + 2);
  require(counts.compute == (test.gap == 0 ? 1U : 0U) && counts.control == 0);
  for (std::size_t element = 0; element < (test.profile == Profile::both ? 2U : 1U); ++element) {
    const bool negative = element == 0 ? test.negative : !test.negative;
    const auto result = test.fixed.writes ? std::uint64_t{test.fixed.result} << 8 : sentinel;
    require(cpu.state().data[element * 2][7] ==
            (lead_a1::SharcKnown40{.value = result, .known = full40}));
    require(cpu.state().data[element * 2][3].value == (product | (negative ? sign : 0)));
    require((cpu.state().system[6 + element].value & multiplier_flags) ==
            (negative ? negative_flag : 0));
    require((cpu.state().system[6 + element].value & cacc) ==
            (test.fixed.compare ? greater_history : history));
  }
  if (test.profile == Profile::producer_copy || test.profile == Profile::consumer_copy) {
    require(cpu.state().data[0][2].value == one);
  }
}

void fixed_matrix() {
  for (const auto &fixed : fixed_cases) {
    for (const auto profile :
         {Profile::scalar, Profile::both, Profile::producer_copy, Profile::consumer_copy}) {
      if (profile == Profile::consumer_copy && (fixed.opcode >> class_shift) == 2) { continue; }
      for (unsigned int gap = 0; gap < 3; ++gap) {
        matrix_case({.fixed = fixed, .profile = profile, .gap = gap});
        matrix_case({.fixed = fixed, .profile = profile, .gap = gap, .negative = true});
      }
    }
  }
}

void conditional_bank_case(unsigned int activation, bool alternate) {
  constexpr std::uint64_t if_eq_add = 0x010000001756;
  constexpr std::uint32_t alternate_low = 0x400;
  constexpr std::uint32_t peer_input = 20;
  constexpr std::uint32_t peer_result = 24;
  SharcMemory memory;
  program(memory, if_eq_add);
  auto state = initial();
  state.system[2].value = simd | (alternate ? alternate_low : 0);
  for (std::size_t element = 0; element < 2; ++element) {
    const bool active = ((activation >> element) & 1) != 0;
    state.system[6 + element].value = history | (active ? 1U : 0U);
    const auto bank = (element * 2) + (alternate ? 1U : 0U);
    state.data[bank][5] = active ? word(element == 0 ? 10U : peer_input) : lead_a1::SharcKnown40{};
    state.data[bank][6] = active ? word(element == 0 ? 3U : 4U) : lead_a1::SharcKnown40{};
  }
  state.effective = state.system;
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, sequenced());
  require(run_until(cpu, fetch, base + 2).compute == 1);
  for (std::size_t element = 0; element < 2; ++element) {
    const bool active = ((activation >> element) & 1) != 0;
    const auto bank = (element * 2) + (alternate ? 1U : 0U);
    const auto value = element == 0 ? 13U : peer_result;
    const auto expected = active ? std::uint64_t{value} << 8 : sentinel;
    require(cpu.state().data[bank][7] ==
            (lead_a1::SharcKnown40{.value = expected, .known = full40}));
  }
}

void conditional_banks() {
  for (unsigned int activation = 0; activation < 4; ++activation) {
    conditional_bank_case(activation, false);
    conditional_bank_case(activation, true);
  }
}

void positive_zero_products() {
  for (const auto &fixed : fixed_cases) {
    SharcMemory memory;
    program(memory, fixed.opcode);
    auto state = initial();
    state.data[0][1] = {.known = full40};
    Sharc cpu(memory, state);
    SharcFetch fetch(memory, sequenced());
    require(run_until(cpu, fetch, base + 2).compute == 1);
    require(cpu.state().data[0][3] == (lead_a1::SharcKnown40{.known = full40}));
    const auto result = fixed.writes ? std::uint64_t{fixed.result} << 8 : sentinel;
    require(cpu.state().data[0][7] == (lead_a1::SharcKnown40{.value = result, .known = full40}));
    require((cpu.state().system[6].value & multiplier_flags) == 0);
  }
}

void independent_registers() {
  constexpr std::uint32_t ordinary = 0x12345678;
  constexpr std::array<std::uint32_t, 16> results{ordinary,
                                                  0x3fc00000,
                                                  0x40200000,
                                                  0x40700000,
                                                  0x3f800000,
                                                  10,
                                                  3,
                                                  ordinary,
                                                  ordinary,
                                                  ordinary,
                                                  ordinary,
                                                  ordinary,
                                                  ordinary,
                                                  ordinary,
                                                  ordinary,
                                                  ordinary};
  constexpr std::uint64_t pass = 0x013e00021700;
  for (unsigned int reg = 0; reg < results.size(); ++reg) {
    SharcMemory memory;
    program(memory, pass | (std::uint64_t{reg} << 4));
    Sharc cpu(memory, initial());
    SharcFetch fetch(memory, sequenced());
    require(run_until(cpu, fetch, base + 2).compute == 1);
    require(cpu.state().data[0][7] ==
            (lead_a1::SharcKnown40{.value = std::uint64_t{results[reg]} << 8, .known = full40}));
  }
}

struct SafeCase {
  std::uint64_t consumer{};
  std::uint64_t expected{sentinel};
};

void unrelated_formats_and_producers() {
  constexpr std::array safe{SafeCase{.consumer = 0x013e00081756, .expected = three},
                            SafeCase{.consumer = 0x013e00082756, .expected = sign | one},
                            SafeCase{.consumer = 0x013e00130756, .expected = two},
                            SafeCase{.consumer = parallel_copy},
                            SafeCase{.consumer = 0x0f0700001756, .expected = 0x175600},
                            SafeCase{.consumer = 0}};
  for (const auto &item : safe) {
    SharcMemory memory;
    program(memory, item.consumer);
    auto state = initial();
    state.data[0][5] = {.value = one, .known = full40};
    state.data[0][6] = {.value = two, .known = full40};
    Sharc cpu(memory, state);
    SharcFetch fetch(memory, sequenced());
    const auto counts = run_until(cpu, fetch, base + 2);
    require(counts.compute == 0 && counts.control == 0);
    require(cpu.state().data[0][7] ==
            (lead_a1::SharcKnown40{.value = item.expected, .known = full40}));
  }
  constexpr std::array<std::uint64_t, 3> producers{0x013e00081312, 0x013e00082312, 0x0f0300130312};
  for (const auto producer : producers) {
    SharcMemory memory;
    program(memory);
    code(memory, base, producer);
    Sharc cpu(memory, initial());
    SharcFetch fetch(memory, sequenced());
    require(run_until(cpu, fetch, base + 2).compute == 0);
    require(cpu.state().data[0][5].value == nine);
  }
}

void warmup(Sharc &cpu, SharcFetch &fetch) {
  for (unsigned int clock = 0; clock < 4; ++clock) {
    require(fetch.advance_sequenced(cpu).address_accepted);
    require(!fetch.state().execute);
  }
}

void failed_producer_and_consumer() {
  SharcMemory memory;
  program(memory);
  auto state = initial();
  state.data[0][1] = {};
  Sharc missing_input(memory, state);
  SharcFetch fetch(memory, sequenced());
  warmup(missing_input, fetch);
  require(fetch.advance_sequenced(missing_input).compute_stall);
  const auto before_cpu = missing_input.state();
  const auto before_fetch = fetch.state();
  rejected([&] { static_cast<void>(missing_input.step(fetch)); }, "floating register operand");
  require(missing_input.state() == before_cpu && fetch.state() == before_fetch && !memory.active());
  rejected([&] { static_cast<void>(fetch.advance_sequenced(missing_input)); }, "has not retired");
  state = before_cpu;
  state.data[0][1] = {.value = one_half, .known = full40};
  state.data[0][5] = {};
  Sharc repaired(memory, state);
  code(memory, base, invalid);
  require(repaired.step(fetch).opcode == multiply);
  check_clock(repaired, fetch, 2);
  check_clock(repaired, fetch, 3);
  const auto producer_done = repaired.state();
  const auto consumer_held = fetch.state();
  rejected([&] { static_cast<void>(repaired.step(fetch)); }, "register operand bits");
  require(repaired.state() == producer_done && fetch.state() == consumer_held && !memory.active());
  state = producer_done;
  state.data[0][5] = {.value = ten, .known = full40};
  Sharc retry(memory, state);
  code(memory, base + 1, invalid);
  require(retry.step(fetch).opcode == decrement && retry.state().data[0][5].value == nine);
  require(retry.state().data[0][3].value == product);
}

void release_clock_ownership() {
  SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  Sharc cpu(memory, initial(), &control);
  SharcFetch fetch(memory, sequenced());
  warmup(cpu, fetch);
  require(fetch.advance_sequenced(cpu).compute_stall);
  retire(cpu, fetch);
  const auto before_cpu = cpu.state();
  const auto before_fetch = fetch.state();
  const auto before_control = control.snapshot();
  control.begin_instruction();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "during a transaction");
  require(cpu.state() == before_cpu && fetch.state() == before_fetch);
  require(control.state() == before_control);
  control.abort_instruction();
  check_clock(cpu, fetch, 2);
  check_clock(cpu, fetch, 3);
  retire(cpu, fetch);
  require(cpu.state().data[0][5].value == nine);
}

void serial_compute_and_branch_holds() {
  constexpr std::uint32_t target = base + 32;
  constexpr std::uint64_t if_ne = 0x062000000000;
  SharcMemory memory;
  program(memory);
  code(memory, base + 2, if_ne | target);
  auto state = initial();
  state.data[0][5] = {.value = 512, .known = full40};
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, sequenced());
  const auto counts = run_until(cpu, fetch, target + 1);
  require(counts.compute == 1 && counts.control == 1);
  require(cpu.state().data[0][5].value == 256 && cpu.state().data[0][3].value == product);
}

void dual_load_and_later_cache_stall() {
  constexpr std::uint64_t dual_add = 0x294b2b001756;
  constexpr std::uint32_t dm_address = 0x80000;
  constexpr std::uint32_t pm_address = 0xe0001;
  constexpr std::uint64_t dm_input = 0x11223344;
  constexpr std::uint64_t pm_input = 0x89abcdef;
  constexpr unsigned int cells = 64;
  constexpr std::uint32_t disabled_cache = 0x10;
  SharcMemory memory;
  program(memory, dual_add); // R7=old R5+R6; R5=DM(I4,M5), R6=PM(I12,M13).
  memory.map({.bank = 0, .halfwords = cells, .name = "owned fixed-transition DM data"});
  memory.map({.bank = 3, .halfwords = cells, .name = "owned fixed-transition PM data"});
  memory.write({.address = dm_address}, dm_input);
  memory.write({.address = pm_address}, pm_input);
  auto state = initial();
  state.system[4].value &= ~disabled_cache;
  state.effective = state.system;
  state.dag[0][0][4] = {.value = dm_address, .known = full32};
  state.dag[0][0][12] = {.value = pm_address, .known = full32};
  state.dag[0][1][5] = {.value = 1, .known = full32};
  state.dag[0][1][13] = {.value = full32, .known = full32};
  lead_a1::SharcControl control(true);
  Sharc cpu(memory, state, &control);
  SharcFetch fetch(memory, sequenced());
  warmup(cpu, fetch);
  require(fetch.advance_sequenced(cpu).compute_stall);
  retire(cpu, fetch);
  const auto request = fetch.advance_sequenced(cpu, lead_a1::SharcFetchBus::pm_data);
  require(request.address_accepted && !request.stalled && !request.cache_hit);
  const auto miss = fetch.advance_sequenced(cpu);
  require(miss.stalled && !miss.compute_stall && !miss.control_stall);
  retire(cpu, fetch);
  require(cpu.state().data[0][7].value == std::uint64_t{13} << 8);
  require(cpu.state().data[0][5].value == dm_input << 8);
  require(cpu.state().data[0][6].value == pm_input << 8);
  require(cpu.state().data[0][3].value == product);
  require(cpu.state().dag[0][0][4].value == dm_address + 1);
  require(cpu.state().dag[0][0][12].value == pm_address - 1);
  static_cast<void>(fetch.advance_sequenced(cpu));
  static_cast<void>(fetch.advance_sequenced(cpu));
  retire(cpu, fetch);
  constexpr std::uint32_t cached = 0x100; // F1 base+4: set4, first way8.
  require(cpu.state().cache.valid.value == cached && cpu.state().cache.valid.known == full32);
  require(cpu.state().pc == base + 3);
}

void guarded_boundaries() {
  for (const bool cache : {false, true}) {
    SharcMemory memory;
    program(memory);
    if (cache) {
      constexpr std::uint64_t dual_multiply = 0x200000130312;
      code(memory, base, dual_multiply);
    }
    Sharc cpu(memory, initial());
    SharcFetch fetch(memory, sequenced());
    for (unsigned int clock = 0; clock < 3; ++clock) {
      static_cast<void>(fetch.advance_sequenced(cpu));
    }
    const auto bus = cache ? lead_a1::SharcFetchBus::pm_data : lead_a1::SharcFetchBus::available;
    require(fetch.advance_sequenced(cpu, bus).address_accepted);
    const auto before_cpu = cpu.state();
    const auto before_fetch = fetch.state();
    rejected(
        [&] { static_cast<void>(fetch.advance_sequenced(cpu, lead_a1::SharcFetchBus::pm_data)); },
        cache ? "simultaneous cache/control" : "control stall with PM/cache");
    require(cpu.state() == before_cpu && fetch.state() == before_fetch);
  }
  SharcMemory memory;
  constexpr std::uint64_t unknown_compute = 0x013e00300756;
  program(memory, unknown_compute);
  Sharc cpu(memory, initial());
  SharcFetch fetch(memory, sequenced());
  warmup(cpu, fetch);
  const auto before_cpu = cpu.state();
  const auto before_fetch = fetch.state();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "input-format interlock");
  require(cpu.state() == before_cpu && fetch.state() == before_fetch && !memory.active());
}

void pending_mode_guard() {
  SharcMemory memory;
  program(memory);
  Sharc cpu(memory, initial());
  SharcFetch fetch(memory, sequenced());
  warmup(cpu, fetch);
  auto state = cpu.state();
  state.pending[2] = state.system[2];
  Sharc pending(memory, state);
  const auto before = fetch.state();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(pending)); },
           "floating-to-fixed interlock with mode/control/loop");
  require(pending.state() == state && fetch.state() == before && !memory.active());
}

void changed_loop_body_guard() {
  constexpr std::uint64_t five = 0x0c0005000001;
  SharcMemory memory;
  program(memory, multiply);
  code(memory, base, five);
  Sharc cpu(memory, initial());
  SharcFetch fetch(memory, sequenced());
  warmup(cpu, fetch);
  code(memory, base + 1, decrement); // Held copies stay MUL; later real F2 reads change.
  for (unsigned int clock = 0; clock < limit; ++clock) {
    const auto before_cpu = cpu.state();
    const auto before_fetch = fetch.state();
    try {
      static_cast<void>(fetch.advance_sequenced(cpu));
    } catch (const std::exception &error) {
      require(
          std::string(error.what()).find("floating-to-fixed interlock with mode/control/loop") !=
          std::string::npos);
      require(cpu.state() == before_cpu && fetch.state() == before_fetch && !memory.active());
      return;
    }
    retire(cpu, fetch);
  }
  throw std::runtime_error("Missing owned changed-loop input-format guard");
}

}

int main() try {
  literal_phases();
  fixed_matrix();
  conditional_banks();
  positive_zero_products();
  independent_registers();
  unrelated_formats_and_producers();
  failed_producer_and_consumer();
  release_clock_ownership();
  serial_compute_and_branch_holds();
  dual_load_and_later_cache_stall();
  guarded_boundaries();
  pending_mode_guard();
  changed_loop_body_guard();
  std::cout << "SHARC floating-to-fixed compute stall passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
