// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

using lead_a1::Sharc;
using lead_a1::SharcFetch;
using lead_a1::SharcKnown32;
using lead_a1::SharcMemory;
constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t target = base + 16;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t alu = 0x43f;
constexpr std::uint32_t cacc = 0xff000000;
constexpr std::uint32_t preserved = 0x478c0;
constexpr std::uint32_t old_history = 0xa5000000;
constexpr std::uint32_t shifted_history = 0x52000000;
constexpr std::uint32_t newest = 0x80000000;
constexpr std::uint32_t saturation = 0x2000;
constexpr std::uint32_t floating = 0x400;
constexpr std::uint32_t simd = 0x200000;
constexpr std::uint32_t sign = 0x80000000;
constexpr std::uint64_t signed_compare = 0x013e0000af12; // COMP(R1,R2), ignored n=15.
constexpr std::uint64_t unsigned_compare = 0x013e0000bf12;
constexpr std::uint64_t branch = 0x060000000000;
constexpr std::uint64_t pass = 0x010000021010; // Conditional R0=PASS R1.
constexpr unsigned int condition_shift = 33;
constexpr unsigned int clock_limit = 96;
constexpr std::array<unsigned int, 4> relational_codes{1, 2, 17, 18};

// Bits0..3 are LT/LE/GE/GT. Index bits are AF,AN,AV,AZ,ALUSAT.
// Literal PGR4-93 truth table, independently read from the original PDF bars.
constexpr std::array<unsigned int, 32> relational_truth{12, 12, 3, 3, 3, 12, 12, 3, 6, 6,  3,
                                                        6,  3,  6, 6, 6, 12, 12, 3, 3, 12, 12,
                                                        3,  3,  6, 6, 3, 6,  6,  6, 3, 6};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC compare expectation at line " + std::to_string(where.line()));
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
  throw std::runtime_error("Missing SHARC compare rejection: " + reason);
}

void code(SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(SharcMemory &memory, std::uint64_t opcode = signed_compare) {
  constexpr unsigned int cells = 192;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned compare program"});
  for (unsigned int index = 0; index < cells / 3; ++index) { code(memory, base + index, 0); }
  code(memory, base, opcode);
}

lead_a1::SharcKnown40 word(std::uint32_t value) {
  constexpr std::uint64_t extension = 0xb7;
  return {.value = (std::uint64_t{value} << 8) | extension, .known = full40};
}

lead_a1::SharcState initial() {
  auto state = Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t disabled_cache = 0x10;
  constexpr std::uint32_t sticky_events = 0x3e7;
  state.system[4].value |= disabled_cache;
  state.system[6].value = preserved | old_history | alu;
  state.system[8].value |= sticky_events;
  state.effective = state.system;
  return state;
}

lead_a1::SharcFetchState flow() {
  lead_a1::SharcFetchState state{};
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

struct Boundary {
  std::uint32_t left{};
  std::uint32_t right{};
  std::uint32_t signed_flags{};
  std::uint32_t unsigned_flags{};
  bool signed_greater{};
  bool unsigned_greater{};
};

constexpr std::array boundaries{
    Boundary{.left = 0, .right = 0, .signed_flags = 1, .unsigned_flags = 1},
    Boundary{.left = 1, .right = 0, .signed_greater = true, .unsigned_greater = true},
    Boundary{.left = 0, .right = 1, .signed_flags = 4, .unsigned_flags = 4},
    Boundary{.left = 0xffffffff, .right = 0, .signed_flags = 4, .unsigned_greater = true},
    Boundary{.left = 0, .right = 0xffffffff, .unsigned_flags = 4, .signed_greater = true},
    Boundary{.left = 0x80000000, .right = 0x7fffffff, .signed_flags = 4, .unsigned_greater = true},
    Boundary{.left = 0x7fffffff, .right = 0x80000000, .unsigned_flags = 4, .signed_greater = true},
    Boundary{.left = 0xfffffffe, .right = 0xffffffff, .signed_flags = 4, .unsigned_flags = 4},
    Boundary{
        .left = 0xffffffff, .right = 0xfffffffe, .signed_greater = true, .unsigned_greater = true},
    Boundary{.left = 0x80000000, .right = 0x80000000, .signed_flags = 1, .unsigned_flags = 1},
    Boundary{.left = 0x7fffffff, .right = 0, .signed_greater = true, .unsigned_greater = true}};

void boundary_case(Boundary test, bool unsigned_mode) {
  SharcMemory memory;
  program(memory, unsigned_mode ? unsigned_compare : signed_compare);
  auto state = initial();
  state.data[0][1] = word(test.left);
  state.data[0][2] = word(test.right);
  // Neither the ignored high n-field nor fixed compares need these modes.
  constexpr std::uint32_t irrelevant = 0x18080 | saturation;
  state.system[2].known &= ~irrelevant;
  state.effective = state.system;
  Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  const auto flags = unsigned_mode ? test.unsigned_flags : test.signed_flags;
  const bool greater = unsigned_mode ? test.unsigned_greater : test.signed_greater;
  const auto expected = preserved | shifted_history | (greater ? newest : 0) | flags;
  require(cpu.state().system[6] == (SharcKnown32{.value = expected, .known = full32}));
  require(cpu.state().effective[6] == cpu.state().system[6]);
  require(cpu.state().data == state.data && cpu.state().system[8] == state.system[8]);
  require(cpu.state().system[7] == state.system[7] && cpu.state().system[9] == state.system[9]);
}

void literal_boundaries() {
  for (const auto &test : boundaries) {
    boundary_case(test, false);
    boundary_case(test, true);
  }
}

void history_and_known_bits() {
  constexpr std::array<std::uint32_t, 8> left{5, 7, 0, 9, 1, 3, 1, 8};
  constexpr std::array<std::uint32_t, 8> right{2, 7, 3, 1, 1, 2, 9, 0};
  constexpr std::array<std::uint32_t, 8> history{0x80, 0x40, 0x20, 0x90, 0x48, 0xa4, 0x52, 0xa9};
  constexpr std::array<std::uint32_t, 8> knowledge{0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff};
  constexpr std::array<std::uint32_t, 8> flags{0, 1, 4, 0, 1, 0, 4, 0};
  constexpr std::uint64_t compare_base = 0x013e0000a000;
  SharcMemory memory;
  program(memory);
  auto state = initial();
  state.system[6] = {.value = preserved, .known = full32 & ~cacc};
  state.effective = state.system;
  for (unsigned int index = 0; index < left.size(); ++index) {
    const auto reg = index * 2;
    state.data[0][reg] = word(left[index]);
    state.data[0][reg + 1] = word(right[index]);
    code(memory, base + index, compare_base | (std::uint64_t{reg} << 4) | (reg + 1));
  }
  Sharc cpu(memory, state);
  for (unsigned int index = 0; index < left.size(); ++index) {
    static_cast<void>(cpu.step());
    const auto expected_value = preserved | (history[index] << 24) | flags[index];
    const auto expected_known = (full32 & ~cacc) | (knowledge[index] << 24);
    require(cpu.state().system[6] ==
            (SharcKnown32{.value = expected_value, .known = expected_known}));
    require(cpu.state().effective[6] == cpu.state().system[6]);
    require(cpu.state().data == state.data && cpu.state().system[8] == state.system[8]);
  }
}

std::uint32_t flag_pattern(unsigned int pattern) {
  return ((pattern & 1) != 0 ? floating : 0) | ((pattern & 2) != 0 ? 4U : 0U) |
         ((pattern & 4) != 0 ? 2U : 0U) | ((pattern & 8) != 0 ? 1U : 0U);
}

struct ConditionCase {
  unsigned int x{};
  unsigned int y{};
  unsigned int condition{};
  bool saturated{};
  bool simd_mode{};
};

bool expected_condition(ConditionCase test, unsigned int pattern) {
  const auto index = pattern | (test.saturated ? 16U : 0U);
  return ((relational_truth[index] >> test.condition) & 1) != 0;
}

lead_a1::SharcState condition_state(ConditionCase test) {
  auto state = initial();
  state.system[2].value = (test.saturated ? saturation : 0) | (test.simd_mode ? simd : 0);
  state.system[6].value = flag_pattern(test.x);
  state.system[7].value = flag_pattern(test.y);
  state.effective = state.system;
  return state;
}

void condition_case(ConditionCase test) {
  const bool x = expected_condition(test, test.x);
  const bool y = expected_condition(test, test.y);
  SharcMemory memory;
  const auto encoded = std::uint64_t{relational_codes[test.condition]} << condition_shift;
  program(memory, branch | encoded | target);
  auto state = condition_state(test);
  Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  const bool taken = x && (!test.simd_mode || y);
  require(cpu.state().pc == (taken ? target : base + 1));
  require(cpu.state().data == state.data && cpu.state().system == state.system);
  code(memory, base, pass | encoded);
  constexpr std::uint32_t first = 0x12345678;
  constexpr std::uint32_t second = 0x76543210;
  if (x) { state.data[0][1] = word(first); }
  if (test.simd_mode && y) { state.data[2][1] = word(second); }
  Sharc conditional(memory, state);
  static_cast<void>(conditional.step());
  const lead_a1::SharcKnown40 result_x{.value = std::uint64_t{first} << 8, .known = full40};
  const lead_a1::SharcKnown40 result_y{.value = std::uint64_t{second} << 8, .known = full40};
  require(conditional.state().data[0][0] == (x ? result_x : state.data[0][0]));
  require(conditional.state().data[2][0] == (test.simd_mode && y ? result_y : state.data[2][0]));
}

void relational_truth_matrix() {
  for (const bool saturated : {false, true}) {
    for (unsigned int condition = 0; condition < relational_codes.size(); ++condition) {
      for (unsigned int x = 0; x < 16; ++x) {
        condition_case({.x = x, .condition = condition, .saturated = saturated});
        for (unsigned int y = 0; y < 16; ++y) {
          condition_case(
              {.x = x, .y = y, .condition = condition, .saturated = saturated, .simd_mode = true});
        }
      }
    }
  }
}

struct SimdCase {
  unsigned int active{};
  bool alternate{};
  bool unsigned_mode{};
};

void simd_compare_case(SimdCase test) {
  SharcMemory memory;
  constexpr std::uint64_t without_condition = 0x003e00000000;
  const auto compare =
      (test.unsigned_mode ? unsigned_compare : signed_compare) & ~without_condition;
  program(memory, compare); // IF EQ COMP; destination remains ignored.
  auto state = initial();
  constexpr std::uint32_t alternate_low = 0x400;
  constexpr std::uint32_t high_select = 0x80;
  state.system[2] = {.value = simd | (test.alternate ? alternate_low : 0),
                     .known = full32 & ~high_select};
  for (unsigned int element = 0; element < 2; ++element) {
    const bool active = (test.active & (1U << element)) != 0;
    state.system[6 + element].value = preserved | old_history | floating | (active ? 1U : 0U);
    if (!active) { continue; }
    const auto bank = (element * 2) + (test.alternate ? 1U : 0U);
    state.data[bank][1] = word(element == 0 ? full32 : 1U);
    state.data[bank][2] = word(0);
  }
  state.effective = state.system;
  Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  for (unsigned int element = 0; element < 2; ++element) {
    const bool active = (test.active & (1U << element)) != 0;
    if (!active) {
      require(cpu.state().system[6 + element] == state.system[6 + element]);
      continue;
    }
    const bool less = element == 0 && !test.unsigned_mode;
    const auto flags = less ? 4U : 0U;
    const auto history = less ? shifted_history : shifted_history | newest;
    require(cpu.state().system[6 + element].value == (preserved | history | flags));
    require(cpu.state().effective[6 + element] == cpu.state().system[6 + element]);
  }
  require(cpu.state().data == state.data);
  require(cpu.state().system[8] == state.system[8] && cpu.state().system[9] == state.system[9]);
}

void simd_compare_matrix() {
  for (unsigned int active = 0; active < 4; ++active) {
    for (const bool alternate : {false, true}) {
      simd_compare_case({.active = active, .alternate = alternate});
      simd_compare_case({.active = active, .alternate = alternate, .unsigned_mode = true});
    }
  }
}

unsigned int run_until(Sharc &cpu, SharcFetch &fetch, std::uint32_t stop) {
  unsigned int interlocks{};
  for (unsigned int clock = 0; clock < clock_limit; ++clock) {
    const auto event = fetch.advance_sequenced(cpu);
    if (event.control_stall) { ++interlocks; }
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
    if (cpu.state().pc == stop) { return interlocks; }
  }
  throw std::runtime_error("Missing actual compare pipeline frontier");
}

void compare_branch_case(unsigned int condition, bool gap) {
  SharcMemory memory;
  program(memory);
  const auto branch_at = base + (gap ? 2U : 1U);
  code(memory,
       branch_at,
       branch | (std::uint64_t{relational_codes[condition]} << condition_shift) | target);
  auto state = initial();
  state.data[0][1] = word(full32); // Signed -1 <0; AV clear, AF clear.
  state.data[0][2] = word(0);
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, flow());
  const bool taken = condition < 2;
  const auto stop = taken ? target : branch_at + 1;
  require(run_until(cpu, fetch, stop) == (gap ? 0U : 1U));
  require((cpu.state().system[6].value & alu) == 4);
  require((cpu.state().system[6].value & cacc) == shifted_history);
}

void compare_branch_pipeline() {
  for (unsigned int condition = 0; condition < relational_codes.size(); ++condition) {
    compare_branch_case(condition, false);
    compare_branch_case(condition, true);
  }
}

void retained_branch_decision(bool taken) {
  constexpr std::uint64_t if_lt = 0x060200000000;
  SharcMemory memory;
  SharcMemory rebound;
  program(memory);
  program(rebound);
  code(memory, base + 1, if_lt | target);
  code(rebound, base + 1, if_lt | target);
  auto state = initial();
  state.data[0][1] = word(taken ? full32 : 1U);
  state.data[0][2] = word(0);
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, flow());
  for (unsigned int clock = 0; clock < clock_limit; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
    const auto address = fetch.state().address;
    if (!address || address->address != base + 1) { continue; }
    require(address->branch && address->branch->taken == taken);
    auto altered = cpu.state();
    altered.system[6] = {};
    altered.effective[6] = {};
    altered.system[2].known &= ~saturation;
    altered.effective[2].known &= ~saturation;
    Sharc restored(rebound, altered);
    SharcFetch continuation(rebound, fetch.state());
    static_cast<void>(continuation.advance_sequenced(restored));
    const auto execute = continuation.state().execute;
    require(execute && execute->address == base + 1);
    static_cast<void>(restored.step(continuation));
    require(restored.state().pc == (taken ? target : base + 2));
    return;
  }
  throw std::runtime_error("Missing actual retained relational decision");
}

void arithmetic_overflow_relations() {
  constexpr std::uint64_t add = 0x013e00001312;
  constexpr std::uint64_t subtract = 0x013e00002312;
  constexpr std::uint32_t maximum = 0x7fffffff;
  for (const bool saturated : {false, true}) {
    for (const bool negative : {false, true}) {
      SharcMemory memory;
      program(memory, negative ? subtract : add);
      const auto condition = negative ? 1U : 18U;
      code(memory, base + 1, branch | (std::uint64_t{condition} << condition_shift) | target);
      auto state = initial();
      state.system[2].value = saturated ? saturation : 0;
      state.effective = state.system;
      state.data[0][1] = word(negative ? sign : maximum);
      state.data[0][2] = word(1);
      Sharc cpu(memory, state);
      SharcFetch fetch(memory, flow());
      require(run_until(cpu, fetch, target) == 1);
      require((cpu.state().system[6].value & 2) != 0);
    }
  }
}

void next_loop_body(Sharc &cpu, SharcFetch &fetch, std::uint32_t expected) {
  for (unsigned int clock = 0; clock < clock_limit; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (!fetch.state().execute) { continue; }
    const auto step = cpu.step(fetch);
    if (step.pc != base + 1) { continue; }
    require((cpu.state().system[6].value & cacc) == expected);
    return;
  }
  throw std::runtime_error("Missing actual compare loop body");
}

void clocked_history_and_checkpoint() {
  constexpr std::uint64_t five = 0x0c0005000001;
  SharcMemory memory;
  SharcMemory rebound;
  program(memory, five);
  program(rebound, five);
  code(memory, base + 1, signed_compare);
  code(rebound, base + 1, signed_compare);
  auto state = initial();
  state.data[0][1] = word(5);
  state.data[0][2] = word(2);
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, flow());
  constexpr std::uint32_t first = 0xd2000000;
  constexpr std::uint32_t second = 0xe9000000;
  next_loop_body(cpu, fetch, first);
  next_loop_body(cpu, fetch, second);
  Sharc restored(rebound, cpu.state());
  SharcFetch continuation(rebound, fetch.state());
  constexpr std::array<std::uint32_t, 3> rest{0xf4000000, 0xfa000000, 0xfd000000};
  for (const auto expected : rest) {
    next_loop_body(cpu, fetch, expected);
    next_loop_body(restored, continuation, expected);
    require(cpu.state() == restored.state() && fetch.state() == continuation.state());
  }
  require(cpu.state().pc == base + 2 && !cpu.state().count_loop && cpu.state().depth == 0);
  require(cpu.state().data == state.data);
}

void compare_does_not_produce_multiplier_data() {
  constexpr std::uint64_t ignored_n1 = 0x013e0000a112;
  constexpr std::uint64_t square = 0x013e00130311;
  constexpr std::uint64_t one_half = 0x3fc0000000;
  constexpr std::uint64_t two_half = 0x4020000000;
  constexpr std::uint64_t expected = 0x4010000000;
  SharcMemory memory;
  program(memory, ignored_n1);
  code(memory, base + 1, square);
  auto state = initial();
  state.data[0][1] = {.value = one_half, .known = full40};
  state.data[0][2] = {.value = two_half, .known = full40};
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, flow());
  require(run_until(cpu, fetch, base + 2) == 0);
  require(cpu.state().data[0][3].value == expected && cpu.state().data[0][1] == state.data[0][1]);
}

void parallel_old_inputs() {
  constexpr std::uint64_t compare_and_copy = 0x707e0100af12;
  SharcMemory memory;
  program(memory, compare_and_copy); // COMP(old R1,old R2), R2=old R4.
  auto state = initial();
  state.data[0][1] = word(5);
  state.data[0][2] = word(2);
  state.data[0][4] = word(7);
  Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  require((cpu.state().system[6].value & alu) == 0);
  require((cpu.state().system[6].value & cacc) == (shifted_history | newest));
  require(cpu.state().data[0][2] == state.data[0][4]);
  require(cpu.state().data[0][15] == state.data[0][15]);
}

void unknown_input_and_irrelevant_bits() {
  SharcMemory memory;
  program(memory);
  auto state = initial();
  constexpr std::uint64_t upper32 = 0xffffffff00;
  state.data[0][1] = {.value = 256, .known = upper32};
  state.data[0][2] = {.value = 512, .known = upper32};
  Sharc ignored_extension(memory, state);
  static_cast<void>(ignored_extension.step());
  require((ignored_extension.state().system[6].value & alu) == 4);
  state.data[0][2] = {};
  state.pending[2] = state.system[2];
  Sharc failing(memory, state);
  const auto before = failing.state();
  rejected([&] { static_cast<void>(failing.step()); }, "Unknown SHARC register operand");
  require(failing.state() == before && !memory.active());
  state = initial();
  state.system[2].value = simd;
  state.effective = state.system;
  state.data[0][1] = word(2);
  state.data[0][2] = word(1);
  state.data[2][1] = word(1); // PEx has already shifted its tentative CACC.
  Sharc second_lane(memory, state);
  const auto paired = second_lane.state();
  rejected([&] { static_cast<void>(second_lane.step()); }, "Unknown SHARC register operand");
  require(second_lane.state() == paired && !memory.active());
}

void partial_condition_controls() {
  SharcMemory memory;
  constexpr unsigned int lt = 1;
  constexpr unsigned int le = 2;
  program(memory, branch | (std::uint64_t{lt} << condition_shift) | target);
  auto state = initial();
  state.system[6] = {.value = floating,
                     .known = floating | 4}; // Float AN=0 makes LT false; AZ unused.
  state.system[2].known &= ~saturation;
  state.effective = state.system;
  Sharc false_float(memory, state);
  static_cast<void>(false_float.step());
  require(false_float.state().pc == base + 1);
  state.system[6] = {.value = floating | 4, .known = floating | 4}; // Float AN=1 makes LE true.
  state.effective = state.system;
  code(memory, base, branch | (std::uint64_t{le} << condition_shift) | target);
  Sharc true_float(memory, state);
  static_cast<void>(true_float.step());
  require(true_float.state().pc == target);
  state.system[6] = {.known = floating | 4 | 2}; // Fixed AV=0 ignores ALUSAT.
  state.effective = state.system;
  code(memory, base, branch | (std::uint64_t{lt} << condition_shift) | target);
  Sharc fixed(memory, state);
  static_cast<void>(fixed.step());
  require(fixed.state().pc == base + 1);
  state.system[6] = {.value = 2, .known = floating | 4 | 2}; // AV=1 needs actual ALUSAT.
  state.effective = state.system;
  Sharc unknown_saturation(memory, state);
  const auto before = unknown_saturation.state();
  rejected([&] { static_cast<void>(unknown_saturation.step()); }, "Unknown SHARC control/status");
  require(unknown_saturation.state() == before && !memory.active());
}

}

int main() try {
  literal_boundaries();
  history_and_known_bits();
  relational_truth_matrix();
  simd_compare_matrix();
  compare_branch_pipeline();
  retained_branch_decision(false);
  retained_branch_decision(true);
  arithmetic_overflow_relations();
  clocked_history_and_checkpoint();
  compare_does_not_produce_multiplier_data();
  parallel_old_inputs();
  unknown_input_and_irrelevant_bits();
  partial_condition_controls();
  std::cout << "SHARC signed/unsigned compare, CACC and relational conditions passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
