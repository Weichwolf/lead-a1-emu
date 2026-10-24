// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_float.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

using lead_a1::Sharc;
using lead_a1::SharcFetch;
using lead_a1::SharcFloatFormat;
using lead_a1::SharcFloatResult;
using lead_a1::SharcFloatRounding;
using lead_a1::SharcKnown40;
using lead_a1::SharcMemory;
constexpr auto wide = SharcFloatFormat::extended40;
constexpr auto narrow = SharcFloatFormat::binary32;
constexpr auto nearest = SharcFloatRounding::nearest_even;
constexpr auto truncate = SharcFloatRounding::toward_zero;
constexpr std::uint32_t base = 0xc0000;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t sign = 0x8000000000;
constexpr std::uint64_t one = 0x3f80000000;
constexpr std::uint64_t two = 0x4000000000;
constexpr std::uint64_t three = 0x4040000000;
constexpr std::uint64_t half = 0x3f00000000;
constexpr std::uint64_t one_half = 0x3fc0000000;
constexpr std::uint64_t two_half = 0x4020000000;
constexpr std::uint64_t three_quarters = 0x4070000000;
constexpr std::uint64_t minimum = 0x0080000000;
constexpr std::uint64_t maximum = 0x7f7fffffff;
constexpr std::uint64_t infinity = 0x7f80000000;
constexpr std::uint64_t half_left = 0x3f80008000;
constexpr std::uint64_t half_right = 0x3f80018000;
constexpr std::uint32_t round32 = 0x10000;
constexpr std::uint32_t truncation = 0x8000;
constexpr std::uint32_t simd = 0x200000;
constexpr std::uint32_t multiplier_flags = 0x3c0;
constexpr std::uint32_t multiply_negative = 0x40;
constexpr std::uint32_t alu_flags = 0x43f;
constexpr std::uint64_t multiply = 0x013e00130312; // F3=F1*F2, CU001/op30, PGR12-6.
constexpr std::uint64_t in_place = 0x013e00130112;
constexpr std::uint64_t conditional = 0x010000130312;
constexpr unsigned int clock_limit = 64;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC multiply expectation at line " + std::to_string(where.line()));
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
  throw std::runtime_error("Missing SHARC multiply rejection: " + reason);
}

SharcKnown40 known(std::uint64_t value) {
  return {.value = value, .known = full40};
}

void code(SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(SharcMemory &memory, std::uint64_t opcode = multiply) {
  constexpr unsigned int cells = 192;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned floating multiply program"});
  for (unsigned int index = 0; index < cells / 3; ++index) { code(memory, base + index, 0); }
  code(memory, base, opcode);
}

lead_a1::SharcState initial() {
  auto state = Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t disabled_cache = 0x10;
  state.system[4].value |= disabled_cache;
  state.effective = state.system;
  state.data[0][1] = known(one_half);
  state.data[0][2] = known(two_half);
  return state;
}

lead_a1::SharcFetchState flow() {
  lead_a1::SharcFetchState state{};
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

struct NumericCase {
  std::uint64_t left{};
  std::uint64_t right{};
  SharcFloatFormat format{wide};
  SharcFloatRounding rounding{nearest};
  SharcFloatResult expected;
};

constexpr std::array literals{
    NumericCase{.left = one_half, .right = two_half, .expected = {.value = three_quarters}},
    NumericCase{
        .left = sign | one_half, .right = two_half, .expected = {.value = sign | three_quarters}},
    NumericCase{
        .left = sign | one_half, .right = sign | two_half, .expected = {.value = three_quarters}},
    NumericCase{.left = 0, .right = two, .expected = {.value = 0}},
    NumericCase{.left = sign, .right = two, .expected = {.value = sign}},
    NumericCase{.left = sign, .right = sign, .expected = {.value = 0}},
    NumericCase{.left = 1, .right = maximum, .expected = {.value = 0}},
    NumericCase{.left = sign | 1, .right = maximum, .expected = {.value = sign}},
    NumericCase{.left = minimum, .right = one, .expected = {.value = minimum}},
    NumericCase{.left = minimum, .right = half, .expected = {.value = 0, .underflow = true}},
    NumericCase{
        .left = sign | minimum, .right = half, .expected = {.value = sign, .underflow = true}},
    NumericCase{.left = maximum, .right = two, .expected = {.value = infinity, .overflow = true}},
    NumericCase{.left = maximum,
                .right = two,
                .rounding = truncate,
                .expected = {.value = maximum, .overflow = true}},
    NumericCase{.left = maximum,
                .right = two,
                .format = narrow,
                .rounding = truncate,
                .expected = {.value = maximum & ~std::uint64_t{255}, .overflow = true}},
    NumericCase{
        .left = one + 255, .right = one + 255, .format = narrow, .expected = {.value = one}},
    NumericCase{.left = one + 255, .right = one + 255, .expected = {.value = one + 510}},
    NumericCase{.left = half_left, .right = half_left, .expected = {.value = 0x3f80010000}},
    NumericCase{.left = half_left, .right = half_right, .expected = {.value = 0x3f80020002}},
    NumericCase{.left = half_left,
                .right = half_right,
                .rounding = truncate,
                .expected = {.value = 0x3f80020001}},
    NumericCase{.left = 0x3f80080000,
                .right = 0x3f80080000,
                .format = narrow,
                .expected = {.value = 0x3f80100000}},
    NumericCase{.left = 0x3f80080000,
                .right = 0x3f80180000,
                .format = narrow,
                .expected = {.value = 0x3f80200200}},
    NumericCase{.left = 0x3f80080000,
                .right = 0x3f80180000,
                .format = narrow,
                .rounding = truncate,
                .expected = {.value = 0x3f80200100}},
    NumericCase{.left = 0x3fffffffff, .right = 0x3fffffffff, .expected = {.value = 0x407ffffffe}}};

void numeric_literals() {
  for (const auto &item : literals) {
    require(lead_a1::sharc_float_multiply(item.left, item.right, item.format, item.rounding) ==
            item.expected);
  }
}

void native_rounding_and_status() {
  constexpr std::uint32_t unrelated = 0xa5003bff; // Seed MUL flags; ALU/CACC/shifter set, AF clear.
  for (const bool short_input : {false, true}) {
    for (const bool toward_zero : {false, true}) {
      SharcMemory memory;
      program(memory);
      auto state = initial();
      state.system[2].value = (short_input ? round32 : 0) | (toward_zero ? truncation : 0);
      state.system[6].value = unrelated;
      state.effective = state.system;
      state.data[0][1] = known(half_left);
      state.data[0][2] = known(half_right);
      Sharc cpu(memory, state);
      static_cast<void>(cpu.step());
      constexpr std::uint64_t rounded32 = 0x3f80020000;
      auto expected = rounded32;
      if (!short_input) { expected += toward_zero ? 1U : 2U; }
      require(cpu.state().data[0][3] == known(expected));
      require(cpu.state().system[6] ==
              (lead_a1::SharcKnown32{.value = unrelated & ~multiplier_flags, .known = full32}));
      require(cpu.state().effective[6] == cpu.state().system[6]);
      require(cpu.state().system[8] == state.system[8]);
      require(cpu.state().data[2] == state.data[2]);
    }
  }
  SharcMemory memory;
  program(memory);
  auto state = initial();
  state.system[6] = {.value = multiplier_flags, .known = full32 & ~alu_flags};
  state.effective = state.system;
  state.data[0][1] = known(sign | one_half);
  Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][3] == known(sign | three_quarters));
  require(cpu.state().system[6] ==
          (lead_a1::SharcKnown32{.value = multiply_negative, .known = full32 & ~alu_flags}));
}

void simd_case(bool alternate, unsigned int pattern) {
  constexpr std::uint32_t alternate_banks = 0x480;
  SharcMemory memory;
  program(memory, conditional);
  auto state = initial();
  state.data = {};
  state.system[2].value = simd | (alternate ? alternate_banks : 0);
  for (unsigned int element = 0; element < 2; ++element) {
    state.system[6 + element].value = (pattern >> element) & 1U;
    if ((pattern & (1U << element)) == 0) { continue; }
    const auto bank = (element * 2) + (alternate ? 1U : 0U);
    const auto negative = element == 0 ? 0 : sign;
    state.data[bank][1] = known(negative | one_half);
    state.data[bank][2] = known(two_half);
  }
  state.effective = state.system;
  Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  for (unsigned int element = 0; element < 2; ++element) {
    const auto bank = (element * 2) + (alternate ? 1U : 0U);
    const bool active = (pattern & (1U << element)) != 0;
    const auto negative = element == 0 ? 0 : sign;
    const auto expected = active ? known(negative | three_quarters) : state.data[bank][3];
    require(cpu.state().data[bank][3] == expected);
    require(cpu.state().system[8 + element] == state.system[8 + element]);
  }
}

void simd_conditions_and_banks() {
  for (const bool alternate : {false, true}) {
    for (unsigned int pattern = 0; pattern < 4; ++pattern) { simd_case(alternate, pattern); }
  }
}

void known_bits_and_failure_atomicity() {
  SharcMemory memory;
  program(memory);
  auto state = initial();
  state.system[2].value = round32;
  state.effective = state.system;
  constexpr std::uint64_t upper32 = 0xffffffff00;
  state.data[0][1] = {.value = one, .known = upper32};
  state.data[0][2] = {.value = two, .known = upper32};
  Sharc short_cpu(memory, state);
  static_cast<void>(short_cpu.step());
  require(short_cpu.state().data[0][3] == known(two));
  state.system[2].value = 0;
  state.effective = state.system;
  Sharc missing_extension(memory, state);
  const auto before = missing_extension.state();
  rejected([&] { static_cast<void>(missing_extension.step()); }, "Unknown SHARC floating register");
  require(missing_extension.state() == before && !memory.active());
  state = initial();
  state.system[2].value = simd;
  state.effective = state.system;
  state.data[2][1] = known(one);
  state.pending[2] = state.system[2];
  Sharc second_lane(memory, state);
  const auto paired = second_lane.state();
  rejected([&] { static_cast<void>(second_lane.step()); }, "Unknown SHARC floating register");
  require(second_lane.state() == paired && !memory.active());
}

void guarded_native_results() {
  for (const auto left : {sign, minimum, maximum, infinity, full40}) {
    SharcMemory memory;
    program(memory);
    auto state = initial();
    state.data[0][1] = known(left);
    state.data[0][2] = known(left == minimum ? half : two);
    Sharc cpu(memory, state);
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(cpu.step()); }, "SHARC floating");
    require(cpu.state() == before && !memory.active());
  }
  SharcMemory memory;
  program(memory, conditional);
  auto state = initial();
  state.system[2].known &= ~(round32 | truncation);
  state.effective = state.system;
  state.data = {};
  Sharc inactive(memory, state);
  static_cast<void>(inactive.step());
  require(inactive.state().data == state.data && inactive.state().system == state.system);
  code(memory, base, multiply);
  state = initial();
  state.data[0][1] = {};
  state.data[0][2] = known(0); // Unknown X times zero cannot hide a possible NAN.
  Sharc unknown_zero(memory, state);
  const auto previous = unknown_zero.state();
  rejected([&] { static_cast<void>(unknown_zero.step()); }, "Unknown SHARC floating register");
  require(unknown_zero.state() == previous && !memory.active());
}

void positive_zero_products() {
  for (const auto left : {std::uint64_t{0}, sign, std::uint64_t{1}, sign | 1}) {
    SharcMemory memory;
    program(memory);
    auto state = initial();
    state.system[6].value = multiplier_flags | alu_flags;
    state.effective = state.system;
    state.data[0][1] = known(left);
    state.data[0][2] = known((left & sign) != 0 ? sign | two : two);
    Sharc cpu(memory, state);
    static_cast<void>(cpu.step());
    require(cpu.state().data[0][3] == known(0));
    require(cpu.state().system[6].value == alu_flags);
    require(cpu.state().system[8] == state.system[8]);
  }
}

void precision_latency_and_in_place() {
  SharcMemory memory;
  constexpr std::uint64_t mode_write = 0x0f7200010000;
  program(memory, mode_write);
  code(memory, base + 1, in_place);
  code(memory, base + 2, in_place);
  auto state = initial();
  state.data[0][1] = known(one + 1);
  state.data[0][2] = known(one + 1);
  Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][1] == known(one + 2));
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][1] == known(one));
}

void parallel_register_precedence() {
  constexpr std::uint64_t old_source = one + 1;
  constexpr std::array<std::uint64_t, 2> instructions{0x707e01130430, 0x707e82130430};
  for (const auto instruction : instructions) {
    SharcMemory memory;
    program(memory, instruction);
    auto state = initial();
    state.system[6].value = alu_flags;
    state.effective = state.system;
    state.data[0][0] = known(two_half);
    state.data[0][3] = known(one_half);
    state.data[0][4] = known(old_source);
    state.data[0][5] = known(old_source);
    Sharc cpu(memory, state);
    static_cast<void>(cpu.step());
    const bool conflict = instruction == instructions[1];
    require(cpu.state().data[0][4] == known(conflict ? old_source : three_quarters));
    if (!conflict) { require(cpu.state().data[0][2] == known(old_source)); }
    require(cpu.state().system[6].value == alu_flags);
  }
}

void next_body(Sharc &cpu, SharcFetch &fetch, std::uint64_t expected) {
  for (unsigned int clock = 0; clock < clock_limit; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (!fetch.state().execute) { continue; }
    const auto step = cpu.step(fetch);
    if (step.pc != base + 1) { continue; }
    require(cpu.state().data[0][1] == known(expected));
    return;
  }
  throw std::runtime_error("Missing actual multiply loop execution");
}

void clocked_loop_and_checkpoint() {
  constexpr std::uint64_t five = 0x0c0005000001;
  SharcMemory memory;
  SharcMemory rebound;
  program(memory, five);
  program(rebound, five);
  code(memory, base + 1, in_place);
  code(rebound, base + 1, in_place);
  auto state = initial();
  state.data[0][1] = known(one);
  state.data[0][2] = known(two);
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, flow());
  next_body(cpu, fetch, two);
  constexpr std::uint64_t four = 0x4080000000;
  next_body(cpu, fetch, four);
  Sharc restored(rebound, cpu.state());
  SharcFetch continuation(rebound, fetch.state());
  constexpr std::array<std::uint64_t, 3> remaining{0x4100000000, 0x4180000000, 0x4200000000};
  for (const auto expected : remaining) {
    next_body(cpu, fetch, expected);
    next_body(restored, continuation, expected);
    require(cpu.state() == restored.state() && fetch.state() == continuation.state());
  }
  require(cpu.state().pc == base + 2 && !cpu.state().count_loop && cpu.state().depth == 0);
}

unsigned int run_until(Sharc &cpu, SharcFetch &fetch, std::uint32_t stop) {
  unsigned int control_stalls{};
  for (unsigned int clock = 0; clock < clock_limit; ++clock) {
    const auto event = fetch.advance_sequenced(cpu);
    if (event.control_stall) { ++control_stalls; }
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
    if (cpu.state().pc == stop) { return control_stalls; }
  }
  throw std::runtime_error("Missing actual multiply pipeline frontier");
}

void multiplier_branch_flags() {
  constexpr std::uint32_t target = base + 16;
  constexpr std::uint64_t if_mn = 0x060c00000000;
  constexpr std::uint64_t if_eq = 0x060000000000;
  for (const bool multiplier_condition : {false, true}) {
    SharcMemory memory;
    program(memory);
    code(memory, base + 1, (multiplier_condition ? if_mn : if_eq) | target);
    auto state = initial();
    state.system[6].value = 1; // AZ set, MN clear. MUL must preserve AZ.
    state.effective = state.system;
    state.data[0][1] = known(sign | one_half);
    Sharc cpu(memory, state);
    SharcFetch fetch(memory, flow());
    require(run_until(cpu, fetch, target) == (multiplier_condition ? 1U : 0U));
    require(cpu.state().data[0][3] == known(sign | three_quarters));
    require((cpu.state().system[6].value & multiplier_flags) == multiply_negative);
    require((cpu.state().system[6].value & alu_flags) == 1);
  }
}

void expect_forwarding_guard(Sharc &cpu, SharcFetch &fetch) {
  for (unsigned int clock = 0; clock < clock_limit; ++clock) {
    const auto before_cpu = cpu.state();
    const auto before_fetch = fetch.state();
    try {
      static_cast<void>(fetch.advance_sequenced(cpu));
    } catch (const std::exception &error) {
      require(std::string(error.what()).find("dual multiplier operand forwarding") !=
              std::string::npos);
      require(cpu.state() == before_cpu && fetch.state() == before_fetch);
      rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); },
               "dual multiplier operand forwarding");
      return;
    }
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
  }
  throw std::runtime_error("Missing multiplier dual-input timing guard");
}

void dual_forwarding_guards() {
  constexpr std::uint64_t self_square = 0x013e00130311;
  constexpr std::uint64_t add_f1 = 0x013e00081112;
  constexpr std::uint64_t parallel_pair = 0x707e01021140;
  for (const auto producer : {add_f1, in_place, parallel_pair}) {
    SharcMemory memory;
    program(memory, producer);
    code(memory, base + 1, producer == parallel_pair ? multiply : self_square);
    auto state = initial();
    state.data[0][4] = known(one);
    Sharc cpu(memory, state);
    SharcFetch fetch(memory, flow());
    expect_forwarding_guard(cpu, fetch);
    require(!memory.active());
  }
}

void safe_forwarding_and_gap() {
  constexpr std::uint64_t add_f1 = 0x013e00081112;
  constexpr std::uint64_t self_square = 0x013e00130311;
  constexpr std::uint64_t six = 0x40c0000000;
  constexpr std::uint64_t nine = 0x4110000000;
  for (const bool gap : {false, true}) {
    SharcMemory memory;
    program(memory, add_f1);
    code(memory, base + (gap ? 2U : 1U), gap ? self_square : multiply);
    auto state = initial();
    state.data[0][1] = known(one);
    state.data[0][2] = known(two);
    Sharc cpu(memory, state);
    SharcFetch fetch(memory, flow());
    require(run_until(cpu, fetch, base + (gap ? 3U : 2U)) == 0);
    require(cpu.state().data[0][3] == known(gap ? nine : six));
    require(cpu.state().data[0][1] == known(three));
  }
}

void immediate_payload_is_not_multiply() {
  constexpr std::uint64_t add_f1 = 0x013e00081112;
  constexpr std::uint64_t immediate = 0x0f0400130311;
  constexpr std::uint64_t stored = 0x13031100;
  SharcMemory memory;
  program(memory, add_f1);
  code(memory, base + 1, immediate);
  auto state = initial();
  Sharc cpu(memory, state);
  SharcFetch fetch(memory, flow());
  require(run_until(cpu, fetch, base + 2) == 0);
  require(cpu.state().data[0][4] == known(stored));
}

void malformed_numeric_inputs() {
  rejected([] { static_cast<void>(lead_a1::sharc_float_multiply(full40 + 1, one, wide, nearest)); },
           "bits");
  rejected([] { static_cast<void>(lead_a1::sharc_float_multiply(0, infinity, wide, nearest)); },
           "NAN/infinity");
  rejected([] { static_cast<void>(lead_a1::sharc_float_multiply(full40, one, narrow, nearest)); },
           "NAN/infinity");
  rejected(
      [] {
        const auto invalid = std::bit_cast<SharcFloatFormat>(std::uint8_t{2});
        static_cast<void>(lead_a1::sharc_float_multiply(one, two, invalid, nearest));
      },
      "precision");
  rejected(
      [] {
        const auto invalid = std::bit_cast<SharcFloatRounding>(std::uint8_t{2});
        static_cast<void>(lead_a1::sharc_float_multiply(one, two, wide, invalid));
      },
      "rounding");
}

void rational_oracle(const char *path) {
  std::ifstream source(path);
  if (!source) { throw std::runtime_error("Cannot read product oracle fixture"); }
  std::uint64_t left{};
  std::uint64_t right{};
  std::uint64_t value{};
  unsigned int flags{};
  bool short_input{};
  bool toward_zero{};
  std::uint64_t count{};
  while (source >> left >> right >> value >> flags >> short_input >> toward_zero) {
    const auto actual = lead_a1::sharc_float_multiply(
        left, right, short_input ? narrow : wide, toward_zero ? truncate : nearest);
    const SharcFloatResult expected{
        .value = value, .underflow = (flags & 1) != 0, .overflow = (flags & 2) != 0};
    if (actual != expected) {
      throw std::runtime_error("Exact product mismatch case " + std::to_string(count) +
                               " inputs=" + std::to_string(left) + "," + std::to_string(right) +
                               " expected=" + std::to_string(value) +
                               " actual=" + std::to_string(actual.value));
    }
    ++count;
  }
  constexpr std::uint64_t expected_cases = 230400;
  require(source.eof() && count == expected_cases);
  std::cout << count << " exact rational floating products passed\n";
}

}

int main(int argc, char **argv) try {
  if (argc == 2) {
    rational_oracle(argv[1]);
    return 0;
  }
  require(argc == 1);
  numeric_literals();
  native_rounding_and_status();
  simd_conditions_and_banks();
  known_bits_and_failure_atomicity();
  guarded_native_results();
  positive_zero_products();
  precision_latency_and_in_place();
  malformed_numeric_inputs();
  parallel_register_precedence();
  clocked_loop_and_checkpoint();
  multiplier_branch_flags();
  dual_forwarding_guards();
  safe_forwarding_and_gap();
  immediate_payload_is_not_multiply();
  std::cout << "SHARC finite products and guarded native floating multiply passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
