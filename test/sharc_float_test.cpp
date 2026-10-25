// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_float.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

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
constexpr std::uint32_t full = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t sign = 0x8000000000;
constexpr std::uint64_t one = 0x3f80000000;
constexpr std::uint64_t two = 0x4000000000;
constexpr std::uint64_t three = 0x4040000000;
constexpr std::uint64_t half_ulp40 = 0x2f80000000; // 2^-32, half of a 40-bit ULP at 1.
constexpr std::uint64_t half_ulp32 = 0x3380000000; // 2^-24.
constexpr std::uint64_t minimum = 0x0080000000;
constexpr std::uint64_t maximum = 0x7f7fffffff;
constexpr std::uint64_t infinity = 0x7f80000000;
constexpr std::uint64_t overflow_half40 = 0x6f00000000; // 2^95.
constexpr std::uint64_t overflow_half32 = 0x7300000000; // 2^103.
constexpr std::uint32_t round32 = 0x10000;
constexpr std::uint32_t truncation = 0x8000;
constexpr std::uint32_t simd = 0x200000;
constexpr std::uint32_t alu_flags = 0x43f;
constexpr std::uint32_t preserved = 0xa50020c0;
constexpr std::uint64_t add = 0x013e00081312; // F3=F1+F2, ALU 0x81, PGR12-4.
constexpr std::uint64_t subtract = 0x013e00082312;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC floating expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC floating rejection: " + reason);
}

void map(SharcMemory &memory, std::uint64_t opcode = add) {
  memory.map({.bank = 2, .halfwords = 16, .name = "owned floating ALU program"});
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, opcode);
}

SharcKnown40 known(std::uint64_t value) {
  return {.value = value, .known = full40};
}

struct NumericCase {
  std::uint64_t left{};
  std::uint64_t right{};
  SharcFloatFormat format{wide};
  SharcFloatRounding rounding{nearest};
  bool subtract{};
  SharcFloatResult expected;
};

constexpr std::array literals{
    NumericCase{.left = one, .right = two, .expected = {.value = three}},
    NumericCase{.left = one, .right = two, .subtract = true, .expected = {.value = sign | one}},
    NumericCase{.left = one, .right = one, .subtract = true, .expected = {.value = 0}},
    NumericCase{.left = sign, .right = sign, .expected = {.value = sign}},
    NumericCase{.left = sign, .right = 0, .expected = {.value = 0}},
    NumericCase{.left = 0, .right = 0, .subtract = true, .expected = {.value = 0}},
    NumericCase{.left = 1, .right = one, .expected = {.value = one}},
    NumericCase{.left = sign | 1, .right = sign, .expected = {.value = sign}},
    NumericCase{.left = one, .right = half_ulp40, .expected = {.value = one}},
    NumericCase{.left = one + 1, .right = half_ulp40, .expected = {.value = one + 2}},
    NumericCase{.left = one, .right = half_ulp40 + 1, .expected = {.value = one + 1}},
    NumericCase{
        .left = one, .right = half_ulp40 + 1, .rounding = truncate, .expected = {.value = one}},
    NumericCase{.left = one, .right = half_ulp32, .format = narrow, .expected = {.value = one}},
    NumericCase{
        .left = one + 256, .right = half_ulp32, .format = narrow, .expected = {.value = one + 512}},
    NumericCase{.left = one + 255, .right = 0, .format = narrow, .expected = {.value = one}},
    NumericCase{.left = minimum,
                .right = minimum + 1,
                .subtract = true,
                .expected = {.value = sign, .underflow = true}},
    NumericCase{
        .left = maximum, .right = maximum, .expected = {.value = infinity, .overflow = true}},
    NumericCase{.left = maximum,
                .right = maximum,
                .rounding = truncate,
                .expected = {.value = maximum, .overflow = true}},
    NumericCase{.left = sign | maximum,
                .right = sign | maximum,
                .format = narrow,
                .expected = {.value = sign | infinity, .overflow = true}},
    NumericCase{.left = maximum,
                .right = overflow_half40,
                .expected = {.value = infinity, .overflow = true}},
    NumericCase{.left = maximum,
                .right = overflow_half40,
                .rounding = truncate,
                .expected = {.value = maximum}},
    NumericCase{.left = maximum,
                .right = overflow_half32,
                .format = narrow,
                .expected = {.value = infinity, .overflow = true}},
    NumericCase{.left = maximum,
                .right = overflow_half32,
                .format = narrow,
                .rounding = truncate,
                .expected = {.value = maximum & ~std::uint64_t{255}}},
    NumericCase{.left = minimum, .right = minimum, .expected = {.value = minimum * 2}},
    NumericCase{.left = one,
                .right = minimum,
                .rounding = truncate,
                .subtract = true,
                .expected = {.value = one - 1}}};

void numeric_literals() {
  for (const auto &item : literals) {
    require(lead_a1::sharc_float_add(
                item.left, item.right, item.format, item.rounding, item.subtract) == item.expected);
  }
}

void native_modes_and_flags() {
  for (const bool short_input : {false, true}) {
    for (const bool toward_zero : {false, true}) {
      SharcMemory memory;
      map(memory);
      auto initial = Sharc::reset_reference_214xx(base);
      initial.system[2].value = (short_input ? round32 : 0) | (toward_zero ? truncation : 0);
      initial.system[6] = {.value = preserved | alu_flags, .known = full};
      initial.effective = initial.system;
      initial.data[0][1] = known(one + 1);
      initial.data[0][2] = known(half_ulp40 + 1);
      Sharc cpu(memory, initial);
      static_cast<void>(cpu.step());
      auto expected = one;
      if (!short_input) { expected += toward_zero ? 1U : 2U; }
      require(cpu.state().data[0][3] == known(expected));
      require(cpu.state().system[6] ==
              (lead_a1::SharcKnown32{.value = preserved | 0x400, .known = full}));
      require(cpu.state().effective[6] == cpu.state().system[6]);
      require(cpu.state().system[8] == initial.system[8]);
      require(cpu.state().data[2] == initial.data[2]);
    }
  }
  SharcMemory memory;
  map(memory, subtract);
  auto initial = Sharc::reset_reference_214xx(base);
  initial.data[0][1] = known(one);
  initial.data[0][2] = known(two);
  Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][3] == known(sign | one));
  constexpr std::uint32_t negative_float = 0x404;
  require((cpu.state().system[6].value & alu_flags) == negative_float);
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, add);
  initial = Sharc::reset_reference_214xx(base);
  initial.system[2].value = truncation;
  initial.effective = initial.system;
  initial.data[0][1] = known(maximum);
  initial.data[0][2] = known(overflow_half40);
  Sharc postrounded(memory, initial);
  static_cast<void>(postrounded.step());
  require(postrounded.state().data[0][3] == known(maximum));
  require(postrounded.state().system[8] == initial.system[8]);
}

void ignored_extension_and_rejection() {
  SharcMemory memory;
  map(memory);
  auto initial = Sharc::reset_reference_214xx(base);
  initial.system[2].value = round32;
  initial.effective = initial.system;
  constexpr std::uint64_t upper32 = 0xffffffff00;
  initial.data[0][1] = {.value = one, .known = upper32};
  initial.data[0][2] = {.value = two, .known = upper32};
  Sharc narrow_cpu(memory, initial);
  static_cast<void>(narrow_cpu.step());
  require(narrow_cpu.state().data[0][3] == known(three));
  initial.system[2].value = 0;
  initial.effective = initial.system;
  Sharc wide_cpu(memory, initial);
  const auto before = wide_cpu.state();
  rejected([&] { static_cast<void>(wide_cpu.step()); }, "Unknown SHARC floating register");
  require(wide_cpu.state() == before && !memory.active());
  initial.data[0][1] = known(one);
  initial.data[0][2] = known(two);
  initial.system[2].known &= ~truncation;
  initial.effective = initial.system;
  Sharc unknown_mode(memory, initial);
  const auto previous = unknown_mode.state();
  rejected([&] { static_cast<void>(unknown_mode.step()); }, "rounding/precision mode");
  require(unknown_mode.state() == previous && !memory.active());
}

void guarded_native_boundaries() {
  for (const auto right : {sign | one, sign | (minimum - 1), maximum, infinity, full40}) {
    SharcMemory memory;
    map(memory);
    auto initial = Sharc::reset_reference_214xx(base);
    initial.data[0][1] = known(right == maximum ? maximum : one);
    initial.data[0][2] = known(right);
    if (right == (sign | (minimum - 1))) {
      initial.data[0][1] = known(minimum);
      initial.data[0][2] = known(sign | (minimum + 1));
    }
    initial.pending[2] = initial.system[2];
    Sharc cpu(memory, initial);
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(cpu.step()); }, "SHARC floating");
    require(cpu.state() == before && !memory.active());
  }
}

void simd_conditions_and_rollback() {
  constexpr std::uint64_t conditional = 0x010000081312;
  for (unsigned int pattern = 0; pattern < 4; ++pattern) {
    SharcMemory memory;
    map(memory, conditional);
    auto initial = Sharc::reset_reference_214xx(base);
    initial.system[2].value = simd;
    for (unsigned int element = 0; element < 2; ++element) {
      const bool active = (pattern & (1U << element)) != 0;
      initial.system[6 + element].value = active ? 1 : 0;
      if (active) {
        const auto bank = element * 2;
        const auto negative = element == 0 ? 0 : sign;
        initial.data[bank][1] = known(negative | one);
        initial.data[bank][2] = known(negative | two);
      }
    }
    initial.effective = initial.system;
    Sharc cpu(memory, initial);
    static_cast<void>(cpu.step());
    for (unsigned int element = 0; element < 2; ++element) {
      const bool active = (pattern & (1U << element)) != 0;
      const auto expected = element == 0 ? three : sign | three;
      const auto bank = element * 2;
      require(cpu.state().data[bank][3] == (active ? known(expected) : initial.data[bank][3]));
      require(cpu.state().system[8 + element] == initial.system[8 + element]);
    }
  }
  SharcMemory memory;
  map(memory);
  auto initial = Sharc::reset_reference_214xx(base);
  initial.system[2].value = simd;
  initial.effective = initial.system;
  initial.data[0][1] = known(one);
  initial.data[0][2] = known(two);
  initial.data[2][1] = known(one); // Second PE is missing its other input.
  Sharc cpu(memory, initial);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Unknown SHARC floating register");
  require(cpu.state() == before && !memory.active());
}

void precision_latency_and_banks() {
  SharcMemory memory;
  constexpr std::uint64_t mode_write = 0x0f7200010000;
  constexpr std::uint64_t in_place = 0x013e00081112;
  map(memory, mode_write);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, in_place);
  memory.write({.address = base + 2, .width = lead_a1::SharcWidth::wide48}, in_place);
  auto initial = Sharc::reset_reference_214xx(base);
  initial.data[0][1] = known(one);
  initial.data[0][2] = known(half_ulp40 + 1);
  Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][1] == known(one + 1));
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][1] == known(one));
  constexpr std::uint32_t low_secondary = 0x400;
  initial = Sharc::reset_reference_214xx(base + 1);
  initial.system[2].value = low_secondary;
  initial.effective = initial.system;
  initial.data[1][1] = known(one);
  initial.data[1][2] = known(two);
  Sharc banked(memory, initial);
  static_cast<void>(banked.step());
  require(banked.state().data[1][1] == known(three));
  require(banked.state().data[0] == initial.data[0]);
}

void parallel_register_precedence() {
  constexpr std::uint64_t old_source = one + 1;
  // PGR Type5a fields: F4=F3+F0 while R2=old R4, then R4=old R5.
  constexpr std::array<std::uint64_t, 2> instructions{0x707e01081430, 0x707e82081430};
  for (const auto instruction : instructions) {
    SharcMemory memory;
    map(memory, instruction);
    auto initial = Sharc::reset_reference_214xx(base);
    initial.data[0][0] = known(two);
    initial.data[0][3] = known(one);
    initial.data[0][4] = known(old_source);
    initial.data[0][5] = known(old_source);
    Sharc cpu(memory, initial);
    static_cast<void>(cpu.step());
    const bool conflict = instruction == instructions[1];
    require(cpu.state().data[0][4] == known(conflict ? old_source : three));
    if (!conflict) { require(cpu.state().data[0][2] == known(old_source)); }
    constexpr std::uint32_t floating_flag = 0x400;
    require((cpu.state().system[6].value & alu_flags) == floating_flag);
  }
}

void inactive_and_unknown_alias() {
  SharcMemory memory;
  constexpr std::uint64_t conditional = 0x010000081312;
  map(memory, conditional);
  auto initial = Sharc::reset_reference_214xx(base);
  initial.system[2].known &= ~(round32 | truncation);
  initial.effective = initial.system;
  Sharc inactive(memory, initial); // EQ false, inputs and rounding are unknown.
  static_cast<void>(inactive.step());
  require(inactive.state().data == initial.data && inactive.state().system == initial.system);
  constexpr std::uint64_t self_subtract = 0x013e00082311;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, self_subtract);
  initial = Sharc::reset_reference_214xx(base);
  Sharc unknown(memory, initial);
  const auto before = unknown.state();
  rejected([&] { static_cast<void>(unknown.step()); }, "Unknown SHARC floating register");
  require(unknown.state() == before && !memory.active());
}

void loop_program(SharcMemory &memory) {
  constexpr unsigned int cells = 96;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned floating loop"});
  for (unsigned int index = 0; index < cells / 3; ++index) {
    memory.write({.address = base + index, .width = lead_a1::SharcWidth::wide48}, 0);
  }
  constexpr std::uint64_t five = 0x0c0005000001;
  constexpr std::uint64_t accumulate = 0x013e00081112;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, five);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, accumulate);
}

void next_body(Sharc &cpu, lead_a1::SharcFetch &fetch, std::uint64_t expected) {
  constexpr unsigned int limit = 32;
  for (unsigned int clock = 0; clock < limit; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (!fetch.state().execute) { continue; }
    const auto step = cpu.step(fetch);
    if (step.pc != base + 1) { continue; }
    require(cpu.state().data[0][1] == known(expected));
    return;
  }
  throw std::runtime_error("Missing actual floating loop execution");
}

void clocked_loop_and_checkpoint() {
  SharcMemory memory;
  SharcMemory restored_memory;
  loop_program(memory);
  loop_program(restored_memory);
  auto initial = Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t cache_disabled = 0x10;
  initial.system[4].value |= cache_disabled;
  initial.effective = initial.system;
  initial.data[0][1] = known(one);
  initial.data[0][2] = known(one);
  Sharc cpu(memory, initial);
  lead_a1::SharcFetchState flow{};
  flow.flow = lead_a1::SharcFetchFlow{.next = base};
  lead_a1::SharcFetch fetch(memory, flow);
  next_body(cpu, fetch, two);
  next_body(cpu, fetch, three);
  Sharc restored(restored_memory, cpu.state());
  lead_a1::SharcFetch continuation(restored_memory, fetch.state());
  constexpr std::array<std::uint64_t, 3> remaining{0x4080000000, 0x40a0000000, 0x40c0000000};
  for (const auto expected : remaining) {
    next_body(cpu, fetch, expected);
    next_body(restored, continuation, expected);
    require(cpu.state() == restored.state() && fetch.state() == continuation.state());
  }
  require(cpu.state().pc == base + 2 && !cpu.state().count_loop && cpu.state().depth == 0);
}

void malformed_numeric_input() {
  rejected([] { static_cast<void>(lead_a1::sharc_float_add(full40 + 1, one, wide, nearest)); },
           "bits");
  rejected([] { static_cast<void>(lead_a1::sharc_float_add(infinity, one, wide, nearest)); },
           "NAN/infinity");
  rejected([] { static_cast<void>(lead_a1::sharc_float_add(one, full40, narrow, nearest)); },
           "NAN/infinity");
  rejected(
      [] {
        const auto invalid = std::bit_cast<SharcFloatFormat>(std::uint8_t{2});
        static_cast<void>(lead_a1::sharc_float_add(one, two, invalid, nearest));
      },
      "precision");
  rejected(
      [] {
        const auto invalid = std::bit_cast<SharcFloatRounding>(std::uint8_t{2});
        static_cast<void>(lead_a1::sharc_float_add(one, two, wide, invalid));
      },
      "rounding");
}

void rational_oracle(const char *path) {
  std::ifstream source(path);
  if (!source) { throw std::runtime_error("Cannot read exact floating oracle fixture"); }
  std::uint64_t left{};
  std::uint64_t right{};
  std::uint64_t value{};
  unsigned int flags{};
  bool short_input{};
  bool toward_zero{};
  bool subtract_input{};
  std::uint64_t count{};
  while (source >> left >> right >> value >> flags >> short_input >> toward_zero >>
         subtract_input) {
    const auto actual = lead_a1::sharc_float_add(
        left, right, short_input ? narrow : wide, toward_zero ? truncate : nearest, subtract_input);
    const SharcFloatResult expected{
        .value = value, .underflow = (flags & 1) != 0, .overflow = (flags & 2) != 0};
    if (actual != expected) {
      throw std::runtime_error("Exact Fraction mismatch case " + std::to_string(count) +
                               " inputs=" + std::to_string(left) + "," + std::to_string(right) +
                               " expected=" + std::to_string(value) +
                               " actual=" + std::to_string(actual.value));
    }
    ++count;
  }
  constexpr std::uint64_t expected_cases = 460800;
  require(source.eof() && count == expected_cases);
  std::cout << count << " exact rational floating expectations passed\n";
}

}

int main(int argc, char **argv) try {
  if (argc == 2) {
    rational_oracle(argv[1]);
    return 0;
  }
  require(argc == 1);
  numeric_literals();
  native_modes_and_flags();
  ignored_extension_and_rejection();
  guarded_native_boundaries();
  simd_conditions_and_rollback();
  precision_latency_and_banks();
  parallel_register_precedence();
  inactive_and_unknown_alias();
  clocked_loop_and_checkpoint();
  malformed_numeric_input();
  std::cout << "SHARC finite floating sums and guarded native normal add/sub passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
