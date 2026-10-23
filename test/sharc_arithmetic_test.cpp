// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t full = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t saturation = 0x2000;
constexpr std::uint32_t simd = 0x200000;
constexpr std::uint32_t alu_mask = 0x43f;
constexpr std::uint32_t preserved = 0xa50000c0;
constexpr std::uint64_t add = 0x013e00001312; // R3=R1+R2, PGR12-2/3.
constexpr std::uint64_t subtract = 0x013e00002312;
constexpr std::uint64_t increment = 0x013e00029312; // Ry2 is unused.
constexpr std::uint64_t decrement = 0x013e0002a312;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC arithmetic expectation at line " +
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
  throw std::runtime_error("Missing SHARC arithmetic rejection: " + reason);
}

void map(lead_a1::SharcMemory &memory, std::uint64_t opcode) {
  constexpr std::uint32_t cells = 96;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned integer ALU program"});
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, opcode);
}

lead_a1::SharcKnown40 word(std::uint32_t value) {
  constexpr std::uint64_t dirty_extension = 0xa5;
  return {.value = (std::uint64_t{value} << 8) | dirty_extension, .known = full40};
}

struct ArithmeticCase {
  std::uint64_t opcode;
  std::uint32_t left;
  std::uint32_t right;
  std::uint32_t wrapped;
  std::uint32_t clamped;
  std::uint32_t flags;
  std::uint32_t clamped_flags;
};

// Independent literal results: AZ=1,AV=2,AN=4,AC=8. SUB carry is no-borrow.
constexpr std::array<ArithmeticCase, 22> cases{{
    {.opcode = add,
     .left = 0,
     .right = 0,
     .wrapped = 0,
     .clamped = 0,
     .flags = 1,
     .clamped_flags = 1},
    {.opcode = add,
     .left = 0xffffffff,
     .right = 1,
     .wrapped = 0,
     .clamped = 0,
     .flags = 9,
     .clamped_flags = 9},
    {.opcode = add,
     .left = 0x7fffffff,
     .right = 1,
     .wrapped = 0x80000000,
     .clamped = 0x7fffffff,
     .flags = 6,
     .clamped_flags = 2},
    {.opcode = add,
     .left = 0x80000000,
     .right = 0x80000000,
     .wrapped = 0,
     .clamped = 0x80000000,
     .flags = 11,
     .clamped_flags = 14},
    {.opcode = add,
     .left = 0xffffffff,
     .right = 0xffffffff,
     .wrapped = 0xfffffffe,
     .clamped = 0xfffffffe,
     .flags = 12,
     .clamped_flags = 12},
    {.opcode = add,
     .left = 0x80000000,
     .right = 0xffffffff,
     .wrapped = 0x7fffffff,
     .clamped = 0x80000000,
     .flags = 10,
     .clamped_flags = 14},
    {.opcode = subtract,
     .left = 0,
     .right = 1,
     .wrapped = 0xffffffff,
     .clamped = 0xffffffff,
     .flags = 4,
     .clamped_flags = 4},
    {.opcode = subtract,
     .left = 1,
     .right = 1,
     .wrapped = 0,
     .clamped = 0,
     .flags = 9,
     .clamped_flags = 9},
    {.opcode = subtract,
     .left = 0,
     .right = 0,
     .wrapped = 0,
     .clamped = 0,
     .flags = 9,
     .clamped_flags = 9},
    {.opcode = subtract,
     .left = 0x80000000,
     .right = 1,
     .wrapped = 0x7fffffff,
     .clamped = 0x80000000,
     .flags = 10,
     .clamped_flags = 14},
    {.opcode = subtract,
     .left = 0x7fffffff,
     .right = 0xffffffff,
     .wrapped = 0x80000000,
     .clamped = 0x7fffffff,
     .flags = 6,
     .clamped_flags = 2},
    {.opcode = subtract,
     .left = 0xffffffff,
     .right = 0xffffffff,
     .wrapped = 0,
     .clamped = 0,
     .flags = 9,
     .clamped_flags = 9},
    {.opcode = subtract,
     .left = 0,
     .right = 0x80000000,
     .wrapped = 0x80000000,
     .clamped = 0x7fffffff,
     .flags = 6,
     .clamped_flags = 2},
    {.opcode = increment,
     .left = 0,
     .right = 0,
     .wrapped = 1,
     .clamped = 1,
     .flags = 0,
     .clamped_flags = 0},
    {.opcode = increment,
     .left = 0x7fffffff,
     .right = 0,
     .wrapped = 0x80000000,
     .clamped = 0x7fffffff,
     .flags = 6,
     .clamped_flags = 2},
    {.opcode = increment,
     .left = 0xffffffff,
     .right = 0,
     .wrapped = 0,
     .clamped = 0,
     .flags = 9,
     .clamped_flags = 9},
    {.opcode = increment,
     .left = 0x80000000,
     .right = 0,
     .wrapped = 0x80000001,
     .clamped = 0x80000001,
     .flags = 4,
     .clamped_flags = 4},
    {.opcode = decrement,
     .left = 0,
     .right = 0,
     .wrapped = 0xffffffff,
     .clamped = 0xffffffff,
     .flags = 4,
     .clamped_flags = 4},
    {.opcode = decrement,
     .left = 1,
     .right = 0,
     .wrapped = 0,
     .clamped = 0,
     .flags = 9,
     .clamped_flags = 9},
    {.opcode = decrement,
     .left = 0x80000000,
     .right = 0,
     .wrapped = 0x7fffffff,
     .clamped = 0x80000000,
     .flags = 10,
     .clamped_flags = 14},
    {.opcode = decrement,
     .left = 0xffffffff,
     .right = 0,
     .wrapped = 0xfffffffe,
     .clamped = 0xfffffffe,
     .flags = 12,
     .clamped_flags = 12},
    {.opcode = decrement,
     .left = 0x7fffffff,
     .right = 0,
     .wrapped = 0x7ffffffe,
     .clamped = 0x7ffffffe,
     .flags = 8,
     .clamped_flags = 8},
}};

void literal_boundaries() {
  for (const auto item : cases) {
    for (const bool saturate : {false, true}) {
      lead_a1::SharcMemory memory;
      map(memory, item.opcode);
      auto initial = lead_a1::Sharc::reset_reference_214xx(base);
      initial.system[2].value = saturate ? saturation : 0;
      initial.system[6] = {.value = preserved | alu_mask, .known = full};
      initial.effective = initial.system;
      initial.data[0][1] = word(item.left);
      if (item.opcode == add || item.opcode == subtract) { initial.data[0][2] = word(item.right); }
      lead_a1::Sharc cpu(memory, initial);
      static_cast<void>(cpu.step());
      const auto result = saturate ? item.clamped : item.wrapped;
      const auto flags = saturate ? item.clamped_flags : item.flags;
      require((cpu.state().data[0][3] ==
               lead_a1::SharcKnown40{.value = std::uint64_t{result} << 8, .known = full40}));
      require(cpu.state().system[6].value == (preserved | flags));
      require(cpu.state().system[6].known == full);
      require(cpu.state().effective[6] == cpu.state().system[6]);
      const auto sticky = initial.system[8].value | ((item.flags & 2) != 0 ? 4U : 0U);
      require(cpu.state().system[8].value == sticky && cpu.state().system[8].known == full);
      require(cpu.state().effective[8] == cpu.state().system[8]);
      require(cpu.state().system[7] == initial.system[7] &&
              cpu.state().system[9] == initial.system[9]);
      require(cpu.state().data[2] == initial.data[2]);
    }
  }
}

void sticky_overflow_and_unused_operand() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t increment_high_unused = 0x013e0002931c;
  constexpr std::uint64_t decrement_r5 = 0x013e0002a540;
  map(memory, increment_high_unused);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, decrement_r5);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  // Unknown high-bank choice cannot be consulted for an unused Ry12 field.
  constexpr std::uint32_t high_bank = 0x80;
  initial.system[2].known &= ~high_bank;
  initial.system[8].known &= ~std::uint32_t{4};
  initial.effective = initial.system;
  constexpr std::uint32_t maximum = 0x7fffffff;
  initial.data[0][1] = word(maximum);
  initial.data[0][4] = word(1);
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  require((cpu.state().system[8].value & 4) != 0 && (cpu.state().system[8].known & 4) != 0);
  const auto sticky = cpu.state().system[8];
  static_cast<void>(cpu.step());
  require((cpu.state().system[6].value & alu_mask) == 9 && cpu.state().system[8] == sticky);
  require(cpu.state().data[0][5].value == 0 && cpu.state().data[0][5].known == full40);
}

void check_simd_element(const lead_a1::SharcState &actual,
                        const lead_a1::SharcState &initial,
                        unsigned int element) {
  const auto bank = element * 2;
  if (initial.system[6 + element].value == 0) {
    require(actual.data[bank] == initial.data[bank]);
    require(actual.system[6 + element] == initial.system[6 + element]);
    require(actual.system[8 + element] == initial.system[8 + element]);
    return;
  }
  constexpr std::uint64_t maximum = 0x7fffffff;
  const auto value = element == 0 ? maximum << 8 : 0;
  const auto flags = element == 0 ? 2U : 9U;
  const auto sticky = element == 0 ? 4U : 0U;
  require(actual.data[bank][3].value == value && actual.data[bank][3].known == full40);
  require((actual.system[6 + element].value & alu_mask) == flags);
  require(actual.system[8 + element].value == (initial.system[8 + element].value | sticky));
}

void simd_conditions() {
  constexpr std::uint64_t if_eq_add = 0x010000001312;
  constexpr std::uint32_t maximum = 0x7fffffff;
  for (unsigned int pattern = 0; pattern < 4; ++pattern) {
    lead_a1::SharcMemory memory;
    map(memory, if_eq_add);
    auto initial = lead_a1::Sharc::reset_reference_214xx(base);
    initial.system[2].value = simd | saturation;
    for (unsigned int element = 0; element < 2; ++element) {
      const auto bank = element * 2;
      const bool executes = (pattern & (1U << element)) != 0;
      initial.system[6 + element].value = executes ? 1 : 0;
      if (executes) {
        initial.data[bank][1] = word(element == 0 ? maximum : full);
        initial.data[bank][2] = word(1);
      }
    }
    initial.effective = initial.system;
    lead_a1::Sharc cpu(memory, initial);
    static_cast<void>(cpu.step());
    check_simd_element(cpu.state(), initial, 0);
    check_simd_element(cpu.state(), initial, 1);
  }
}

void bank_selection_and_in_place() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t add_r9 = 0x013e0000191a;
  constexpr std::uint64_t inplace_decrement = 0x013e0002a992;
  constexpr std::uint32_t low_secondary = 0x400;
  map(memory, add_r9);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, inplace_decrement);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[2].value = low_secondary;
  initial.effective = initial.system;
  constexpr std::uint32_t value = 0x12345678;
  initial.data[1][1] = word(value);
  initial.data[0][10] = word(1);
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][9].value == (std::uint64_t{value + 1} << 8));
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][9].value == (std::uint64_t{value} << 8));
  require(cpu.state().data[1][9] == initial.data[1][9]);
}

void saturation_effect_latency() {
  constexpr std::uint64_t mode_write = 0x0f7200000000;
  constexpr std::uint32_t maximum = 0x7fffffff;
  constexpr std::uint32_t minimum = 0x80000000;
  for (const bool old_saturation : {false, true}) {
    lead_a1::SharcMemory memory;
    map(memory, mode_write | (old_saturation ? 0 : saturation));
    memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, increment);
    memory.write({.address = base + 2, .width = lead_a1::SharcWidth::wide48}, increment);
    auto initial = lead_a1::Sharc::reset_reference_214xx(base);
    initial.system[2].value = old_saturation ? saturation : 0;
    initial.effective = initial.system;
    initial.data[0][1] = word(maximum);
    lead_a1::Sharc cpu(memory, initial);
    static_cast<void>(cpu.step());
    require(cpu.state().system[2].value != cpu.state().effective[2].value);
    static_cast<void>(cpu.step());
    require(cpu.state().data[0][3].value ==
            (std::uint64_t{old_saturation ? maximum : minimum} << 8));
    static_cast<void>(cpu.step());
    require(cpu.state().data[0][3].value ==
            (std::uint64_t{old_saturation ? minimum : maximum} << 8));
  }
}

void unknown_inputs_and_rollback() {
  lead_a1::SharcMemory memory;
  map(memory, add);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[2].value = simd;
  initial.effective = initial.system;
  constexpr std::uint32_t maximum = 0x7fffffff;
  initial.data[0][1] = word(maximum);
  initial.data[0][2] = word(1);
  initial.data[2][1] = word(1); // PEy R2 remains unknown, after tentative PEx overflow.
  initial.pending[2] = initial.system[2];
  lead_a1::Sharc failing(memory, initial);
  const auto before = failing.state();
  rejected([&] { static_cast<void>(failing.step()); }, "Unknown SHARC register operand");
  require(failing.state() == before && !memory.active());
  initial.system[2].value = 0;
  initial.effective = initial.system;
  initial.effective[2].known &= ~saturation;
  initial.pending[2].reset();
  lead_a1::Sharc unknown_mode(memory, initial);
  const auto unknown = unknown_mode.state();
  rejected([&] { static_cast<void>(unknown_mode.step()); }, "Unknown SHARC ALU saturation");
  require(unknown_mode.state() == unknown && !memory.active());
  initial.data[0][1] = word(1); // A nonoverflowing result does not need ALUSAT.
  initial.system[8].known &= ~std::uint32_t{4};
  initial.effective[8] = initial.system[8];
  lead_a1::Sharc irrelevant_mode(memory, initial);
  static_cast<void>(irrelevant_mode.step());
  require(irrelevant_mode.state().data[0][3].value == (std::uint64_t{2} << 8));
  require(irrelevant_mode.state().system[8] == initial.system[8]);
}

void arithmetic_short_loop() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t three_iterations = 0x0c0003000001;
  constexpr std::uint64_t inplace_increment = 0x013e00029112;
  map(memory, three_iterations);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, inplace_increment);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.data[0][1] = word(0);
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  for (unsigned int index = 1; index <= 3; ++index) {
    static_cast<void>(cpu.step());
    require(cpu.state().data[0][1].value == (std::uint64_t{index} << 8));
  }
  require(cpu.state().pc == base + 2 && !cpu.state().count_loop && cpu.state().depth == 0);
}

void alias_cancellation() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t self_subtract = 0x013e00002311;
  map(memory, self_subtract);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[2].known &= ~saturation;
  initial.effective = initial.system;
  lead_a1::Sharc cpu(memory, initial); // Both operands are the same unknown R1.
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][3].value == 0 && cpu.state().data[0][3].known == full40);
  require(cpu.state().system[6].value == 9); // Zero, no borrow, no overflow.
  require(cpu.state().data[0][1] == initial.data[0][1]);
  require(cpu.state().system[8] == initial.system[8]);
}

}

int main() {
  try {
    literal_boundaries();
    sticky_overflow_and_unused_operand();
    simd_conditions();
    bank_selection_and_in_place();
    saturation_effect_latency();
    unknown_inputs_and_rollback();
    arithmetic_short_loop();
    alias_cancellation();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
