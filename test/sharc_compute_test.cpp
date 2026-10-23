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
constexpr std::uint32_t target = 0xc0010;
constexpr std::uint32_t full = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t simd = 0x200000;
constexpr std::uint32_t preserved_flags = 0xa5047bc0;
constexpr std::uint32_t alu_mask = 0x43f;
constexpr std::uint64_t pass = 0x013e00021312; // R3=PASS R1; unused Ry2.
constexpr std::uint64_t bit_and = 0x013e00040312;
constexpr std::uint64_t bit_or = 0x013e00041312;
constexpr std::uint64_t bit_xor = 0x013e00042312;
constexpr std::uint64_t bit_not = 0x013e00043312;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC compute expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC compute rejection: " + reason);
}

void map(lead_a1::SharcMemory &memory, std::uint64_t opcode) {
  constexpr std::uint32_t cells = 96;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned native ALU program"});
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, opcode);
}

lead_a1::SharcKnown40 word(std::uint32_t value) {
  constexpr std::uint64_t dirty_extension = 0xa7;
  return {.value = (std::uint64_t{value} << 8) | dirty_extension, .known = full40};
}

struct LogicalCase {
  std::uint64_t opcode;
  std::uint32_t left;
  std::uint32_t right;
  std::uint32_t expected;
  std::uint32_t flags;
};

void logical_literals() {
  constexpr std::array<LogicalCase, 8> cases{
      {{.opcode = pass, .left = 0xf0f0ff00, .right = 0, .expected = 0xf0f0ff00, .flags = 4},
       {.opcode = bit_and,
        .left = 0xf0f0ff00,
        .right = 0x0ff00ff0,
        .expected = 0x00f00f00,
        .flags = 0},
       {.opcode = bit_or,
        .left = 0xf0f0ff00,
        .right = 0x0ff00ff0,
        .expected = 0xfff0fff0,
        .flags = 4},
       {.opcode = bit_xor,
        .left = 0xf0f0ff00,
        .right = 0x0ff00ff0,
        .expected = 0xff00f0f0,
        .flags = 4},
       {.opcode = bit_not, .left = 0xf0f0ff00, .right = 0, .expected = 0x0f0f00ff, .flags = 0},
       {.opcode = bit_not, .left = 0xffffffff, .right = 0, .expected = 0, .flags = 1},
       {.opcode = bit_and, .left = 0x80000000, .right = 0x7fffffff, .expected = 0, .flags = 1},
       {.opcode = pass, .left = 0x80000000, .right = 0, .expected = 0x80000000, .flags = 4}}};
  for (const auto item : cases) {
    lead_a1::SharcMemory memory;
    map(memory, item.opcode);
    auto initial = lead_a1::Sharc::reset_reference_214xx(base);
    initial.system[6] = {.value = preserved_flags | alu_mask, .known = full};
    initial.effective = initial.system;
    initial.data[0][1] = word(item.left);
    initial.data[0][2] = word(item.right);
    lead_a1::Sharc cpu(memory, initial);
    static_cast<void>(cpu.step());
    require((cpu.state().data[0][3] ==
             lead_a1::SharcKnown40{.value = std::uint64_t{item.expected} << 8, .known = full40}));
    require(cpu.state().system[6].value == (preserved_flags | item.flags));
    require(cpu.state().effective[6] == cpu.state().system[6]);
    require(cpu.state().system[8] == initial.system[8] &&
            cpu.state().system[9] == initial.system[9]);
    require(cpu.state().data[2][3].known == 0 && cpu.state().system[7] == initial.system[7]);
  }
}

void per_element_conditions() {
  constexpr std::uint64_t if_eq_pass = 0x010000021312;
  constexpr std::uint32_t x_input = 0x87654321;
  constexpr std::uint32_t y_input = 0x12345678;
  for (unsigned int pattern = 0; pattern < 4; ++pattern) {
    lead_a1::SharcMemory memory;
    map(memory, if_eq_pass);
    auto initial = lead_a1::Sharc::reset_reference_214xx(base);
    initial.system[2].value = simd;
    initial.system[6].value = (pattern & 1) != 0 ? 1 : 0;
    initial.system[7].value = (pattern & 2) != 0 ? 1 : 0;
    initial.effective = initial.system;
    initial.data[0][1] = word(x_input);
    initial.data[2][1] = word(y_input);
    lead_a1::Sharc cpu(memory, initial);
    static_cast<void>(cpu.step());
    for (unsigned int element = 0; element < 2; ++element) {
      const auto bank = element * 2;
      const bool executes = (pattern & (1U << element)) != 0;
      if (executes) {
        constexpr std::uint64_t low_mask = 0xff;
        require(cpu.state().data[bank][3].value == (initial.data[bank][1].value & ~low_mask));
        require(cpu.state().data[bank][3].known == full40);
        require((cpu.state().system[6 + element].value & alu_mask) == (element == 0 ? 4U : 0U));
      } else {
        require(cpu.state().data[bank][3].known == 0);
        require(cpu.state().system[6 + element] == initial.system[6 + element]);
      }
    }
  }
}

void partial_knowledge_and_aliases() {
  lead_a1::SharcMemory memory;
  map(memory, bit_and);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.data[0][2] = word(0); // Unknown AND zero is independently known zero.
  lead_a1::Sharc zero(memory, initial);
  static_cast<void>(zero.step());
  require(zero.state().data[0][3].known == full40 && zero.state().data[0][3].value == 0);
  require((zero.state().system[6].value & alu_mask) == 1);
  constexpr std::uint64_t xor_self = 0x013e00042311;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, xor_self);
  lead_a1::Sharc alias(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(alias.step());
  require(alias.state().data[0][3].known == full40 && alias.state().data[0][3].value == 0);
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, pass);
  lead_a1::Sharc unknown(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(unknown.step());
  constexpr std::uint64_t extension = 0xff;
  require(unknown.state().data[0][3].known == extension);
  require((unknown.state().system[6].known & 5) == 0);
  require((unknown.state().system[6].known & (alu_mask & ~5U)) == (alu_mask & ~5U));
  // In-place NOT reads old source bits before overwriting the register.
  constexpr std::uint64_t not_in_place = 0x013e0004311f;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, not_in_place);
  initial.data[0][1] = word(full);
  initial.data[0][15] = {}; // Unused Ry must not become an operand.
  lead_a1::Sharc in_place(memory, initial);
  static_cast<void>(in_place.step());
  require(in_place.state().data[0][1].value == 0 && in_place.state().data[0][1].known == full40);
}

void bank_halves() {
  constexpr std::uint64_t pass_high = 0x013e0002191f; // R9=PASS R1.
  constexpr std::array<std::uint32_t, 4> modes{0, 0x80, 0x400, 0x480};
  for (const auto mode : modes) {
    lead_a1::SharcMemory memory;
    map(memory, pass_high);
    auto initial = lead_a1::Sharc::reset_reference_214xx(base);
    initial.system[2].value = mode;
    initial.effective = initial.system;
    initial.data[0][1] = word(1);
    initial.data[1][1] = word(2);
    lead_a1::Sharc cpu(memory, initial);
    static_cast<void>(cpu.step());
    constexpr std::uint32_t low_secondary = 0x400;
    constexpr std::uint32_t high_secondary = 0x80;
    const auto bank = (mode & high_secondary) != 0 ? 1U : 0U;
    const auto expected = (mode & low_secondary) != 0 ? 2U : 1U;
    require(cpu.state().data[bank][9].value == (std::uint64_t{expected} << 8));
    require(cpu.state().data[1 - bank][9].known == 0);
  }
}

void flag_bit_regression() {
  constexpr std::uint64_t jump_ac = 0x0606000c0010;
  constexpr std::uint64_t jump_av = 0x0608000c0010;

  struct Case {
    std::uint64_t opcode;
    std::uint32_t astat;
    bool taken;
  };

  constexpr std::array<Case, 5> cases{
      {{.opcode = jump_ac, .astat = 4, .taken = false}, // AN is not AC.
       {.opcode = jump_ac, .astat = 8, .taken = true},
       {.opcode = jump_ac, .astat = 2, .taken = false},
       {.opcode = jump_av, .astat = 8, .taken = false}, // AC is not AV.
       {.opcode = jump_av, .astat = 2, .taken = true}}};
  for (const auto item : cases) {
    lead_a1::SharcMemory memory;
    map(memory, item.opcode);
    auto initial = lead_a1::Sharc::reset_reference_214xx(base);
    initial.system[6].value = item.astat;
    initial.effective = initial.system;
    lead_a1::Sharc cpu(memory, initial);
    require(cpu.step().next_pc == (item.taken ? target : base + 1));
  }
}

void guards_and_rollback() {
  lead_a1::SharcMemory memory;
  map(memory, pass);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[2].value = simd;
  initial.effective = initial.system;
  initial.data[0][1] = word(1);
  initial.effective[2].known &= ~std::uint32_t{0x400}; // Cannot choose the bank.
  lead_a1::Sharc unknown_bank(memory, initial);
  const auto before = unknown_bank.state();
  rejected([&] { static_cast<void>(unknown_bank.step()); }, "bank selection");
  require(unknown_bank.state() == before && !memory.active());
  constexpr std::uint64_t unsupported_float = 0x013e00081312;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, unsupported_float);
  lead_a1::Sharc floating(memory, lead_a1::Sharc::reset_reference_214xx(base));
  rejected([&] { static_cast<void>(floating.step()); }, "arithmetic/float/compare");
  constexpr std::uint64_t reserved = 0x013e00821312;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, reserved);
  lead_a1::Sharc invalid(memory, lead_a1::Sharc::reset_reference_214xx(base));
  rejected([&] { static_cast<void>(invalid.step()); }, "reserved compute-only");
  constexpr std::uint64_t empty = 0x013e00000000;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, empty);
  lead_a1::Sharc no_compute(memory, lead_a1::Sharc::reset_reference_214xx(base));
  rejected([&] { static_cast<void>(no_compute.step()); }, "empty compute-only form");
}

void inactive_operands_and_unknown_conditions() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t if_eq_pass = 0x010000021312;
  map(memory, if_eq_pass);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[7] = {}; // SISD must not evaluate the inactive PEy condition.
  initial.effective = initial.system;
  lead_a1::Sharc inactive(memory, initial);
  require(inactive.step().next_pc == base + 1);
  require(inactive.state().data == initial.data && inactive.state().system == initial.system);
  initial.system[2].value = simd;
  initial.system[6].value = 1;
  initial.effective = initial.system;
  initial.data[0][1] = word(1);
  lead_a1::Sharc unknown_y(memory, initial);
  const auto before = unknown_y.state();
  rejected([&] { static_cast<void>(unknown_y.step()); }, "Unknown SHARC control/status bits");
  require(unknown_y.state() == before && !memory.active());
}

void logical_short_loop() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t do5 = 0x0c0005000001;
  constexpr std::uint64_t not_r1 = 0x013e0004311f;
  map(memory, do5);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, not_r1);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.data[0][1] = word(0);
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  constexpr std::array<std::uint64_t, 5> expected{0xffffffff00, 0, 0xffffffff00, 0, 0xffffffff00};
  for (const auto value : expected) {
    static_cast<void>(cpu.step());
    require(cpu.state().data[0][1].value == value);
  }
  require(cpu.state().pc == base + 2 && !cpu.state().count_loop && cpu.state().depth == 0);
}

void simd_enable_latency() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t enable_simd = 0x0f7200200000;
  constexpr std::uint64_t pass_r4 = 0x013e0002141f;
  map(memory, enable_simd);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, pass);
  memory.write({.address = base + 2, .width = lead_a1::SharcWidth::wide48}, pass_r4);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.data[0][1] = word(1);
  initial.data[2][1] = word(2);
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][3].value == (std::uint64_t{1} << 8));
  require(cpu.state().data[2][3].known == 0 && cpu.state().effective[2].value == simd);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][4].value == (std::uint64_t{1} << 8));
  require(cpu.state().data[2][4].value == (std::uint64_t{2} << 8));
}

}

int main() {
  try {
    logical_literals();
    per_element_conditions();
    partial_knowledge_and_aliases();
    bank_halves();
    flag_bit_regression();
    guards_and_rollback();
    inactive_operands_and_unknown_conditions();
    logical_short_loop();
    simd_enable_latency();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
