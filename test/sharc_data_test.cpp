// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
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
constexpr std::uint32_t data_base = 0x80000;
constexpr std::uint32_t all32 = 0xffffffff;
constexpr std::uint64_t all40 = 0xffffffffff;
constexpr std::uint64_t literal_move = 0x0f0000000000;
constexpr std::uint32_t marker = 0x13572468;
constexpr std::uint64_t store_r3_i2_plus1 = 0x64fe09800000;
constexpr std::uint64_t load_r4_i2_minus1_pre = 0x643ffa000000;
constexpr std::uint64_t store_r5_i0 = 0x60fe02800000;
constexpr std::uint64_t load_r6_i0 = 0x607e03000000;
constexpr std::uint64_t store_r7_i0_plus1 = 0x60fe0b800000;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC data expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC data rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void map(lead_a1::SharcMemory &memory) {
  constexpr std::uint32_t cells = 512;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned native data program"});
  memory.map({.bank = 0, .halfwords = cells, .name = "owned data RAM"});
}

void immediate_register_matrix(unsigned int selected) {
  constexpr unsigned int uregs = 96;
  constexpr unsigned int complementary = 80;
  constexpr unsigned int reg_shift = 32;
  for (unsigned int reg = 0; reg < uregs; ++reg) {
    lead_a1::SharcMemory memory;
    map(memory);
    code(memory, base, literal_move | (std::uint64_t{reg} << reg_shift) | marker);
    auto initial = lead_a1::Sharc::reset_reference_214xx(base);
    constexpr std::uint32_t all_secondary = 0x4f8;
    initial.system[2].value = selected == 0 ? 0 : all_secondary;
    initial.effective = initial.system;
    lead_a1::Sharc cpu(memory, initial);
    static_cast<void>(cpu.step());
    const auto index = reg & 15;
    if (reg < 16 || reg >= complementary) {
      const auto bank = (reg < 16 ? 0U : 2U) + selected;
      require((cpu.state().data[bank][index] ==
               lead_a1::SharcKnown40{.value = std::uint64_t{marker} << 8, .known = all40}));
    } else {
      const auto kind = (reg / 16) - 1;
      require((cpu.state().dag[selected][kind][index] ==
               lead_a1::SharcKnown32{.value = marker, .known = all32}));
      require(cpu.state().dag[1 - selected][kind][index].known == 0);
      if (kind == 3) { require(cpu.state().dag[selected][0][index].value == marker); }
    }
  }
}

void alternate_banks_and_base_copy() {
  lead_a1::SharcMemory memory;
  map(memory);
  constexpr std::array<std::uint64_t, 10> program{
      0x0f1000012345, // I0 primary.
      0x0f72000004f8, // All DAG/data alternate bank selects (no SIMD).
      0x0f1412340000, // I4 still primary in the delay instruction.
      0x0f1000054321, // I0 secondary.
      0x0f4400008765, // B4 secondary, also I4 secondary.
      0x0f1400005678, // I4 secondary; does not overwrite B4.
      0x0f001234abcd, // R0 secondary low half.
      0x0f0f5678abcd, // R15 secondary high half.
      0x0f50aabbccdd, // S0 secondary low half.
      0x0f5feeffaabb  // S15 secondary high half.
  };
  for (unsigned int index = 0; index < program.size(); ++index) {
    code(memory, base + index, program[index]);
  }
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  for (const auto opcode : program) { require(cpu.step().opcode == opcode); }
  constexpr std::uint32_t primary_i0 = 0x12345;
  constexpr std::uint32_t secondary_i0 = 0x54321;
  constexpr std::uint32_t primary_i4 = 0x12340000;
  constexpr std::uint32_t secondary_i4 = 0x5678;
  constexpr std::uint32_t secondary_b4 = 0x8765;
  require(cpu.state().dag[0][0][0].value == primary_i0);
  require(cpu.state().dag[1][0][0].value == secondary_i0);
  require(cpu.state().dag[0][0][4].value == primary_i4);
  require(cpu.state().dag[1][0][4].value == secondary_i4);
  require(cpu.state().dag[1][3][4].value == secondary_b4);
  constexpr std::uint64_t secondary_r0 = 0x1234abcd00;
  constexpr std::uint64_t secondary_r15 = 0x5678abcd00;
  constexpr std::uint64_t secondary_s0 = 0xaabbccdd00;
  constexpr std::uint64_t secondary_s15 = 0xeeffaabb00;
  require(cpu.state().data[1][0].value == secondary_r0 && cpu.state().data[0][0].known == 0);
  require(cpu.state().data[1][15].value == secondary_r15 && cpu.state().data[0][15].known == 0);
  require(cpu.state().data[3][0].value == secondary_s0 && cpu.state().data[2][0].known == 0);
  require(cpu.state().data[3][15].value == secondary_s15 && cpu.state().data[2][15].known == 0);
}

void normal_and_extended_moves() {
  lead_a1::SharcControl control(true);
  lead_a1::SharcMemory memory;
  map(memory);
  constexpr std::uint32_t sysctl = 0x30024;
  memory.map_io({.first_word = sysctl, .words = 1}, control);
  constexpr std::array<std::uint64_t, 6> program{
      store_r3_i2_plus1,
      load_r4_i2_minus1_pre,
      0x0f7100000204, // USTAT2 = IIVT | IMDW0.
      0x117100030024, // DM(SYSCTL) = USTAT2.
      store_r5_i0,    // Still NW32 during effect latency.
      load_r6_i0};    // Now 40-bit, independently seeded physical48 word.
  for (unsigned int index = 0; index < program.size(); ++index) {
    code(memory, base + index, program[index]);
  }
  constexpr std::uint64_t physical = 0x9182736455ee;
  memory.write({.address = data_base, .width = lead_a1::SharcWidth::wide48}, physical);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.dag[0][0][0] = {.value = data_base, .known = all32};
  initial.dag[0][0][2] = {.value = data_base + 4, .known = all32};
  constexpr std::uint64_t upper32 = 0xffffffff00;
  initial.data[0][3] = {.value = std::uint64_t{marker} << 8, .known = upper32};
  constexpr std::uint64_t other = 0xa1b2c3d47f;
  initial.data[0][5] = {.value = other, .known = all40};
  lead_a1::Sharc cpu(memory, initial, &control);
  static_cast<void>(cpu.step());
  require(memory.read({.address = data_base + 4}) == marker);
  require(cpu.state().dag[0][0][2].value == data_base + 5);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][4].value == (std::uint64_t{marker} << 8));
  require(cpu.state().dag[0][0][2].value == data_base + 5); // Pre-modify leaves I2.
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(control.state().control != control.state().effective);
  static_cast<void>(cpu.step());
  constexpr std::uint32_t stored32 = 0xa1b2c3d4;
  require(memory.read({.address = data_base}) == stored32);
  static_cast<void>(cpu.step());
  constexpr std::uint64_t loaded40 = 0x9182a1b2c3;
  require(cpu.state().data[0][6].value == loaded40 && cpu.state().data[0][6].known == all40);
  code(memory, base + 6, store_r5_i0);
  static_cast<void>(cpu.step());
  require(memory.read({.address = data_base, .width = lead_a1::SharcWidth::wide48}) ==
          (other << 8));
}

void pm_and_negative_modifier() {
  lead_a1::SharcControl control(true);
  lead_a1::SharcMemory memory;
  map(memory);
  constexpr std::uint64_t store_pre = 0x62bff1000000; // DM(-2,I1)=R2, no I1 update.
  constexpr std::uint64_t load_pm = 0x637e0c800000;   // R9=PM(I9,+1).
  code(memory, base, store_pre);
  code(memory, base + 1, load_pm);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.dag[0][0][1] = {.value = data_base + 2, .known = all32};
  initial.dag[0][0][9] = {.value = data_base, .known = all32};
  initial.data[0][2] = {.value = std::uint64_t{marker} << 8, .known = all40};
  lead_a1::Sharc cpu(memory, initial, &control);
  static_cast<void>(cpu.step());
  require(cpu.state().dag[0][0][1].value == data_base + 2);
  require(memory.read({.address = data_base}) == marker);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][9].value == (std::uint64_t{marker} << 8));
  require(cpu.state().dag[0][0][9].value == data_base + 1);
}

void loop_store_and_retry() {
  lead_a1::SharcControl control(true);
  lead_a1::SharcMemory memory;
  map(memory);
  constexpr std::uint64_t do5 = 0x0c0005000001;
  code(memory, base, do5);
  code(memory, base + 1, store_r7_i0_plus1);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.dag[0][0][0] = {.value = data_base, .known = all32};
  lead_a1::Sharc unknown(memory, initial, &control);
  static_cast<void>(unknown.step());
  const auto before = unknown.state();
  const auto before_control = control.state();
  rejected([&] { static_cast<void>(unknown.step()); }, "Unknown SHARC data register store bits");
  require(unknown.state() == before && control.state() == before_control && !memory.active());
  rejected([&] { static_cast<void>(memory.read({.address = data_base})); }, "Unknown SHARC memory");
  auto repaired = before;
  repaired.data[0][7] = {.value = std::uint64_t{marker} << 8, .known = all40};
  lead_a1::Sharc cpu(memory, repaired, &control);
  for (unsigned int index = 0; index < 5; ++index) {
    static_cast<void>(cpu.step());
    require(cpu.state().dag[0][0][0].value == data_base + index + 1);
    require(memory.read({.address = data_base + index}) == marker);
  }
  require(cpu.state().pc == base + 2 && !cpu.state().count_loop && cpu.state().depth == 0);
}

void guards_and_conditions() {
  lead_a1::SharcMemory memory;
  map(memory);
  code(memory, base, store_r3_i2_plus1);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.dag[0][0][2] = {.value = data_base, .known = all32};
  initial.data[0][3] = {.value = std::uint64_t{marker} << 8, .known = all40};
  lead_a1::Sharc unbound(memory, initial);
  rejected([&] { static_cast<void>(unbound.step()); }, "requires a bound SYSCTL reference");
  require(unbound.state().pc == base);
  lead_a1::SharcControl control(true);
  for (const std::uint32_t mode : {2U, 0x1000000U, 0x200000U, 0x400000U}) {
    auto state = initial;
    state.system[2].value = mode;
    state.effective = state.system;
    lead_a1::Sharc cpu(memory, state, &control);
    // BR0 affects I0, not this I2 access.
    if (mode == 2) {
      static_cast<void>(cpu.step());
    } else {
      rejected([&] { static_cast<void>(cpu.step()); }, "data addressing not implemented");
      require(cpu.state() == state);
    }
  }
  constexpr std::uint64_t if_eq_store = 0x64c009800000;
  code(memory, base, if_eq_store);
  lead_a1::Sharc false_condition(memory, lead_a1::Sharc::reset_reference_214xx(base));
  require(false_condition.step().next_pc == base + 1); // No I2/data/SYSCTL required.
  auto simd = lead_a1::Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t pey_enable = 0x200000;
  simd.system[2].value = pey_enable;
  simd.system[7].value = 1; // EQ false in PEx, true in PEy.
  simd.effective = simd.system;
  lead_a1::Sharc per_element(memory, simd, &control);
  rejected([&] { static_cast<void>(per_element.step()); }, "SIMD data addressing");
  require(per_element.state() == simd);
  auto bad = initial;
  bad.data[0][0].known = all40 + 1;
  rejected([&] { const lead_a1::Sharc invalid(memory, bad); }, "40-bit register knowledge");
}

void partial_word_store_rollback() {
  lead_a1::SharcControl control(true);
  lead_a1::SharcMemory memory;
  constexpr std::uint32_t instruction_cells = 8;
  memory.map({.bank = 2, .halfwords = instruction_cells, .name = "owned failed loop"});
  const std::array<lead_a1::SharcCell, 1> first{{{.value = 0x5678, .known = 0xffff}}};
  const std::array<lead_a1::SharcCell, 1> second{{{.value = 0x1234, .known = 0xffff}}};
  memory.map({.bank = 0, .halfwords = 1, .name = "owned writable first column"}, first);
  memory.map({.bank = 0,
              .first_halfword = 1,
              .halfwords = 1,
              .name = "owned read-only second column",
              .writable = false},
             second);
  constexpr std::uint64_t do5 = 0x0c0005000001;
  code(memory, base, do5);
  code(memory, base + 1, store_r7_i0_plus1);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.dag[0][0][0] = {.value = data_base, .known = all32};
  initial.data[0][7] = {.value = std::uint64_t{marker} << 8, .known = all40};
  lead_a1::Sharc cpu(memory, initial, &control);
  static_cast<void>(cpu.step());
  constexpr std::uint32_t extended_other_bank = 0x1004;
  control.write({.size = 4}, extended_other_bank);
  const auto before = cpu.state();
  const auto before_control = control.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Read-only SHARC region");
  require(cpu.state() == before && control.state() == before_control && !memory.active());
  constexpr std::uint32_t original = 0x12345678;
  require(memory.read({.address = data_base}) == original);
}

void signed_modifier_fields() {
  constexpr std::int32_t minimum = -32;
  constexpr std::int32_t maximum = 31;
  constexpr std::uint32_t center = data_base + 96;
  constexpr std::uint64_t pre_store = 0x64be01800000;  // DM(data6,I2)=R3.
  constexpr std::uint64_t post_store = 0x64fe01800000; // DM(I2,data6)=R3.
  constexpr unsigned int displacement_shift = 27;
  constexpr std::uint64_t field = 63;
  for (const bool post : {false, true}) {
    for (std::int32_t displacement = minimum; displacement <= maximum; ++displacement) {
      lead_a1::SharcControl control(true);
      lead_a1::SharcMemory memory;
      map(memory);
      const auto encoded = static_cast<std::uint64_t>(displacement) & field;
      code(memory, base, (post ? post_store : pre_store) | (encoded << displacement_shift));
      auto initial = lead_a1::Sharc::reset_reference_214xx(base);
      initial.dag[0][0][2] = {.value = center, .known = all32};
      initial.data[0][3] = {.value = std::uint64_t{marker} << 8, .known = all40};
      lead_a1::Sharc cpu(memory, initial, &control);
      static_cast<void>(cpu.step());
      const auto sum = static_cast<std::uint32_t>(std::int64_t{center} + displacement);
      require(cpu.state().dag[0][0][2].value == (post ? sum : center));
      require(memory.read({.address = post ? center : sum}) == marker);
    }
  }
}

}

int main() {
  try {
    immediate_register_matrix(0);
    immediate_register_matrix(1);
    alternate_banks_and_base_copy();
    normal_and_extended_moves();
    pm_and_negative_modifier();
    loop_store_and_retry();
    guards_and_conditions();
    partial_word_store_rollback();
    signed_modifier_fields();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
