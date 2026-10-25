// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_control.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint64_t register_move = 0x703e00000000;
constexpr std::uint64_t load_low_immediate = 0x0f6c89abcdef;
constexpr std::uint64_t load_high_immediate = 0x0f6d01234567;
constexpr std::uint64_t r3_from_px = 0x76bf81800000;
constexpr std::uint64_t r5_from_px2 = 0x76fe82800000;
constexpr std::uint64_t r6_from_px1 = 0x76fe03000000;
constexpr std::uint64_t low_word = 0x89abcdef;
constexpr std::uint64_t whole_word = 0x0123456789abcdef;
constexpr std::uint64_t data_word = 0x0123456789;
constexpr std::uint64_t loaded_px = 0xfedcba9876540000;
constexpr std::uint64_t high_data_word = 0xfedcba9800;
constexpr std::uint64_t low_data_word = 0x7654000000;
constexpr unsigned int combined_px = 0x6b;
constexpr std::uint64_t px_from_r0 = 0x703e35800000;
constexpr std::uint64_t partial_px_known = 0xffffff00ffffffff;
constexpr std::uint64_t stored_px = 0x0123456789ab0000;
constexpr std::uint64_t memory_bits = 0xffffffffffff0000;
constexpr std::uint64_t three_iterations = 0x0c0003000001;
constexpr std::uint64_t indexed_px_store = 0x51beb5800000;

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t data_base = 0x80000;
constexpr std::uint64_t all64 = 0xffffffffffffffff;
constexpr std::uint64_t all40 = 0xffffffffff;
constexpr std::uint32_t all32 = 0xffffffff;
constexpr std::uint64_t extension_bits = 0xff;
constexpr std::uint64_t payload = 0x0123456789ab;
constexpr std::uint64_t load_px = 0x126b00000000;
constexpr std::uint64_t store_px = 0x136b00000000;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC PX expectation at line " + std::to_string(where.line()));
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
  throw std::runtime_error("Missing SHARC PX rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, bool data = true) {
  constexpr unsigned int cells = 256;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned PX code"});
  if (data) { memory.map({.bank = 0, .halfwords = cells, .name = "owned PX data"}); }
}

std::uint64_t move(unsigned int source, unsigned int destination) {
  constexpr unsigned int high_shift = 38;
  constexpr unsigned int low_shift = 31;
  constexpr unsigned int destination_shift = 23;
  return register_move | (std::uint64_t{source >> 2} << high_shift) |
         (std::uint64_t{source & 3} << low_shift) |
         (std::uint64_t{destination} << destination_shift);
}

void literal_transfers() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, load_low_immediate);      // PX1 immediate.
  code(memory, base + 1, load_high_immediate); // PX2 immediate.
  code(memory, base + 2, r3_from_px);          // R3=PX, upper40 including data low8.
  code(memory, base + 3, store_px | (data_base + 4));
  code(memory, base + 4, load_px | (data_base + 5));
  code(memory, base + 5, r5_from_px2); // R5=PX2.
  code(memory, base + 6, r6_from_px1); // R6=PX1.
  constexpr std::uint64_t loaded = 0xfedcba987654;
  memory.write({.address = data_base + 5, .width = lead_a1::SharcWidth::wide48}, loaded);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(cpu.step());
  require(cpu.state().px.value == low_word && cpu.state().px.known == all32);
  static_cast<void>(cpu.step());
  require(cpu.state().px.value == whole_word && cpu.state().px.known == all64);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][3].value == data_word && cpu.state().data[0][3].known == all40);
  static_cast<void>(cpu.step());
  require(memory.read({.address = data_base + 4, .width = lead_a1::SharcWidth::wide48}) == payload);
  static_cast<void>(cpu.step());
  require(cpu.state().px.value == loaded_px && cpu.state().px.known == all64);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][5].value == high_data_word && cpu.state().data[0][5].known == all40);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][6].value == low_data_word && cpu.state().data[0][6].known == all40);
  require(move(combined_px, 3) == r3_from_px && move(0, combined_px) == px_from_r0);
}

void data_banks() {
  for (unsigned int index = 0; index < 32; ++index) {
    for (unsigned int selection = 0; selection < 4; ++selection) {
      lead_a1::SharcMemory memory;
      program(memory);
      const auto reg = index < 16 ? index : index + 64;
      code(memory, base, move(reg, combined_px));
      code(memory, base + 1, move(combined_px, reg));
      auto state = lead_a1::Sharc::reset_reference_214xx(base);
      constexpr unsigned int low_bank_shift = 10;
      constexpr unsigned int high_bank_shift = 7;
      state.system[2].value =
          ((selection & 1) << low_bank_shift) | ((selection >> 1) << high_bank_shift);
      state.effective = state.system;
      const auto bank =
          ((index & 15) < 8 ? selection & 1 : selection >> 1) + (index < 16 ? 0U : 2U);
      constexpr std::uint64_t partial_value = 0x0123456089;
      constexpr std::uint64_t partial_known = 0xffffff00ff;
      state.data[bank][index & 15] = {.value = partial_value, .known = partial_known};
      // Canonical partial bits: the unknown middle byte must have value zero.
      state.data[bank][index & 15].value &= partial_known;
      const auto expected = state.data[bank][index & 15];
      const auto flags = state.system;
      lead_a1::Sharc cpu(memory, state);
      static_cast<void>(cpu.step());
      require(cpu.state().px.value == (expected.value * 0x1000000));
      require(cpu.state().px.known == partial_px_known);
      static_cast<void>(cpu.step());
      require(cpu.state().data[bank][index & 15] == expected);
      require(cpu.state().system == flags);
    }
  }
}

void width_independence() {
  for (const std::uint32_t widths : {0U, 0x1e00U}) {
    lead_a1::SharcMemory memory;
    program(memory);
    lead_a1::SharcControl control(true);
    control.write({.address = 0, .size = 4}, widths | 4);
    control.begin_core_instruction();
    control.commit_core_instruction();
    code(memory, base, load_px | (data_base + 3));
    code(memory, base + 1, store_px | (data_base + 6));
    memory.write({.address = data_base + 3, .width = lead_a1::SharcWidth::wide48}, payload);
    lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base), &control);
    static_cast<void>(cpu.step());
    require(cpu.state().px.value == stored_px);
    static_cast<void>(cpu.step());
    require(memory.read({.address = data_base + 6, .width = lead_a1::SharcWidth::wide48}) ==
            payload);
  }
}

void unknown_half_aliases() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr unsigned int px1 = 0x6c;
  constexpr unsigned int px2 = 0x6d;
  code(memory, base, load_low_immediate);
  code(memory, base + 1, move(px2, 0));
  code(memory, base + 2, move(1, px2));
  code(memory, base + 3, move(px1, 2));
  code(memory, base + 4, move(px2, 3));
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  constexpr std::uint64_t upper_data = 0x7654000000;
  constexpr std::uint64_t upper_known = 0xffff0000ff;
  constexpr std::uint64_t merged_value = 0x7654000089abcdef;
  constexpr std::uint64_t merged_known = 0xffff0000ffffffff;
  constexpr std::uint64_t low_data = 0x89abcdef00;
  state.data[0][1] = {.value = upper_data, .known = upper_known};
  lead_a1::Sharc cpu(memory, state);
  require(cpu.state().px.known == 0);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][0].value == 0 && cpu.state().data[0][0].known == extension_bits);
  require(cpu.state().px.value == low_word && cpu.state().px.known == all32);
  static_cast<void>(cpu.step());
  require(cpu.state().px.value == merged_value && cpu.state().px.known == merged_known);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][2].value == low_data && cpu.state().data[0][2].known == all40);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][3] == state.data[0][1]);
}

void false_predicate_and_invalid_state() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t conditions = std::uint64_t{31} << 33;
  code(memory, base, px_from_r0 & ~conditions); // IF EQ with AZ clear: no operand read.
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.px = {.value = low_word, .known = all32};
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  require(cpu.state().px == state.px && cpu.state().data == state.data);
  state.px = {.value = 1};
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "Invalid SHARC PX");
}

void partial_store(bool missing_bit) {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, store_px | data_base);
  constexpr std::uint64_t old = 0xabcd98765432;
  memory.write({.address = data_base, .width = lead_a1::SharcWidth::wide48}, old);
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.px = {.value = stored_px, .known = memory_bits};
  if (missing_bit) {
    state.px.known &= ~std::uint64_t{0x10000};
    state.px.value &= state.px.known;
  }
  lead_a1::Sharc cpu(memory, state);
  const auto before = cpu.state();
  if (missing_bit) {
    rejected([&] { static_cast<void>(cpu.step()); }, "Unknown SHARC PX memory store bits");
    require(cpu.state() == before && !memory.active());
  } else {
    static_cast<void>(cpu.step());
    require(cpu.state().px == before.px);
  }
  require(memory.read({.address = data_base, .width = lead_a1::SharcWidth::wide48}) ==
          (missing_bit ? old : payload));
}

void read_only_rollback() {
  lead_a1::SharcMemory memory;
  program(memory, false);
  memory.map({.bank = 0, .halfwords = 2, .name = "owned writable columns"});
  memory.map({.bank = 0,
              .first_halfword = 2,
              .halfwords = 1,
              .name = "owned read-only column",
              .writable = false});
  code(memory, base, store_px | data_base);
  constexpr std::uint64_t old = 0xdeadbeef;
  memory.write({.address = data_base}, old);
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.px = {.value = whole_word, .known = all64};
  lead_a1::Sharc cpu(memory, state);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Read-only SHARC region");
  require(cpu.state() == before && !memory.active());
  require(memory.read({.address = data_base}) == old);
}

void indexed_loop() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, three_iterations);     // Three real single-instruction iterations.
  code(memory, base + 1, indexed_px_store); // DM(I0,M6)=PX, native Type3a.
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.px = {.value = stored_px, .known = memory_bits};
  state.dag[0][0][0] = {.value = data_base, .known = all32};
  state.dag[0][1][6] = {.value = 1, .known = all32};
  lead_a1::Sharc cpu(memory, state);
  for (unsigned int step = 0; step < 4; ++step) { static_cast<void>(cpu.step()); }
  require(cpu.state().pc == base + 2 && !cpu.state().count_loop);
  require(cpu.state().dag[0][0][0].value == data_base + 3);
  for (unsigned int index = 0; index < 3; ++index) {
    require(memory.read({.address = data_base + index, .width = lead_a1::SharcWidth::wide48}) ==
            payload);
  }
}

void guarded_transfers() {
  struct GuardCase {
    std::uint64_t instruction;
    const char *reason;
  };

  const std::array<GuardCase, 5> cases{
      {{.instruction = load_px | 0x100000,
        .reason = "PX transfer outside normal-word RAM not qualified"},
       {.instruction = load_px | 0x40000,
        .reason = "PX transfer outside normal-word RAM not qualified"},
       {.instruction = load_px | 0x8000000000, .reason = "forced-LW register transfer"},
       {.instruction = move(combined_px, 16), .reason = "combined PX/non-data register width"},
       {.instruction = move(16, combined_px), .reason = "combined PX/non-data register width"}}};
  for (const auto &item : cases) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, base, item.instruction);
    lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(cpu.step()); }, item.reason);
    require(cpu.state() == before && !memory.active());
  }
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, load_px | data_base);
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.system[2].value = 0x200000;
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  rejected([&] { static_cast<void>(cpu.step()); }, "SIMD PX");
}

}

int main() {
  try {
    literal_transfers();
    data_banks();
    width_independence();
    unknown_half_aliases();
    false_predicate_and_invalid_state();
    partial_store(false);
    partial_store(true);
    read_only_rollback();
    indexed_loop();
    guarded_transfers();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
