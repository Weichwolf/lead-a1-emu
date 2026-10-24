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
constexpr std::uint32_t short_base = 0x100000;
constexpr std::uint32_t normal_base = 0x80000;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t halfword = 0xffff;
constexpr std::uint32_t sse = 0x4000;
constexpr std::uint32_t alternate = 0x4f8;
constexpr std::uint64_t direct_read = 0x100300100008; // R3=DM(SW+8).
constexpr std::uint64_t direct_write = 0x110300100008;
constexpr std::uint64_t indexed_read = 0x51be01800000; // R3=DM(I0,M6), post.
constexpr std::uint64_t indexed_write = 0x51be81800000;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC RAM-move expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC RAM-move rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void map(lead_a1::SharcMemory &memory, std::uint64_t opcode) {
  constexpr unsigned int cells = 256;
  memory.map({.bank = 0, .halfwords = cells, .name = "owned mixed-width data"});
  memory.map({.bank = 2, .halfwords = cells, .name = "owned native RAM-move program"});
  code(memory, base, opcode);
}

lead_a1::SharcState initial(std::uint32_t mode = 0) {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.system[2].value = mode;
  state.effective = state.system;
  return state;
}

void short_literals() {
  struct Literal {
    std::uint16_t input;
    lead_a1::SharcKnown32 mode;
    lead_a1::SharcKnown40 expected;
  };

  constexpr std::array<Literal, 8> cases{
      {{.input = 0, .mode = {.known = full32}, .expected = {.known = full40}},
       {.input = 0x7fff,
        .mode = {.value = sse, .known = full32},
        .expected = {.value = 0x7fff00, .known = full40}},
       {.input = 0x8000,
        .mode = {.known = full32},
        .expected = {.value = 0x800000, .known = full40}},
       {.input = 0xffff,
        .mode = {.known = full32},
        .expected = {.value = 0xffff00, .known = full40}},
       {.input = 0x8001,
        .mode = {.value = sse, .known = full32},
        .expected = {.value = 0xffff800100, .known = full40}},
       {.input = 0xffff,
        .mode = {.value = sse, .known = full32},
        .expected = {.value = 0xffffffff00, .known = full40}},
       {.input = 0x7fff,
        .mode = {.known = full32 & ~sse},
        .expected = {.value = 0x7fff00, .known = full40}},
       {.input = 0x8001,
        .mode = {.known = full32 & ~sse},
        .expected = {.value = 0x800100, .known = 0xffffff}}}};
  for (const auto &test : cases) {
    lead_a1::SharcMemory memory;
    map(memory, direct_read);
    memory.write({.address = short_base + 8, .width = lead_a1::SharcWidth::short16}, test.input);
    auto state = initial();
    state.system[2] = test.mode;
    state.effective = state.system;
    lead_a1::Sharc cpu(memory, state); // SW does not require SYSCTL.
    static_cast<void>(cpu.step());
    require(cpu.state().data[0][3] == test.expected);
    for (unsigned int reg = 6; reg <= 9; ++reg) {
      require(cpu.state().system[reg] == state.system[reg]);
    }
  }
}

void all_short_values() {
  lead_a1::SharcMemory memory;
  map(memory, direct_read);
  for (const bool signed_mode : {false, true}) {
    for (std::uint32_t value = 0; value <= halfword; ++value) {
      memory.write({.address = short_base + 8, .width = lead_a1::SharcWidth::short16}, value);
      lead_a1::Sharc cpu(memory, initial(signed_mode ? sse : 0));
      static_cast<void>(cpu.step());
      // Independent integer-threshold oracle, not the implementation's bit fill.
      constexpr std::uint32_t negative_begin = 32768;
      constexpr std::uint64_t negative_offset = 0xffff0000;
      const auto expected = value + (signed_mode && value >= negative_begin ? negative_offset : 0);
      require(cpu.state().data[0][3].value == expected * 256);
      require(cpu.state().data[0][3].known == full40);
    }
  }
}

void short_store_and_aliases() {
  lead_a1::SharcMemory memory;
  map(memory, direct_write);
  constexpr std::uint64_t original = 0x12345678;
  memory.write({.address = normal_base + 4}, original);
  auto state = initial();
  constexpr std::uint64_t value = 0xbeef00;
  state.data[0][3] = {.value = value, .known = halfword << 8};
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  constexpr std::uint64_t expected = 0x1234beef;
  require(memory.read({.address = normal_base + 4}) == expected);
  require(cpu.state().data == state.data);
  state.data[0][3].known &= ~std::uint64_t{256};
  state.data[0][3].value &= state.data[0][3].known;
  lead_a1::Sharc unknown(memory, state);
  rejected([&] { static_cast<void>(unknown.step()); }, "short-word store bits");
  require(unknown.state() == state && memory.read({.address = normal_base + 4}) == expected);
}

void short_mode_latency_and_explicit_sx() {
  lead_a1::SharcMemory memory;
  constexpr std::array<std::uint64_t, 5> program{
      0x0f7200004000, // MODE1.SSE takes effect after one following instruction.
      direct_read,
      0x100400100008,
      0x0f7200000080,  // Select high secondary data bank; clear SSE.
      0x125900100008}; // S9=PM(SW+8), still old SSE/high bank in delay slot.
  map(memory, program[0]);
  for (unsigned int index = 1; index < program.size(); ++index) {
    code(memory, base + index, program[index]);
  }
  constexpr std::uint64_t value = 0x8001;
  memory.write({.address = short_base + 8, .width = lead_a1::SharcWidth::short16}, value);
  lead_a1::SharcControl control(true);
  constexpr std::uint64_t normal_words_are_40 = 0x204;
  control.write({.size = 4}, normal_words_are_40);
  control.begin_core_instruction();
  control.commit_core_instruction();
  lead_a1::Sharc cpu(memory, initial(), &control); // SW remains16 regardless of IMDW.
  for (unsigned int index = 0; index < program.size(); ++index) { static_cast<void>(cpu.step()); }
  constexpr std::uint64_t zero_extended = 0x800100;
  constexpr std::uint64_t sign_extended = 0xffff800100;
  require(cpu.state().data[0][3].value == zero_extended);
  require(cpu.state().data[0][4].value == sign_extended);
  require(cpu.state().data[2][9].value == sign_extended && cpu.state().data[3][9].known == 0);
  code(memory, base + 5, program[4]);
  static_cast<void>(cpu.step());
  require(cpu.state().data[3][9].value == zero_extended);
}

std::uint64_t type3(unsigned int index, unsigned int modifier, bool post, bool write) {
  constexpr std::uint64_t prefix = 0x403e01800000; // TRUE, UREG R3, no compute/LW.
  constexpr unsigned int index_shift = 41;
  constexpr unsigned int modifier_shift = 38;
  constexpr unsigned int update_shift = 44;
  constexpr unsigned int direction_shift = 31;
  return prefix | (std::uint64_t{index & 7} << index_shift) |
         (std::uint64_t{modifier & 7} << modifier_shift) | (std::uint64_t{index / 8} << 32) |
         (std::uint64_t{post} << update_shift) | (std::uint64_t{write} << direction_shift);
}

void indexed_case(
    unsigned int index, unsigned int modifier, bool post, bool write, lead_a1::SharcWidth width) {
  const bool short_word = width == lead_a1::SharcWidth::short16;
  const bool extended = width == lead_a1::SharcWidth::extended40;
  const auto center = (short_word ? short_base + 64 : normal_base + 32);
  const auto magnitude = (modifier & 7) + 1;
  const auto delta = (modifier & 1) != 0 ? -static_cast<std::int64_t>(magnitude)
                                         : static_cast<std::int64_t>(magnitude);
  const auto changed = static_cast<std::uint32_t>(std::int64_t{center} + delta);
  const auto target = post ? center : changed;
  const auto selected = index & 1;
  auto state = initial((selected != 0 ? alternate : 0) | sse);
  state.dag[selected][0][index] = {.value = center, .known = full32};
  state.dag[selected][1][modifier] = {.value = static_cast<std::uint32_t>(delta), .known = full32};
  lead_a1::SharcControl control(true);
  if (extended) {
    constexpr std::uint64_t imdw0 = 0x204;
    control.write({.size = 4}, imdw0);
    control.begin_core_instruction();
    control.commit_core_instruction();
  }
  lead_a1::SharcMemory memory;
  map(memory, type3(index, modifier, post, write));
  constexpr std::uint64_t source = 0x12345678ab;
  state.data[selected][3] = {.value = source, .known = full40};
  constexpr std::uint64_t physical = 0xaabbccddee11;
  constexpr std::uint64_t normal = 0x10203040;
  constexpr std::uint64_t short_value = 0x8001;
  const auto narrow = short_word ? short_value : normal;
  const auto stored = extended ? physical : narrow;
  const auto physical_width = extended ? lead_a1::SharcWidth::wide48 : width;
  memory.write({.address = target, .width = physical_width}, stored);
  lead_a1::Sharc cpu(memory, state, &control);
  static_cast<void>(cpu.step());
  require(cpu.state().dag[selected][0][index].value == (post ? changed : center));
  constexpr std::uint64_t expected40 = 0x12345678ab00;
  constexpr std::uint64_t expected32 = 0x12345678;
  constexpr std::uint64_t expected16 = 0x5678;
  constexpr std::uint64_t loaded40 = 0xaabbccddee;
  constexpr std::uint64_t loaded16 = 0xffff800100;
  if (write) {
    const auto normal_or_short = short_word ? expected16 : expected32;
    const auto expected = extended ? expected40 : normal_or_short;
    require(memory.read({.address = target, .width = physical_width}) == expected);
    require(cpu.state().data == state.data);
  } else {
    const auto normal_or_short = short_word ? loaded16 : normal * 256;
    const auto expected = extended ? loaded40 : normal_or_short;
    require(cpu.state().data[selected][3].value == expected);
    require(cpu.state().data[selected][3].known == full40);
  }
}

void modifier_matrix() {
  for (unsigned int index = 0; index < 16; ++index) {
    for (unsigned int offset = 0; offset < 8; ++offset) {
      const auto modifier = ((index / 8) * 8) + offset;
      for (const bool post : {false, true}) {
        for (const bool write : {false, true}) {
          indexed_case(index, modifier, post, write, lead_a1::SharcWidth::short16);
          indexed_case(index, modifier, post, write, lead_a1::SharcWidth::normal32);
          indexed_case(index, modifier, post, write, lead_a1::SharcWidth::extended40);
        }
      }
    }
  }
}

void indexed_failure_and_predicate() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t if_eq_read = 0x518001800000;
  map(memory, if_eq_read);
  lead_a1::Sharc skipped(memory, initial()); // No operands or SYSCTL needed.
  require(skipped.step().next_pc == base + 1);
  code(memory, base, indexed_write);
  auto state = initial();
  state.dag[0][0][0] = {.value = short_base + 8, .known = full32};
  state.dag[0][1][6] = {.value = 1, .known = full32};
  memory.write({.address = short_base + 8, .width = lead_a1::SharcWidth::short16}, halfword);
  lead_a1::Sharc unknown(memory, state);
  rejected([&] { static_cast<void>(unknown.step()); }, "store bits");
  require(unknown.state() == state && !memory.active());
  require(memory.read({.address = short_base + 8, .width = lead_a1::SharcWidth::short16}) ==
          halfword);
  constexpr std::uint64_t known_value = 0x123400;
  state.data[0][3] = {.value = known_value, .known = halfword << 8};
  state.dag[0][1][6].known = 0;
  state.dag[0][1][6].value = 0;
  lead_a1::Sharc unknown_modify(memory, state);
  rejected([&] { static_cast<void>(unknown_modify.step()); }, "Unknown SHARC register operand");
  require(unknown_modify.state() == state);
}

void mixed_banks_and_explicit_sx() {
  lead_a1::SharcMemory memory;
  map(memory, indexed_write);
  constexpr std::uint32_t i0_secondary = 0x10;
  auto state = initial(i0_secondary);
  state.dag[1][0][0] = {.value = short_base + 8, .known = full32};
  state.dag[0][1][6] = {.value = 1, .known = full32};
  constexpr std::uint64_t value = 0xbeef00;
  state.data[0][3] = {.value = value, .known = halfword << 8};
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  require(cpu.state().dag[1][0][0].value == short_base + 9);
  require(cpu.state().dag[0][0][0].known == 0 && cpu.state().dag[1][1][6].known == 0);
  constexpr std::uint64_t sx_pm_store = 0x51bfac800000; // PM(I8,M14)=S9, post.
  code(memory, base, sx_pm_store);
  constexpr std::uint32_t i8_and_sx_high_secondary = 0xc0;
  state = initial(i8_and_sx_high_secondary);
  state.dag[1][0][8] = {.value = short_base + 8, .known = full32};
  state.dag[0][1][14] = {.value = full32, .known = full32};
  state.data[3][9] = {.value = value, .known = halfword << 8};
  lead_a1::Sharc other(memory, state);
  static_cast<void>(other.step());
  require(other.state().dag[1][0][8].value == short_base + 7);
  require(other.state().data[2][9].known == 0 && other.state().dag[0][0][8].known == 0);
  constexpr std::uint64_t stored = 0xbeef;
  require(memory.read({.address = short_base + 8, .width = lead_a1::SharcWidth::short16}) ==
          stored);
}

void full_modifier_and_wrap() {
  struct Literal {
    std::uint32_t current;
    std::uint32_t delta;
    std::uint32_t after;
    bool post;
  };

  constexpr std::array<Literal, 6> cases{
      {{.current = short_base + 8, .delta = 0, .after = short_base + 8, .post = true},
       {.current = short_base + 8, .delta = full32, .after = short_base + 7, .post = true},
       {.current = short_base + 8, .delta = 0x7fffffff, .after = 0x80100007, .post = true},
       {.current = short_base + 8, .delta = 0x80000000, .after = 0x80100008, .post = true},
       {.current = 0xfffffff0, .delta = 0x100018, .after = 0xfffffff0, .post = false},
       {.current = 0x80100008, .delta = 0x80000000, .after = 0x80100008, .post = false}}};
  constexpr std::uint64_t update = std::uint64_t{1} << 44;
  for (const auto &[current, delta, after, post] : cases) {
    lead_a1::SharcMemory memory;
    map(memory, post ? indexed_read : indexed_read & ~update);
    constexpr std::uint64_t value = 0x1234;
    memory.write({.address = short_base + 8, .width = lead_a1::SharcWidth::short16}, value);
    auto state = initial();
    state.dag[0][0][0] = {.value = current, .known = full32};
    state.dag[0][1][6] = {.value = delta, .known = full32};
    lead_a1::Sharc cpu(memory, state);
    static_cast<void>(cpu.step());
    require(cpu.state().dag[0][0][0].value == after);
    require(cpu.state().data[0][3].value == value * 256);
  }
}

void partial_store_rollback() {
  lead_a1::SharcMemory memory;
  constexpr unsigned int cells = 6;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned failing native store"});
  const std::array<lead_a1::SharcCell, 1> low{{{.value = 0x7777, .known = 0xffff}}};
  const std::array<lead_a1::SharcCell, 1> high{{{.value = 0x6666, .known = 0xffff}}};
  memory.map({.bank = 0, .halfwords = 1, .name = "owned writable column"}, low);
  memory.map({.bank = 0,
              .first_halfword = 1,
              .halfwords = 1,
              .name = "owned read-only column",
              .writable = false},
             high);
  code(memory, base, indexed_write);
  auto state = initial();
  state.dag[0][0][0] = {.value = normal_base, .known = full32};
  state.dag[0][1][6] = {.value = 1, .known = full32};
  constexpr std::uint64_t value = 0x1234567800;
  state.data[0][3] = {.value = value, .known = full40};
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  rejected([&] { static_cast<void>(cpu.step()); }, "Read-only");
  require(cpu.state() == state && !memory.active());
  constexpr std::uint64_t original = 0x66667777;
  require(memory.read({.address = normal_base}) == original);
}

void unsupported_forms() {
  struct Invalid {
    std::uint64_t opcode;
    const char *reason;
  };

  constexpr std::array<Invalid, 4> cases{
      {{.opcode = indexed_write | 0x40000000, .reason = "forced-LW"},
       {.opcode = indexed_write | 1, .reason = "parallel compute"},
       {.opcode = 0x51be88000000, .reason = "same-DAG"},        // DM(I0,M6)=I0.
       {.opcode = 0x51be8c000000, .reason = "non-data UREG"}}}; // I8 is other DAG.
  for (const auto &[opcode, reason] : cases) {
    lead_a1::SharcMemory memory;
    map(memory, opcode);
    auto state = initial();
    state.dag[0][0][0] = {.value = short_base + 8, .known = full32};
    state.dag[0][1][6] = {.known = full32};
    lead_a1::Sharc cpu(memory, state);
    rejected([&] { static_cast<void>(cpu.step()); }, reason);
    require(cpu.state() == state && !memory.active());
  }
  for (const std::uint32_t mode : {2U, 0x1000000U, 0x200000U, 0x400000U}) {
    lead_a1::SharcMemory memory;
    map(memory, indexed_read);
    lead_a1::Sharc cpu(memory, initial(mode));
    rejected([&] { static_cast<void>(cpu.step()); }, "data addressing not implemented");
  }
}

void real_single_instruction_loop() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t do3 = 0x0c0003000001;
  map(memory, do3);
  code(memory, base + 1, indexed_write);
  auto state = initial();
  state.dag[0][0][0] = {.value = short_base + 8, .known = full32};
  state.dag[0][1][6] = {.value = 1, .known = full32};
  constexpr std::uint64_t value = 0x123400;
  state.data[0][3] = {.value = value, .known = halfword << 8};
  lead_a1::Sharc cpu(memory, state);
  for (unsigned int index = 0; index < 4; ++index) { static_cast<void>(cpu.step()); }
  require(cpu.state().pc == base + 2 && !cpu.state().count_loop && cpu.state().depth == 0);
  for (unsigned int index = 8; index < 11; ++index) {
    constexpr std::uint64_t expected = 0x1234;
    require(memory.read({.address = short_base + index, .width = lead_a1::SharcWidth::short16}) ==
            expected);
  }
  require(cpu.state().dag[0][0][0].value == short_base + 11);
}

}

int main() {
  try {
    short_literals();
    all_short_values();
    short_store_and_aliases();
    short_mode_latency_and_explicit_sx();
    modifier_matrix();
    indexed_failure_and_predicate();
    mixed_banks_and_explicit_sx();
    full_modifier_and_wrap();
    partial_store_rollback();
    unsupported_forms();
    real_single_instruction_loop();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
