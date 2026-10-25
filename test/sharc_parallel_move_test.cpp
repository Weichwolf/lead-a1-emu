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

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t prefix = 0x703e00000000;
constexpr std::uint32_t bank_mask = 0x480;
constexpr std::uint32_t rnd32 = 0x10000;
constexpr std::uint32_t alu_mask = 0x43f;
constexpr std::uint64_t pass_r3_to_r4 = 0x21430;
constexpr std::uint64_t dependent_move = 0x707e01021430; // R4=PASS R3, R2=old R4.
constexpr std::uint64_t conflict_move = 0x707e82021430;  // R4=PASS R3, R4=old R5.
constexpr std::uint64_t old_source = 0x12345678c9;
constexpr std::uint64_t negative = 0xaabbccdda7;
constexpr std::uint64_t fixed_negative = 0xaabbccdd00;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC parallel-move expectation at line " +
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
  throw std::runtime_error("Missing SHARC parallel-move rejection: " + reason);
}

void program(lead_a1::SharcMemory &memory, std::uint64_t opcode) {
  constexpr unsigned int cells = 96;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned parallel register moves"});
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, opcode);
}

std::uint64_t move(unsigned int source, unsigned int destination) {
  constexpr unsigned int high_shift = 38;
  constexpr unsigned int low_shift = 31;
  constexpr unsigned int destination_shift = 23;
  return prefix | (std::uint64_t{source >> 2} << high_shift) |
         (std::uint64_t{source & 3} << low_shift) |
         (std::uint64_t{destination} << destination_shift);
}

unsigned int selector(unsigned int index) {
  constexpr unsigned int complementary_begin = 64;
  return index < 16 ? index : index + complementary_begin;
}

unsigned int bank(unsigned int reg, unsigned int selectors) {
  return ((reg & 15) < 8 ? selectors & 1 : selectors >> 1) + (reg < 16 ? 0U : 2U);
}

void all_data_file_pairs() {
  constexpr unsigned int register_count = 32;
  for (unsigned int source = 0; source < register_count; ++source) {
    for (unsigned int destination = 0; destination < register_count; ++destination) {
      for (unsigned int selectors = 0; selectors < 4; ++selectors) {
        for (const auto rounding : {0U, rnd32}) {
          lead_a1::SharcMemory memory;
          program(memory, move(selector(source), selector(destination)));
          auto state = lead_a1::Sharc::reset_reference_214xx(base);
          constexpr unsigned int low_bank_shift = 10;
          constexpr unsigned int high_bank_shift = 7;
          state.system[2].value = ((selectors & 1) << low_bank_shift) |
                                  ((selectors >> 1) << high_bank_shift) | rounding;
          state.effective = state.system;
          constexpr std::uint64_t known = 0xffffff00f3;
          constexpr std::uint64_t value = old_source & known;
          state.data[bank(source, selectors)][source & 15] = {.value = value, .known = known};
          auto expected = state.data;
          expected[bank(destination, selectors)][destination & 15] =
              state.data[bank(source, selectors)][source & 15];
          lead_a1::Sharc cpu(memory, state);
          static_cast<void>(cpu.step());
          require(cpu.state().data == expected && cpu.state().system == state.system);
        }
      }
    }
  }
}

void old_source_and_destination_priority() {
  require((move(4, 2) | pass_r3_to_r4) == dependent_move);
  require((move(5, 4) | pass_r3_to_r4) == conflict_move);
  lead_a1::SharcMemory memory;
  program(memory, dependent_move);
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.data[0][3] = {.value = negative, .known = full40};
  state.data[0][4] = {.value = old_source, .known = full40};
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][2] == state.data[0][4]);
  require(cpu.state().data[0][4].value == fixed_negative);
  require((cpu.state().system[6].value & alu_mask) == 4);

  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, conflict_move);
  state.data[0][3] = {.known = full40};
  state.data[0][5] = {.value = negative, .known = full40};
  lead_a1::Sharc conflict(memory, state);
  static_cast<void>(conflict.step());
  require(conflict.state().data[0][4] == state.data[0][5]);
  require((conflict.state().system[6].value & alu_mask) == 1); // AZ from zero computation.
}

void status_capture_and_dag_side_effects() {
  lead_a1::SharcMemory memory;
  constexpr unsigned int astat = 0x76;
  program(memory, move(astat, 5) | pass_r3_to_r4);
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t previous_flags = 0x6c21;
  state.system[6].value = previous_flags;
  state.effective = state.system;
  state.data[0][3] = {.value = negative, .known = full40};
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][5].value == std::uint64_t{previous_flags} * 256);
  require((cpu.state().system[6].value & alu_mask) == 4);

  constexpr unsigned int b0 = 0x40;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48},
               move(4, b0) | pass_r3_to_r4);
  state.data[0][4] = {.value = old_source, .known = full40};
  lead_a1::Sharc dag(memory, state);
  static_cast<void>(dag.step());
  constexpr std::uint32_t old_upper = 0x12345678;
  require(dag.state().dag[0][3][0].value == old_upper);
  require(dag.state().dag[0][0][0] == dag.state().dag[0][3][0]);
  require(dag.state().data[0][4].value == fixed_negative);
}

void delayed_bank_and_rollback() {
  lead_a1::SharcMemory memory;
  constexpr unsigned int mode1 = 0x72;
  constexpr unsigned int ustat1 = 0x70;
  program(memory, move(ustat1, mode1) | pass_r3_to_r4);
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.system[0] = {.value = bank_mask, .known = full32};
  state.effective = state.system;
  state.data[0][3] = {.value = negative, .known = full40};
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  require(cpu.state().effective[2].value == 0 && cpu.state().system[2].value == bank_mask);
  require(cpu.state().data[0][4].value == fixed_negative);
  require(cpu.state().data[1][4].known == 0 && cpu.state().pending[2]);

  constexpr unsigned int unsupported_status = 0x76;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48},
               move(4, unsupported_status) | pass_r3_to_r4);
  state.data[0][4] = {.value = old_source, .known = full40};
  state.pending[2] = {.value = bank_mask, .known = full32};
  lead_a1::SharcControl control(true);
  lead_a1::Sharc failure(memory, state, &control);
  const auto before_control = control.state();
  rejected([&] { static_cast<void>(failure.step()); }, "system register write semantics");
  require(failure.state() == state && control.state() == before_control && !memory.active());
}

void false_predicate_and_unsupported_compute() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t conditions = std::uint64_t{31} << 33;
  program(memory, dependent_move & ~conditions); // IF EQ, AZ is initially clear.
  const auto state = lead_a1::Sharc::reset_reference_214xx(base);
  lead_a1::Sharc skipped(memory, state);
  static_cast<void>(skipped.step());
  require(skipped.state().data == state.data && skipped.state().system == state.system);
  constexpr std::uint64_t multiplier = 0x100000;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, move(4, 2) | multiplier);
  lead_a1::Sharc invalid(memory, state);
  rejected([&] { static_cast<void>(invalid.step()); }, "multiplier/shifter/multifunction");
  require(invalid.state() == state && !memory.active());
}

}

int main() {
  try {
    all_data_file_pairs();
    old_source_and_destination_priority();
    status_capture_and_dag_side_effects();
    delayed_bank_and_rollback();
    false_predicate_and_unsupported_compute();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
