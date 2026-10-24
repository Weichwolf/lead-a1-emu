// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t dm_address = base + 128;
constexpr std::uint32_t pm_address = 0xe0080;
constexpr std::uint32_t sysctl = 0x30024;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t dm_input = 0x11223344;
constexpr std::uint64_t pm_input = 0x89abcdef;
constexpr std::uint64_t pass_r2_r3 = 0x21320;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC dual-transfer expectation at line " +
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
  throw std::runtime_error("Missing SHARC dual-transfer rejection: " + reason);
}

std::uint64_t instruction(unsigned int dm_reg, unsigned int pm_reg, unsigned int writes) {
  constexpr unsigned int dm_register_shift = 33;
  constexpr unsigned int pm_register_shift = 23;
  constexpr unsigned int dm_direction_shift = 44;
  constexpr unsigned int pm_direction_shift = 37;
  // I4/M5 and I12/M13, independently read from PGR10-5's fields.
  constexpr std::uint64_t fields = 0x294128000000;
  return fields | (std::uint64_t{dm_reg} << dm_register_shift) |
         (std::uint64_t{pm_reg} << pm_register_shift) |
         (std::uint64_t{writes & 1} << dm_direction_shift) |
         (std::uint64_t{writes >> 1} << pm_direction_shift);
}

void program(lead_a1::SharcMemory &memory, std::uint64_t opcode) {
  constexpr unsigned int cells = 1024;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned dual DM program/data"});
  memory.map({.bank = 3, .halfwords = cells, .name = "owned dual PM data"});
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, opcode);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, 0);
  memory.write({.address = dm_address}, dm_input);
  memory.write({.address = pm_address}, pm_input);
}

unsigned int data_bank(unsigned int reg, unsigned int selectors) {
  return reg < 8 ? selectors & 1 : selectors >> 1;
}

lead_a1::SharcState initial(unsigned int selectors = 0) {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  constexpr unsigned int low_bank_shift = 10;
  constexpr unsigned int high_bank_shift = 7;
  state.system[2].value |=
      ((selectors & 1) << low_bank_shift) | ((selectors >> 1) << high_bank_shift);
  state.effective = state.system;
  for (unsigned int bank = 0; bank < 2; ++bank) {
    for (unsigned int reg = 0; reg < 16; ++reg) {
      constexpr std::uint64_t marker = 0x1020304077;
      constexpr std::uint64_t bank_offset = 0x1000000000;
      state.data[bank][reg] = {.value = marker + (bank * bank_offset) + (std::uint64_t{reg} * 256),
                               .known = full40};
    }
  }
  state.dag[0][0][4] = {.value = dm_address, .known = full32};
  state.dag[0][0][12] = {.value = pm_address, .known = full32};
  state.dag[0][1][5] = {.value = full32, .known = full32};
  state.dag[0][1][13] = {.value = 2, .known = full32};
  return state;
}

struct MatrixCase {
  unsigned int selectors;
  unsigned int writes;
  unsigned int dm_reg;
  unsigned int pm_reg;
};

void matrix_case(MatrixCase test) {
  const auto [selectors, writes, dm_reg, pm_reg] = test;
  lead_a1::SharcMemory memory;
  program(memory, instruction(dm_reg, pm_reg, writes) | pass_r2_r3);
  const auto state = initial(selectors);
  const auto old_dm = state.data[data_bank(dm_reg, selectors)][dm_reg];
  const auto old_pm = state.data[data_bank(pm_reg, selectors)][pm_reg];
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  static_cast<void>(cpu.step());
  auto expected = state.data;
  const auto computed = state.data[data_bank(2, selectors)][2].value & ~std::uint64_t{255};
  expected[data_bank(3, selectors)][3] = {.value = computed, .known = full40};
  if ((writes & 2) == 0) {
    expected[data_bank(pm_reg, selectors)][pm_reg] = {.value = pm_input * 256, .known = full40};
  }
  if ((writes & 1) == 0) {
    expected[data_bank(dm_reg, selectors)][dm_reg] = {.value = dm_input * 256, .known = full40};
  }
  require(cpu.state().data == expected);
  require(memory.read({.address = dm_address}) ==
          ((writes & 1) != 0 ? old_dm.value / 256 : dm_input));
  require(memory.read({.address = pm_address}) ==
          ((writes & 2) != 0 ? old_pm.value / 256 : pm_input));
  require(cpu.state().dag[0][0][4].value == dm_address - 1 &&
          cpu.state().dag[0][0][12].value == pm_address + 2);
  constexpr std::uint32_t alu_mask = 0x43f;
  require((cpu.state().system[6].value & alu_mask) == 0);
  require(cpu.state().cache.valid.known == 0);
}

void direction_and_alias_matrix() {
  for (unsigned int selectors = 0; selectors < 4; ++selectors) {
    for (unsigned int writes = 0; writes < 4; ++writes) {
      for (unsigned int dm_reg = 0; dm_reg < 16; ++dm_reg) {
        for (unsigned int pm_reg = 0; pm_reg < 16; ++pm_reg) {
          matrix_case(
              {.selectors = selectors, .writes = writes, .dm_reg = dm_reg, .pm_reg = pm_reg});
        }
      }
    }
  }
}

void same_block_order_and_empty_compute() {
  for (const auto writes : {0U, 1U, 2U, 3U}) {
    lead_a1::SharcMemory memory;
    program(memory, instruction(6, 8, writes));
    auto state = initial();
    state.dag[0][0][12].value = dm_address;
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, state, &control);
    static_cast<void>(cpu.step());
    const auto dm_stored = state.data[0][6].value / 256;
    const auto pm_stored = state.data[0][8].value / 256;
    const auto after_dm = (writes & 1) != 0 ? dm_stored : dm_input;
    require(memory.read({.address = dm_address}) == ((writes & 2) != 0 ? pm_stored : after_dm));
    if ((writes & 1) == 0) { require(cpu.state().data[0][6].value == dm_input * 256); }
    if ((writes & 2) == 0) {
      require(cpu.state().data[0][8].value == ((writes & 1) != 0 ? dm_stored : dm_input) * 256);
    }
    require(cpu.state().system == state.system);
  }
}

void core_control_effect_and_rollback() {
  lead_a1::SharcControl control(true);
  lead_a1::SharcMemory memory;
  program(memory, instruction(6, 8, 3));
  memory.map_io({.first_word = sysctl, .words = 1}, control);
  auto state = initial();
  state.dag[0][0][4].value = sysctl;
  constexpr std::uint32_t widened = 0x804;
  state.data[0][6] = {.value = std::uint64_t{widened} * 256, .known = full40};
  lead_a1::Sharc cpu(memory, state, &control);
  static_cast<void>(cpu.step());
  require(control.state().control == widened && control.state().effective == 4 &&
          control.state().delayed == widened);
  require(memory.read({.address = pm_address}) == state.data[0][8].value / 256);
  static_cast<void>(cpu.step());
  require(control.state().effective == widened && !control.state().delayed);

  // Reuse a separate instruction with a later PM fault: the earlier SYSCTL write rolls back.
  constexpr std::uint32_t unmapped_pm = 0xf0000;
  state.dag[0][0][12].value = unmapped_pm;
  lead_a1::Sharc failing(memory, state, &control);
  const auto before = control.state();
  rejected([&] { static_cast<void>(failing.step()); }, "Unmapped SHARC physical RAM/ROM cell");
  require(failing.state() == state && control.state() == before && !memory.active());
}

void data_width_case(unsigned int widths, bool write) {
  lead_a1::SharcControl control(true);
  constexpr std::uint32_t dm_wide = 0x800;
  constexpr std::uint32_t pm_wide = 0x1000;
  control.write({.size = 4},
                4 | ((widths & 1) != 0 ? dm_wide : 0) | ((widths & 2) != 0 ? pm_wide : 0));
  control.begin_core_instruction();
  control.commit_core_instruction();
  lead_a1::SharcMemory memory;
  program(memory, instruction(6, 8, write ? 3U : 0U));
  constexpr std::uint64_t physical_dm = 0xa1b2c3d4e5f6;
  constexpr std::uint64_t physical_pm = 0x9182736455ee;
  if ((widths & 1) != 0) {
    memory.write({.address = dm_address, .width = lead_a1::SharcWidth::wide48}, physical_dm);
  }
  if ((widths & 2) != 0) {
    memory.write({.address = pm_address, .width = lead_a1::SharcWidth::wide48}, physical_pm);
  }
  const auto state = initial();
  lead_a1::Sharc cpu(memory, state, &control);
  static_cast<void>(cpu.step());
  if (write) {
    const bool dm_extended = (widths & 1) != 0;
    const bool pm_extended = (widths & 2) != 0;
    require(memory.read({.address = dm_address,
                         .width = dm_extended ? lead_a1::SharcWidth::wide48
                                              : lead_a1::SharcWidth::normal32}) ==
            (dm_extended ? state.data[0][6].value * 256 : state.data[0][6].value / 256));
    require(memory.read({.address = pm_address,
                         .width = pm_extended ? lead_a1::SharcWidth::wide48
                                              : lead_a1::SharcWidth::normal32}) ==
            (pm_extended ? state.data[0][8].value * 256 : state.data[0][8].value / 256));
  } else {
    require(cpu.state().data[0][6].value ==
            ((widths & 1) != 0 ? physical_dm / 256 : dm_input * 256));
    require(cpu.state().data[0][8].value ==
            ((widths & 2) != 0 ? physical_pm / 256 : pm_input * 256));
  }
}

void independent_data_widths() {
  for (unsigned int widths = 0; widths < 4; ++widths) {
    for (const bool write : {false, true}) { data_width_case(widths, write); }
  }
}

void late_compute_failure_and_peripheral_guard() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t unimplemented_multiplier = 0x100000;
  program(memory, instruction(6, 8, 3) | unimplemented_multiplier);
  auto state = initial();
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  rejected([&] { static_cast<void>(cpu.step()); }, "multiplier/shifter/multifunction");
  require(cpu.state() == state && !memory.active());
  require(memory.read({.address = dm_address}) == dm_input &&
          memory.read({.address = pm_address}) == pm_input);
  constexpr std::uint32_t spi_dma_index = 0x1080;
  state.dag[0][0][4].value = spi_dma_index;
  lead_a1::Sharc peripheral(memory, state, &control);
  rejected([&] { static_cast<void>(peripheral.step()); }, "IOP target/domain not qualified");
  require(peripheral.state() == state && !memory.active());
}

void floating_old_inputs_and_load_priority() {
  constexpr std::uint64_t float_add = 0x81312; // F3=old F1+old F2.
  constexpr std::uint64_t one = 0x3f80000000;
  constexpr std::uint64_t two = 0x4000000000;
  constexpr std::uint64_t three = 0x4040000000;
  constexpr std::uint32_t float_status = 0x400;
  constexpr std::uint32_t alu_mask = 0x43f;
  for (const bool conflict : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory, instruction(conflict ? 3U : 1U, conflict ? 3U : 4U, 0) | float_add);
    auto state = initial();
    state.data[0][1] = {.value = one, .known = full40};
    state.data[0][2] = {.value = two, .known = full40};
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, state, &control);
    static_cast<void>(cpu.step());
    require(cpu.state().data[0][3].value == (conflict ? dm_input << 8 : three));
    if (!conflict) {
      require(cpu.state().data[0][1].value == dm_input << 8);
      require(cpu.state().data[0][4].value == pm_input << 8);
    }
    require((cpu.state().system[6].value & alu_mask) == float_status);
    require(cpu.state().system[8] == state.system[8]);
  }
  lead_a1::SharcMemory memory;
  program(memory, instruction(6, 8, 3) | float_add);
  auto state = initial();
  state.data[0][1] = {}; // Real stores precede the unknown arithmetic input.
  lead_a1::SharcControl control(true);
  const auto before_control = control.snapshot();
  lead_a1::Sharc failing(memory, state, &control);
  rejected([&] { static_cast<void>(failing.step()); }, "Unknown SHARC floating register");
  require(failing.state() == state && !memory.active() && control.snapshot() == before_control);
  require(memory.read({.address = dm_address}) == dm_input);
  require(memory.read({.address = pm_address}) == pm_input);
}

void flush_follower_and_simd_guards() {
  lead_a1::SharcMemory memory;
  program(memory, instruction(6, 8, 3));
  auto state = initial();
  state.cache.pending_flush = true;
  lead_a1::SharcControl control(true);
  lead_a1::Sharc pending(memory, state, &control);
  rejected([&] { static_cast<void>(pending.step()); }, "immediately after FLUSH CACHE prohibited");
  require(pending.state() == state && !memory.active());
  state.cache.pending_flush = false;
  constexpr std::uint32_t simd = 0x200000;
  state.system[2].value |= simd;
  state.effective = state.system;
  lead_a1::Sharc parallel(memory, state, &control);
  rejected([&] { static_cast<void>(parallel.step()); }, "SIMD data addressing not implemented");
  require(parallel.state() == state && !memory.active());
  require(memory.read({.address = dm_address}) == dm_input &&
          memory.read({.address = pm_address}) == pm_input);
}

}

int main() {
  try {
    direction_and_alias_matrix();
    same_block_order_and_empty_compute();
    core_control_effect_and_rollback();
    independent_data_widths();
    late_compute_failure_and_peripheral_guard();
    floating_old_inputs_and_load_priority();
    flush_follower_and_simd_guards();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
