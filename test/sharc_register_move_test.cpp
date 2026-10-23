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
constexpr std::uint32_t simd = 0x200000;
constexpr std::uint64_t data_to_base = 0x703fa0000000;       // B0=R3, PGR10-9/10-32.
constexpr std::uint64_t index_to_data = 0x713e03800000;      // R7=I0.
constexpr std::uint64_t status_to_index = 0x773e08800000;    // I1=USTAT1.
constexpr std::uint64_t other_pe_to_status = 0x753f38000000; // USTAT1=S2.

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC register-move expectation at line " +
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
  throw std::runtime_error("Missing SHARC register-move rejection: " + reason);
}

void map(lead_a1::SharcMemory &memory, std::uint64_t opcode) {
  constexpr unsigned int cells = 48;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned UREG-transfer program"});
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void literal_transfers_and_alignment() {
  lead_a1::SharcMemory memory;
  map(memory, data_to_base);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, index_to_data);
  memory.write({.address = base + 2, .width = lead_a1::SharcWidth::wide48}, other_pe_to_status);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t value = 0x23456789;
  constexpr std::uint32_t other = 0x87654321;
  constexpr std::uint64_t extension = 0xa5;
  initial.data[0][3] = {.value = (std::uint64_t{value} << 8) | extension, .known = full40};
  initial.data[2][2] = {.value = std::uint64_t{other} << 8, .known = std::uint64_t{full} << 8};
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  require((cpu.state().dag[0][3][0] == lead_a1::SharcKnown32{.value = value, .known = full}));
  require(cpu.state().dag[0][0][0] == cpu.state().dag[0][3][0]); // Bx writes Ix.
  static_cast<void>(cpu.step());
  require((cpu.state().data[0][7] ==
           lead_a1::SharcKnown40{.value = std::uint64_t{value} << 8, .known = full40}));
  static_cast<void>(cpu.step());
  require((cpu.state().system[0] == lead_a1::SharcKnown32{.value = other, .known = full}));
  for (unsigned int reg = 6; reg <= 9; ++reg) {
    require(cpu.state().system[reg] == initial.system[reg]);
  }
}

void source_fields_and_partial_knowledge() {
  lead_a1::SharcMemory memory;
  map(memory, status_to_index);
  constexpr std::uint64_t modifier_to_status = 0x72ff3f800000; // USTAT4=M14.
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, modifier_to_status);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t known = 0xffff00ff;
  constexpr std::uint32_t value = 0x12340078;
  initial.system[0] = {.value = value, .known = known};
  initial.effective[0] = initial.system[0];
  initial.dag[0][1][14] = {.value = full, .known = full};
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  require(cpu.state().dag[0][0][1] == initial.system[0]);
  static_cast<void>(cpu.step());
  require(cpu.state().system[15] == initial.dag[0][1][14]);
  require(cpu.state().dag[0][1][14] == initial.dag[0][1][14]);
}

void false_condition_and_bank_selection() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t if_eq_status_to_index = 0x770008800000;
  map(memory, if_eq_status_to_index);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[0] = {};
  constexpr std::uint32_t dag_bank = 0x10;
  initial.system[2].known &= ~dag_bank;
  initial.effective = initial.system;
  lead_a1::Sharc skipped(memory, initial);
  static_cast<void>(skipped.step());
  require(skipped.state().system == initial.system && skipped.state().dag == initial.dag);
  initial.system[6].value = 1;
  initial.effective = initial.system;
  lead_a1::Sharc failing(memory, initial);
  const auto before = failing.state();
  rejected([&] { static_cast<void>(failing.step()); }, "bank selection");
  require(failing.state() == before && !memory.active());
  initial.system[2] = {.value = dag_bank, .known = full};
  initial.effective = initial.system;
  constexpr std::uint32_t value = 0x5a5a5a5a;
  initial.system[0] = {.value = value, .known = full};
  initial.effective[0] = initial.system[0];
  lead_a1::Sharc secondary(memory, initial);
  static_cast<void>(secondary.step());
  require(secondary.state().dag[1][0][1] == initial.system[0]);
  require(secondary.state().dag[0] == initial.dag[0]);
}

void system_effect_latency() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t status_to_mode = 0x773e39000000;
  map(memory, status_to_mode);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, 0);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[0] = {.value = simd, .known = full};
  initial.effective = initial.system;
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  require(cpu.state().system[2].value == simd && cpu.state().effective[2].value == 0);
  require(cpu.state().pending[2] == cpu.state().system[2]);
  static_cast<void>(cpu.step());
  require(cpu.state().effective[2].value == simd && !cpu.state().pending[2]);
}

void unsupported_fields_and_modes() {
  constexpr std::uint64_t reserved = 0x703fe0000000; // Bit30 is not a source bit.
  constexpr std::uint64_t swap = 0x783e00000000;
  constexpr std::uint64_t parallel_compute = 0x703fa0000001;
  constexpr std::uint64_t data_copy = 0x703e81800000; // R3=R1: separate40-bit qualification.
  constexpr std::array<std::uint64_t, 4> opcodes{reserved, swap, parallel_compute, data_copy};
  for (const auto opcode : opcodes) {
    lead_a1::SharcMemory memory;
    map(memory, opcode);
    lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
    const auto before = cpu.state();
    const auto *reason =
        opcode == data_copy ? "width not qualified" : "swap/reserved/parallel-compute";
    rejected([&] { static_cast<void>(cpu.step()); }, reason);
    require(cpu.state() == before && !memory.active());
  }
  lead_a1::SharcMemory memory;
  map(memory, data_to_base);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[2].value = simd;
  initial.effective = initial.system;
  lead_a1::Sharc cpu(memory, initial);
  rejected([&] { static_cast<void>(cpu.step()); }, "SIMD register transfer");
  require(cpu.state() == initial && !memory.active());
}

}

int main() {
  try {
    literal_transfers_and_alignment();
    source_fields_and_partial_knowledge();
    false_condition_and_bank_selection();
    system_effect_latency();
    unsupported_fields_and_modes();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
