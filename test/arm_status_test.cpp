// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/arm.hpp"
#include "lead_a1/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t base = 0x1000;
constexpr std::uint32_t spacing = 256;
constexpr std::uint32_t supervisor_cpsr = 0xd3;
constexpr unsigned int pc_index = 15;
constexpr std::uint32_t carry_flag = 0x20000000;
constexpr std::uint32_t nv_flags = 0x90000000;
constexpr std::uint32_t fiq_saved = 0xa00000d2;
constexpr std::uint32_t irq_saved = 0x500000d1;
constexpr std::uint32_t supervisor_sp = 0x400;
constexpr std::uint32_t supervisor_lr = 0x40;
constexpr std::uint32_t fiq_sp = 0x500;
constexpr std::uint32_t fiq_lr = 0x50;
constexpr std::uint32_t irq_sp = 0x600;
constexpr std::uint32_t irq_lr = 0x60;
constexpr std::uint32_t user_sp = 0x700;
constexpr std::uint32_t user_lr = 0x70;
constexpr std::uint32_t user_cpsr = 0xd0;
constexpr std::uint32_t system_cpsr = 0xdf;
constexpr std::uint32_t irq_cpsr = 0xd2;
constexpr std::uint32_t enabled_system_cpsr = 0x1f;
constexpr std::uint32_t q_flag = 0x08000000;
constexpr std::uint32_t fully_known_status = 0xffffffff;

void require(bool value) {
  if (!value) { throw std::runtime_error("Independent ARM status/bank expectation failed"); }
}

std::vector<std::byte> read_program(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Cannot read owned status program"); }
  const auto data = std::as_bytes(std::span(bytes));
  return {data.begin(), data.end()};
}

lead_a1::ArmState initial(unsigned int index) {
  lead_a1::ArmState result;
  result.cpsr = supervisor_cpsr;
  result.registers[pc_index] = base + (index * spacing);
  return result;
}

void step(lead_a1::Arm &cpu, unsigned int count = 1) {
  for (unsigned int index = 0; index < count; ++index) { static_cast<void>(cpu.step()); }
}

void reject(lead_a1::Arm &cpu, const std::string &expected) {
  const auto before = cpu.state();
  bool rejected = false;
  try {
    step(cpu);
  } catch (const std::runtime_error &error) {
    require(std::string(error.what()).find(expected) != std::string::npos);
    rejected = true;
  }
  require(rejected && cpu.state() == before);
}

void status_values(lead_a1::Memory &memory, const std::filesystem::path &spec) {
  lead_a1::Arm cpu(memory, spec, initial(0));
  step(cpu);
  require(cpu.state().registers[0] == supervisor_cpsr);
  step(cpu, 3);
  require(cpu.state().registers[2] == 1);
  require(cpu.state().cpsr == (supervisor_cpsr | carry_flag));
  step(cpu);
  require(cpu.state().registers[3] == (supervisor_cpsr | carry_flag));
  step(cpu, 2);
  require(cpu.state().registers[4] == (supervisor_cpsr | nv_flags));
  step(cpu, 2);
  require(cpu.state().registers[5] == (enabled_system_cpsr | nv_flags));
  require(!cpu.state().registers[13]);
  step(cpu, 2);
  require(cpu.state().registers[6] == (enabled_system_cpsr | q_flag));
  step(cpu, 2);
  require(cpu.state().registers[7] == enabled_system_cpsr);
}

void banks(lead_a1::Memory &memory, const std::filesystem::path &spec) {
  auto state = initial(1);
  state.registers[0] = fiq_saved;
  state.registers[2] = irq_saved;
  state.registers[3] = supervisor_cpsr;
  lead_a1::Arm cpu(memory, spec, state);
  step(cpu, 5);
  require(!cpu.state().registers[8] && !cpu.state().registers[12]);
  require(!cpu.state().registers[13] && !cpu.state().registers[14]);
  require(cpu.state().banks.modes[3].sp == supervisor_sp);
  require(cpu.state().banks.modes[3].lr == supervisor_lr);
  step(cpu, 6);
  require(cpu.state().registers[8] == 1 && cpu.state().registers[12] == 2);
  require(!cpu.state().registers[13] && !cpu.state().registers[14]);
  require(cpu.state().banks.modes[1].sp == fiq_sp);
  require(cpu.state().banks.modes[1].lr == fiq_lr);
  require(cpu.state().banks.modes[1].saved.bits == fiq_saved);
  step(cpu, 4);
  require(cpu.state().registers[13] == supervisor_sp && cpu.state().registers[14] == supervisor_lr);
  require(cpu.state().banks.modes[2].sp == irq_sp);
  require(cpu.state().banks.modes[2].lr == irq_lr);
  require(cpu.state().banks.modes[2].saved.bits == irq_saved);
  const auto snapshot = cpu.state().banks;
  step(cpu);
  require(!cpu.state().registers[13] && !cpu.state().registers[14]);
  step(cpu, 3);
  require(cpu.state().cpsr == user_cpsr);
  require(cpu.state().registers[13] == user_sp && cpu.state().registers[14] == user_lr);
  step(cpu, 2);
  require(cpu.state().registers[4] == user_cpsr);
  step(cpu, 3);
  require(cpu.state().registers[6] == 1);
  require(cpu.state().cpsr == (user_cpsr | carry_flag));
  auto restored = initial(2);
  restored.banks = snapshot;
  lead_a1::Arm saved(memory, spec, restored);
  step(saved, 2);
  require(saved.state().registers[0] == fiq_saved);
  require(saved.state().registers[8] == 3 && saved.state().registers[12] == 4);
  require(saved.state().registers[13] == fiq_sp && saved.state().registers[14] == fiq_lr);
  step(saved, 2);
  require(saved.state().registers[1] == irq_saved);
  require(saved.state().registers[13] == irq_sp && saved.state().registers[14] == irq_lr);
}

void saved_and_predication(lead_a1::Memory &memory, const std::filesystem::path &spec) {
  lead_a1::Arm conditional(memory, spec, initial(3));
  step(conditional, 4);
  require(!conditional.state().registers[0] && !conditional.state().registers[1]);
  require(conditional.state().registers[2] == supervisor_cpsr);
  lead_a1::Arm partial(memory, spec, initial(4));
  step(partial);
  require(partial.state().banks.modes[3].saved.bits == irq_cpsr);
  reject(partial, "Uninitialized SPSR");
  auto incomplete = initial(5);
  incomplete.banks.modes[3].saved = partial.state().banks.modes[3].saved;
  lead_a1::Arm complete(memory, spec, incomplete);
  step(complete, 2);
  require(complete.state().registers[0] == (irq_cpsr | carry_flag));
  for (auto mode : std::array{user_cpsr, system_cpsr}) {
    auto state = initial(6);
    state.cpsr = mode;
    lead_a1::Arm absent(memory, spec, state);
    reject(absent, "User/System");
  }
  lead_a1::Arm untaken(memory, spec, initial(16));
  step(untaken);
  require(!untaken.state().registers[14]);
  require(untaken.state().cpsr == supervisor_cpsr);
}

void invalid(lead_a1::Memory &memory, const std::filesystem::path &spec) {
  constexpr std::array<unsigned int, 4> cases{7, 8, 10, 11};
  constexpr std::array<const char *, 4> messages{
      "processor mode", "execution-state", "destination PC", "operand PC"};
  for (std::size_t index = 0; index < cases.size(); ++index) {
    lead_a1::Arm cpu(memory, spec, initial(cases[index]));
    reject(cpu, messages[index]);
  }
  auto reserved = initial(9);
  constexpr std::uint32_t reserved_bit = 0x100;
  reserved.registers[0] = reserved_bit;
  lead_a1::Arm reserved_cpu(memory, spec, reserved);
  reject(reserved_cpu, "reserved PSR");
  lead_a1::Arm unknown(memory, spec, initial(9));
  reject(unknown, "Uninitialized ARM status operand");
  auto malformed_saved = initial(6);
  malformed_saved.banks.modes[3].saved = {.bits = supervisor_cpsr | reserved_bit,
                                          .known_mask = fully_known_status};
  lead_a1::Arm bad_saved(memory, spec, malformed_saved);
  reject(bad_saved, "reserved saved PSR");
  auto masked = initial(12);
  masked.registers[0] = 0;
  lead_a1::Arm ignored(memory, spec, masked);
  step(ignored);
  require(ignored.state().cpsr == supervisor_cpsr);
  for (auto index : std::array{14U, 15U}) {
    auto state = initial(index);
    state.registers[0] = base;
    state.registers[14] = base + 4;
    constexpr std::uint32_t implemented_psr_bits = 0xf90000ff;
    state.banks.modes[3].saved = {.bits = user_cpsr, .known_mask = implemented_psr_bits};
    lead_a1::Arm unsupported(memory, spec, state);
    reject(unsupported, "exception return or banked block");
  }
}

void abort_undefined(lead_a1::Memory &memory, const std::filesystem::path &spec) {
  auto state = initial(13);
  state.registers[13] = supervisor_sp;
  state.registers[14] = supervisor_lr;
  lead_a1::Arm cpu(memory, spec, state);
  step(cpu, 8);
  require(cpu.state().registers[0] == supervisor_sp);
  require(cpu.state().registers[14] == supervisor_lr);
  constexpr std::uint32_t abort_sp = 0x800;
  constexpr std::uint32_t abort_lr = 0x80;
  constexpr std::uint32_t undefined_sp = 0x900;
  constexpr std::uint32_t undefined_lr = 0x90;
  step(cpu, 2);
  require(cpu.state().registers[1] == abort_sp && cpu.state().registers[14] == abort_lr);
  step(cpu, 2);
  require(cpu.state().registers[2] == undefined_sp && cpu.state().registers[14] == undefined_lr);
}

void conditions(lead_a1::Memory &memory, const std::filesystem::path &spec) {
  // Independent condition truth masks over N/Z/C/V encoded as a four-bit index.
  constexpr std::array<std::uint16_t, 15> truth{0xf0f0,
                                                0x0f0f,
                                                0xcccc,
                                                0x3333,
                                                0xff00,
                                                0x00ff,
                                                0xaaaa,
                                                0x5555,
                                                0x0c0c,
                                                0xf3f3,
                                                0xaa55,
                                                0x55aa,
                                                0x0a05,
                                                0xf5fa,
                                                0xffff};
  constexpr std::uint32_t msr_flags_r0 = 0x0128f000;
  constexpr unsigned int flag_shift = 28;
  for (unsigned int condition = 0; condition < truth.size(); ++condition) {
    memory.write({.address = base, .size = 4}, msr_flags_r0 | (condition << flag_shift));
    for (unsigned int flags = 0; flags < 16; ++flags) {
      auto state = initial(0);
      state.cpsr |= flags << flag_shift;
      const auto changed = (flags ^ 15U) << flag_shift;
      state.registers[0] = changed;
      lead_a1::Arm cpu(memory, spec, state);
      step(cpu);
      const bool passed = ((truth[condition] >> flags) & 1U) != 0;
      require(cpu.state().cpsr == (passed ? supervisor_cpsr | changed : state.cpsr));
    }
  }
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    const auto bytes = read_program(argv[2]);
    lead_a1::Memory memory;
    memory.map({.address = base, .size = bytes.size(), .name = "owned status tests"}, bytes);
    status_values(memory, argv[1]);
    banks(memory, argv[1]);
    saved_and_predication(memory, argv[1]);
    invalid(memory, argv[1]);
    abort_undefined(memory, argv[1]);
    conditions(memory, argv[1]);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
