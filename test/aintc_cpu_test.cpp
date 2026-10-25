// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm.hpp"
#include "arm/core/arm_system.hpp"
#include "arm/peripherals/aintc.hpp"
#include "common/memory.hpp"

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

constexpr std::uint32_t code_base = 0x1000;
constexpr std::uint32_t mmio_base = 0xfffee000;
constexpr std::uint32_t vector_register = 0xfffef604;
constexpr std::uint32_t spacing = 256;

void require(bool condition) {
  if (!condition) { throw std::runtime_error("Independent CPU/MMIO expectation failed"); }
}

std::vector<std::byte> read_program(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Missing owned MMIO program"); }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

lead_a1::ArmSystemInitial profile() {
  constexpr std::uint32_t control = 0x50078;
  return {.control = control,
          .instruction_cache_bytes = {},
          .data_cache_bytes = {},
          .instruction_cache_invalid = true,
          .data_cache_invalid = true,
          .instruction_lockdown = {},
          .data_lockdown = {},
          .write_buffer = std::vector<lead_a1::ArmBufferedWrite>{},
          .writeback_buffer_empty = true,
          .mmu = {}};
}

void check(const std::filesystem::path &spec, std::span<const std::byte> program, bool store) {
  auto initial = lead_a1::Aintc::reset_reference();
  constexpr unsigned int source = 3;
  initial.control = 0x10;
  initial.channels[source] = 2;
  initial.vector_base = 0x4000;
  initial.vector_size_programmed = true;
  initial.enabled[0] = 8;
  initial.pending[0] = 8;
  lead_a1::Aintc device(initial);
  lead_a1::Memory memory;
  memory.map_device(
      {.address = mmio_base, .size = 0x2000, .name = "owned AINTC fixture", .executable = false},
      device);
  memory.map({.address = code_base, .size = program.size(), .name = "owned MMIO caller"}, program);
  constexpr std::uint32_t selected_vector = 0x400c;
  if (store) { require(memory.read({.address = vector_register, .size = 4}) == selected_vector); }
  const auto before = device.state();
  lead_a1::ArmState state;
  constexpr std::uint32_t supervisor_status = 0xd3;
  state.cpsr = supervisor_status;
  state.registers[15] = code_base + ((store ? 3 : 2) * spacing);
  state.registers[0] = vector_register;
  state.registers[1] = 7;
  state.registers[2] = 8;
  lead_a1::Arm cpu(memory, spec, state, profile());
  const auto cpu_before = cpu.state();
  const auto system_before = cpu.system_state();
  bool failed{};
  try {
    static_cast<void>(cpu.step());
  } catch (const std::runtime_error &error) {
    require(std::string(error.what()).find("address=0xfffef608") != std::string::npos);
    failed = true;
  }
  require(failed && cpu.state() == cpu_before && cpu.system_state() == system_before);
  require(device.state() == before);
  device.pulse(4);
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    const auto program = read_program(argv[2]);
    for (const bool store : {false, true}) { check(argv[1], program, store); }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
