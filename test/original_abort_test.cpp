// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm.hpp"
#include "arm/core/arm_mmu.hpp"
#include "arm/core/arm_system.hpp"
#include "common/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t original_code = 0xc0000000;
constexpr std::uint32_t local = 0xffff0000;
constexpr std::uint32_t arm0 = 0xffff0020;
constexpr std::uint32_t owned_code = 0x80000000;
constexpr std::uint32_t owned_entry = owned_code + 256;
constexpr std::uint32_t missing = 0xa0000000;
constexpr std::uint32_t directory = 0x10000;
constexpr std::uint32_t section = 0xc12;
constexpr std::uint32_t stack = 0x9000;
constexpr std::uint32_t caller_status = 0xb800001f;
constexpr std::uint32_t saved_far = 0x11223344;

struct Inputs {
  std::filesystem::path specification;
  std::filesystem::path images;
  std::filesystem::path owned;
};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Original abort expectation at line " + std::to_string(where.line()));
  }
}

std::vector<std::byte> read(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Missing original/owned abort input"); }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

lead_a1::ArmSystemInitial profile(bool translated) {
  constexpr std::uint32_t high_control = 0x52078;
  return {.control = high_control | (translated ? 1 : 0),
          .instruction_cache_bytes = {},
          .data_cache_bytes = {},
          .instruction_cache_invalid = true,
          .data_cache_invalid = true,
          .instruction_lockdown = {},
          .data_lockdown = {},
          .write_buffer = std::vector<lead_a1::ArmBufferedWrite>{},
          .writeback_buffer_empty = true,
          .mmu = lead_a1::ArmMmuInitial{.translation_table = directory,
                                        .domain_access = 3,
                                        .fcse_pid = 0,
                                        .context_id = {},
                                        .tlb_lockdown = 0,
                                        .tlb_invalid = true,
                                        .locked_tlb_invalid = true},
          .faults = {.data = lead_a1::ArmFaultStatus{.status = 6, .domain = 7},
                     .instruction = lead_a1::ArmFaultStatus{.status = 8, .domain = 9},
                     .address = saved_far}};
}

lead_a1::ArmState initial() {
  lead_a1::ArmState state;
  state.cpsr = caller_status;
  for (unsigned int reg = 0; reg < 13; ++reg) { state.registers[reg] = reg + 1; }
  state.registers[0] = missing;
  state.registers[13] = stack;
  state.registers[14] = 0x1000;
  state.registers[15] = owned_entry;
  constexpr std::uint32_t abort_stack = 0x7000;
  state.banks.modes[4].sp = abort_stack;
  return state;
}

void map(lead_a1::Memory &memory,
         std::span<const std::byte> code,
         std::span<const std::byte> boot,
         std::span<const std::byte> owned) {
  memory.map(
      {.address = original_code, .size = code.size(), .name = "original CODE", .writable = false},
      code);
  constexpr unsigned int prefix = 32;
  memory.map(
      {.address = local, .size = boot.size() + prefix, .name = "original ARM0"}, boot, prefix);
  memory.map({.address = owned_code, .size = owned.size(), .name = "owned fault source"}, owned);
  const std::vector<std::byte> zeros(0x4000);
  memory.map({.address = directory, .size = zeros.size(), .name = "owned translation directory"},
             zeros);
  constexpr unsigned int stack_bytes = 128;
  const std::vector<std::byte> pattern(stack_bytes, std::byte{0xa5});
  memory.map({.address = stack - stack_bytes, .size = pattern.size(), .name = "owned stack"},
             pattern);
  constexpr std::uint32_t owned_descriptor = 0x12000;
  constexpr std::uint32_t high_descriptor = 0x13ffc;
  constexpr std::uint32_t high_section = 0xfff00000;
  memory.write({.address = directory, .size = 4}, section);
  memory.write({.address = owned_descriptor, .size = 4}, owned_code | section);
  memory.write({.address = high_descriptor, .size = 4}, high_section | section);
  constexpr std::uint32_t section_bytes = 0x100000;
  constexpr unsigned int section_shift = 20;
  for (std::size_t offset = 0; offset < code.size(); offset += section_bytes) {
    const auto address = original_code + static_cast<std::uint32_t>(offset);
    const auto descriptor = directory + ((address >> section_shift) * 4);
    memory.write({.address = descriptor, .size = 4}, address | section);
  }
}

void install_vectors(lead_a1::Memory &memory, const Inputs &inputs, unsigned int part) {
  constexpr std::array<std::uint32_t, 2> installers{0xc0172238, 0xc002d83c};
  auto state = initial();
  constexpr std::uint32_t masked_system = 0xb80000df;
  state.cpsr = masked_system;
  state.registers[15] = installers[part];
  lead_a1::Arm cpu(memory, inputs.specification, state, profile(false));
  constexpr unsigned int limit = 512;
  unsigned int count{};
  while (cpu.state().registers[15] != 0x1000 && count < limit) {
    require(!cpu.step().exception);
    ++count;
  }
  require(cpu.state().registers[15] == 0x1000 && count < limit);
}

void execute(const Inputs &inputs, unsigned int part, bool prefetch) {
  const auto code = read(inputs.images / (std::to_string(part) + "-CODE.bin"));
  const auto boot = read(inputs.images / (std::to_string(part) + "-ARM0.bin"));
  const auto owned = read(inputs.owned);
  lead_a1::Memory memory;
  map(memory, code, boot, owned);
  install_vectors(memory, inputs, part);
  auto state = initial();
  if (prefetch) { state.registers[15] = missing; }
  lead_a1::Arm cpu(memory, inputs.specification, state, profile(true));
  const auto event = cpu.step();
  const auto kind =
      prefetch ? lead_a1::ArmException::prefetch_abort : lead_a1::ArmException::data_abort;
  require(event.exception == kind && event.after == local + (prefetch ? 12 : 16));
  constexpr std::uint32_t expected_status = 0xb8000097;
  require(cpu.state().cpsr == expected_status &&
          cpu.state().banks.modes[4].saved.bits == caller_status);
  const auto source_pc = prefetch ? missing : owned_entry;
  require(cpu.state().registers[14] == source_pc + (prefetch ? 4 : 8));
  const auto faults = cpu.system_state().faults;
  const auto status = prefetch ? faults.instruction : faults.data;
  if (!status) { throw std::runtime_error("Missing original abort status"); }
  require(status->status == 5 && !status->domain);
  require(faults.address == (prefetch ? saved_far : missing));
  const auto before = profile(true).faults;
  require(prefetch ? faults.data == before.data : faults.instruction == before.instruction);
  constexpr std::array<std::uint32_t, 2> fatal{0xc0173ce4, 0xc002f7d8};
  constexpr std::uint32_t prefetch_entry = 0xffff0130;
  constexpr std::uint32_t data_entry = 0xffff012c;
  constexpr std::uint32_t trampoline = 0xffff0280;
  const std::array<std::uint32_t, 3> destinations{
      prefetch ? prefetch_entry : data_entry, trampoline, fatal[part]};
  for (const auto destination : destinations) {
    require(!cpu.step().exception && cpu.state().registers[15] == destination);
  }
  require(cpu.state().registers[0] == missing && cpu.state().cpsr == expected_status);
  for (unsigned int byte = 0; byte < boot.size(); ++byte) {
    require(memory.peek(arm0 + byte) == boot[byte]);
  }
  std::cout << "Part " << part << ": original " << (prefetch ? "prefetch" : "data")
            << " abort vector/trampoline reaches original fatal CODE entry in three instructions\n";
}

}

int main(int argc, char **argv) {
  if (argc != 4) { return 1; }
  try {
    const Inputs inputs{.specification = argv[1], .images = argv[2], .owned = argv[3]};
    for (const unsigned int part : {0U, 1U}) {
      for (const bool prefetch : {false, true}) { execute(inputs, part, prefetch); }
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
