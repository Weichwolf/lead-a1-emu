// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/arm.hpp"
#include "lead_a1/arm_mmu.hpp"
#include "lead_a1/arm_system.hpp"
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
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t code_base = 0xc0000000;
constexpr std::uint32_t caller_base = 0x01000000;
constexpr std::uint32_t table_base = 0x10000;
constexpr std::uint32_t data_virtual = 0x90000000;
constexpr std::uint32_t data_physical = 0x20000000;
constexpr std::uint32_t second_offset = 0x4000;
constexpr std::array<std::uint32_t, 2> sweep_entries{0xc018ba5c, 0xc002e064};
constexpr std::array<std::uint32_t, 2> drain_entries{0xc018badc, 0xc002e0e4};
constexpr std::array<std::uint32_t, 2> markers{0x11223344, 0xaabbccdd};
constexpr std::uint32_t second_initial = 0x20202020;
constexpr std::uint32_t control = 0x5007d;
constexpr std::uint32_t caller_cpsr = 0xd3;
constexpr std::uint32_t final_cpsr = 0x600000d3;
constexpr std::uint32_t unlocked = 0xfff0;
constexpr std::size_t line_bytes = 32;
constexpr unsigned int pc_index = 15;

struct Inputs {
  std::filesystem::path spec;
  std::filesystem::path directory;
  std::filesystem::path owned;
};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Original cache-helper expectation failed at line " +
                             std::to_string(where.line()));
  }
}

std::vector<std::byte> read_file(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  constexpr std::size_t maximum_bytes = std::size_t{16} * 1024 * 1024;
  if (!input || bytes.empty() || bytes.size() > maximum_bytes) {
    throw std::runtime_error("Missing or oversized original/owned cache-test input");
  }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

void map_fixture(lead_a1::Memory &memory,
                 std::span<const std::byte> code,
                 std::span<const std::byte> caller) {
  memory.map(
      {.address = code_base, .size = code.size(), .name = "original CODE", .writable = false},
      code);
  memory.map(
      {.address = caller_base, .size = caller.size(), .name = "owned caller", .writable = false},
      caller);
  const std::vector<std::byte> zero(0x4000);
  memory.map({.address = table_base, .size = zero.size(), .name = "owned MMU directory"}, zero);
  constexpr std::uint32_t caller_descriptor = 0x10040;
  constexpr std::uint32_t code_descriptor = 0x13000;
  constexpr std::uint32_t data_descriptor = 0x12400;
  constexpr std::uint32_t uncached_attributes = 0xc12;
  constexpr std::uint32_t cached_attributes = 0xc1e;
  constexpr std::uint32_t section_bytes = 0x100000;
  memory.write({.address = caller_descriptor, .size = 4}, caller_base | uncached_attributes);
  for (unsigned int section = 0; section < 16; ++section) {
    memory.write({.address = code_descriptor + (section * 4), .size = 4},
                 (code_base + (section * section_bytes)) | uncached_attributes);
  }
  memory.write({.address = data_descriptor, .size = 4}, data_physical | cached_attributes);
  const std::array<std::byte, 2> patterns{std::byte{0x10}, std::byte{0x20}};
  for (unsigned int index = 0; index < patterns.size(); ++index) {
    const std::vector<std::byte> bytes(line_bytes, patterns[index]);
    memory.map({.address = data_physical + (index * second_offset),
                .size = bytes.size(),
                .name = "owned data line"},
               bytes);
  }
}

lead_a1::ArmSystemInitial profile() {
  constexpr std::size_t cache_bytes = std::size_t{16} * 1024;
  return {.control = control,
          .instruction_cache_bytes = cache_bytes,
          .data_cache_bytes = cache_bytes,
          .instruction_cache_invalid = true,
          .data_cache_invalid = true,
          .instruction_lockdown = unlocked,
          .data_lockdown = unlocked,
          .write_buffer = std::vector<lead_a1::ArmBufferedWrite>{},
          .writeback_buffer_empty = true,
          .mmu = lead_a1::ArmMmuInitial{.translation_table = table_base,
                                        .domain_access = 3,
                                        .fcse_pid = 0,
                                        .context_id = {},
                                        .tlb_lockdown = 0,
                                        .tlb_invalid = true,
                                        .locked_tlb_invalid = true}};
}

lead_a1::ArmState initial(unsigned int part) {
  lead_a1::ArmState state;
  state.cpsr = caller_cpsr;
  state.registers[0] = data_virtual;
  state.registers[2] = markers[0];
  state.registers[4] = data_virtual + second_offset;
  state.registers[5] = sweep_entries[part];
  state.registers[6] = markers[1];
  state.registers[7] = drain_entries[part];
  state.registers[pc_index] = caller_base;
  return state;
}

void steps(lead_a1::Arm &cpu, unsigned int count) {
  for (unsigned int index = 0; index < count; ++index) { static_cast<void>(cpu.step()); }
}

void check_data(const lead_a1::Memory &memory, unsigned int committed_lines) {
  constexpr unsigned int byte_bits = 8;
  constexpr std::uint32_t byte_mask = 255;
  constexpr std::array<unsigned int, 2> patterns{0x10, 0x20};
  for (unsigned int line = 0; line < patterns.size(); ++line) {
    for (unsigned int byte = 0; byte < line_bytes; ++byte) {
      const auto expected = line < committed_lines && byte < 4
                                ? ((markers[line] >> (byte * byte_bits)) & byte_mask)
                                : patterns[line];
      require(memory.read({.address = data_physical + (line * second_offset) + byte, .size = 1}) ==
              expected);
    }
  }
}

void run_sweep(lead_a1::Arm &cpu, unsigned int part) {
  constexpr std::uint32_t return_offset = 24;
  constexpr unsigned int instruction_bound = 3000;
  constexpr unsigned int sets = 128;
  constexpr unsigned int ways = 4;
  constexpr unsigned int way_shift = 30;
  constexpr unsigned int stride = 32;
  unsigned int operations{};
  unsigned int count{};
  while (cpu.state().registers[pc_index] != caller_base + return_offset &&
         count < instruction_bound) {
    const auto before = cpu.state();
    if (before.registers[pc_index] == sweep_entries[part] + 12) {
      require(operations < sets * ways);
      const auto expected = ((operations / sets) << way_shift) | ((operations % sets) * stride);
      require(before.registers[2] == expected);
      ++operations;
    }
    steps(cpu, 1);
    ++count;
  }
  constexpr unsigned int expected_instructions = 2578;
  require(count == expected_instructions && operations == sets * ways);
  require(cpu.state().registers[pc_index] == caller_base + return_offset);
  require(cpu.state().registers[0] == 0x1000 && cpu.state().registers[1] == 0);
  constexpr std::uint32_t last_index = 0xc0000fe0;
  require(cpu.state().registers[2] == last_index && cpu.state().cpsr == final_cpsr);
  const auto data = cpu.system_state().data;
  require(data.cached_lines == 0 && data.dirty_halves == 0 && data.writeback_words == 4);
  require(data.writebacks == 2 && data.invalidations == static_cast<std::uint64_t>(sets) * ways);
}

void check_code(const lead_a1::Memory &memory, std::span<const std::byte> code) {
  for (std::size_t index = 0; index < code.size(); ++index) {
    require(memory.peek(code_base + static_cast<std::uint32_t>(index)) == code[index]);
  }
}

void run(const Inputs &inputs, unsigned int part) {
  const auto code = read_file(inputs.directory / (std::to_string(part) + "-CODE.bin"));
  const auto caller = read_file(inputs.owned);
  lead_a1::Memory memory;
  map_fixture(memory, code, caller);
  const auto entry = initial(part);
  lead_a1::Arm cpu(memory, inputs.spec, entry, profile());
  steps(cpu, 4);
  require(cpu.system_state().data.cached_lines == 2 && cpu.system_state().data.dirty_halves == 2);
  require(cpu.system_state().data.read_misses == 2 && cpu.system_state().data.write_hits == 2);
  check_data(memory, 0);
  steps(cpu, 2);
  require(cpu.state().registers[pc_index] == sweep_entries[part]);
  run_sweep(cpu, part);
  check_data(memory, 1);
  steps(cpu, 2);
  require(cpu.state().registers[pc_index] == drain_entries[part]);
  steps(cpu, 3);
  require(cpu.state().registers[pc_index] == caller_base + line_bytes);
  require(cpu.state().registers[14] == caller_base + line_bytes);
  require(cpu.state().registers[3] == second_initial && cpu.state().cpsr == final_cpsr);
  for (unsigned int reg = 4; reg < 14; ++reg) {
    require(cpu.state().registers[reg] == entry.registers[reg]);
  }
  const auto system = cpu.system_state();
  require(system.control == control && system.data.explicit_drains == 1);
  require(system.data.writeback_words == 0 && system.data.buffered_stores == 0);
  check_data(memory, 2);
  check_code(memory, code);
  std::cout << "Part " << part
            << ": original 512-operation cache sweep and drain preserve "
               "CODE/registers and commit two dirty half-lines under owned MMU/caller inputs\n";
}

}

int main(int argc, char **argv) {
  if (argc != 4) { return 1; }
  try {
    const Inputs inputs{.spec = argv[1], .directory = argv[2], .owned = argv[3]};
    for (const unsigned int part : {0U, 1U}) { run(inputs, part); }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
