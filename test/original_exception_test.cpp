// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm.hpp"
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

constexpr std::uint32_t code_base = 0x1000;
constexpr std::uint32_t local_base = 0xffff0000;
constexpr std::uint32_t original_entry = 0xffff0020;
constexpr std::uint32_t handler = 0xffff002c;
constexpr std::uint32_t vector = 0xffff0008;
constexpr std::uint32_t stack_top = 0x8000;
constexpr std::uint32_t user_sp = 0x9000;
constexpr std::uint32_t user_lr = 0x11223344;
constexpr std::uint32_t caller_status = 0xb8000050;
constexpr std::uint32_t system_status = 0xb800005f;
constexpr unsigned int pc_index = 15;
constexpr unsigned int spacing = 256;

struct Inputs {
  std::filesystem::path spec;
  std::filesystem::path images;
  std::filesystem::path owned;
};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Original exception expectation at line " +
                             std::to_string(where.line()));
  }
}

std::vector<std::byte> read(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) {
    throw std::runtime_error("Missing original/owned exception input");
  }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

lead_a1::ArmSystemInitial profile() {
  constexpr std::uint32_t high_control = 0x52078;
  return {.control = high_control,
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

void map(lead_a1::Memory &memory,
         std::span<const std::byte> original,
         std::span<const std::byte> caller) {
  constexpr unsigned int prefix = 32;
  memory.map({.address = local_base,
              .size = original.size() + prefix,
              .name = "original ARM0 and owned vector"},
             original,
             prefix);
  memory.map(
      {.address = code_base, .size = caller.size(), .name = "owned SVC caller", .writable = false},
      caller);
  constexpr std::uint32_t branch_opcode = 0xea000000;
  constexpr auto displacement = (handler - vector - 8) / 4;
  memory.write({.address = vector, .size = 4}, branch_opcode | displacement);
  constexpr unsigned int stack_extent = 96;
  constexpr unsigned int guard_below = 80;
  const std::vector<std::byte> pattern(stack_extent, std::byte{0xa5});
  memory.map(
      {.address = stack_top - guard_below, .size = pattern.size(), .name = "owned SVC stack"},
      pattern);
}

lead_a1::ArmState initial(unsigned int index) {
  lead_a1::ArmState state;
  state.cpsr = caller_status;
  constexpr std::uint32_t first = 0x10203000;
  constexpr std::uint32_t increment = 0x101;
  for (unsigned int reg = 0; reg < 13; ++reg) { state.registers[reg] = first + (reg * increment); }
  state.registers[13] = user_sp;
  state.registers[14] = user_lr;
  state.registers[pc_index] = code_base + (index * spacing);
  state.banks.modes[3].sp = stack_top;
  return state;
}

void check_stack(const lead_a1::Memory &memory,
                 const lead_a1::ArmState &caller,
                 std::uint32_t return_address) {
  constexpr unsigned int frame_bytes = 56;
  for (unsigned int reg = 0; reg < 13; ++reg) {
    require(memory.read({.address = stack_top - frame_bytes + (reg * 4), .size = 4}) ==
            caller.registers[reg]);
  }
  require(memory.read({.address = stack_top - 4, .size = 4}) == return_address);
  constexpr unsigned int scratch_offset = 60;
  constexpr std::uint32_t guard = 0xa5a5a5a5;
  require(memory.read({.address = stack_top - scratch_offset, .size = 4}) == guard);
  require(memory.read({.address = stack_top, .size = 4}) == guard);
}

void run(const Inputs &inputs, unsigned int part) {
  const auto original = read(inputs.images / (std::to_string(part) + "-ARM0.bin"));
  constexpr std::size_t expected_bytes = 616;
  require(original.size() == expected_bytes);
  const auto owned = read(inputs.owned);
  constexpr unsigned int select_system = 38;
  constexpr unsigned int other_selector = 39;
  for (const auto index : {select_system, other_selector}) {
    lead_a1::Memory memory;
    map(memory, original, owned);
    const auto caller = initial(index);
    lead_a1::Arm cpu(memory, inputs.spec, caller, profile());
    const auto event = cpu.step();
    require(event.exception == lead_a1::ArmException::software_interrupt && event.after == vector);
    require(cpu.state().registers[13] == stack_top);
    static_cast<void>(cpu.step());
    require(cpu.state().registers[pc_index] == handler);
    const auto resume = code_base + (index * spacing) + 4;
    unsigned int count{};
    constexpr unsigned int limit = 32;
    while (cpu.state().registers[pc_index] != resume && count < limit) {
      require(!cpu.step().exception);
      ++count;
    }
    require(count == (index == select_system ? 13 : 9));
    require(cpu.state().registers[pc_index] == resume);
    const auto expected_status = index == select_system ? system_status : caller_status;
    require(cpu.state().cpsr == expected_status);
    for (unsigned int reg = 0; reg < pc_index; ++reg) {
      require(cpu.state().registers[reg] == caller.registers[reg]);
    }
    require(cpu.state().banks.modes[3].sp == stack_top);
    require(cpu.state().banks.modes[3].lr == resume);
    require(cpu.state().banks.modes[3].saved.bits == expected_status);
    check_stack(memory, caller, resume);
    for (std::size_t byte = 0; byte < original.size(); ++byte) {
      require(memory.peek(original_entry + static_cast<std::uint32_t>(byte)) == original[byte]);
    }
    static_cast<void>(cpu.step());
    require(cpu.state().registers[0] == 7 && cpu.state().cpsr == expected_status);
  }
  std::cout << "Part " << part
            << ": original ARM0 SVC handler preserves caller/frame/flags and "
               "returns to System only for selector 0x050000 (13/9 instructions)\n";
}

}

int main(int argc, char **argv) {
  if (argc != 4) { return 1; }
  try {
    const Inputs inputs{.spec = argv[1], .images = argv[2], .owned = argv[3]};
    for (const unsigned int part : {0U, 1U}) { run(inputs, part); }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
