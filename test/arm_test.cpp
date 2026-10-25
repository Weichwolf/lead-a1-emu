// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm.hpp"
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
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t code_base = 0x1000;
constexpr std::uint32_t data_base = 0x100;
constexpr std::uint32_t initial_cpsr = 0xd3;
constexpr std::size_t case_spacing = 256;
constexpr unsigned int pc_index = 15;
constexpr std::uint32_t minimum_signed_value = 0x80000000;
constexpr std::uint32_t maximum_signed_value = 0x7fffffff;
constexpr std::uint32_t overflow_negative_flags = 0x90000000;
constexpr std::uint32_t compare_flags = 0x60000000;
constexpr std::uint32_t branch_destination_offset = 40;

void require(bool condition) {
  if (!condition) { throw std::runtime_error("Independent ARM expectation failed"); }
}

std::vector<std::byte> read_program(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Cannot read owned ARM test program"); }
  const auto data = std::as_bytes(std::span(bytes));
  return {data.begin(), data.end()};
}

lead_a1::ArmState initial(unsigned int index) {
  lead_a1::ArmState result;
  result.cpsr = initial_cpsr;
  result.registers[pc_index] = code_base + static_cast<std::uint32_t>(index * case_spacing);
  return result;
}

void map_program(lead_a1::Memory &memory, std::span<const std::byte> bytes) {
  memory.map({.address = code_base, .size = bytes.size(), .name = "owned test code"}, bytes);
  memory.map({.address = data_base, .size = 16, .name = "uninitialized owned RAM"});
}

template <typename Function> void rejected(Function function, const std::string &expected) {
  try {
    function();
  } catch (const std::runtime_error &error) {
    require(std::string(error.what()).find(expected) != std::string::npos);
    return;
  }
  throw std::runtime_error("Expected ARM boundary failure did not occur");
}

void arithmetic(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  lead_a1::Arm cpu(memory, spec, initial(0));
  static_cast<void>(cpu.step());
  require(cpu.state().registers[0] == maximum_signed_value);
  static_cast<void>(cpu.step());
  require(cpu.state().registers[1] == minimum_signed_value);
  require(cpu.state().cpsr == (initial_cpsr | overflow_negative_flags));
  for (unsigned int step = 0; step < 4; ++step) { static_cast<void>(cpu.step()); }
  require(cpu.state().registers[3] == minimum_signed_value);
  require(cpu.state().cpsr == (initial_cpsr | compare_flags));
  require(memory.read({.address = data_base, .size = 4}) == minimum_signed_value);
  require(memory.read({.address = data_base + 3}) == 128);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(cpu.state().registers[4] == 9);
  require(!cpu.state().registers[5]);
  const auto branch = cpu.step();
  require(branch.after == code_base + branch_destination_offset);
  static_cast<void>(cpu.step());
  require(cpu.state().registers[7] == 11);
  require(!cpu.state().registers[6]);
}

void rollback(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  for (const auto index : std::array{2U, 3U, 4U, 5U}) {
    auto state = initial(index);
    state.registers[0] = 7;
    if (index == 3 || index == 4) { state.registers[1] = data_base; }
    if (index == 3) { memory.write({.address = data_base, .size = 4}, 11); }
    lead_a1::Arm cpu(memory, spec, state);
    constexpr std::array<const char *, 6> messages{
        "", "", "uninitialized", "uninitialized", "uninitialized", "CP15"};
    const std::string message = messages[index];
    rejected([&cpu] { static_cast<void>(cpu.step()); }, message);
    require(cpu.state().registers == state.registers && cpu.state().cpsr == state.cpsr);
    if (index == 4) { require(memory.read({.address = data_base, .size = 4}) == 11); }
    rejected([&cpu] { static_cast<void>(cpu.step()); }, message);
  }
}

void reused_snapshot_rollback(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr std::uint32_t second_first_word = 19;
  constexpr std::uint32_t second_last_word = 23;
  constexpr std::uint32_t program_bytes = 20;
  lead_a1::Memory memory;
  map_program(memory, bytes);
  memory.write({.address = data_base, .size = 4}, 11);
  memory.write({.address = data_base + 8, .size = 4}, second_first_word);
  lead_a1::Arm cpu(memory, spec, initial(14));
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(cpu.state().registers[0] == 7 && cpu.state().registers[1] == data_base);
  require(!cpu.state().registers[2] && !cpu.state().registers[3]);
  const auto first = cpu.state();
  for (unsigned int attempt = 0; attempt < 2; ++attempt) {
    rejected([&cpu] { static_cast<void>(cpu.step()); }, "uninitialized");
    require(cpu.state() == first);
  }
  memory.write({.address = data_base + 4, .size = 4}, 13);
  static_cast<void>(cpu.step());
  require(cpu.state().registers[0] == 11 && cpu.state().registers[2] == 13);
  require(cpu.state().registers[1] == data_base + 8 && !cpu.state().registers[3]);
  static_cast<void>(cpu.step());
  require(cpu.state().registers[2] == 14 && cpu.state().cpsr == initial_cpsr);
  const auto second = cpu.state();
  for (unsigned int attempt = 0; attempt < 2; ++attempt) {
    rejected([&cpu] { static_cast<void>(cpu.step()); }, "uninitialized");
    require(cpu.state() == second);
  }
  memory.write({.address = data_base + 12, .size = 4}, second_last_word);
  static_cast<void>(cpu.step());
  require(cpu.state().registers[0] == second_first_word &&
          cpu.state().registers[3] == second_last_word);
  require(cpu.state().registers[1] == data_base + 16 && cpu.state().registers[2] == 14);
  require(cpu.state().registers[pc_index] == code_base + (14 * case_spacing) + program_bytes);
  require(cpu.state().cpsr == initial_cpsr);
}

void mutations(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  lead_a1::Arm cpu(memory, spec, initial(6));
  static_cast<void>(cpu.step());
  require(cpu.state().registers[0] == 1);
  constexpr std::uint32_t mov_r0_three = 0xe3a00003;
  const auto pc = code_base + (6 * case_spacing) + 4;
  memory.write({.address = pc, .size = 4}, mov_r0_three);
  static_cast<void>(cpu.step());
  require(cpu.state().registers[0] == 3);
  lead_a1::Memory isolated;
  const auto last_instruction = bytes.subspan(7 * case_spacing, 4);
  isolated.map({.address = code_base, .size = 4, .name = "isolated final instruction"},
               last_instruction);
  lead_a1::Arm final(isolated, spec, initial(0));
  static_cast<void>(final.step());
  require(final.state().registers[0] == 13);
  rejected([&final] { static_cast<void>(final.step()); }, "unmapped");
}

void interworking(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  const auto target = code_base + (7 * case_spacing);
  auto state = initial(8);
  state.registers[0] = target;
  lead_a1::Arm even(memory, spec, state);
  require(even.step().after == target);
  require(even.state().cpsr == initial_cpsr);
  static_cast<void>(even.step());
  require(even.state().registers[0] == 13);
  state.registers[0] = target + 1;
  lead_a1::Arm odd(memory, spec, state);
  rejected([&odd] { static_cast<void>(odd.step()); }, "Unsupported Thumb");
  require(odd.state().registers == state.registers && odd.state().cpsr == state.cpsr);
}

void decode_equivalence(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory cached_memory;
  lead_a1::Memory direct_memory;
  map_program(cached_memory, bytes);
  map_program(direct_memory, bytes);
  lead_a1::Arm cached(cached_memory, spec, initial(9));
  lead_a1::Arm direct(direct_memory, spec, initial(9), std::nullopt, {.cache_translations = false});
  constexpr std::uint32_t loop_address = code_base + (9 * case_spacing) + 12;
  constexpr std::uint32_t subtract_opcode = 0xe0400002;
  constexpr std::uint32_t add_opcode = 0xe0800002;
  constexpr unsigned int total_steps = 256;
  constexpr unsigned int mutation_step = 64;
  constexpr unsigned int restoration_step = 128;
  for (unsigned int index = 0; index < total_steps; ++index) {
    if (index == mutation_step || index == restoration_step) {
      const auto word = index == mutation_step ? subtract_opcode : add_opcode;
      cached_memory.write({.address = loop_address, .size = 4}, word);
      direct_memory.write({.address = loop_address, .size = 4}, word);
    }
    const auto left = cached.step();
    const auto right = direct.step();
    require(left.before == right.before && left.after == right.after);
    require(left.assembly == right.assembly);
    require(cached.state() == direct.state() && cached.system_state() == direct.system_state());
    for (std::uint32_t offset = 0; offset < 4; ++offset) {
      require(cached_memory.peek(data_base + offset) == direct_memory.peek(data_base + offset));
    }
  }
  constexpr std::uint32_t final_accumulator = 23;
  constexpr std::uint32_t final_stored_value = 22;
  require(cached.state().registers[0] == final_accumulator);
  require(cached_memory.read({.address = data_base, .size = 4}) == final_stored_value);
  constexpr std::uint64_t distinct_translations = 10;
  require(cached.decode_statistics().translations == distinct_translations);
  require(cached.decode_statistics().hits == total_steps - distinct_translations);
  require(direct.decode_statistics().translations == total_steps);
  require(direct.decode_statistics().hits == 0 && direct.decode_statistics().entries == 0);
}

void bounded_decoding(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  lead_a1::Arm cpu(memory, spec, initial(10));
  constexpr unsigned int distinct_addresses = 129;
  constexpr unsigned int cycles = 2;
  constexpr unsigned int cache_bound = 128;
  constexpr auto branch_address =
      code_base + (10 * case_spacing) + (std::size_t{distinct_addresses} * 4);
  for (unsigned int index = 0; index < (distinct_addresses + 1) * cycles; ++index) {
    const auto step = cpu.step();
    if (step.before != branch_address) { require(cpu.state().registers[0] == step.before); }
    require(cpu.decode_statistics().entries <= cache_bound);
  }
  require(cpu.state().registers[pc_index] == code_base + (10 * case_spacing));
}

void indirect_calls(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory cached_memory;
  lead_a1::Memory direct_memory;
  map_program(cached_memory, bytes);
  map_program(direct_memory, bytes);
  lead_a1::Arm cached(cached_memory, spec, initial(13));
  lead_a1::Arm direct(
      direct_memory, spec, initial(13), std::nullopt, {.cache_translations = false});
  constexpr unsigned int instruction_count = 14;
  for (unsigned int index = 0; index < instruction_count; ++index) {
    const auto left = cached.step();
    const auto right = direct.step();
    require(left.before == right.before && left.after == right.after);
    require(cached.state() == direct.state());
  }
  require(cached.state().registers[1] == 2 && cached.state().registers[2] == 7);
  constexpr std::uint32_t return_offset = 32;
  require(cached.state().registers[14] == code_base + (13 * case_spacing) + return_offset);
  constexpr std::uint32_t target_offset = 36;
  require(cached_memory.read({.address = data_base, .size = 4}) ==
          code_base + (13 * case_spacing) + target_offset);
  require(cached.state().cpsr == initial_cpsr);
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    const auto bytes = read_program(argv[2]);
    arithmetic(argv[1], bytes);
    rollback(argv[1], bytes);
    reused_snapshot_rollback(argv[1], bytes);
    mutations(argv[1], bytes);
    interworking(argv[1], bytes);
    decode_equivalence(argv[1], bytes);
    bounded_decoding(argv[1], bytes);
    indirect_calls(argv[1], bytes);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
