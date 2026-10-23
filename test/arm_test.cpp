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
        "", "", "uninitialized", "uninitialized", "uninitialized", "intrinsic"};
    const std::string message = messages[index];
    rejected([&cpu] { static_cast<void>(cpu.step()); }, message);
    require(cpu.state().registers == state.registers && cpu.state().cpsr == state.cpsr);
    if (index == 4) { require(memory.read({.address = data_base, .size = 4}) == 11); }
    rejected([&cpu] { static_cast<void>(cpu.step()); }, message);
  }
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

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    const auto bytes = read_program(argv[2]);
    arithmetic(argv[1], bytes);
    rollback(argv[1], bytes);
    mutations(argv[1], bytes);
    interworking(argv[1], bytes);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
