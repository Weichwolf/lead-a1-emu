// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/arm.hpp"
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
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t code_base = 0x1000;
constexpr std::uint32_t data_base = 0x100;
constexpr std::uint32_t control_reset = 0x50078;
constexpr std::uint32_t control_cached = control_reset | 0x1000;
constexpr std::uint32_t initial_cpsr = 0xd3;
constexpr std::uint32_t case_spacing = 256;
constexpr unsigned int pc_index = 15;

void require(bool condition) {
  if (!condition) { throw std::runtime_error("Independent ARM system expectation failed"); }
}

std::vector<std::byte> read_program(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Cannot read owned ARM system program"); }
  const auto data = std::as_bytes(std::span(bytes));
  return {data.begin(), data.end()};
}

lead_a1::ArmState initial(unsigned int index) {
  lead_a1::ArmState result;
  result.cpsr = initial_cpsr;
  result.registers[pc_index] = code_base + (index * case_spacing);
  return result;
}

lead_a1::ArmSystemInitial profile() {
  return {.control = control_reset,
          .instruction_cache_bytes = 128,
          .instruction_cache_invalid = true,
          .data_cache_invalid = false,
          .write_buffer = std::vector<lead_a1::ArmBufferedWrite>{}};
}

void map_program(lead_a1::Memory &memory, std::span<const std::byte> bytes) {
  memory.map({.address = code_base, .size = bytes.size(), .name = "owned system program"}, bytes);
  memory.map({.address = data_base, .size = 16, .name = "owned data RAM"});
}

void steps(lead_a1::Arm &cpu, unsigned int count) {
  for (unsigned int index = 0; index < count; ++index) { static_cast<void>(cpu.step()); }
}

void rejected(lead_a1::Arm &cpu, const std::string &reason) {
  const auto before = cpu.state();
  const auto system = cpu.system_state();
  try {
    steps(cpu, 1);
  } catch (const std::runtime_error &error) {
    const std::string message = error.what();
    require(message.find(reason) != std::string::npos);
    require(message.find("pc=0x") != std::string::npos);
    require(cpu.state() == before && cpu.system_state() == system);
    return;
  }
  throw std::runtime_error("Expected ARM system failure did not occur");
}

void startup(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  auto system = profile();
  system.instruction_cache_invalid = false;
  lead_a1::Arm cpu(memory, spec, initial(0), system);
  steps(cpu, 8);
  require(cpu.state().registers[1] == control_cached);
  require(cpu.state().registers[2] == 1);
  require(cpu.state().registers[3] == control_cached);
  const auto state = cpu.system_state();
  require(state.control == control_cached && state.instruction_cache_known);
  require(state.data_cache_invalid && state.write_buffer_known);
  require(state.instruction_invalidations == 1 && state.data_invalidations == 1);
  require(state.write_buffer_drains == 1 && state.buffered_writes == 0);
  require(state.cached_lines == 1 && state.instruction_misses == 1 && state.instruction_hits == 1);
}

void stale_instructions(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  auto state = initial(1);
  constexpr auto base = code_base + case_spacing;
  constexpr std::uint32_t retained_instruction_offset = 24;
  state.registers[2] = 0;
  state.registers[5] = control_reset;
  state.registers[7] = control_cached;
  state.registers[8] = base + retained_instruction_offset;
  state.registers[9] = base + 4;
  auto system = profile();
  system.control = control_cached;
  lead_a1::Arm cpu(memory, spec, state, system);
  steps(cpu, 1);
  constexpr std::uint32_t mov_r1_nine = 0xe3a01009;
  memory.write({.address = base + 4, .size = 4}, mov_r1_nine);
  steps(cpu, 1);
  require(cpu.state().registers[1] == 2);
  steps(cpu, 1);
  require(cpu.system_state().cached_lines == 0);
  constexpr std::uint32_t mov_r3_seven = 0xe3a03007;
  memory.write({.address = base + 12, .size = 4}, mov_r3_seven);
  steps(cpu, 1);
  require(cpu.state().registers[3] == 7);
  constexpr std::uint32_t mov_r1_eleven = 0xe3a0100b;
  memory.write({.address = base + 4, .size = 4}, mov_r1_eleven);
  steps(cpu, 1);
  require(cpu.state().registers[4] == mov_r1_eleven);
  steps(cpu, 1);
  constexpr std::uint32_t mov_r6_nine = 0xe3a06009;
  memory.write({.address = base + retained_instruction_offset, .size = 4}, mov_r6_nine);
  steps(cpu, 1);
  require(cpu.state().registers[6] == 9);
  steps(cpu, 3);
  require(cpu.state().registers[6] == 6);
}

void unsupported(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  auto system = profile();
  system.control = control_cached;
  constexpr std::array indexes{2U, 7U, 8U, 9U};
  constexpr std::array reasons{"register read", "opcode 1", "zero operand", "cache operation"};
  for (std::size_t index = 0; index < indexes.size(); ++index) {
    auto state = initial(indexes[index]);
    state.registers[0] = indexes[index] == 8 ? 1 : 0;
    lead_a1::Arm cpu(memory, spec, state, system);
    rejected(cpu, reasons[index]);
  }
  lead_a1::Arm unknown_operand(memory, spec, initial(8), profile());
  rejected(unknown_operand, "Unknown CP15 operand");
  auto user = initial(3);
  constexpr std::uint32_t user_cpsr = 0xd0;
  user.cpsr = user_cpsr;
  lead_a1::Arm unprivileged(memory, spec, user, profile());
  rejected(unprivileged, "unprivileged");
}

void controls(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  constexpr std::array<std::uint32_t, 5> values{
      control_reset | 1, control_reset | 0x80, control_reset | 0x8000, 0, control_reset | 0x800};
  constexpr std::array reasons{"MMU", "endian", "legacy", "SBO", "reserved"};
  for (std::size_t index = 0; index < values.size(); ++index) {
    auto state = initial(4);
    state.registers[0] = values[index];
    lead_a1::Arm cpu(memory, spec, state, profile());
    rejected(cpu, reasons[index]);
  }
  for (const bool geometry_known : {false, true}) {
    auto state = initial(4);
    state.registers[0] = control_cached;
    auto system = profile();
    system.instruction_cache_invalid = !geometry_known;
    if (!geometry_known) { system.instruction_cache_bytes.reset(); }
    lead_a1::Arm cpu(memory, spec, state, system);
    rejected(cpu, geometry_known ? "unknown ARM instruction cache" : "geometry");
  }
}

void unknowns_and_flags(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  lead_a1::Arm unconfigured(memory, spec, initial(3));
  rejected(unconfigured, "not configured");
  auto system = profile();
  system.control.reset();
  lead_a1::Arm unknown(memory, spec, initial(3), system);
  rejected(unknown, "Unknown initial ARM control");
  lead_a1::Arm skipped(memory, spec, initial(6));
  steps(skipped, 1);
  require(skipped.state().registers[pc_index] == code_base + (6 * case_spacing) + 4);
  require(!skipped.state().registers[0]);
  auto state = initial(10);
  constexpr std::uint32_t flags = 0xf8000000;
  state.cpsr |= flags;
  lead_a1::Arm flags_cpu(memory, spec, state, profile());
  steps(flags_cpu, 1);
  constexpr std::uint32_t q_flag = 0x08000000;
  require(flags_cpu.state().cpsr == (initial_cpsr | q_flag));
}

void unknown_fetch(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  auto system = profile();
  system.control = control_cached;
  system.instruction_cache_invalid = false;
  lead_a1::Arm unknown_validity(memory, spec, initial(3), system);
  rejected(unknown_validity, "cache validity");
  system.instruction_cache_invalid = true;
  system.instruction_cache_bytes.reset();
  lead_a1::Arm unknown_geometry(memory, spec, initial(3), system);
  rejected(unknown_geometry, "cache geometry");
  lead_a1::Memory partial;
  constexpr std::size_t line_bytes = 32;
  partial.map({.address = code_base, .size = line_bytes, .name = "partially known cache line"},
              bytes.first(4));
  system.instruction_cache_bytes = 128;
  lead_a1::Arm partial_line(partial, spec, initial(0), system);
  rejected(partial_line, "uninitialized");
}

void drains(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  constexpr std::uint32_t prior_value = 99;
  memory.write({.address = data_base, .size = 4}, prior_value);
  auto state = initial(5);
  state.registers[0] = 0;
  auto system = profile();
  system.write_buffer = std::vector<lead_a1::ArmBufferedWrite>{
      {.access = {.address = data_base, .size = 4}, .value = 7},
      {.access = {.address = data_base + 4, .size = 4}, .value = 11}};
  lead_a1::Arm good(memory, spec, state, system);
  steps(good, 1);
  require(memory.read({.address = data_base, .size = 4}) == 7);
  require(memory.read({.address = data_base + 4, .size = 4}) == 11);
  require(good.system_state().buffered_writes == 0 && good.system_state().write_buffer_drains == 1);
  system.write_buffer->back().access.address = data_base + 16;
  system.write_buffer->front().value = 11;
  lead_a1::Arm bad(memory, spec, state, system);
  rejected(bad, "unmapped");
  require(memory.read({.address = data_base, .size = 4}) == 7);
  system.write_buffer.reset();
  lead_a1::Arm unknown(memory, spec, state, system);
  rejected(unknown, "Unknown initial ARM write buffer");
}

void replacement(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  auto system = profile();
  system.control = control_cached;
  lead_a1::Arm random(memory, spec, initial(11), system);
  steps(random, 4);
  require(random.system_state().cached_lines == 4);
  rejected(random, "random cache replacement");
  constexpr std::uint32_t round_robin = 0x4000;
  system.control = control_cached | round_robin;
  lead_a1::Arm cyclic(memory, spec, initial(11), system);
  steps(cyclic, 4);
  require(cyclic.system_state().cached_lines == 4);
  rejected(cyclic, "round-robin cache replacement");
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    const auto bytes = read_program(argv[2]);
    startup(argv[1], bytes);
    stale_instructions(argv[1], bytes);
    unsupported(argv[1], bytes);
    controls(argv[1], bytes);
    unknowns_and_flags(argv[1], bytes);
    unknown_fetch(argv[1], bytes);
    drains(argv[1], bytes);
    replacement(argv[1], bytes);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
