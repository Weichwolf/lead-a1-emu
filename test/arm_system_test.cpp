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
constexpr std::uint32_t unlocked_cache = 0xfff0;

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
          .instruction_lockdown = unlocked_cache,
          .data_lockdown = unlocked_cache,
          .write_buffer = std::vector<lead_a1::ArmBufferedWrite>{},
          .mmu = std::nullopt};
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

void lockdown_registers(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  lead_a1::Arm cpu(memory, spec, initial(16), profile());
  steps(cpu, 8);
  constexpr std::uint32_t locked_instruction_way = 0xfff1;
  constexpr std::uint32_t locked_data_way = 0xfff2;
  require(cpu.state().registers[2] == locked_instruction_way);
  require(cpu.state().registers[3] == locked_data_way);
  require(cpu.system_state().instruction_lockdown == locked_instruction_way);
  require(cpu.system_state().data_lockdown == locked_data_way);
  auto system = profile();
  system.instruction_lockdown.reset();
  constexpr unsigned int instruction_read_case = 19;
  constexpr unsigned int data_read_case = 20;
  constexpr unsigned int tcm_case = 18;
  constexpr unsigned int write_case = 17;
  lead_a1::Arm unknown_instruction(memory, spec, initial(instruction_read_case), system);
  rejected(unknown_instruction, "Unknown initial ARM cache-lockdown");
  system.data_lockdown.reset();
  lead_a1::Arm unknown_data(memory, spec, initial(data_read_case), system);
  rejected(unknown_data, "Unknown initial ARM cache-lockdown");
  system.control = control_cached;
  lead_a1::Arm unknown_allocation(memory, spec, initial(instruction_read_case), system);
  rejected(unknown_allocation, "Unknown initial ARM instruction-cache lockdown");
  lead_a1::Arm tcm(memory, spec, initial(tcm_case), profile());
  rejected(tcm, "cache-lockdown selector");
  constexpr std::array<std::uint32_t, 2> invalid{0, 0xffffffff};
  for (const auto value : invalid) {
    auto state = initial(write_case);
    state.registers[0] = value;
    lead_a1::Arm bad(memory, spec, state, profile());
    rejected(bad, "lockdown SBO/reserved");
  }
  lead_a1::Arm unknown_operand(memory, spec, initial(write_case), profile());
  rejected(unknown_operand, "Unknown CP15 operand");
}

void allocation_masks(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  constexpr std::array<unsigned int, 16> available_ways{
      4, 3, 3, 2, 3, 2, 2, 1, 3, 2, 2, 1, 2, 1, 1, 1};
  constexpr std::uint32_t lockdown_sbo = 0xfff0;
  for (std::uint32_t mask = 0; mask < available_ways.size(); ++mask) {
    auto system = profile();
    system.control = control_cached;
    system.instruction_lockdown = lockdown_sbo | mask;
    lead_a1::Arm cpu(memory, spec, initial(11), system);
    if (available_ways[mask] == 1) {
      steps(cpu, 5);
      require(cpu.state().registers[0] == 15);
      require(cpu.system_state().cached_lines == 1);
      require(cpu.system_state().instruction_evictions == 4);
    } else {
      steps(cpu, available_ways[mask]);
      require(cpu.system_state().cached_lines == available_ways[mask]);
      rejected(cpu, "random cache replacement");
    }
  }
}

struct LockedExpectation {
  std::uint32_t initial;
  std::uint32_t later;
  std::uint32_t result;
  std::size_t lines;
  std::uint64_t evictions;
  std::uint64_t misses;
};

void locked_line_preservation(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr std::array<LockedExpectation, 3> cases{
      {{.initial = 0xfffe, .later = 0xfff7, .result = 1, .lines = 3, .evictions = 1, .misses = 4},
       {.initial = 0xffff, .later = 0xfffe, .result = 1, .lines = 3, .evictions = 1, .misses = 4},
       {.initial = 0xffff, .later = 0xfff7, .result = 9, .lines = 2, .evictions = 3, .misses = 5}}};
  constexpr unsigned int preservation_case = 21;
  constexpr std::uint32_t base = code_base + (preservation_case * case_spacing);
  constexpr std::uint32_t third_line = base + (2 * case_spacing);
  constexpr std::uint32_t mov_r0_nine = 0xe3a00009;
  for (const auto &expected : cases) {
    lead_a1::Memory memory;
    map_program(memory, bytes);
    auto state = initial(preservation_case);
    constexpr std::uint32_t way_one_only = 0xfffd;
    state.registers[1] = way_one_only;
    state.registers[2] = base + case_spacing;
    state.registers[4] = third_line;
    state.registers[5] = expected.later;
    state.registers[6] = base + (3 * case_spacing);
    state.registers[7] = base;
    state.registers[9] = 2;
    auto system = profile();
    system.control = control_cached;
    system.instruction_lockdown = expected.initial;
    lead_a1::Arm cpu(memory, spec, state, system);
    steps(cpu, 1);
    memory.write({.address = base, .size = 4}, mov_r0_nine);
    steps(cpu, 5);
    memory.write({.address = third_line, .size = 4}, mov_r0_nine);
    steps(cpu, 6);
    require(cpu.state().registers[0] == 4);
    steps(cpu, 7);
    require(cpu.state().registers[0] == expected.result);
    require(cpu.system_state().cached_lines == expected.lines);
    require(cpu.system_state().instruction_evictions == expected.evictions);
    require(cpu.system_state().instruction_misses == expected.misses);
  }
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
    lockdown_registers(argv[1], bytes);
    allocation_masks(argv[1], bytes);
    locked_line_preservation(argv[1], bytes);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
