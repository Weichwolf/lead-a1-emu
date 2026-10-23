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
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t shared_base = 0x80000000;
constexpr std::uint32_t code_base = 0xc0000000;
constexpr std::uint32_t entry = 0xffff0020;
constexpr std::uint32_t reference_cpsr = 0xd3;
constexpr std::size_t maximum_section_bytes = std::size_t{16} * 1024 * 1024;
constexpr unsigned int pc_index = 15;
constexpr std::uint32_t maintenance_frontier_offset = 0x10;

std::vector<char> read_section(const std::filesystem::path &path) {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) { throw std::runtime_error("Missing decoded reference section"); }
  const auto length = stream.tellg();
  if (length <= 0 || length > static_cast<std::streamoff>(maximum_section_bytes)) {
    throw std::runtime_error("Unsupported decoded reference size");
  }
  std::vector<char> bytes(static_cast<std::size_t>(length));
  stream.seekg(0);
  if (!stream.read(bytes.data(), static_cast<std::streamsize>(bytes.size()))) {
    throw std::runtime_error("Cannot read complete decoded reference");
  }
  return bytes;
}

void require(bool condition) {
  if (!condition) { throw std::runtime_error("Original ARM software-entry expectation failed"); }
}

void map_sections(lead_a1::Memory &memory,
                  unsigned int part,
                  const std::filesystem::path &directory) {
  constexpr std::array<const char *, 3> names{"SRAM", "CODE", "ARM0"};
  constexpr std::array bases{shared_base, code_base, entry};
  for (std::size_t index = 0; index < names.size(); ++index) {
    const auto data =
        read_section(directory / (std::to_string(part) + "-" + names[index] + ".bin"));
    memory.map({.address = bases[index], .size = data.size(), .name = names[index]},
               std::as_bytes(std::span(data)));
  }
}

void run_part(const std::filesystem::path &spec,
              unsigned int part,
              const std::filesystem::path &directory) {
  lead_a1::Memory memory;
  map_sections(memory, part, directory);
  lead_a1::ArmState initial;
  initial.registers[pc_index] = entry;
  initial.cpsr = reference_cpsr;
  lead_a1::Arm cpu(memory, spec, initial);
  const auto first = cpu.step();
  require(first.before == entry && first.after == entry + 4);
  require(cpu.state().registers[0] == code_base);
  const auto second = cpu.step();
  require(second.before == entry + 4 && second.after == code_base);
  require(cpu.state().cpsr == reference_cpsr);
  require(!cpu.state().registers[13]);
  // Original MRS, mask update, MSR and operand initialization precede CP15.
  for (unsigned int index = 0; index < 4; ++index) { static_cast<void>(cpu.step()); }
  require(cpu.state().registers[0] == 0);
  require(cpu.state().registers[pc_index] == code_base + maintenance_frontier_offset);
  require(cpu.state().cpsr == reference_cpsr);
  const auto before = cpu.state();
  bool stopped = false;
  try {
    static_cast<void>(cpu.step());
  } catch (const std::runtime_error &error) {
    const std::string message = error.what();
    require(message.find("pc=0xc0000010 width=4") != std::string::npos);
    require(message.find("CP15 write opc1=0 crn=7 crm=7 opc2=0 rt=0") != std::string::npos);
    stopped = true;
  }
  require(stopped);
  require(cpu.state() == before);
  std::cout
      << "Part " << part
      << ": original ARM0/CODE entry executes status updates; CP15 frontier remains explicit\n";
}

void configured_startup(const std::filesystem::path &spec,
                        unsigned int part,
                        const std::filesystem::path &directory) {
  lead_a1::Memory memory;
  map_sections(memory, part, directory);
  lead_a1::ArmState initial;
  initial.registers[pc_index] = entry;
  initial.cpsr = reference_cpsr;
  // Explicit reference inputs, not a claim about the physical software-entry state.
  constexpr std::uint32_t reference_control = 0x50078;
  constexpr std::size_t reference_icache_bytes = std::size_t{16} * 1024;
  const lead_a1::ArmSystemInitial system{.control = reference_control,
                                         .instruction_cache_bytes = reference_icache_bytes,
                                         .instruction_cache_invalid = false,
                                         .data_cache_invalid = false,
                                         .write_buffer = std::vector<lead_a1::ArmBufferedWrite>{}};
  lead_a1::Arm cpu(memory, spec, initial, system);
  constexpr std::uint32_t memory_initializer = code_base + 0x158;
  constexpr unsigned int instruction_limit = 256;
  unsigned int count{};
  while (cpu.state().registers[pc_index] != memory_initializer && count < instruction_limit) {
    static_cast<void>(cpu.step());
    ++count;
  }
  const auto state = cpu.state();
  require(state.registers[pc_index] == memory_initializer);
  constexpr std::uint32_t system_cpsr = 0xdf;
  require(state.cpsr == system_cpsr);
  constexpr std::array<std::array<std::uint32_t, 6>, 2> stack_tops{
      {{0x8001e7a0, 0x8001efa0, 0x8001eba0, 0x8001efc0, 0x8001f000, 0x8001efe0},
       {0x8001f790, 0x8001ff90, 0x8001fb90, 0x8001ffb0, 0x8001fff0, 0x8001ffd0}}};
  for (std::size_t bank = 0; bank < stack_tops[part].size(); ++bank) {
    require(state.banks.modes[bank].sp == stack_tops[part][bank]);
  }
  constexpr unsigned int sp_index = 13;
  require(state.registers[sp_index] == stack_tops[part][0]);
  const auto cache = cpu.system_state();
  constexpr std::uint32_t icache_enable = 0x1000;
  require(cache.control == (reference_control | icache_enable));
  require(cache.instruction_invalidations == 1 && cache.data_invalidations == 1);
  require(cache.write_buffer_drains == 1 && cache.instruction_misses != 0);
  std::cout << "Part " << part << ": original cache setup and all six stack banks reach CODE"
            << "+0x158 in " << count << " instructions under explicit reference inputs\n";
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    run_part(argv[1], 0, argv[2]);
    run_part(argv[1], 1, argv[2]);
    configured_startup(argv[1], 0, argv[2]);
    configured_startup(argv[1], 1, argv[2]);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
