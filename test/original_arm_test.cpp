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

void run_part(const std::filesystem::path &spec,
              unsigned int part,
              const std::filesystem::path &directory) {
  lead_a1::Memory memory;
  constexpr std::array<const char *, 3> names{"SRAM", "CODE", "ARM0"};
  constexpr std::array bases{shared_base, code_base, entry};
  for (std::size_t index = 0; index < names.size(); ++index) {
    const auto data =
        read_section(directory / (std::to_string(part) + "-" + names[index] + ".bin"));
    memory.map({.address = bases[index], .size = data.size(), .name = names[index]},
               std::as_bytes(std::span(data)));
  }
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
    require(message.find("coprocessor_moveto") != std::string::npos);
    stopped = true;
  }
  require(stopped);
  require(cpu.state() == before);
  std::cout
      << "Part " << part
      << ": original ARM0/CODE entry executes status updates; CP15 frontier remains explicit\n";
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    run_part(argv[1], 0, argv[2]);
    run_part(argv[1], 1, argv[2]);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
