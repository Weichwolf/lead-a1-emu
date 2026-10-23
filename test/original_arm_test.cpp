// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/arm.hpp"
#include "lead_a1/arm_system.hpp"
#include "lead_a1/memory.hpp"

#include <array>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

namespace {

constexpr std::uint32_t shared_base = 0x80000000;
constexpr std::uint32_t code_base = 0xc0000000;
constexpr std::uint32_t entry = 0xffff0020;
constexpr std::uint32_t reference_cpsr = 0xd3;
constexpr std::size_t maximum_section_bytes = std::size_t{16} * 1024 * 1024;
constexpr unsigned int pc_index = 15;
constexpr std::uint32_t maintenance_frontier_offset = 0x10;
constexpr std::size_t shared_ram_bytes = std::size_t{128} * 1024;
constexpr std::size_t local_ram_bytes = std::size_t{8} * 1024;
constexpr std::uint32_t local_base = 0xffff0000;
constexpr std::uint32_t runtime_checkpoint = code_base + 0x298;
constexpr std::array<std::uint32_t, 2> bss_lengths{0x474578, 0x1f258};
constexpr std::array<unsigned int, 2> initialization_steps{2335493, 63861};
constexpr std::size_t guard_bytes = 16;
constexpr char guard_value = static_cast<char>(0xa5);
constexpr std::array<std::uint32_t, 3> saved_register_values{0x33445566, 0x778899aa, 0xbbccddee};
constexpr std::uint32_t preserved_r9 = 0x99aabbcc;
constexpr std::uint32_t preserved_r10 = 0xaabbccdd;

struct RuntimeProbe {
  unsigned int limit{};
  bool single_cache_way{};
};

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

std::vector<char> map_reference_ram(lead_a1::Memory &memory,
                                    unsigned int part,
                                    const std::filesystem::path &directory) {
  const auto prefix = directory / std::to_string(part);
  const auto code = read_section(prefix.string() + "-CODE.bin");
  auto initialized = code;
  initialized.resize(code.size() + bss_lengths[part] + guard_bytes, guard_value);
  memory.map({.address = code_base, .size = maximum_section_bytes, .name = "reference DDR"},
             std::as_bytes(std::span(initialized)));
  const auto shared = read_section(prefix.string() + "-SRAM.bin");
  memory.map({.address = shared_base, .size = shared_ram_bytes, .name = "reference shared RAM"},
             std::as_bytes(std::span(shared)));
  const auto arm0 = read_section(prefix.string() + "-ARM0.bin");
  memory.map({.address = local_base, .size = local_ram_bytes, .name = "reference local RAM"},
             std::as_bytes(std::span(arm0)),
             entry - local_base);
  return code;
}

unsigned int run_until(lead_a1::Arm &cpu, std::uint32_t target) {
  constexpr unsigned int instruction_limit = 3000000;
  constexpr unsigned int probe_interval = 4096;
  constexpr unsigned int progress_interval = 262144;
  const auto begin = std::chrono::steady_clock::now();
  auto pc = cpu.state().registers[pc_index];
  unsigned int count{};
  while (pc != target && count < instruction_limit) {
    pc = cpu.step().after;
    ++count;
    if (count == probe_interval || count % progress_interval == 0) {
      const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin);
      std::cout << "Original entry: " << count << " instructions in " << elapsed.count()
                << " seconds\n"
                << std::flush;
    }
  }
  require(pc == target);
  return count;
}

void check_initialized_memory(const lead_a1::Memory &memory,
                              unsigned int part,
                              const std::vector<char> &code) {
  for (std::size_t offset = 0; offset < code.size(); ++offset) {
    const auto address = code_base + static_cast<std::uint32_t>(offset);
    require(memory.peek(address) == static_cast<std::byte>(code[offset]));
  }
  const auto bss_start = code_base + static_cast<std::uint32_t>(code.size());
  constexpr unsigned int chunk_bytes = 8;
  for (std::uint32_t offset = 0; offset < bss_lengths[part]; offset += chunk_bytes) {
    require(memory.read({.address = bss_start + offset, .size = chunk_bytes}) == 0);
  }
  const auto bss_end = bss_start + bss_lengths[part];
  for (std::uint32_t offset = 0; offset < guard_bytes; ++offset) {
    require(memory.peek(bss_end + offset) == static_cast<std::byte>(guard_value));
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

unsigned int parse_probe_limit(std::string_view text) {
  unsigned int result{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  constexpr unsigned int maximum_instructions = 3000000;
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || result == 0 ||
      result > maximum_instructions) {
    throw std::invalid_argument("Runtime probe limit must be an integer in 1..3000000");
  }
  return result;
}

void print_probe_state(const lead_a1::ArmState &state) {
  for (std::size_t index = 0; index < state.registers.size(); ++index) {
    std::cout << " r" << index << '=';
    if (const auto value = state.registers[index]) {
      std::cout << "0x" << std::hex << *value << std::dec;
    } else {
      std::cout << "unknown";
    }
  }
  std::cout << " cpsr=0x" << std::hex << state.cpsr << std::dec << '\n' << std::flush;
}

void check_available_vectors(const lead_a1::Memory &memory) {
  constexpr std::array<std::uint32_t, 8> targets{0xffff0020,
                                                 0xffff0128,
                                                 0xffff002c,
                                                 0xffff0130,
                                                 0xffff012c,
                                                 0xffff0014,
                                                 0xffff008c,
                                                 0xffff0060};
  constexpr unsigned int word_bytes = 4;
  for (std::uint32_t offset = 0; offset < targets.size() * word_bytes; ++offset) {
    if (!memory.peek(local_base + offset)) {
      std::cout << "High-vector table not fully initialized at this checkpoint\n";
      return;
    }
  }
  for (std::size_t index = 0; index < targets.size(); ++index) {
    const auto pc = local_base + static_cast<std::uint32_t>(index * word_bytes);
    const auto word = static_cast<std::uint32_t>(memory.read({.address = pc, .size = word_bytes}));
    constexpr std::uint32_t branch_opcode = 0xea000000;
    constexpr std::uint32_t opcode_mask = 0xff000000;
    require((word & opcode_mask) == branch_opcode);
    constexpr std::uint32_t immediate_mask = 0x00ffffff;
    const auto encoded = (word & immediate_mask) << 2;
    constexpr std::uint32_t negative_displacement = 0x02000000;
    constexpr std::uint32_t displacement_range = 0x04000000;
    const auto displacement = (encoded & negative_displacement) == 0
                                  ? static_cast<std::int64_t>(encoded)
                                  : static_cast<std::int64_t>(encoded) - displacement_range;
    const auto destination =
        static_cast<std::uint64_t>(static_cast<std::int64_t>(pc) + 8 + displacement);
    require((destination & std::numeric_limits<std::uint32_t>::max()) == targets[index]);
  }
  std::cout << "All eight original high vectors match independent branch targets\n";
}

void probe_runtime(lead_a1::Arm &cpu,
                   const lead_a1::Memory &memory,
                   unsigned int instruction_limit) {
  for (unsigned int index = 0; index < instruction_limit; ++index) {
    const auto before = cpu.state();
    const auto system = cpu.system_state();
    try {
      const auto step = cpu.step();
      std::cout << "Runtime probe pc=0x" << std::hex << step.before << ": " << step.assembly
                << std::dec << '\n';
    } catch (const std::runtime_error &error) {
      require(cpu.state() == before && cpu.system_state() == system);
      std::cout << "Runtime probe stopped after " << index
                << " committed instructions: " << error.what() << '\n';
      print_probe_state(before);
      check_available_vectors(memory);
      return;
    }
  }
  std::cout << "Runtime probe instruction budget reached; no boot claim\n" << std::flush;
  print_probe_state(cpu.state());
  check_available_vectors(memory);
}

void check_saved_frame(const lead_a1::Memory &memory,
                       const lead_a1::ArmState &state,
                       std::uint32_t stack_top) {
  constexpr std::uint32_t frame_bytes = 16;
  constexpr unsigned int sp_index = 13;
  const auto frame_base = stack_top - frame_bytes;
  require(state.registers[sp_index] == frame_base);
  for (std::size_t index = 0; index < saved_register_values.size(); ++index) {
    require(memory.read({.address = frame_base + static_cast<std::uint32_t>(index * 4),
                         .size = 4}) == saved_register_values[index]);
  }
  constexpr std::uint32_t hook_return = code_base + 0xa4;
  require(memory.read({.address = frame_base + 12, .size = 4}) == hook_return);
}

void configured_startup(const std::filesystem::path &spec,
                        unsigned int part,
                        const std::filesystem::path &directory,
                        const RuntimeProbe &probe) {
  lead_a1::Memory memory;
  const auto code = map_reference_ram(memory, part, directory);
  lead_a1::ArmState initial;
  initial.registers[pc_index] = entry;
  initial.cpsr = reference_cpsr;
  // Explicit reference inputs, not a claim about the physical software-entry state.
  constexpr std::uint32_t reference_control = 0x50078;
  constexpr std::size_t reference_icache_bytes = std::size_t{16} * 1024;
  constexpr std::uint32_t all_ways_available = 0xfff0;
  constexpr std::uint32_t way_zero_available = 0xfffe;
  const lead_a1::ArmSystemInitial system{
      .control = reference_control,
      .instruction_cache_bytes = reference_icache_bytes,
      .instruction_cache_invalid = false,
      .data_cache_invalid = false,
      .instruction_lockdown = probe.single_cache_way ? way_zero_available : all_ways_available,
      .data_lockdown = all_ways_available,
      .write_buffer = std::vector<lead_a1::ArmBufferedWrite>{}};
  std::cout << "Explicit reference I-cache lockdown=0x" << std::hex << *system.instruction_lockdown
            << std::dec << '\n';
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
  const auto before = cpu.state();
  const auto system_before = cpu.system_state();
  bool stopped = false;
  try {
    static_cast<void>(cpu.step());
  } catch (const std::runtime_error &error) {
    require(std::string(error.what()).find("uninitialized") != std::string::npos);
    stopped = true;
  }
  require(stopped && cpu.state() == before && cpu.system_state() == system_before);
  initial.registers[3] = saved_register_values[0];
  initial.registers[7] = saved_register_values[1];
  initial.registers[9] = preserved_r9;
  initial.registers[10] = preserved_r10;
  initial.registers[11] = saved_register_values[2];
  lead_a1::Arm concrete(memory, spec, initial, system);
  const auto total = run_until(concrete, runtime_checkpoint);
  require(total == initialization_steps[part]);
  check_initialized_memory(memory, part, code);
  constexpr std::uint32_t completed_region_flags = 0x60000000;
  require(concrete.state().cpsr == (system_cpsr | completed_region_flags));
  require(concrete.state().registers[6] == 1 && concrete.state().registers[7] == 1);
  require(concrete.state().registers[9] == preserved_r9);
  require(concrete.state().registers[10] == preserved_r10);
  check_saved_frame(memory, concrete.state(), stack_tops[part][0]);
  std::cout << "Part " << part << ": original BSS clearing preserves CODE and tail guard; " << total
            << " instructions reach CODE+0x298\n"
            << std::flush;
  if (probe.limit != 0) { probe_runtime(concrete, memory, probe.limit); }
}

}

int main(int argc, char **argv) {
  if (argc < 4 || argc > 7) { return 1; }
  const std::string selection = argv[3];
  if (selection != "0" && selection != "1") { return 1; }
  const auto part = selection == "0" ? 0U : 1U;
  const bool probe = argc >= 5;
  if (probe && std::string(argv[4]) != "--probe-runtime") { return 1; }
  try {
    constexpr unsigned int default_probe_limit = 4096;
    RuntimeProbe options{.limit = 0, .single_cache_way = argc == 7};
    if (probe) { options.limit = argc >= 6 ? parse_probe_limit(argv[5]) : default_probe_limit; }
    if (options.single_cache_way && std::string(argv[6]) != "--single-cache-way") { return 1; }
    run_part(argv[1], part, argv[2]);
    configured_startup(argv[1], part, argv[2], options);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
