// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm.hpp"
#include "arm/core/arm_mmu.hpp"
#include "arm/core/arm_system.hpp"
#include "common/memory.hpp"

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

constexpr std::uint32_t virtual_code = 0x80000000;
constexpr std::uint32_t physical_code = 0x20000000;
constexpr std::uint32_t clone_code = 0x30000000;
constexpr std::uint32_t identity_code = 0x1000;
constexpr std::uint32_t virtual_data = 0x90000000;
constexpr std::uint32_t physical_data = 0x40000000;
constexpr std::uint32_t alternate_data = 0x50000000;
constexpr std::uint32_t table_base = 0x10000;
constexpr std::uint32_t coarse_base = 0x18000;
constexpr std::uint32_t code_descriptor_address = 0x12000;
constexpr std::uint32_t data_descriptor_address = 0x12400;
constexpr std::uint32_t uncached_section = 0xc12;
constexpr std::uint32_t cached_section = 0xc1a;
constexpr std::uint32_t marker = 0x11223344;
constexpr std::uint32_t alternate_marker = 0xaabbccdd;
constexpr std::uint32_t control_base = 0x50078;
constexpr std::uint32_t control_mmu = control_base | 1;
constexpr std::uint32_t control_cached = control_mmu | 0x1000;
constexpr std::uint32_t cpsr = 0xd3;
constexpr unsigned int pc_index = 15;
constexpr std::uint32_t case_bytes = 256;

void require(bool condition,
             const std::source_location &location = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Independent MMU CPU expectation failed at line " +
                             std::to_string(location.line()));
  }
}

std::vector<std::byte> read_program(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Cannot read owned MMU test program"); }
  const auto data = std::as_bytes(std::span(bytes));
  return {data.begin(), data.end()};
}

void map_fixture(lead_a1::Memory &memory,
                 std::span<const std::byte> program,
                 std::uint32_t code_address) {
  const std::vector<std::byte> zero(0x4000);
  memory.map({.address = table_base, .size = zero.size(), .name = "owned page directory"}, zero);
  memory.map({.address = coarse_base, .size = 0x400, .name = "owned page table"},
             std::span(zero).first(0x400));
  memory.map({.address = code_address, .size = program.size(), .name = "owned physical code"},
             program);
  memory.map({.address = clone_code, .size = program.size(), .name = "owned alternate code"},
             program);
  memory.map({.address = physical_data, .size = 0x1000, .name = "owned RAM A"});
  memory.map({.address = alternate_data, .size = 0x1000, .name = "owned RAM B"});
  memory.write({.address = code_descriptor_address, .size = 4}, physical_code | cached_section);
  memory.write({.address = data_descriptor_address, .size = 4}, physical_data | uncached_section);
  memory.write({.address = physical_data, .size = 4}, marker);
  memory.write({.address = alternate_data, .size = 4}, alternate_marker);
}

lead_a1::ArmSystemInitial profile() {
  constexpr std::uint32_t single_way = 0xfffe;
  constexpr std::uint32_t unlocked = 0xfff0;
  return {.control = control_mmu,
          .instruction_cache_bytes = 128,
          .data_cache_bytes = std::nullopt,
          .instruction_cache_invalid = true,
          .data_cache_invalid = true,
          .instruction_lockdown = single_way,
          .data_lockdown = unlocked,
          .write_buffer = std::vector<lead_a1::ArmBufferedWrite>{},
          .writeback_buffer_empty = true,
          .mmu = lead_a1::ArmMmuInitial{.translation_table = table_base,
                                        .domain_access = 3,
                                        .fcse_pid = 0,
                                        .context_id = std::nullopt,
                                        .tlb_lockdown = 0,
                                        .tlb_invalid = true,
                                        .locked_tlb_invalid = true}};
}

lead_a1::ArmState initial(std::uint32_t pc) {
  lead_a1::ArmState state;
  state.registers[pc_index] = pc;
  state.cpsr = cpsr;
  return state;
}

void steps(lead_a1::Arm &cpu, unsigned int count) {
  for (unsigned int index = 0; index < count; ++index) { static_cast<void>(cpu.step()); }
}

void rejected(lead_a1::Arm &cpu, const std::string &reason) {
  const auto state = cpu.state();
  const auto system = cpu.system_state();
  try {
    steps(cpu, 1);
  } catch (const std::runtime_error &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    require(cpu.state() == state && cpu.system_state() == system);
    return;
  }
  throw std::runtime_error("Expected MMU CPU rejection did not occur");
}

void control_and_virtual_ram(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_fixture(memory, bytes, identity_code);
  memory.write({.address = table_base, .size = 4}, uncached_section);
  auto state = initial(identity_code);
  state.registers[0] = table_base;
  state.registers[1] = 3;
  state.registers[2] = 0;
  state.registers[5] = control_mmu;
  state.registers[7] = virtual_data;
  state.registers[8] = virtual_data + 4;
  auto system = profile();
  system.control = control_base;
  system.mmu = lead_a1::ArmMmuInitial{.translation_table = {},
                                      .domain_access = {},
                                      .fcse_pid = {},
                                      .context_id = {},
                                      .tlb_lockdown = {},
                                      .tlb_invalid = false,
                                      .locked_tlb_invalid = true};
  lead_a1::Arm cpu(memory, spec, state, system);
  steps(cpu, 12);
  require(cpu.state().registers[3] == table_base && cpu.state().registers[4] == 3);
  require(cpu.state().registers[6] == marker);
  require(memory.read({.address = physical_data + 4, .size = 4}) == marker);
  require(!memory.peek(virtual_data));
  require(cpu.system_state().control == control_mmu);
  require(cpu.system_state().mmu.entries == 2);
}

void data_tlb_and_instruction_cache(const std::filesystem::path &spec,
                                    std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_fixture(memory, bytes, physical_code);
  auto state = initial(virtual_code + case_bytes);
  state.registers[1] = virtual_data;
  state.registers[2] = virtual_data;
  state.registers[9] = 0;
  auto system = profile();
  system.control = control_cached;
  lead_a1::Arm cpu(memory, spec, state, system);
  steps(cpu, 1);
  require(cpu.state().registers[0] == marker);
  memory.write({.address = data_descriptor_address, .size = 4}, alternate_data | uncached_section);
  constexpr std::uint32_t mov_r4_nine = 0xe3a04009;
  memory.write({.address = physical_code + case_bytes + 16, .size = 4}, mov_r4_nine);
  steps(cpu, 2);
  require(cpu.state().registers[3] == alternate_marker);
  steps(cpu, 2);
  require(cpu.state().registers[4] == 9);
}

void virtual_instruction_tags(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_fixture(memory, bytes, physical_code);
  constexpr auto offset = 2 * case_bytes;
  auto state = initial(virtual_code + offset);
  state.registers[2] = virtual_code;
  state.registers[9] = 0;
  auto system = profile();
  system.control = control_cached;
  lead_a1::Arm cpu(memory, spec, state, system);
  steps(cpu, 1);
  require(!cpu.state().registers[0]);
  constexpr std::uint32_t mov_r0_nine = 0xe3a00009;
  constexpr std::uint32_t mov_r1_seven = 0xe3a01007;
  memory.write({.address = clone_code + offset + 8, .size = 4}, mov_r0_nine);
  memory.write({.address = clone_code + offset + 16, .size = 4}, mov_r1_seven);
  memory.write({.address = code_descriptor_address, .size = 4}, clone_code | cached_section);
  steps(cpu, 2);
  require(cpu.state().registers[0] == 1);
  steps(cpu, 2);
  require(cpu.state().registers[1] == 7);
}

void atomic_page_faults(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_fixture(memory, bytes, physical_code);
  constexpr std::uint32_t coarse_descriptor = coarse_base | 0x11;
  constexpr std::uint32_t first_subpage_only = physical_data | 0x32;
  memory.write({.address = data_descriptor_address, .size = 4}, coarse_descriptor);
  memory.write({.address = coarse_base, .size = 4}, first_subpage_only);
  constexpr std::uint32_t last_word = 0x3fc;
  memory.write({.address = physical_data + last_word, .size = 4}, 7);
  for (const auto index : {3U, 4U}) {
    auto state = initial(virtual_code + (index * case_bytes));
    state.registers[0] = virtual_data + last_word;
    state.registers[1] = marker;
    state.registers[2] = alternate_marker;
    auto system = profile();
    if (!system.mmu) { throw std::runtime_error("Missing MMU fixture state"); }
    system.mmu->domain_access = 1;
    lead_a1::Arm cpu(memory, spec, state, system, {.deliver_mmu_aborts = false});
    rejected(cpu, "status=0xf");
    require(memory.read({.address = physical_data + last_word, .size = 4}) == 7);
    require(cpu.system_state().mmu.entries == 0);
  }
}

void unsupported_transitions(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_fixture(memory, bytes, identity_code);
  memory.write({.address = table_base, .size = 4}, physical_code | uncached_section);
  auto state = initial(identity_code + (5 * case_bytes));
  state.registers[0] = control_mmu;
  auto system = profile();
  system.control = control_base;
  lead_a1::Arm cpu(memory, spec, state, system);
  rejected(cpu, "nonidentity ARM MMU");
  constexpr std::uint32_t data_cache_bit = 4;
  memory.write({.address = table_base, .size = 4}, uncached_section);
  state.registers[0] = control_mmu | data_cache_bit;
  lead_a1::Arm cached_data(memory, spec, state, system);
  rejected(cached_data, "data cache geometry");
}

void privilege_and_alignment(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_fixture(memory, bytes, physical_code);
  constexpr std::uint32_t privileged_section = 0x412;
  memory.write({.address = data_descriptor_address, .size = 4}, physical_data | privileged_section);
  for (const auto index : {6U, 7U}) {
    auto state = initial(virtual_code + (index * case_bytes));
    state.registers[0] = virtual_data;
    state.registers[1] = alternate_marker;
    auto system = profile();
    if (!system.mmu) { throw std::runtime_error("Missing MMU fixture state"); }
    system.mmu->domain_access = 1;
    lead_a1::Arm cpu(memory, spec, state, system, {.deliver_mmu_aborts = false});
    rejected(cpu, "status=0xd");
    require(memory.read({.address = physical_data, .size = 4}) == marker);
  }
  auto state = initial(virtual_code + case_bytes);
  state.registers[1] = virtual_data;
  constexpr std::uint32_t user_cpsr = 0xd0;
  state.cpsr = user_cpsr;
  auto system = profile();
  if (!system.mmu) { throw std::runtime_error("Missing MMU fixture state"); }
  system.mmu->domain_access = 1;
  lead_a1::Arm user(memory, spec, state, system, {.deliver_mmu_aborts = false});
  rejected(user, "status=0xd");
  state.cpsr = cpsr;
  state.registers[1] = virtual_data + 1;
  constexpr std::uint32_t alignment_bit = 2;
  system.control = control_mmu | alignment_bit;
  lead_a1::Arm unaligned(memory, spec, state, system, {.deliver_mmu_aborts = false});
  rejected(unaligned, "status=0x1");
}

lead_a1::ArmSystemInitial data_profile() {
  auto system = profile();
  constexpr std::uint32_t data_cache_bit = 4;
  constexpr std::uint32_t one_way = 0xfffe;
  system.control = control_mmu | data_cache_bit;
  system.data_cache_bytes = 128;
  system.data_lockdown = one_way;
  return system;
}

void data_writeback(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_fixture(memory, bytes, physical_code);
  constexpr std::uint32_t writeback_section = 0xc1e;
  memory.write({.address = data_descriptor_address, .size = 4}, physical_data | writeback_section);
  for (unsigned int offset = 4; offset < 32; offset += 4) {
    memory.write({.address = physical_data + offset, .size = 4}, 0);
  }
  auto state = initial(virtual_code + (8 * case_bytes));
  state.registers[1] = virtual_data;
  state.registers[2] = alternate_marker;
  state.registers[9] = 0;
  lead_a1::Arm cpu(memory, spec, state, data_profile());
  steps(cpu, 3);
  require(cpu.state().registers[0] == marker && cpu.state().registers[3] == alternate_marker);
  require(cpu.system_state().data.dirty_halves == 1);
  require(memory.read({.address = physical_data, .size = 4}) == marker);
  steps(cpu, 1);
  require(cpu.system_state().data.writeback_words == 4);
  require(memory.read({.address = physical_data, .size = 4}) == marker);
  steps(cpu, 1);
  require(memory.read({.address = physical_data, .size = 4}) == alternate_marker);
  steps(cpu, 1);
  require(cpu.system_state().data_cache_invalid && cpu.system_state().data.cached_lines == 0);
  steps(cpu, 1);
  require(cpu.state().registers[4] == alternate_marker && cpu.state().cpsr == cpsr);
}

void cached_store_rollback(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_fixture(memory, bytes, physical_code);
  constexpr std::uint32_t coarse_descriptor = coarse_base | 0x11;
  constexpr std::uint32_t first_subpage_only = physical_data | 0x3e;
  constexpr std::uint32_t last_word = 0x3fc;
  constexpr std::uint32_t last_line = 0x3e0;
  memory.write({.address = data_descriptor_address, .size = 4}, coarse_descriptor);
  memory.write({.address = coarse_base, .size = 4}, first_subpage_only);
  for (std::uint32_t offset = last_line; offset <= last_word; offset += 4) {
    memory.write({.address = physical_data + offset, .size = 4}, 7);
  }
  constexpr auto code_offset = 9 * case_bytes;
  auto state = initial(virtual_code + code_offset);
  state.registers[0] = virtual_data + last_word;
  state.registers[1] = marker;
  state.registers[2] = alternate_marker;
  auto system = data_profile();
  if (!system.mmu) { throw std::runtime_error("Missing MMU fixture state"); }
  system.mmu->domain_access = 1;
  lead_a1::Arm cpu(memory, spec, state, system, {.deliver_mmu_aborts = false});
  steps(cpu, 1);
  require(cpu.state().registers[3] == 7 && cpu.system_state().data.dirty_halves == 0);
  rejected(cpu, "status=0xf");
  require(memory.read({.address = physical_data + last_word, .size = 4}) == 7);
  const auto load = memory.read({.address = physical_code + code_offset, .size = 4});
  memory.write({.address = physical_code + code_offset + 4, .size = 4}, load);
  steps(cpu, 1);
  require(cpu.state().registers[3] == 7 && cpu.system_state().data.dirty_halves == 0);
}

void data_disable_and_buffered_tables(const std::filesystem::path &spec,
                                      std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_fixture(memory, bytes, physical_code);
  memory.write({.address = data_descriptor_address, .size = 4}, physical_data | cached_section);
  for (unsigned int offset = 4; offset < 32; offset += 4) {
    memory.write({.address = physical_data + offset, .size = 4}, 0);
  }
  auto state = initial(virtual_code + (10 * case_bytes));
  state.registers[1] = virtual_data;
  state.registers[5] = control_mmu;
  state.registers[6] = data_profile().control;
  lead_a1::Arm cpu(memory, spec, state, data_profile());
  steps(cpu, 1);
  memory.write({.address = physical_data, .size = 4}, alternate_marker);
  steps(cpu, 4);
  require(cpu.state().registers[0] == marker && cpu.state().registers[2] == alternate_marker);
  require(cpu.state().registers[3] == marker);
  constexpr std::uint32_t uncached_buffered = 0xc16;
  constexpr std::uint32_t target_descriptor = 0x12800;
  constexpr std::uint32_t target_virtual = 0xa0000000;
  memory.write({.address = table_base, .size = 4}, uncached_buffered);
  state = initial(virtual_code + (11 * case_bytes));
  state.registers[1] = target_descriptor;
  state.registers[2] = physical_data | uncached_section;
  state.registers[4] = target_virtual;
  lead_a1::Arm buffered(memory, spec, state, data_profile());
  steps(buffered, 1);
  require(buffered.system_state().buffered_writes == 1);
  require(memory.read({.address = target_descriptor, .size = 4}) == 0);
  steps(buffered, 1);
  require(buffered.state().registers[3] == alternate_marker);
  require(buffered.system_state().buffered_writes == 0);
  require(memory.read({.address = target_descriptor, .size = 4}) ==
          (physical_data | uncached_section));
  memory.write({.address = target_descriptor, .size = 4}, 0);
  state.registers[2] = physical_data;
  state.registers[9] = 0;
  lead_a1::Arm invalid_table(memory, spec, state, data_profile(), {.deliver_mmu_aborts = false});
  steps(invalid_table, 1);
  rejected(invalid_table, "status=0x5");
  require(invalid_table.system_state().buffered_writes == 1);
  require(memory.read({.address = target_descriptor, .size = 4}) == 0);
  constexpr auto drain_offset = (8 * case_bytes) + 16;
  const auto drain = memory.read({.address = physical_code + drain_offset, .size = 4});
  memory.write({.address = physical_code + (11 * case_bytes) + 4, .size = 4}, drain);
  steps(invalid_table, 1);
  require(invalid_table.system_state().buffered_writes == 0);
  require(memory.read({.address = target_descriptor, .size = 4}) == physical_data);
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    const auto bytes = read_program(argv[2]);
    control_and_virtual_ram(argv[1], bytes);
    data_tlb_and_instruction_cache(argv[1], bytes);
    virtual_instruction_tags(argv[1], bytes);
    atomic_page_faults(argv[1], bytes);
    unsupported_transitions(argv[1], bytes);
    privilege_and_alignment(argv[1], bytes);
    data_writeback(argv[1], bytes);
    cached_store_rollback(argv[1], bytes);
    data_disable_and_buffered_tables(argv[1], bytes);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
