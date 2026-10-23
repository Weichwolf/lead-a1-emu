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

constexpr std::uint32_t code = 0x80000000;
constexpr std::uint32_t data = 0x90000000;
constexpr std::uint32_t physical = 0x40000000;
constexpr std::uint32_t directory = 0x10000;
constexpr std::uint32_t coarse = 0x18000;
constexpr std::uint32_t code_descriptor = 0x12000;
constexpr std::uint32_t data_descriptor = 0x12400;
constexpr std::uint32_t section = 0xc12;
constexpr std::uint32_t denied = 0x12;
constexpr std::uint32_t high_vectors = 0xffff0000;
constexpr std::uint32_t old_far = 0x12345678;
constexpr std::uint32_t old_r1 = 0x11223344;
constexpr std::uint32_t old_r2 = 0x55667788;
constexpr std::uint32_t marker = 0xaabbccdd;
constexpr std::uint32_t user_status = 0xb8000010;
constexpr std::uint32_t abort_status = 0xb8000097;
constexpr std::uint32_t supervisor_status = 0xb80000d3;
constexpr unsigned int spacing = 256;
constexpr std::uint32_t user_stack = 0x5000;
constexpr std::uint32_t user_link = 0x6000;
constexpr std::uint32_t abort_stack = 0x7000;
constexpr std::uint32_t prefetch_slot = 44;
constexpr std::uint32_t data_slot = 48;
constexpr std::uint32_t high_section = 0xfff00000;
constexpr std::uint32_t mode_mask = 0x1f;
constexpr std::uint32_t abort_control = 0x97;
constexpr std::uint32_t repaired_base = 0x104;
constexpr unsigned int data_handler = 19;
constexpr unsigned int prefetch_handler = 20;
constexpr unsigned int skipped_multiple_case = 23;
constexpr unsigned int exception_return_case = 24;
constexpr unsigned int skipped_register_case = 25;
constexpr unsigned int flags_register_case = 26;
constexpr unsigned int reserved_write_case = 17;
constexpr unsigned int invalid_selector_case = 18;
constexpr std::uint32_t repeated_abort_link = 20;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Independent abort expectation at line " +
                             std::to_string(where.line()));
  }
}

std::vector<std::byte> read_program(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Missing owned abort program"); }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

struct Fixture {
  lead_a1::Memory memory;
  lead_a1::ArmState state;
  lead_a1::ArmSystemInitial system;
  std::uint32_t vectors{};

  Fixture(std::span<const std::byte> program, unsigned int index, bool high = false) {
    vectors = high ? high_vectors : 0;
    const std::vector<std::byte> zeros(0x4000);
    memory.map({.address = directory, .size = zeros.size(), .name = "owned directory"}, zeros);
    memory.map({.address = coarse, .size = 0x400, .name = "owned coarse table"},
               std::span(zeros).first(0x400));
    memory.map({.address = code, .size = program.size(), .name = "owned abort program"}, program);
    memory.map({.address = physical, .size = 0x1000, .name = "owned data"});
    constexpr unsigned int vector_bytes = 64;
    memory.map({.address = vectors, .size = vector_bytes, .name = "owned vectors"},
               std::span(zeros).first(vector_bytes));
    for (unsigned int byte = 0; byte < 32; ++byte) {
      memory.write({.address = vectors + byte, .size = 1},
                   std::to_integer<unsigned int>(program[byte]));
    }
    memory.write({.address = vectors + prefetch_slot, .size = 4},
                 code + (prefetch_handler * spacing));
    memory.write({.address = vectors + data_slot, .size = 4}, code + (data_handler * spacing));
    memory.write({.address = directory, .size = 4}, section);
    memory.write({.address = code_descriptor, .size = 4}, code | section);
    constexpr std::uint32_t high_descriptor = 0x13ffc;
    memory.write({.address = high_descriptor, .size = 4}, high_section | section);
    memory.write({.address = data_descriptor, .size = 4}, physical | section);
    memory.write({.address = physical, .size = 4}, marker);
    state.cpsr = user_status;
    state.registers[0] = data;
    state.registers[1] = old_r1;
    state.registers[2] = old_r2;
    state.registers[3] = physical | section;
    state.registers[4] = data_descriptor;
    state.registers[5] = data;
    state.registers[13] = user_stack;
    state.registers[14] = user_link;
    state.registers[15] = code + (index * spacing);
    state.banks.modes[4].sp = abort_stack;
    constexpr std::uint32_t control = 0x50079;
    system.control = control | (high ? 0x2000 : 0);
    system.instruction_cache_invalid = true;
    system.data_cache_invalid = true;
    system.write_buffer = std::vector<lead_a1::ArmBufferedWrite>{};
    system.writeback_buffer_empty = true;
    system.mmu = lead_a1::ArmMmuInitial{.translation_table = directory,
                                        .domain_access = 1,
                                        .fcse_pid = 0,
                                        .context_id = {},
                                        .tlb_lockdown = 0,
                                        .tlb_invalid = true,
                                        .locked_tlb_invalid = true};
    system.faults = {.data = lead_a1::ArmFaultStatus{.status = 6, .domain = 7},
                     .instruction = lead_a1::ArmFaultStatus{.status = 8, .domain = 9},
                     .address = old_far};
  }
};

void steps(lead_a1::Arm &cpu, unsigned int count) {
  for (unsigned int index = 0; index < count; ++index) { require(!cpu.step().exception); }
}

void rejected(lead_a1::Arm &cpu, const std::string &reason) {
  const auto before = cpu.state();
  const auto system = cpu.system_state();
  try {
    static_cast<void>(cpu.step());
  } catch (const std::runtime_error &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    require(cpu.state() == before && cpu.system_state() == system);
    return;
  }
  throw std::runtime_error("Expected abort-reference diagnostic");
}

void data_entry(lead_a1::Arm &cpu,
                const Fixture &fixture,
                std::uint8_t status,
                std::optional<std::uint8_t> domain,
                std::uint32_t address) {
  const auto before = cpu.state();
  const auto event = cpu.step();
  require(event.exception == lead_a1::ArmException::data_abort &&
          event.after == fixture.vectors + 16);
  require(cpu.state().cpsr == ((before.cpsr & ~mode_mask) | abort_control));
  const auto fault_pc = before.registers[15];
  if (!fault_pc) { throw std::logic_error("Missing fault PC"); }
  require(cpu.state().registers[14] == *fault_pc + 8);
  require(cpu.state().registers[13] == abort_stack);
  require(cpu.state().banks.modes[4].saved.bits == before.cpsr);
  const auto faults = cpu.system_state().faults;
  require(faults.data == (lead_a1::ArmFaultStatus{.status = status, .domain = domain}));
  require(faults.address == address && faults.instruction == fixture.system.faults.instruction);
}

void scalar_permission(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr std::array<unsigned int, 6> cases{1, 2, 9, 10, 11, 12};
  for (const bool high : {false, true}) {
    for (const auto index : cases) {
      Fixture fixture(bytes, index, high);
      fixture.memory.write({.address = data_descriptor, .size = 4}, physical | denied);
      lead_a1::Arm cpu(fixture.memory, spec, fixture.state, fixture.system);
      constexpr std::uint8_t permission_status = 13;
      data_entry(cpu, fixture, permission_status, 0, data + (index == 2 ? 4 : 0));
      require(cpu.state().registers[0] == data && cpu.state().registers[1] == old_r1);
      require(cpu.state().registers[2] == old_r2);
      require(fixture.memory.read({.address = physical, .size = 4}) == marker);
    }
  }
}

void user_and_swap(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  for (const unsigned int index : {6U, 7U, 8U}) {
    Fixture fixture(bytes, index);
    constexpr std::uint32_t user_read_only = 0x812;
    constexpr std::uint32_t supervisor_only = 0x412;
    fixture.memory.write({.address = data_descriptor, .size = 4},
                         physical | (index == 6 ? user_read_only : supervisor_only));
    if (index != 6) { fixture.state.cpsr = supervisor_status; }
    lead_a1::Arm cpu(fixture.memory, spec, fixture.state, fixture.system);
    constexpr std::uint8_t permission_status = 13;
    data_entry(cpu, fixture, permission_status, 0, data);
    require(cpu.state().registers[0] == data && cpu.state().registers[1] == old_r1);
    require(fixture.memory.read({.address = physical, .size = 4}) == marker);
  }
}

void partial_multiple(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr std::uint32_t last_word = 0x3fc;
  constexpr std::uint32_t second_word = 0x400;
  constexpr std::uint32_t coarse_descriptor = coarse | 0x11;
  constexpr std::uint32_t first_only = physical | 0x32;
  for (const unsigned int index : {3U, 4U, 5U, 24U}) {
    Fixture fixture(bytes, index);
    fixture.memory.write({.address = data_descriptor, .size = 4}, coarse_descriptor);
    fixture.memory.write({.address = coarse, .size = 4}, first_only);
    fixture.memory.write({.address = physical + last_word, .size = 4}, marker);
    fixture.memory.write({.address = physical + second_word, .size = 4}, 7);
    fixture.state.registers[0] = data + last_word;
    if (index == exception_return_case) {
      constexpr std::uint32_t irq_status = 0xb80000d2;
      constexpr std::uint32_t status_mask = 0xf90000ff;
      fixture.state.cpsr = irq_status;
      fixture.state.banks.modes[2].saved = {.bits = user_status, .known_mask = status_mask};
    }
    lead_a1::Arm cpu(fixture.memory, spec, fixture.state, fixture.system);
    constexpr std::uint8_t permission_page = 15;
    data_entry(cpu, fixture, permission_page, 0, data + second_word);
    require(cpu.state().registers[0] == data + last_word);
    if (index == 3 || index == exception_return_case) {
      require(cpu.state().registers[1] == marker);
    }
    if (index == 5) { require(cpu.state().registers[1] == old_r1); }
    const auto expected = index == 4 ? old_r1 : marker;
    require(fixture.memory.read({.address = physical + last_word, .size = 4}) == expected);
    require(fixture.memory.read({.address = physical + second_word, .size = 4}) == 7);
  }
}

struct FaultCase {
  std::uint32_t first;
  std::uint32_t second;
  std::uint32_t domains;
  std::uint8_t status;
  std::optional<std::uint8_t> domain;
  std::optional<std::uint32_t> readback;
};

void status_matrix(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr std::array<FaultCase, 6> cases{
      {{.first = 0, .second = 0, .domains = 3, .status = 5, .domain = {}, .readback = {}},
       {.first = coarse | 0x11, .second = 0, .domains = 3, .status = 7, .domain = 0, .readback = 7},
       {.first = physical | 0xc32,
        .second = 0,
        .domains = 3,
        .status = 9,
        .domain = 1,
        .readback = 0x19},
       {.first = coarse | 0x31,
        .second = physical | 0xff2,
        .domains = 3,
        .status = 11,
        .domain = 1,
        .readback = 0x1b},
       {.first = physical | 0x32,
        .second = 0,
        .domains = 7,
        .status = 13,
        .domain = 1,
        .readback = 0x1d},
       {.first = coarse | 0x31,
        .second = physical | 2,
        .domains = 7,
        .status = 15,
        .domain = 1,
        .readback = 0x1f}}};
  for (const auto &entry : cases) {
    Fixture fixture(bytes, 1);
    if (!fixture.system.mmu) { throw std::logic_error("Missing fixture MMU"); }
    fixture.system.mmu->domain_access = entry.domains;
    fixture.memory.write({.address = data_descriptor, .size = 4}, entry.first);
    fixture.memory.write({.address = coarse, .size = 4}, entry.second);
    lead_a1::Arm cpu(fixture.memory, spec, fixture.state, fixture.system);
    data_entry(cpu, fixture, entry.status, entry.domain, data);
    auto saved = cpu.state();
    saved.registers[15] = code + (14 * spacing);
    fixture.system.faults = cpu.system_state().faults;
    lead_a1::Arm inspect(fixture.memory, spec, saved, fixture.system);
    if (entry.readback) {
      steps(inspect, 1);
      require(inspect.state().registers[0] == entry.readback);
    } else {
      rejected(inspect, "Unknown ARM fault domain");
    }
  }
}

void guest_data_repair(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  Fixture fixture(bytes, 1, true);
  fixture.memory.write({.address = data_descriptor, .size = 4}, 0);
  lead_a1::Arm cpu(fixture.memory, spec, fixture.state, fixture.system);
  data_entry(cpu, fixture, 5, {}, data);
  steps(cpu, 5);
  require(cpu.state().cpsr == user_status && cpu.state().registers[15] == code + spacing);
  steps(cpu, 1);
  require(cpu.state().registers[1] == marker && cpu.state().registers[0] == data + 4);
  require(cpu.state().registers[6] == data);
  require(fixture.memory.read({.address = data_descriptor, .size = 4}) == (physical | section));
}

void guest_prefetch_repair(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr std::uint32_t missing_code = 0xa0000000;
  constexpr std::uint32_t target_descriptor = 0x12800;
  constexpr std::uint32_t repaired_physical = 0x60000000;
  constexpr unsigned int resumed_case = 22;
  for (const bool permission : {false, true}) {
    Fixture fixture(bytes, 1, true);
    fixture.memory.map({.address = repaired_physical, .size = 4, .name = "owned repaired code"},
                       bytes.subspan(static_cast<std::size_t>(resumed_case) * spacing, 4));
    fixture.state.registers[15] = missing_code;
    fixture.state.registers[3] = repaired_physical | section;
    fixture.state.registers[4] = target_descriptor;
    fixture.state.registers[5] = missing_code;
    fixture.memory.write({.address = target_descriptor, .size = 4},
                         permission ? repaired_physical | denied : 0);
    lead_a1::Arm cpu(fixture.memory, spec, fixture.state, fixture.system);
    require(cpu.step().exception == lead_a1::ArmException::prefetch_abort);
    require(cpu.state().registers[15] == high_vectors + 12 &&
            cpu.state().registers[14] == missing_code + 4);
    require(cpu.state().cpsr == abort_status && cpu.decode_statistics().translations == 0);
    const lead_a1::ArmFaultStatus expected{.status = static_cast<std::uint8_t>(permission ? 13 : 5),
                                           .domain = permission ? std::optional<std::uint8_t>{0}
                                                                : std::nullopt};
    require(cpu.system_state().faults.instruction == expected);
    require(cpu.system_state().faults.address == old_far);
    require(cpu.system_state().faults.data == fixture.system.faults.data);
    steps(cpu, 4);
    require(cpu.state().cpsr == user_status && cpu.state().registers[15] == missing_code);
    steps(cpu, 1);
    require(cpu.state().registers[8] == 9 && cpu.state().registers[15] == missing_code + 4);
  }
}

void alignment_and_unknown(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  Fixture fixture(bytes, 1, true);
  constexpr std::uint32_t pid = 0x02000000;
  constexpr std::uint32_t target_descriptor = 0x10080;
  constexpr std::uint32_t misaligned = 0x103;
  constexpr std::uint32_t modified = 0x02000103;
  if (!fixture.system.control) { throw std::logic_error("Missing fixture control"); }
  fixture.system.control = *fixture.system.control | 2;
  if (!fixture.system.mmu) { throw std::logic_error("Missing fixture MMU"); }
  fixture.system.mmu->fcse_pid = pid;
  fixture.memory.write({.address = target_descriptor, .size = 4}, physical | section);
  fixture.memory.write({.address = physical + 0x100, .size = 4}, marker);
  constexpr unsigned int alignment_handler = 21;
  fixture.memory.write({.address = high_vectors + data_slot, .size = 4},
                       code + (alignment_handler * spacing));
  fixture.state.registers[0] = misaligned;
  lead_a1::Arm cpu(fixture.memory, spec, fixture.state, fixture.system);
  data_entry(cpu, fixture, 1, {}, modified);
  steps(cpu, 3);
  steps(cpu, 1);
  require(cpu.state().registers[0] == repaired_base && cpu.state().registers[1] == marker);
  Fixture unknown(bytes, 1);
  unknown.state.registers[0] = data + 16;
  lead_a1::Arm diagnostic(unknown.memory, spec, unknown.state, unknown.system);
  rejected(diagnostic, "uninitialized RAM");
  Fixture skipped(bytes, skipped_multiple_case);
  skipped.state.registers[0].reset();
  lead_a1::Arm no_access(skipped.memory, spec, skipped.state, skipped.system);
  steps(no_access, 1);
  require(!no_access.state().registers[0] &&
          no_access.system_state().faults == skipped.system.faults);
}

void exception_boundaries(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  Fixture unconfigured_vector(bytes, 1, true);
  constexpr std::uint32_t high_descriptor = 0x13ffc;
  unconfigured_vector.memory.write({.address = data_descriptor, .size = 4}, 0);
  unconfigured_vector.memory.write({.address = high_descriptor, .size = 4}, 0);
  lead_a1::Arm recursive(
      unconfigured_vector.memory, spec, unconfigured_vector.state, unconfigured_vector.system);
  data_entry(recursive, unconfigured_vector, 5, {}, data);
  require(recursive.step().exception == lead_a1::ArmException::prefetch_abort);
  require(recursive.state().registers[15] == high_vectors + 12);
  require(recursive.state().registers[14] == high_vectors + repeated_abort_link);
  require(recursive.state().banks.modes[4].saved.bits == abort_status);
  require(recursive.system_state().faults.address == data);
  Fixture untranslated(bytes, 1);
  constexpr std::uint32_t alignment_without_mmu = 0x5007a;
  untranslated.system.control = alignment_without_mmu;
  untranslated.state.registers[0] = data + 1;
  lead_a1::Arm physical_alignment(
      untranslated.memory, spec, untranslated.state, untranslated.system);
  data_entry(physical_alignment, untranslated, 1, {}, data + 1);
  constexpr unsigned int qualification_case = 27;
  Fixture qualification(bytes, qualification_case);
  constexpr std::uint32_t control_off = 0x50078;
  constexpr std::uint32_t control_on = 0x50079;
  qualification.system.control = control_off;
  qualification.state.cpsr = supervisor_status;
  qualification.state.registers[0] = control_on;
  qualification.memory.write({.address = code_descriptor, .size = 4}, 0);
  lead_a1::Arm future_fetch(qualification.memory, spec, qualification.state, qualification.system);
  rejected(future_fetch, "status=0x5");
  Fixture unknown_tlb(bytes, 1);
  if (!unknown_tlb.system.mmu) { throw std::logic_error("Missing fixture MMU"); }
  unknown_tlb.system.mmu->tlb_invalid = false;
  lead_a1::Arm unresolved(unknown_tlb.memory, spec, unknown_tlb.state, unknown_tlb.system);
  rejected(unresolved, "initial TLB contents");
}

void cache_and_buffer_faults(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr unsigned int cached_case = 28;
  constexpr unsigned int clean_handler = 29;
  constexpr std::uint32_t last_line = 0x3e0;
  constexpr std::uint32_t last_word = 0x3fc;
  constexpr std::uint32_t next_subpage = 0x400;
  constexpr std::uint32_t writeback_control = 0x5007d;
  constexpr std::uint32_t single_way = 0xfffe;
  constexpr std::uint32_t first_page = physical | 0x3e;
  constexpr std::uint32_t coarse_descriptor = coarse | 0x11;
  Fixture fixture(bytes, cached_case);
  fixture.system.control = writeback_control;
  fixture.system.data_cache_bytes = 128;
  fixture.system.data_lockdown = single_way;
  fixture.memory.write({.address = data_descriptor, .size = 4}, coarse_descriptor);
  fixture.memory.write({.address = coarse, .size = 4}, first_page);
  for (auto offset = last_line; offset <= next_subpage; offset += 4) {
    fixture.memory.write({.address = physical + offset, .size = 4}, 7);
  }
  fixture.state.registers[0] = data + last_word;
  fixture.memory.write({.address = data_slot, .size = 4}, code + (clean_handler * spacing));
  lead_a1::Arm cpu(fixture.memory, spec, fixture.state, fixture.system);
  steps(cpu, 1);
  data_entry(cpu, fixture, 15, 0, data + next_subpage);
  require(cpu.system_state().data.dirty_halves == 1);
  require(fixture.memory.read({.address = physical + last_word, .size = 4}) == 7);
  steps(cpu, 6);
  require(cpu.system_state().data.dirty_halves == 0 && cpu.system_state().buffered_writes == 0);
  require(cpu.state().registers[4] == old_r1);
  require(fixture.memory.read({.address = physical + last_word, .size = 4}) == old_r1);
  require(fixture.memory.read({.address = physical + next_subpage, .size = 4}) == 7);
  constexpr unsigned int buffer_case = 30;
  constexpr std::uint32_t buffered_section = 0xc16;
  constexpr std::uint32_t target_descriptor = 0x12800;
  constexpr std::uint32_t target_virtual = 0xa0000000;
  for (const bool deliver : {false, true}) {
    Fixture buffered(bytes, buffer_case);
    buffered.memory.write({.address = directory, .size = 4}, buffered_section);
    buffered.memory.write({.address = target_descriptor, .size = 4}, physical | section);
    buffered.state.registers[1] = target_descriptor;
    buffered.state.registers[2] = 0;
    buffered.state.registers[4] = target_virtual;
    lead_a1::Arm queued(
        buffered.memory, spec, buffered.state, buffered.system, {.deliver_mmu_aborts = deliver});
    steps(queued, 1);
    require(queued.system_state().buffered_writes == 1);
    require(buffered.memory.read({.address = target_descriptor, .size = 4}) ==
            (physical | section));
    if (deliver) {
      data_entry(queued, buffered, 5, {}, target_virtual);
      require(queued.system_state().buffered_writes == 0);
      require(buffered.memory.read({.address = target_descriptor, .size = 4}) == 0);
    } else {
      rejected(queued, "status=0x5");
      require(queued.system_state().buffered_writes == 1);
      require(buffered.memory.read({.address = target_descriptor, .size = 4}) ==
              (physical | section));
    }
  }
}

void fault_registers(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  Fixture fixture(bytes, 13);
  fixture.state.cpsr = supervisor_status;
  constexpr std::uint32_t written_data = 0x1ab;
  constexpr std::uint32_t written_instruction = 0x1cd;
  fixture.state.registers[0] = written_data;
  fixture.state.registers[1] = written_instruction;
  fixture.state.registers[2] = marker;
  lead_a1::Arm cpu(fixture.memory, spec, fixture.state, fixture.system);
  steps(cpu, 7);
  constexpr std::uint32_t expected_data = 0xab;
  constexpr std::uint32_t expected_instruction = 0xcd;
  constexpr std::uint32_t expected_status = 0x080000d3;
  require(cpu.state().registers[3] == expected_data &&
          cpu.state().registers[4] == expected_instruction);
  require(cpu.state().registers[5] == marker && cpu.state().cpsr == expected_status);
  for (const unsigned int index : {14U, 15U, 16U, 17U, 18U, 25U, 26U}) {
    Fixture bad(bytes, index);
    bad.state.cpsr = supervisor_status;
    bad.system.faults = {};
    constexpr std::uint32_t reserved = 0x200;
    bad.state.registers[0] = reserved;
    lead_a1::Arm invalid(bad.memory, spec, bad.state, bad.system);
    if (index == skipped_register_case || index == flags_register_case) {
      steps(invalid, 1);
      const auto flags = index == flags_register_case ? expected_status : supervisor_status;
      require(invalid.state().cpsr == flags);
    } else {
      std::string reason = "Unknown ARM fault";
      if (index == reserved_write_case) { reason = "SBZP"; }
      if (index == invalid_selector_case) { reason = "selector"; }
      rejected(invalid, reason);
    }
  }
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    const auto bytes = read_program(argv[2]);
    scalar_permission(argv[1], bytes);
    user_and_swap(argv[1], bytes);
    partial_multiple(argv[1], bytes);
    status_matrix(argv[1], bytes);
    guest_data_repair(argv[1], bytes);
    guest_prefetch_repair(argv[1], bytes);
    alignment_and_unknown(argv[1], bytes);
    fault_registers(argv[1], bytes);
    exception_boundaries(argv[1], bytes);
    cache_and_buffer_faults(argv[1], bytes);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
