// SPDX-License-Identifier: GPL-3.0-only
#include "arm_exception.hpp"
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
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using lead_a1::ArmException;
constexpr std::uint32_t code_base = 0x1000;
constexpr std::uint32_t spacing = 256;
constexpr std::uint32_t high_vectors = 0xffff0000;
constexpr std::uint32_t cpsr_mask = 0xf90000ff;
constexpr std::uint32_t saved_user = 0xf8000010;
constexpr std::uint32_t supervisor = 0xd3;
constexpr std::uint32_t supervisor_sp = 0x8000;
constexpr std::uint32_t user_sp = 0x9000;
constexpr std::uint32_t supervisor_lr = 0x1234;
constexpr std::uint32_t user_lr = 0x5678;
constexpr std::uint32_t control = 0x50078;
constexpr unsigned int pc_index = 15;
constexpr unsigned int lr_index = 14;
constexpr unsigned int visible_case = 36;
constexpr std::uint32_t fully_known = 0xffffffff;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Independent ARM exception expectation at line " +
                             std::to_string(where.line()));
  }
}

std::vector<std::byte> read_program(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Cannot read owned exception program"); }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

std::uint32_t address(unsigned int index) {
  return code_base + (index * spacing);
}

lead_a1::ArmState initial(unsigned int index) {
  lead_a1::ArmState state;
  state.cpsr = supervisor;
  state.registers[pc_index] = address(index);
  state.registers[13] = supervisor_sp;
  state.registers[lr_index] = supervisor_lr;
  state.banks.modes[0].sp = user_sp;
  state.banks.modes[0].lr = user_lr;
  state.banks.modes[3].saved = {.bits = saved_user, .known_mask = cpsr_mask};
  return state;
}

lead_a1::ArmSystemInitial profile(bool high) {
  constexpr std::uint32_t vector_bit = 0x2000;
  return {.control = control | (high ? vector_bit : 0),
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

void map_program(lead_a1::Memory &memory, std::span<const std::byte> bytes) {
  memory.map({.address = code_base, .size = bytes.size(), .name = "owned exception program"},
             bytes);
}

void map_vectors(lead_a1::Memory &memory, std::span<const std::byte> bytes, std::uint32_t base) {
  constexpr unsigned int vector_bytes = 32;
  const std::vector<std::byte> zero(static_cast<std::size_t>(vector_bytes) * 2);
  memory.map({.address = base, .size = zero.size(), .name = "owned vector RAM"}, zero);
  for (unsigned int offset = 0; offset < vector_bytes; ++offset) {
    memory.write({.address = base + offset, .size = 1},
                 std::to_integer<unsigned int>(bytes[offset]));
  }
  for (unsigned int index = 0; index < 8; ++index) {
    memory.write({.address = base + vector_bytes + (index * 4), .size = 4},
                 address(index == 2 ? 2 : 1));
  }
}

void steps(lead_a1::Arm &cpu, unsigned int count) {
  for (unsigned int index = 0; index < count; ++index) { static_cast<void>(cpu.step()); }
}

void rejected(lead_a1::Arm &cpu, const std::string &reason) {
  const auto state = cpu.state();
  const auto system = cpu.system_state();
  const auto lines = cpu.interrupts();
  try {
    steps(cpu, 1);
  } catch (const std::runtime_error &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    require(cpu.state() == state && cpu.system_state() == system && cpu.interrupts() == lines);
    return;
  }
  throw std::runtime_error("Expected ARM exception failure did not occur");
}

struct EntryExpectation {
  ArmException kind;
  std::uint32_t mode;
  std::uint32_t offset;
  std::size_t bank;
  std::uint32_t link;
};

void check_entry(lead_a1::ArmState state,
                 const EntryExpectation &target,
                 std::uint32_t vector_base) {
  const auto before = state;
  lead_a1::detail::enter_arm_exception(state, target.kind, vector_base);
  constexpr std::uint32_t fiq_bit = 0x40;
  constexpr std::uint32_t irq_bit = 0x80;
  constexpr std::uint32_t interrupt_bits = 0xc0;
  constexpr std::uint32_t flags = 0xb8000000;
  const auto mask =
      target.kind == ArmException::fiq ? interrupt_bits : (before.cpsr & fiq_bit) | irq_bit;
  require(state.cpsr == (flags | mask | target.mode));
  require(state.registers[pc_index] == vector_base + target.offset);
  require(state.registers[lr_index] == address(visible_case) + target.link);
  require(state.registers[13] == before.banks.modes[target.bank].sp);
  require(state.banks.modes[target.bank].saved.bits == before.cpsr);
  require(state.banks.modes[target.bank].saved.known_mask == cpsr_mask);
  for (unsigned int reg = 0; reg < 8; ++reg) {
    require(state.registers[reg] == before.registers[reg]);
  }
  for (unsigned int reg = 8; reg < 13; ++reg) { require(!state.registers[reg]); }
}

void entry_matrix() {
  constexpr std::array<EntryExpectation, 6> targets{
      {{.kind = ArmException::undefined_instruction,
        .mode = 0x1b,
        .offset = 4,
        .bank = 5,
        .link = 4},
       {.kind = ArmException::software_interrupt, .mode = 0x13, .offset = 8, .bank = 3, .link = 4},
       {.kind = ArmException::prefetch_abort, .mode = 0x17, .offset = 12, .bank = 4, .link = 4},
       {.kind = ArmException::data_abort, .mode = 0x17, .offset = 16, .bank = 4, .link = 8},
       {.kind = ArmException::irq, .mode = 0x12, .offset = 0x18, .bank = 2, .link = 4},
       {.kind = ArmException::fiq, .mode = 0x11, .offset = 0x1c, .bank = 1, .link = 4}}};
  constexpr std::array<std::uint32_t, 7> modes{0x10, 0x1f, 0x11, 0x12, 0x13, 0x17, 0x1b};
  constexpr std::array<std::size_t, 7> banks{0, 0, 1, 2, 3, 4, 5};
  constexpr std::uint32_t flags = 0xb8000000;
  for (std::size_t source = 0; source < modes.size(); ++source) {
    for (std::uint32_t masks = 0; masks < 4; ++masks) {
      for (const auto vector_base : {0U, high_vectors}) {
        for (const auto &target : targets) {
          auto state = initial(visible_case);
          state.cpsr = flags | modes[source] | (masks << 6);
          for (std::size_t bank = 0; bank < state.banks.modes.size(); ++bank) {
            state.banks.modes[bank].sp = supervisor_sp + static_cast<std::uint32_t>(bank * spacing);
            state.banks.modes[bank].lr = user_sp + static_cast<std::uint32_t>(bank * spacing);
          }
          state.registers[13] = state.banks.modes[banks[source]].sp;
          state.registers[lr_index] = state.banks.modes[banks[source]].lr;
          check_entry(state, target, vector_base);
        }
      }
    }
  }
}

void mask_priority(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr std::array<std::array<int, 4>, 4> expected{
      {{{-1, 0, 1, 1}}, {{-1, 0, -1, 0}}, {{-1, -1, 1, 1}}, {{-1, -1, -1, -1}}}};
  for (unsigned int masks = 0; masks < expected.size(); ++masks) {
    for (unsigned int lines = 0; lines < expected[masks].size(); ++lines) {
      lead_a1::Memory memory;
      map_program(memory, bytes);
      auto state = initial(visible_case);
      constexpr std::uint32_t user = 0x10;
      state.cpsr = user | (masks << 6);
      lead_a1::Arm cpu(memory, spec, state, profile(false));
      const lead_a1::ArmInterruptLines input{.irq = (lines & 1) != 0, .fiq = (lines & 2) != 0};
      cpu.set_interrupts(input);
      const auto step = cpu.step();
      const auto wanted = expected[masks][lines];
      require(cpu.interrupts() == input);
      if (wanted < 0) {
        require(!step.exception && cpu.state().registers[0] == 7);
        require(cpu.decode_statistics().translations == 1);
      } else {
        require(step.exception == (wanted == 0 ? ArmException::irq : ArmException::fiq));
        require(!cpu.state().registers[0] && cpu.decode_statistics().translations == 0);
      }
    }
  }
}

void interrupt_return(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  for (const bool high : {false, true}) {
    lead_a1::Memory memory;
    map_program(memory, bytes);
    map_vectors(memory, bytes, high ? high_vectors : 0);
    auto state = initial(visible_case);
    constexpr std::uint32_t user = 0x10;
    state.cpsr = user;
    state.registers[13] = user_sp;
    state.registers[lr_index] = user_lr;
    state.banks.modes[2].sp = supervisor_sp;
    lead_a1::Arm cpu(memory, spec, state, profile(high));
    cpu.set_interrupts({.irq = true, .fiq = false});
    const auto event = cpu.step();
    constexpr std::uint32_t irq_offset = 24;
    require(event.exception == ArmException::irq &&
            event.after == (high ? high_vectors : 0) + irq_offset);
    constexpr std::uint32_t irq_status = 0x92;
    require(cpu.state().cpsr == irq_status && cpu.state().registers[13] == supervisor_sp);
    steps(cpu, 2);
    require(cpu.state().registers[pc_index] == address(visible_case));
    require(cpu.state().cpsr == user && cpu.state().registers[13] == user_sp);
    require(cpu.state().registers[lr_index] == user_lr);
    require(cpu.step().exception == ArmException::irq);
    cpu.set_interrupts({});
    steps(cpu, 3);
    require(cpu.state().registers[0] == 7 && cpu.state().cpsr == user);
  }
}

void software_interrupt(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  map_vectors(memory, bytes, high_vectors);
  auto state = initial(3);
  constexpr std::uint32_t user = 0xa8000050;
  state.cpsr = user;
  state.registers[13] = user_sp;
  state.registers[lr_index] = user_lr;
  state.banks.modes[3].sp = supervisor_sp;
  lead_a1::Arm cpu(memory, spec, state, profile(true));
  require(cpu.step().exception == ArmException::software_interrupt);
  constexpr std::uint32_t handler_status = 0xa80000d3;
  require(cpu.state().cpsr == handler_status);
  require(cpu.state().registers[pc_index] == high_vectors + 8);
  steps(cpu, 3);
  require(cpu.state().cpsr == user && cpu.state().registers[0] == 7);
  require(cpu.state().registers[13] == user_sp && cpu.state().registers[lr_index] == user_lr);
  lead_a1::Arm skipped(memory, spec, initial(4));
  require(!skipped.step().exception);
  require(skipped.state().registers[pc_index] == address(4) + 4);
}

void return_block_modes(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr std::array<unsigned int, 4> cases{7, 8, 9, 10};
  constexpr std::array<std::uint32_t, 4> before_sp{0x8000, 0x7ffc, 0x800c, 0x8010};
  constexpr std::array<std::uint32_t, 4> after_sp{0x8010, 0x800c, 0x7ffc, 0x8000};
  constexpr std::array<std::uint32_t, 4> words{0x11112222, 0x88889999, 0xccccdddd, 0x12345003};
  constexpr std::uint32_t target = 0x12345000;
  for (std::size_t index = 0; index < cases.size(); ++index) {
    lead_a1::Memory memory;
    map_program(memory, bytes);
    memory.map({.address = supervisor_sp, .size = 16, .name = "owned return frame"});
    for (unsigned int word = 0; word < words.size(); ++word) {
      memory.write({.address = supervisor_sp + (word * 4), .size = 4}, words[word]);
    }
    auto state = initial(cases[index]);
    state.registers[13] = before_sp[index];
    lead_a1::Arm cpu(memory, spec, state);
    require(!cpu.step().exception);
    const auto after = cpu.state();
    require(after.cpsr == saved_user && after.registers[pc_index] == target);
    require(after.registers[0] == words[0] && after.registers[8] == words[1] &&
            after.registers[12] == words[2]);
    require(after.registers[13] == user_sp && after.banks.modes[3].sp == after_sp[index]);
    require(after.banks.modes[3].saved == state.banks.modes[3].saved);
  }
}

void return_arithmetic(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr std::array<std::uint32_t, 12> results{0x02244220,
                                                  0x95511558,
                                                  0x7530eca8,
                                                  0x8acf1354,
                                                  0x99999998,
                                                  0x99999998,
                                                  0x7530eca8,
                                                  0x8acf1354,
                                                  0x97755778,
                                                  0x12345678,
                                                  0x85410100,
                                                  0xedcba984};
  constexpr std::uint32_t left = 0x87654322;
  constexpr std::uint32_t right = 0x12345679;
  for (unsigned int index = 0; index < results.size(); ++index) {
    lead_a1::Memory memory;
    map_program(memory, bytes);
    auto state = initial(16 + index);
    state.registers[0] = left;
    state.registers[1] = right;
    lead_a1::Arm cpu(memory, spec, state);
    steps(cpu, 1);
    require(cpu.state().registers[pc_index] == results[index] && cpu.state().cpsr == saved_user);
  }
  constexpr std::array<unsigned int, 3> cases{21, 22, 23};
  constexpr std::array<std::uint32_t, 3> left_inputs{left, 0x1000, 4};
  constexpr std::array<std::uint32_t, 3> right_inputs{right, 4, 0x1000};
  constexpr std::array<std::array<std::uint32_t, 2>, 3> expected{
      {{{0x99999998, 0x9999999c}}, {{0xff8, 0xffc}}, {{0xff8, 0xffc}}}};
  for (std::size_t index = 0; index < cases.size(); ++index) {
    for (unsigned int carry = 0; carry < 2; ++carry) {
      lead_a1::Memory memory;
      map_program(memory, bytes);
      auto state = initial(cases[index]);
      constexpr unsigned int carry_bit = 29;
      state.cpsr |= carry << carry_bit;
      state.registers[0] = left_inputs[index];
      state.registers[1] = right_inputs[index];
      lead_a1::Arm cpu(memory, spec, state);
      steps(cpu, 1);
      require(cpu.state().registers[pc_index] == expected[index][carry]);
      require(cpu.state().cpsr == saved_user);
    }
  }
}

void return_failures(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  constexpr unsigned int move_case = 5;
  auto state = initial(move_case);
  state.registers[0] = code_base;
  state.banks.modes[3].saved.known_mask = 0;
  lead_a1::Arm unknown(memory, spec, state);
  rejected(unknown, "SPSR bits");
  state = initial(move_case);
  state.registers[0] = code_base;
  constexpr std::uint32_t reserved = 0x100;
  constexpr std::uint32_t invalid_mode = 0x14;
  constexpr std::array<std::uint32_t, 4> invalid{0x30, 0x01000010, reserved, invalid_mode};
  constexpr std::array<const char *, 4> reasons{
      "Thumb/Jazelle", "Thumb/Jazelle", "reserved", "mode"};
  for (std::size_t index = 0; index < invalid.size(); ++index) {
    state.banks.modes[3].saved = {.bits = invalid[index], .known_mask = fully_known};
    lead_a1::Arm bad(memory, spec, state);
    rejected(bad, reasons[index]);
  }
  for (const auto mode : {0x10U, 0x1fU}) {
    state = initial(move_case);
    state.cpsr = mode;
    lead_a1::Arm user(memory, spec, state);
    rejected(user, "User/System");
  }
  for (const auto index : {14U, 15U}) {
    state = initial(index);
    state.banks.modes[3].saved = {};
    state.registers[13].reset();
    state.registers[lr_index].reset();
    lead_a1::Arm skipped(memory, spec, state);
    steps(skipped, 1);
    require(skipped.state().cpsr == supervisor &&
            skipped.state().registers[pc_index] == address(index) + 4);
  }
  constexpr unsigned int bad_shift_case = 35;
  state = initial(bad_shift_case);
  state.registers[0] = code_base;
  state.registers[1] = 0;
  lead_a1::Arm shift(memory, spec, state);
  rejected(shift, "register-shift");
  state = initial(12);
  state.registers[0] = supervisor_sp;
  lead_a1::Arm overlap(memory, spec, state);
  rejected(overlap, "base/writeback");
  memory.map({.address = supervisor_sp, .size = 8, .name = "owned partial return frame"});
  memory.write({.address = supervisor_sp, .size = 4}, 7);
  state = initial(13);
  state.registers[0] = supervisor_sp;
  constexpr std::uint32_t old_register = 99;
  state.registers[1] = old_register;
  lead_a1::Arm partial(memory, spec, state);
  rejected(partial, "uninitialized");
  require(memory.read({.address = supervisor_sp, .size = 4}) == 7);
  require(!memory.peek(supervisor_sp + 4));
  state = initial(visible_case);
  state.cpsr = 0x10;
  lead_a1::Arm unconfigured(memory, spec, state);
  unconfigured.set_interrupts({.irq = true, .fiq = false});
  rejected(unconfigured, "not configured");
  unconfigured.set_interrupts({});
  steps(unconfigured, 1);
  require(unconfigured.state().registers[0] == 7);
}

struct ShiftExpectation {
  unsigned int index;
  std::optional<std::uint32_t> input;
  bool carry;
  std::uint32_t target;
};

void return_shifters(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr std::array<ShiftExpectation, 17> cases{
      {{.index = 28, .input = 0xffffffff, .carry = false, .target = 0x80000000},
       {.index = 29, .input = 0xffffffff, .carry = false, .target = 0},
       {.index = 30, .input = 0x80000001, .carry = false, .target = 0xfffffffc},
       {.index = 30, .input = 7, .carry = false, .target = 0},
       {.index = 31, .input = 0x12345678, .carry = false, .target = 0x78123454},
       {.index = 32, .input = 0x1008, .carry = false, .target = 0x804},
       {.index = 32, .input = 0x1008, .carry = true, .target = 0x80000804},
       {.index = 33, .input = {}, .carry = false, .target = 0x80000000},
       {.index = 34, .input = 4, .carry = false, .target = 0x01000004},
       {.index = 5, .input = 0x12345003, .carry = false, .target = 0x12345000},
       {.index = 40, .input = 0x80000002, .carry = false, .target = 0xc0000000},
       {.index = 40, .input = 8, .carry = false, .target = 4},
       {.index = 41, .input = 0xfffffff8, .carry = false, .target = 0x7ffffffc},
       {.index = 42, .input = 0x80000004, .carry = false, .target = 8},
       {.index = 43, .input = {}, .carry = false, .target = 0x3b08},
       {.index = 44, .input = 0x80000000, .carry = false, .target = 0xfffffffc},
       {.index = 45, .input = {}, .carry = false, .target = 16}}};
  for (const auto &expected : cases) {
    lead_a1::Memory memory;
    map_program(memory, bytes);
    auto state = initial(expected.index);
    state.registers[0] = expected.input;
    constexpr std::uint32_t carry_bit = 0x20000000;
    if (expected.carry) { state.cpsr |= carry_bit; }
    lead_a1::Arm cpu(memory, spec, state);
    steps(cpu, 1);
    require(cpu.state().registers[pc_index] == expected.target && cpu.state().cpsr == saved_user);
  }
  lead_a1::Memory memory;
  map_program(memory, bytes);
  auto state = initial(6);
  state.registers[lr_index] = 4;
  lead_a1::Arm wrapping(memory, spec, state);
  steps(wrapping, 1);
  constexpr std::uint32_t wrapped = 0xfffffffc;
  require(wrapping.state().registers[pc_index] == wrapped && wrapping.state().cpsr == saved_user);
  state = initial(visible_case);
  state.registers[pc_index] = wrapped;
  lead_a1::detail::enter_arm_exception(state, ArmException::irq, 0);
  require(state.registers[lr_index] == 0);
}

void boundary_and_unmapped(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  constexpr unsigned int skipped_case = 14;
  for (const unsigned int offset : {8U, 12U}) {
    lead_a1::Memory memory;
    constexpr std::uint32_t end_base = 0xfffffff0;
    const auto instruction = bytes.subspan(static_cast<std::size_t>(skipped_case) * spacing, 4);
    memory.map({.address = end_base, .size = 16, .name = "owned skipped wrapping return"},
               instruction,
               offset);
    auto state = initial(skipped_case);
    state.registers[pc_index] = end_base + offset;
    state.registers[lr_index].reset();
    state.banks.modes[3].saved = {};
    lead_a1::Arm cpu(memory, spec, state);
    steps(cpu, 1);
    constexpr std::uint32_t last = 0xfffffffc;
    require(cpu.state().registers[pc_index] == (offset == 8 ? last : 0));
    require(!cpu.state().registers[lr_index] && cpu.state().cpsr == supervisor);
  }
  lead_a1::Memory memory;
  auto state = initial(visible_case);
  state.cpsr = 0x10;
  lead_a1::Arm cpu(memory, spec, state, profile(false));
  cpu.set_interrupts({.irq = true, .fiq = false});
  require(cpu.step().exception == ArmException::irq);
  require(cpu.decode_statistics().translations == 0 && !cpu.state().registers[0]);
  rejected(cpu, "unmapped");
}

void fiq_banks(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  map_vectors(memory, bytes, high_vectors);
  auto state = initial(visible_case);
  constexpr std::uint32_t user = 0xb8000010;
  state.cpsr = user;
  state.registers[13] = user_sp;
  state.registers[lr_index] = user_lr;
  for (unsigned int index = 0; index < 5; ++index) {
    state.registers[8 + index] = 8 + index;
    constexpr std::uint32_t private_base = 0xf0000000;
    state.banks.high[1][index] = private_base + index;
  }
  lead_a1::Arm cpu(memory, spec, state, profile(true));
  cpu.set_interrupts({.irq = true, .fiq = true});
  require(cpu.step().exception == ArmException::fiq);
  constexpr std::uint32_t fiq_status = 0xb80000d1;
  require(cpu.state().cpsr == fiq_status && !cpu.state().registers[13]);
  for (unsigned int index = 0; index < 5; ++index) {
    require(cpu.state().registers[8 + index] == state.banks.high[1][index]);
  }
  cpu.set_interrupts({});
  steps(cpu, 2);
  require(cpu.state().cpsr == user && cpu.state().registers[pc_index] == address(visible_case));
  for (unsigned int index = 0; index < 5; ++index) {
    require(cpu.state().registers[8 + index] == 8 + index);
  }
}

void loaded_banks(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  lead_a1::Memory memory;
  map_program(memory, bytes);
  memory.map({.address = supervisor_sp, .size = 16, .name = "owned bank-return frame"});
  constexpr std::array<std::uint32_t, 4> words{0x11112222, 0x88889999, 0xccccdddd, 0x12345003};
  for (unsigned int index = 0; index < words.size(); ++index) {
    memory.write({.address = supervisor_sp + (index * 4), .size = 4}, words[index]);
  }
  auto state = initial(7);
  constexpr std::uint32_t fiq_status = 0xd1;
  constexpr std::uint32_t common_r8 = 0x8080ccdd;
  constexpr std::uint32_t common_r12 = 0xc0c0eeee;
  constexpr std::uint32_t target = 0x12345000;
  state.cpsr = fiq_status;
  state.banks.modes[1].saved = {.bits = saved_user, .known_mask = cpsr_mask};
  state.banks.high[0][0] = common_r8;
  state.banks.high[0][4] = common_r12;
  lead_a1::Arm fiq(memory, spec, state);
  steps(fiq, 1);
  require(fiq.state().registers[8] == common_r8 && fiq.state().registers[12] == common_r12);
  require(fiq.state().banks.high[1][0] == words[1] && fiq.state().banks.high[1][4] == words[2]);
  require(fiq.state().registers[pc_index] == target && fiq.state().cpsr == saved_user);
  constexpr std::array<std::uint32_t, 3> frame{0xd00d0001, 0xeeee0002, 0x12345003};
  for (unsigned int index = 0; index < frame.size(); ++index) {
    memory.write({.address = supervisor_sp + (index * 4), .size = 4}, frame[index]);
  }
  constexpr unsigned int load_banks_case = 37;
  for (const unsigned int index : {11U, load_banks_case}) {
    state = initial(index);
    state.registers[4] = supervisor_sp;
    lead_a1::Arm cpu(memory, spec, state);
    steps(cpu, 1);
    require(cpu.state().registers[pc_index] == target && cpu.state().cpsr == saved_user);
    require(cpu.state().registers[13] == user_sp && cpu.state().registers[lr_index] == user_lr);
    if (index == load_banks_case) {
      require(cpu.state().banks.modes[3].sp == frame[0] &&
              cpu.state().banks.modes[3].lr == frame[1]);
      require(cpu.state().registers[4] == supervisor_sp + 12);
    } else {
      require(cpu.state().registers[0] == frame[0] && cpu.state().registers[1] == frame[1]);
      require(cpu.state().registers[4] == supervisor_sp);
    }
  }
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    const auto bytes = read_program(argv[2]);
    entry_matrix();
    mask_priority(argv[1], bytes);
    interrupt_return(argv[1], bytes);
    software_interrupt(argv[1], bytes);
    return_block_modes(argv[1], bytes);
    return_arithmetic(argv[1], bytes);
    return_failures(argv[1], bytes);
    return_shifters(argv[1], bytes);
    fiq_banks(argv[1], bytes);
    loaded_banks(argv[1], bytes);
    boundary_and_unmapped(argv[1], bytes);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
