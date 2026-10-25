// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr unsigned int flags = 5;
constexpr unsigned int astat = 6;
constexpr std::uint32_t directions = 0xaaaaaaaa;
constexpr std::uint32_t btf = 0x40000;
constexpr std::uint16_t all_flags = 0xffff;
constexpr std::uint16_t non_core_flags = 0xfff0;
constexpr std::uint16_t excluding_flag7 = 0xff70;
constexpr std::uint32_t end_samples = 0x8001;
constexpr std::uint32_t all_bits = 0xffffffff;
// Owned Type18 field literals: FLAGS register 5, SET/CLR/TST/XOR operation.
constexpr std::uint64_t set_flags = 0x140500000000;
constexpr std::uint64_t clear_flags = 0x142500000000;
constexpr std::uint64_t toggle_flags = 0x144500000000;
constexpr std::uint64_t test_flags = 0x148500000000;
constexpr std::uint64_t compare_flags = 0x14a500000000;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC FLAGS expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC FLAGS rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void map(lead_a1::SharcMemory &memory) {
  constexpr std::uint32_t instruction_cells = 48;
  memory.map({.bank = 2, .halfwords = instruction_cells, .name = "owned FLAG program"});
}

void pin_case(unsigned int pin) {
  const auto packed = static_cast<std::uint16_t>(std::uint32_t{1} << pin);
  const auto level = std::uint32_t{1} << (pin * 2);
  const auto direction = level << 1;
  lead_a1::SharcMemory memory;
  map(memory);
  code(memory, base, set_flags | direction);
  code(memory, base + 1, 0);
  code(memory, base + 2, set_flags | level);
  code(memory, base + 3, 0);
  code(memory, base + 4, test_flags | level);
  code(memory, base + 5, test_flags | level);
  code(memory, base + 6, clear_flags | direction);
  code(memory, base + 7, 0);
  code(memory, base + 8, toggle_flags | direction);
  code(memory, base + 9, 0);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[flags] = {.known = directions}; // Owned all-input directions.
  initial.effective = initial.system;
  lead_a1::Sharc cpu(memory, initial);
  require(cpu.flag_drives().high_impedance == all_flags);
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().high_impedance == all_flags && cpu.state().pending[flags]);
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().unresolved == packed);
  require(cpu.flag_drives().high == 0 && cpu.flag_drives().low == 0);
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().unresolved == packed); // One following instruction.
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().high == packed && cpu.flag_drives().unresolved == 0);
  // A driven output is not silently fed into an input sample/readback.
  static_cast<void>(cpu.step());
  require((cpu.state().system[astat].known & btf) == 0);
  cpu.flag_inputs({.value = 0, .known = packed}); // Explicit low signal.
  static_cast<void>(cpu.step());
  require((cpu.state().system[astat].known & btf) != 0);
  require((cpu.state().system[astat].value & btf) == 0 && cpu.flag_drives().high == packed);
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().high == packed);
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().high_impedance == all_flags && cpu.flag_drives().high == 0);
  require((cpu.state().system[flags].value & level) != 0); // Latch retained.
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().high_impedance == all_flags);
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().high == packed); // Direction TGL preserves latch.
}

void samples_and_partial_tests() {
  lead_a1::SharcMemory memory;
  map(memory);
  constexpr std::uint64_t high_flag15 = 0x148540000000;
  constexpr std::uint64_t low_flag0 = 0x148500000001;
  constexpr std::uint64_t unknown_flag8 = 0x148500010000;
  constexpr std::uint64_t compare_all_inputs = 0x14a540000001;
  code(memory, base, high_flag15);
  code(memory, base + 1, low_flag0);
  code(memory, base + 2, unknown_flag8);
  code(memory, base + 3, compare_all_inputs);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  cpu.flag_inputs({.value = 0x8000, .known = end_samples});
  static_cast<void>(cpu.step());
  require((cpu.state().effective[astat].value & btf) != 0);
  static_cast<void>(cpu.step());
  require((cpu.state().effective[astat].known & btf) != 0 &&
          (cpu.state().effective[astat].value & btf) == 0);
  static_cast<void>(cpu.step());
  require((cpu.state().effective[astat].known & btf) == 0);
  static_cast<void>(cpu.step());
  require((cpu.state().effective[astat].known & btf) != 0); // Known mismatch at FLAG0.
  require((cpu.state().effective[astat].value & btf) == 0);
  cpu.flag_inputs({});
  require(cpu.state().flag_inputs.known == 0);
  require(cpu.flag_drives().high_impedance == 15 && cpu.flag_drives().unresolved == non_core_flags);
}

void separate_writes_and_rollback() {
  lead_a1::SharcMemory memory;
  map(memory);
  constexpr std::uint64_t select_flag7 = 0x140500008000;
  constexpr std::uint64_t combined_flag15 = 0x1405c0000000;
  constexpr std::uint64_t drive_flag7_low = 0x142500004000;
  constexpr std::uint64_t toggle_flag7 = 0x144500004000;
  code(memory, base, select_flag7);
  code(memory, base + 1, combined_flag15);
  code(memory, base + 2, 0);
  code(memory, base + 3, toggle_flag7);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(cpu.step());
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "direction and level require separate writes");
  require(cpu.state() == before && !memory.active());
  // Adjacent separate writes are valid; output effects retain instruction order.
  code(memory, base + 1, drive_flag7_low);
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().unresolved == non_core_flags && cpu.flag_drives().low == 0);
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().low == 0x80 && cpu.flag_drives().unresolved == excluding_flag7);
  const auto settled = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "level toggle timing not qualified");
  require(cpu.state() == settled);
  constexpr std::uint64_t write_input_level = 0x140500000100;
  code(memory, base + 3, write_input_level);
  rejected([&] { static_cast<void>(cpu.step()); }, "without known output selection");
  require(cpu.state() == settled);
}

class Source final : public lead_a1::MemoryDevice {
public:
  unsigned int reads{};
  unsigned int saved{};

  std::uint64_t read(lead_a1::MemoryAccess access) override {
    require(access.address == 0 && access.size == 4);
    ++reads;
    return 0;
  }

  void write(lead_a1::MemoryAccess /*access*/, std::uint64_t /*value*/) override {
    throw std::runtime_error("Read-only owned source");
  }

  void begin_instruction() override { saved = reads; }

  void commit_instruction() noexcept override {}

  void abort_instruction() noexcept override { reads = saved; }
};

void direct_read_and_sampling_guards() {
  Source source;
  lead_a1::SharcMemory memory;
  map(memory);
  memory.map_io({.first_word = 0x1000, .words = 1}, source);
  constexpr std::uint64_t load_flags = 0x107500001000;
  code(memory, base, load_flags);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "without known output selection");
  require(source.reads == 0 && cpu.state() == before && !memory.active());
  rejected([&] { cpu.flag_inputs({.value = 1}); }, "Invalid SHARC FLAG input");
  constexpr std::uint32_t invalid_pin = 0x10000;
  rejected([&] { cpu.flag_inputs({.known = invalid_pin}); }, "Invalid SHARC FLAG input");
  require(cpu.state() == before);
  memory.begin_instruction();
  rejected([&] { cpu.flag_inputs({.known = 1}); }, "sampling during a memory transaction");
  require(cpu.state() == before && memory.active());
  memory.abort_instruction();
  auto invalid = before;
  invalid.flag_inputs = {.value = 1};
  rejected([&] { const lead_a1::Sharc bad(memory, invalid); }, "Invalid SHARC FLAG input");
}

void full_register_write() {
  lead_a1::SharcMemory memory;
  map(memory);
  constexpr std::uint64_t all_low = 0x0f75aaaaaaaa;
  constexpr std::uint64_t all_high = 0x0f75ffffffff;
  code(memory, base, all_low);
  code(memory, base + 1, all_high);
  code(memory, base + 2, 0);
  code(memory, base + 3, compare_flags | all_bits);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[flags] = {.value = directions, .known = directions};
  initial.effective = initial.system; // Explicit previously selected outputs.
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().unresolved == all_flags);
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().low == all_flags);
  static_cast<void>(cpu.step());
  require(cpu.flag_drives().high == all_flags);
  cpu.flag_inputs({.value = all_flags, .known = all_flags});
  static_cast<void>(cpu.step());
  require((cpu.state().effective[astat].value & btf) != 0);
}

}

int main() {
  try {
    for (unsigned int pin = 0; pin < 16; ++pin) { pin_case(pin); }
    samples_and_partial_tests();
    separate_writes_and_rollback();
    direct_read_and_sampling_guards();
    full_register_write();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
