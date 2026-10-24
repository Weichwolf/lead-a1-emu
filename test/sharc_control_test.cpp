// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t sysctl = 0x30024;
constexpr lead_a1::MemoryAccess word{.size = 4};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC SYSCTL expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC SYSCTL rejection: " + reason);
}

void boot_inputs_and_widths() {
  lead_a1::SharcControl unknown({});
  rejected([&] { static_cast<void>(unknown.read(word)); }, "Unknown SHARC SYSCTL boot-mode");
  constexpr std::uint32_t normal = 0x80000;
  require(unknown.data_width(normal) == lead_a1::SharcWidth::normal32);
  lead_a1::SharcControl valid(true);
  lead_a1::SharcControl reserved(false);
  require(valid.read(word) == 4 && reserved.read(word) == 0);
  constexpr std::array<std::uint32_t, 4> addresses{0x80000, 0xa0000, 0xc0000, 0xe0000};
  constexpr unsigned int width_shift = 9;
  for (unsigned int selected = 0; selected < addresses.size(); ++selected) {
    valid.write(word, 4 | (std::uint32_t{1} << (width_shift + selected)));
    require(valid.data_width(addresses[selected]) == lead_a1::SharcWidth::normal32);
    valid.begin_core_instruction();
    valid.commit_core_instruction();
    for (unsigned int bank = 0; bank < addresses.size(); ++bank) {
      require(valid.data_width(addresses[bank]) ==
              (bank == selected ? lead_a1::SharcWidth::extended40 : lead_a1::SharcWidth::normal32));
    }
    valid.write(word, 4);
    valid.begin_core_instruction();
    valid.commit_core_instruction();
  }
  constexpr std::uint32_t short_address = 0x100000;
  rejected([&] { static_cast<void>(valid.data_width(short_address)); }, "outside normal-word");
}

void transaction_ownership() {
  lead_a1::SharcControl control(true);
  const auto before = control.state();
  constexpr std::uint32_t extended_bank0 = 0x204;
  control.begin_instruction();
  control.write(word, extended_bank0);
  require(control.read(word) == extended_bank0);
  rejected([&] { control.begin_core_instruction(); }, "already owned");
  rejected([&] { control.begin_instruction(); }, "Nested");
  control.abort_instruction();
  require(control.state() == before);
  control.begin_core_instruction();
  control.begin_instruction(); // Memory joins the same owning instruction.
  control.write(word, extended_bank0);
  control.commit_instruction();
  control.abort_core_instruction();
  require(control.state() == before);
  control.begin_core_instruction();
  control.begin_instruction();
  control.write(word, extended_bank0);
  control.commit_instruction();
  control.commit_core_instruction();
  require(control.state().control == extended_bank0 && control.state().effective == 4);
  control.begin_core_instruction();
  control.begin_instruction();
  control.write(word, 4);
  control.commit_instruction();
  control.commit_core_instruction();
  require(control.state().control == 4 && control.state().effective == extended_bank0);
  control.begin_core_instruction();
  control.commit_core_instruction();
  require(control.state().effective == 4 && !control.state().delayed);
}

void invalid_fields() {
  lead_a1::SharcControl control(true);
  const auto before = control.state();
  constexpr std::array<std::uint64_t, 6> unsupported{
      1, 2, 0x80, 0x10000, 0x1000000, std::uint64_t{1} << 32};
  for (const auto value : unsupported) {
    rejected([&] { control.write(word, value); }, "write not implemented");
    require(control.state() == before);
  }
  rejected([&] { static_cast<void>(control.read({.size = 2})); }, "offset0/read-write32");
  rejected([&] { control.write({.address = 4, .size = 4}, 0); }, "offset0/read-write32");
  require(control.state() == before);
}

void cpu_effects_and_failed_store() {
  lead_a1::SharcControl control(true);
  lead_a1::SharcMemory memory;
  constexpr std::uint32_t cells = 64;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned SYSCTL program"});
  memory.map_io({.first_word = sysctl, .words = 1}, control);
  constexpr std::array<std::uint64_t, 4> program{0x0f7100000204, 0x117100030024, 0x60fe02800000, 0};
  for (unsigned int index = 0; index < program.size(); ++index) {
    memory.write({.address = base + index, .width = lead_a1::SharcWidth::wide48}, program[index]);
  }
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base), &control);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  const auto before = cpu.state();
  const auto pending = control.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Unknown SHARC register operand");
  require(cpu.state() == before && control.state() == pending && !memory.active());
  memory.write({.address = base + 2, .width = lead_a1::SharcWidth::wide48}, 0);
  static_cast<void>(cpu.step());
  constexpr std::uint32_t expected = 0x204;
  require(control.state().effective == expected && !control.state().delayed);
}

void quiet_internal_clock_effects() {
  lead_a1::SharcControl control(true);
  constexpr std::uint32_t extended_bank2 = 0x804;
  control.write(word, extended_bank2);
  const auto before = control.state();
  control.advance_internal_clocks(0);
  require(control.state() == before);
  rejected([&] { control.advance_internal_clocks(1); }, "partial clock effect interval");
  require(control.state() == before);
  control.begin_instruction();
  const auto owned = control.state();
  rejected([&] { control.advance_internal_clocks(4); }, "clock advance during a transaction");
  require(control.state() == owned);
  control.abort_instruction();
  require(control.state() == before);
  control.advance_internal_clocks(2);
  require(control.state().effective == extended_bank2 && !control.state().delayed);
}

}

int main() {
  try {
    boot_inputs_and_widths();
    transaction_ownership();
    invalid_fields();
    cpu_effects_and_failed_store();
    quiet_internal_clock_effects();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
