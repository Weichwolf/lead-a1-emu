// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_visa.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0x180040;
constexpr std::uint32_t native_alias = 0xc0000;
constexpr unsigned int cells = 512;
constexpr std::uint16_t nop = 1;
constexpr std::uint16_t unknown_type = 0xd000;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("VISA expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing VISA rejection: " + reason);
}

void write(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint16_t value) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::short16}, value);
}

void program(lead_a1::SharcMemory &memory) {
  memory.map({.bank = 2, .halfwords = cells, .name = "owned VISA short words"});
  for (unsigned int index = 0; index < cells; ++index) { write(memory, base - 64 + index, nop); }
  memory.set_pc(native_alias);
}

lead_a1::SharcVisaState initial() {
  return {.decode = base, .fetch = base};
}

void expect_instruction(const lead_a1::SharcVisaBuffer &buffer,
                        std::uint32_t address,
                        unsigned int words,
                        std::uint64_t opcode) {
  const auto actual = buffer.instruction();
  require(actual && actual->address == address && actual->words == words &&
          actual->opcode == opcode);
}

void length_formats() {
  struct Example {
    std::array<std::uint16_t, 3> data;
    unsigned int words;
  };

  // Literal owned encodings from the distinct PGR10-5..29 format diagrams.
  constexpr std::array examples{Example{.data = {0x2000, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x2000, 0x003f, 0x1111}, .words = 2},
                                Example{.data = {0x013e, 0x0002, 0x9770}, .words = 3},
                                Example{.data = {0x0182, 0x9770, 0xffff}, .words = 2},
                                Example{.data = {0xc200, 0xffff, 0xffff}, .words = 1},
                                Example{.data = {0x4000, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x4000, 0x003f, 0xffff}, .words = 2},
                                Example{.data = {0x9030, 0xffff, 0xffff}, .words = 1},
                                Example{.data = {0x6000, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x6000, 0x003f, 0xffff}, .words = 2},
                                Example{.data = {0x7000, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x7000, 0x003f, 0xffff}, .words = 2},
                                Example{.data = {0x0200, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x8000, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x0400, 0x003f, 0xffff}, .words = 2},
                                Example{.data = {0x0400, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x063e, 0x000c, 0x0040}, .words = 3},
                                Example{.data = {0x073e, 0x0000, 0x0003}, .words = 3},
                                Example{.data = {0x0800, 0x003f, 0xffff}, .words = 2},
                                Example{.data = {0x0900, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x0a3e, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x0abe, 0xffff, 0xffff}, .words = 1},
                                Example{.data = {0x0bbe, 0xffff, 0xffff}, .words = 1},
                                Example{.data = {0x0c00, 0x0001, 0x0005}, .words = 3},
                                Example{.data = {0x0d00, 0x0000, 0x0005}, .words = 3},
                                Example{.data = {0x0e00, 0x0000, 0x0005}, .words = 3},
                                Example{.data = {0x1000, 0x0008, 0x0040}, .words = 3},
                                Example{.data = {0xab6b, 0xffff, 0xfffd}, .words = 3},
                                Example{.data = {0x9008, 0x0001, 0xffff}, .words = 2},
                                Example{.data = {0x9004, 0x0001, 0xffff}, .words = 2},
                                Example{.data = {0x9000, 0x0000, 0x0001}, .words = 3},
                                Example{.data = {0x0f70, 0x0000, 0x0004}, .words = 3},
                                Example{.data = {0x0ff0, 0x0004, 0xffff}, .words = 2},
                                Example{.data = {0x1400, 0x0000, 0x0001}, .words = 3},
                                Example{.data = {0x1600, 0x0000, 0x0001}, .words = 3},
                                Example{.data = {0x1700, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x0000, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x0001, 0xffff, 0xffff}, .words = 1},
                                Example{.data = {0x0080, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x0081, 0xffff, 0xffff}, .words = 1},
                                Example{.data = {0x0041, 0xffff, 0xffff}, .words = 1},
                                Example{.data = {0x1804, 0x0000, 0x0040}, .words = 3},
                                Example{.data = {0x1900, 0x0000, 0x0000}, .words = 3},
                                Example{.data = {0x1901, 0xffff, 0xffff}, .words = 1}};
  for (const auto &example : examples) {
    require(lead_a1::sharc_visa_words(example.data) == example.words);
  }
  const std::array<std::uint16_t, 1> ambiguous{0x2000};
  require(!lead_a1::sharc_visa_words(ambiguous));
  require(!lead_a1::sharc_visa_words({}));
  const std::array<std::uint16_t, 4> oversized{};
  rejected([&] { static_cast<void>(lead_a1::sharc_visa_words(oversized)); }, "exceeds three");
  for (const auto head : {std::uint16_t{0xd000}, std::uint16_t{0xf000}, std::uint16_t{0x900c}}) {
    const std::array prefix{head};
    rejected([&] { static_cast<void>(lead_a1::sharc_visa_words(prefix)); }, "VISA");
  }
}

void delayed_capacity_and_order() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcVisaBuffer buffer(memory, initial());
  require(!buffer.instruction());
  buffer.capture();
  require(!buffer.instruction() && buffer.state().count == 0 && buffer.state().delay);
  require(!buffer.advance(false));
  require(buffer.state().count == 3 && buffer.can_capture());
  buffer.capture();
  expect_instruction(buffer, base, 1, 0x000100000000);
  require(buffer.advance(true).has_value() && buffer.state().count == 5 && buffer.can_capture());
  buffer.capture();
  const auto full = buffer.state();
  rejected([&] { buffer.capture(); }, "delay register is occupied");
  require(buffer.state() == full);
  // Eight buffered short words occupy the five-word IAB plus its three-word delay.
  constexpr std::array counts{4U, 3U, 5U, 4U, 3U, 2U, 1U, 0U};
  for (unsigned int index = 0; index < counts.size(); ++index) {
    expect_instruction(buffer, base + 1 + index, 1, 0x000100000000);
    const auto accepted = buffer.advance(true);
    require(accepted && accepted->address == base + 1 + index &&
            buffer.state().count == counts[index]);
  }
  require(!buffer.instruction() && buffer.state().decode == base + 9 &&
          buffer.state().fetch == base + 9 && buffer.can_capture());
}

void split_instruction_and_restore() {
  lead_a1::SharcMemory memory;
  program(memory);
  // NOP16, R0=0x12345678 (48), R1=0xabcd (32), short compute16, NOP16.
  constexpr std::array<std::uint16_t, 9> data{
      0x0001, 0x0f00, 0x1234, 0x5678, 0x0f81, 0xabcd, 0xc200, 0x0001, 0x0001};
  for (unsigned int index = 0; index < data.size(); ++index) {
    write(memory, base + index, data[index]);
  }
  lead_a1::SharcVisaBuffer buffer(memory, initial());
  buffer.capture();
  static_cast<void>(buffer.advance(false));
  static_cast<void>(buffer.advance(true));
  require(buffer.state().count == 2 && !buffer.instruction());
  const auto incomplete = buffer.state();
  rejected([&] { static_cast<void>(buffer.advance(true)); }, "complete buffered instruction");
  require(buffer.state() == incomplete);
  buffer.capture();
  lead_a1::SharcMemory other_memory;
  program(other_memory); // Different RAM is not reread in place of saved IAB/delay words.
  lead_a1::SharcVisaBuffer restored(other_memory, buffer.state());
  for (auto *current : {&buffer, &restored}) {
    static_cast<void>(current->advance(false));
    constexpr std::uint64_t immediate32 = 0x0f0012345678;
    constexpr std::uint64_t immediate16 = 0x0f81abcd0000;
    expect_instruction(*current, base + 1, 3, immediate32);
    static_cast<void>(current->advance(true));
    expect_instruction(*current, base + 4, 2, immediate16);
    static_cast<void>(current->advance(true));
    require(!current->instruction() && current->state().decode == base + 6);
  }
  require(buffer.state() == restored.state());
  buffer.capture();
  static_cast<void>(buffer.advance(false));
  constexpr std::uint64_t short_compute = 0xc20000000000;
  expect_instruction(buffer, base + 6, 1, short_compute);
}

void capture_visibility_and_failures() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcVisaBuffer buffer(memory, initial());
  buffer.capture();
  write(memory, base, unknown_type);
  static_cast<void>(buffer.advance(false));
  expect_instruction(buffer, base, 1, 0x000100000000);
  lead_a1::SharcVisaBuffer fresh(memory, initial());
  fresh.capture();
  static_cast<void>(fresh.advance(false));
  const auto bad = fresh.state();
  rejected([&] { static_cast<void>(fresh.instruction()); }, "operation=decode");
  rejected([&] { static_cast<void>(fresh.advance(true)); }, "Unsupported VISA length prefix");
  require(fresh.state() == bad);
  memory.begin_instruction();
  const auto before = buffer.state();
  rejected([&] { buffer.capture(); }, "foreign transaction");
  rejected([&] { static_cast<void>(buffer.advance(true)); }, "foreign transaction");
  require(buffer.state() == before && memory.active());
  memory.abort_instruction();
}

void partial_read_and_bad_snapshots() {
  lead_a1::SharcMemory memory;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned partial VISA burst"});
  write(memory, base, nop);
  write(memory, base + 1, nop);
  lead_a1::SharcVisaBuffer buffer(memory, initial());
  const auto before = buffer.state();
  rejected([&] { buffer.capture(); }, "Unknown SHARC memory bits");
  require(buffer.state() == before && !memory.active());
  write(memory, base + 2, nop);
  buffer.capture();
  static_cast<void>(buffer.advance(false));
  expect_instruction(buffer, base, 1, 0x000100000000);
  auto state = initial();
  state.fetch += 1;
  rejected([&] { const lead_a1::SharcVisaBuffer invalid(memory, state); }, "continuity");
  state = initial();
  state.data[0] = nop;
  rejected([&] { const lead_a1::SharcVisaBuffer invalid(memory, state); }, "unused VISA buffer");
  state = initial();
  state.decode = native_alias;
  rejected([&] { const lead_a1::SharcVisaBuffer invalid(memory, state); }, "addresses/count");
  constexpr std::uint32_t boundary = 0x1bffff;
  lead_a1::SharcVisaBuffer crossing(memory, {.decode = boundary, .fetch = boundary});
  const auto crossing_before = crossing.state();
  rejected([&] { crossing.capture(); }, "cross-block burst arbitration");
  require(crossing.state() == crossing_before);
}

void permissions_and_full_checkpoint() {
  lead_a1::SharcMemory protected_memory;
  protected_memory.map({.bank = 2,
                        .halfwords = cells,
                        .name = "owned non-executable VISA RAM",
                        .executable = false});
  lead_a1::SharcVisaBuffer protected_buffer(protected_memory, initial());
  const auto protected_before = protected_buffer.state();
  rejected([&] { protected_buffer.capture(); }, "Non-executable SHARC physical region");
  require(protected_buffer.state() == protected_before);
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcVisaBuffer buffer(memory, initial());
  buffer.capture();
  static_cast<void>(buffer.advance(false));
  buffer.capture();
  static_cast<void>(buffer.advance(true));
  buffer.capture();
  const auto occupied = buffer.state();
  require(occupied.count == 5 && occupied.delay && !buffer.can_capture());
  lead_a1::SharcMemory other_memory;
  program(other_memory);
  write(other_memory, base + 1, unknown_type);
  lead_a1::SharcVisaBuffer restored(other_memory, occupied);
  constexpr unsigned int remaining = 8;
  for (unsigned int index = 0; index < remaining; ++index) {
    const auto left = buffer.advance(true);
    const auto right = restored.advance(true);
    require(left && left == right && left->address == base + 1 + index);
    require(buffer.state() == restored.state());
  }
  require(!buffer.instruction());
  auto invalid = occupied;
  if (!invalid.delay) { throw std::logic_error("Missing owned delay snapshot"); }
  invalid.delay->address += 1;
  rejected([&] { const lead_a1::SharcVisaBuffer bad(memory, invalid); }, "continuity/delay");
  invalid = occupied;
  invalid.count = 6;
  rejected([&] { const lead_a1::SharcVisaBuffer bad(memory, invalid); }, "addresses/count");
}

}

int main() try {
  length_formats();
  delayed_capacity_and_order();
  split_instruction_and_restore();
  capture_visibility_and_failures();
  partial_read_and_bad_snapshots();
  permissions_and_full_checkpoint();
  std::cout << "VISA lengths and captured five-word IAB/delay inputs pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
