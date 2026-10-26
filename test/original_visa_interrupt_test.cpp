// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/visa/sharc_visa.hpp"

#include <array>
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

constexpr std::uint32_t ivt = 0x8c000;
constexpr std::uint32_t vector_pc = 0x8c044;
constexpr std::uint32_t foreground = 0x180400;
constexpr std::uint32_t expected_frame = 0x2180400;
constexpr unsigned int clock_limit = 32;
constexpr unsigned int instruction_bytes = 6;
constexpr unsigned int class_shift = 40;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t operand_mask = 0xffffff;
constexpr std::uint64_t jump_opcode = 0x063e00000000;
constexpr std::uint64_t compact_rti_opcode = 0x0bbe00000000;

struct Placement {
  unsigned int part;
  unsigned int transport;
  std::uint32_t vector_offset;
  std::uint32_t target;
  std::uint32_t target_offset;
  std::uint32_t missing_data;
};

constexpr std::array placements{Placement{.part = 0,
                                          .transport = 0,
                                          .vector_offset = 0x2e26d0,
                                          .target = 0x1c5b8b,
                                          .target_offset = 0x2e130a,
                                          .missing_data = 0},
                                Placement{.part = 0,
                                          .transport = 1,
                                          .vector_offset = 0x2c690c,
                                          .target = 0x1c47da,
                                          .target_offset = 0x2c2e62,
                                          .missing_data = 0x93fe1},
                                Placement{.part = 1,
                                          .transport = 0,
                                          .vector_offset = 0x74434,
                                          .target = 0x1c0752,
                                          .target_offset = 0x73a48,
                                          .missing_data = 0},
                                Placement{.part = 1,
                                          .transport = 1,
                                          .vector_offset = 0x7175c,
                                          .target = 0x1c0595,
                                          .target_offset = 0x70c5c,
                                          .missing_data = 0x924f9}};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Original VISA ISR expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Optional> auto &present(Optional &value) {
  if (!value) { throw std::runtime_error("Missing captured original VISA ISR input"); }
  return *value;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = foreground};
  return state;
}

std::vector<char> read_image(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) { throw std::runtime_error("Cannot open original CODE: " + path.string()); }
  return {std::istreambuf_iterator<char>(input), {}};
}

std::uint64_t little(std::span<const char> image, unsigned int offset, unsigned int bytes) {
  require(bytes > 0 && bytes <= instruction_bytes && offset <= image.size() &&
          bytes <= image.size() - offset);
  std::uint64_t value{};
  for (unsigned int byte = 0; byte < bytes; ++byte) {
    value |= std::uint64_t{static_cast<unsigned char>(image[offset + byte])} << (byte * 8);
  }
  return value;
}

void load(lead_a1::SharcMemory &memory, std::span<const char> image, Placement placement) {
  memory.map_2148x_5mbit_ram();
  for (unsigned int index = 0; index < 4; ++index) {
    memory.write(
        {.address = vector_pc + index, .width = lead_a1::SharcWidth::wide48},
        little(image, placement.vector_offset + (index * instruction_bytes), instruction_bytes));
  }
  constexpr unsigned int short_words = 32;
  for (unsigned int index = 0; index < short_words; ++index) {
    memory.write({.address = placement.target + index, .width = lead_a1::SharcWidth::short16},
                 little(image, placement.target_offset + (index * 2), 2));
  }
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(foreground);
  state.system[2].value = 0x1000;
  state.system[13].value =
      0x400; // Explicit P6I-enabled test input, not recovered application state.
  state.effective = state.system;
  return state;
}

lead_a1::SharcStep retire(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  for (unsigned int clock = 0; clock < clock_limit; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (fetch.state().execute) { return cpu.step(fetch); }
  }
  throw std::runtime_error("Missing original ISA/VISA ISR retirement");
}

void missing_input(lead_a1::Sharc &cpu,
                   lead_a1::SharcFetch &fetch,
                   lead_a1::SharcVisaBuffer &input,
                   lead_a1::SharcMemory &memory,
                   lead_a1::SharcControl &control,
                   Placement placement,
                   bool clocked) {
  if (clocked) {
    for (unsigned int clock = 0; clock < clock_limit && !fetch.state().execute; ++clock) {
      static_cast<void>(fetch.advance_sequenced(cpu));
    }
    require(fetch.state().execute && present(fetch.state().execute).address == placement.target);
  } else {
    input.capture();
    static_cast<void>(input.advance(false));
  }
  const auto core = cpu.state();
  const auto transport = fetch.state();
  const auto buffer = input.state();
  const auto controller = control.snapshot();
  bool failed{};
  try {
    if (clocked) {
      static_cast<void>(cpu.step(fetch));
    } else {
      static_cast<void>(cpu.step(input));
    }
  } catch (const std::exception &error) {
    const std::string diagnostic(error.what());
    if (diagnostic.find("Unknown SHARC memory bits") == std::string::npos) {
      throw std::runtime_error("Expected the original ISR's missing DM input: " + diagnostic);
    }
    failed = true;
  }
  require(failed && cpu.state() == core && fetch.state() == transport && input.state() == buffer &&
          cpu.state().pc == placement.target && cpu.state().interrupt && cpu.state().depth == 1 &&
          control.snapshot() == controller && !memory.active());
  const auto instruction = clocked ? present(fetch.state().execute).visa : input.instruction();
  require(instruction && instruction->words == 3 && (instruction->opcode >> class_shift) == 16 &&
          static_cast<std::uint32_t>(instruction->opcode) == placement.missing_data);
}

void check(std::span<const char> image, Placement placement, bool clocked) {
  lead_a1::SharcMemory memory;
  load(memory, image, placement);
  lead_a1::SharcControl control(true); // Explicit family SYSCTL=4 reference input.
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  cpu.latch_programmable_interrupt(6);
  const auto entry = clocked ? cpu.service_interrupt(fetch, {.vector_base = ivt})
                             : cpu.service_interrupt({.vector_base = ivt});
  require(entry && entry->source == 1 && entry->vector == vector_pc &&
          entry->return_pc == foreground && entry->bank == lead_a1::SharcInterruptBank::lirptl &&
          cpu.state().pc_stack[0] == expected_frame);
  const auto jump = clocked ? retire(cpu, fetch) : cpu.step();
  require(jump.pc == vector_pc && jump.address_units == 1 && jump.next_pc == placement.target &&
          (jump.opcode & ~operand_mask) == jump_opcode);
  lead_a1::SharcVisaBuffer input(memory, {.decode = placement.target, .fetch = placement.target});
  if (placement.missing_data != 0) {
    missing_input(cpu, fetch, input, memory, control, placement, clocked);
    return;
  }
  if (!clocked) {
    input.capture();
    static_cast<void>(input.advance(false));
  }
  const auto returned = clocked ? retire(cpu, fetch) : cpu.step(input);
  require(returned.pc == placement.target && returned.address_units == 1 &&
          returned.opcode == compact_rti_opcode && returned.next_pc == foreground &&
          !cpu.state().interrupt && cpu.state().depth == 0 &&
          cpu.state().pc_stack[0] == expected_frame &&
          cpu.state().system[12] == (lead_a1::SharcKnown32{.known = full32}) &&
          cpu.state().system[13].value == 0x400);
}

}

int main(int argc, char **argv) try {
  if (argc != 2) { throw std::runtime_error("Usage: original-visa-interrupt-test images"); }
  for (const auto placement : placements) {
    const auto image =
        read_image(std::filesystem::path(argv[1]) / (std::to_string(placement.part) + "-CODE.bin"));
    for (const bool clocked : {false, true}) { check(image, placement, clocked); }
    std::cout << "Part " << placement.part << " transport " << placement.transport
              << ": original P6I vector/VISA entry; "
              << (placement.missing_data == 0 ? "actual compact RTI returned"
                                              : "missing DM input rejected")
              << "; explicit test IRQ controls, no boot/physical routing claim\n";
  }
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
