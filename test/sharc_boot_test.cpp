// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_boot.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/spi.hpp"

#include <algorithm>
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

constexpr std::uint32_t destination = 0x92000;
constexpr std::uint32_t reset_word = 0x8c000;
constexpr unsigned int boot_bytes = 1536;
constexpr unsigned int boot_words = 384;
constexpr unsigned int instruction_bytes = 6;
constexpr std::uint32_t spi_clock = 0x200;
constexpr std::uint32_t spi_output = 0x400;
constexpr std::uint32_t data1 = 0x3c;
constexpr std::uint32_t buffer = 0x40;
constexpr std::uint32_t hold = 0x10000000;
constexpr std::uint32_t pins = 0xe01;
constexpr std::uint32_t boot_format = 0x00120210;
constexpr std::uint32_t enabled = 0x01000003;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC boot expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC boot rejection: " + reason);
}

void bit(lead_a1::SharcSpiBoot &boot, lead_a1::SharcMemory &memory, bool incoming) {
  boot.observe(memory, {.selected = true, .clock = false, .mosi = {}});
  boot.observe(memory, {.selected = true, .clock = true, .mosi = incoming});
}

void explicit_edges_and_failure() {
  auto boot = lead_a1::SharcSpiBoot::slave_reference_214xx();
  lead_a1::SharcMemory memory;
  require(!boot.entry() && !lead_a1::SharcSpiBoot::miso() && boot.state().remaining == boot_words);
  rejected([&] { boot.observe(memory, {.selected = true, .clock = false, .mosi = {}}); },
           "idle-high");
  require(!boot.state().clock && !boot.state().selected);
  boot.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  boot.observe(memory, {.selected = true, .clock = false, .mosi = {}});
  const auto before = boot.state();
  rejected([&] { boot.observe(memory, {.selected = true, .clock = true, .mosi = {}}); },
           "Unknown SHARC boot MOSI");
  require(boot.state() == before);
  boot.observe(memory, {.selected = true, .clock = true, .mosi = true});
  // Stable clock observations never shift bits, even with missing MOSI.
  boot.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  require(boot.state().bits == 1);
  rejected([&] { boot.observe(memory, {.selected = false, .clock = true, .mosi = {}}); },
           "within boot word");
  constexpr unsigned int bits_before_last = 31;
  for (unsigned int index = 1; index < bits_before_last; ++index) { bit(boot, memory, false); }
  boot.observe(memory, {.selected = true, .clock = false, .mosi = {}});
  const auto pending = boot.state();
  rejected([&] { boot.observe(memory, {.selected = true, .clock = true, .mosi = false}); },
           "Unmapped");
  require(boot.state() == pending && !memory.active());
  memory.map_2148x_5mbit_ram();
  boot.observe(memory, {.selected = true, .clock = true, .mosi = false});
  require(memory.read({.address = destination}) == 1);
  require(boot.state().remaining == boot_words - 1 && boot.state().bits == 0);
  boot.observe(memory, {.selected = false, .clock = true, .mosi = {}});
  memory.begin_instruction();
  const auto inactive = boot.state();
  rejected([&] { boot.observe(memory, {.selected = true, .clock = true, .mosi = {}}); },
           "active core instruction");
  require(boot.state() == inactive);
  memory.abort_instruction();
  require(!boot.entry() && !lead_a1::SharcSpiBoot::miso());
}

void write(lead_a1::Spi &spi, std::uint32_t offset, std::uint32_t value) {
  spi.write({.address = offset, .size = 4}, value);
}

void connect(const lead_a1::Spi &host, lead_a1::SharcSpiBoot &boot, lead_a1::SharcMemory &memory) {
  const auto pad = host.pins();
  require((pad.enabled & (spi_clock | spi_output | 1)) == (spi_clock | spi_output | 1));
  boot.observe(memory,
               {.selected = (pad.high & 1) == 0,
                .clock = (pad.high & spi_clock) != 0,
                .mosi = (pad.high & spi_output) != 0});
}

void check_loaded(const lead_a1::SharcMemory &memory, std::span<const std::uint8_t> bytes) {
  for (std::uint32_t offset = 0; offset < boot_bytes; offset += instruction_bytes) {
    std::uint64_t expected{};
    for (unsigned int index = 0; index < instruction_bytes; ++index) {
      expected |= static_cast<std::uint64_t>(bytes[offset + index]) << (index * 8);
    }
    require(memory.read({.address = reset_word + (offset / instruction_bytes),
                         .width = lead_a1::SharcWidth::wide48},
                        lead_a1::AccessKind::fetch) == expected);
  }
  rejected([&] { static_cast<void>(memory.read({.address = destination + boot_words})); },
           "Unknown");
}

void wire_load(std::span<const std::uint8_t> bytes) {
  require(bytes.size() == boot_bytes);
  lead_a1::Spi host({.chip_selects = 1, .previous_hold = false});
  auto boot = lead_a1::SharcSpiBoot::slave_reference_214xx();
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  write(host, 0, 1);
  constexpr std::uint32_t pin_function = 0x14;
  constexpr std::uint32_t format0 = 0x50;
  write(host, pin_function, pins);
  write(host, format0, boot_format);
  write(host, 4, enabled);
  for (unsigned int offset = 0; offset < boot_bytes; offset += 2) {
    const auto halfword = static_cast<std::uint32_t>(bytes[offset]) |
                          (static_cast<std::uint32_t>(bytes[offset + 1]) << 8);
    write(host, data1, halfword | (offset + 2 < boot_bytes ? hold : 0));
    connect(host, boot, memory);
    constexpr unsigned int event_limit = 40;
    for (unsigned int event = 0; host.next_event() && event < event_limit; ++event) {
      // Explicit owned pull-up at the undriven MISO; no claimed Nord pad wiring.
      const bool transmitted = (host.pins().high & spi_output) != 0;
      host.advance({.received = true, .observed_output = transmitted});
      connect(host, boot, memory);
    }
    require(!host.next_event());
    constexpr std::uint32_t high_halfword = 0xffff;
    require(host.read({.address = buffer, .size = 4}) == high_halfword);
    const auto received = (offset + 2) / 4;
    require(boot.state().remaining == boot_words - received);
    require(boot.state().bits == ((offset + 2) % 4) * 8);
    require(boot.entry().has_value() == (offset + 2 == boot_bytes));
  }
  require(boot.entry() == reset_word && !lead_a1::SharcSpiBoot::miso());
  require(boot.state().next_word == destination + boot_words);
  check_loaded(memory, bytes);
  const auto complete = boot.state();
  boot.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  rejected([&] { boot.observe(memory, {.selected = true, .clock = false, .mosi = {}}); },
           "handoff required");
  require(boot.state().remaining == complete.remaining && boot.entry() == reset_word);
}

void owned_kernel() {
  std::array<std::uint8_t, boot_bytes> bytes{};
  // Owned non-program data covers every byte value and physical column rotation.
  constexpr unsigned int byte_mask = 0xff;
  for (unsigned int offset = 0; offset < bytes.size(); ++offset) {
    bytes[offset] = static_cast<std::uint8_t>((offset * 13 + offset / 4) & byte_mask);
  }
  wire_load(bytes);
}

void original_kernels(const std::filesystem::path &directory) {
  constexpr std::array<std::array<unsigned int, 2>, 2> offsets{
      {{0x2ab0f8, 0x2c6d78}, {0x701d8, 0x71bc8}}};
  for (unsigned int part = 0; part < offsets.size(); ++part) {
    std::ifstream file(directory / (std::to_string(part) + "-CODE.bin"), std::ios::binary);
    if (!file) { throw std::runtime_error("Cannot read local original CODE image"); }
    const std::vector<char> characters{std::istreambuf_iterator<char>(file), {}};
    std::vector<std::uint8_t> data;
    data.reserve(characters.size());
    std::ranges::transform(characters, std::back_inserter(data), [](char value) {
      return static_cast<std::uint8_t>(value);
    });
    for (const auto offset : offsets[part]) {
      require(offset <= data.size() && boot_bytes <= data.size() - offset);
      wire_load(std::span(data).subspan(offset, boot_bytes));
    }
  }
}

}

int main(int argc, char **argv) {
  try {
    if (argc == 2) {
      original_kernels(argv[1]);
    } else if (argc == 1) {
      explicit_edges_and_failure();
      owned_kernel();
    } else {
      throw std::invalid_argument("Expected optional directory of verified original images");
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
