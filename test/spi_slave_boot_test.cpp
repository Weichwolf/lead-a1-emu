// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/peripherals/sharc_spi_port.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t destination = 0x92000;
constexpr std::uint32_t kernel_base = 0x8c000;
constexpr std::uint32_t boot_control = 0x4d22;
constexpr unsigned int boot_words = 384;
constexpr unsigned int word_bits = 32;
constexpr std::uint32_t spi_control = 0x1000;
constexpr std::uint32_t spi_index = 0x1080;
constexpr std::uint32_t spi_modify = 0x1081;
constexpr std::uint32_t spi_count = 0x1082;
constexpr std::uint32_t spi_dma_control = 0x1084;
constexpr auto io0 = lead_a1::SharcMemoryMaster::io0;
constexpr auto write_kind = lead_a1::AccessKind::write;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SPI slave boot expectation at line " + std::to_string(where.line()));
  }
}

template <typename Action> void rejected(const Action &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected " + reason + ": " + error.what());
  }
  throw std::runtime_error("Missing SPI boot rejection: " + reason);
}

std::uint32_t pattern(unsigned int index) {
  constexpr std::uint32_t seed = 0xa5f00000;
  return seed ^ index;
}

struct Fixture {
  lead_a1::SharcSpi spi{lead_a1::SharcSpiSlaveBootReference{}};
  lead_a1::SharcSpiPort port{spi};
  lead_a1::SharcMemory memory;
  lead_a1::SharcMemoryBus bus{memory, lead_a1::SharcPeripheralPhase::active};

  explicit Fixture(bool mapped = true) {
    if (mapped) { memory.map_2148x_5mbit_ram(); }
    constexpr unsigned int register_words = 0x85;
    memory.map_io({.first_word = spi_control, .words = register_words}, port);
  }

  void word(std::uint32_t value) {
    spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
    for (unsigned int bit = 0; bit < word_bits; ++bit) {
      spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
      spi.observe(memory, {.selected = true, .clock = true, .mosi = ((value >> bit) & 1) != 0});
    }
  }

  void request() {
    const auto request = spi.dma_word_request();
    if (!request) { throw std::logic_error("Missing actual SPI boot DMA request"); }
    bus.request(io0, *request);
    bus.advance();
    if (!bus.grant(io0)) { bus.advance(); }
    require(bus.grant(io0) == request);
  }

  void grant() {
    request();
    spi.service_dma_word(bus);
  }
};

void reset_profile_and_real_requests() {
  Fixture fixture;
  const auto before = fixture.spi.state();
  require(!before.serial.clock && !before.serial.selected && before.serial.bits == 0 &&
          before.serial.shift == 0 && before.serial.remaining == boot_words &&
          before.serial.next_word == destination && before.modifier == 1 && before.control == 7 &&
          before.port_control == boot_control && before.occupied == 0 && !before.receive &&
          !before.receive_latch && !before.interrupt_pending && !before.transmit &&
          !before.transmit_latch && !before.transmit_shift);
  constexpr std::uint32_t busy = 0x8007;
  require(fixture.memory.read({.address = spi_control}) == boot_control &&
          fixture.memory.read({.address = spi_modify}) == 1 &&
          fixture.memory.read({.address = spi_count}) == boot_words &&
          fixture.memory.read({.address = spi_dma_control}) == busy &&
          fixture.spi.state() == before);
  require(fixture.spi.dma_request() == 0 && !fixture.spi.dma_word_request() &&
          !fixture.spi.interrupt_request() && !fixture.spi.miso());
  rejected([&] { const lead_a1::SharcSpi invalid(before.serial); }, "completed initial boot");
  rejected([&] { fixture.memory.write({.address = spi_index}, destination + 1); },
           "active SPI DMA parameter");
  require(fixture.spi.state() == before && !fixture.memory.active());
  rejected(
      [&] { fixture.spi.observe(fixture.memory, {.selected = true, .clock = false, .mosi = {}}); },
      "idle-high");
  require(fixture.spi.state() == before);
  unsigned int written{};
  for (unsigned int word = 0; word < boot_words; ++word) {
    fixture.word(pattern(word));
    require(fixture.spi.state().serial.remaining == boot_words - written &&
            !fixture.spi.state().interrupt_pending);
    if ((word + 1) % 4 != 0) {
      require(fixture.spi.dma_request() == 0 && !fixture.spi.dma_word_request());
      continue;
    }
    require(fixture.spi.dma_request() == 4);
    for (unsigned int member = 0; member < 4; ++member) {
      const auto request = fixture.spi.dma_word_request();
      require(request == (lead_a1::SharcMemoryRequest{.access = {.address = destination + written},
                                                      .kind = write_kind,
                                                      .value = pattern(written)}));
      fixture.grant();
      require(fixture.memory.read({.address = destination + written}) == pattern(written));
      ++written;
      require(fixture.spi.state().serial.remaining == boot_words - written &&
              fixture.spi.state().interrupt_pending == (written == boot_words));
    }
  }
  constexpr std::uint64_t expected_clocks = 767;
  require(written == boot_words && fixture.bus.state().clocks == expected_clocks &&
          fixture.spi.state().serial.next_word == destination + boot_words &&
          fixture.spi.state().occupied == 0 && !fixture.spi.state().receive &&
          fixture.spi.state().burst_remaining == 0 && fixture.spi.interrupt_request());
  for (unsigned int pair = 0; pair < boot_words / 3; ++pair) {
    const auto first = std::uint64_t{pattern(pair * 3)};
    const auto second = std::uint64_t{pattern((pair * 3) + 1)};
    const auto third = std::uint64_t{pattern((pair * 3) + 2)};
    constexpr std::uint64_t low16 = 0xffff;
    const auto address = kernel_base + (pair * 2);
    require(fixture.memory.read({.address = address, .width = lead_a1::SharcWidth::wide48}) ==
                (first | ((second & low16) << 32)) &&
            fixture.memory.read({.address = address + 1, .width = lead_a1::SharcWidth::wide48}) ==
                ((second >> 16) | (third << 16)));
  }
  // The same mapped port retains the actual event until its owner consumes it.
  require(fixture.memory.read({.address = spi_dma_control}) == 7 &&
          fixture.spi.interrupt_request());
  fixture.spi.acknowledge_interrupt();
  require(!fixture.spi.interrupt_request());
  fixture.memory.write({.address = spi_dma_control}, 0);
  fixture.memory.write({.address = spi_index}, destination + 3);
  fixture.memory.write({.address = spi_modify}, 1);
  fixture.memory.write({.address = spi_count}, 3);
  fixture.memory.write({.address = spi_dma_control}, 7);
  for (unsigned int word = 0; word < 3; ++word) { fixture.word(pattern(boot_words + word)); }
  require(fixture.spi.dma_request() == 3 && !fixture.spi.interrupt_request());
  for (unsigned int word = 0; word < 3; ++word) {
    fixture.grant();
    require(fixture.memory.read({.address = destination + 3 + word}) == pattern(boot_words + word));
    require(fixture.spi.interrupt_request() == (word == 2));
  }
  require(fixture.spi.state().serial.remaining == 0 &&
          fixture.spi.state().port_control == boot_control);
}

void missing_ram_and_no_time_retry() {
  Fixture fixture(false);
  for (unsigned int word = 0; word < 4; ++word) { fixture.word(pattern(word)); }
  fixture.request();
  const auto before = fixture.spi.state();
  const auto bus = fixture.bus.state();
  rejected([&] { fixture.spi.service_dma_word(fixture.bus); }, "word-address=0x92000");
  require(fixture.spi.state() == before && fixture.bus.state() == bus && !fixture.memory.active());
  fixture.memory.map_2148x_5mbit_ram();
  fixture.spi.service_dma_word(fixture.bus);
  require(fixture.memory.read({.address = destination}) == pattern(0) &&
          fixture.spi.state().serial.remaining == boot_words - 1 &&
          fixture.spi.state().burst_remaining == 3 && fixture.spi.state().occupied == 3 &&
          fixture.bus.state().clocks == bus.clocks && !fixture.spi.interrupt_request());
}

void no_implicit_grants_or_overflow_success() {
  Fixture fixture;
  for (unsigned int word = 0; word < 6; ++word) { fixture.word(pattern(word)); }
  require(fixture.spi.state().serial.remaining == boot_words && fixture.spi.state().occupied == 4 &&
          fixture.spi.state().receive == pattern(4) && fixture.spi.state().overflow &&
          !fixture.spi.interrupt_request());
  rejected([&] { static_cast<void>(fixture.memory.read({.address = destination})); }, "Unknown");
  for (unsigned int word = 0; word < 4; ++word) {
    fixture.grant();
    require(fixture.memory.read({.address = destination + word}) == pattern(word));
  }
  require(fixture.spi.state().serial.remaining == boot_words - 4 &&
          fixture.spi.state().occupied == 1 && !fixture.spi.state().receive &&
          fixture.spi.state().fifo[0] == pattern(4) && fixture.spi.dma_request() == 0 &&
          !fixture.spi.interrupt_request());
}

}

int main() try {
  reset_profile_and_real_requests();
  missing_ram_and_no_time_retry();
  no_implicit_grants_or_overflow_success();
  std::cout << "One SPI slave-boot port, real FIFO/word grants, completion event and continued DMA "
               "pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
