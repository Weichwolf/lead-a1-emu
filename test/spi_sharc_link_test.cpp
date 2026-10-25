// SPDX-License-Identifier: GPL-3.0-only
#include "arm/peripherals/gpio.hpp"
#include "arm/peripherals/spi.hpp"
#include "board/spi_sharc_link.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_boot.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/peripherals/sharc_spi_port.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t select_bit = 0x80;
constexpr std::uint32_t gpio_direction = 0x60;
constexpr std::uint32_t gpio_set = 0x68;
constexpr std::uint32_t gpio_clear = 0x6c;
constexpr std::uint32_t serial_direction = 24;
constexpr std::uint32_t serial_output = 32;
constexpr std::uint32_t clock_bit = 0x200;
constexpr std::uint32_t data_bit = 0x400;
constexpr std::uint32_t both_bits = 0x600;
constexpr std::uint32_t first_word = 0x92000;
constexpr std::uint32_t boot_end = 0x92180;
constexpr unsigned int word_bits = 32;
constexpr unsigned int boot_words = 384;
constexpr std::uint32_t mask16 = 0xffff;
constexpr unsigned int pin_count = 80;
constexpr std::uint32_t reset_word = 0x8c000;
constexpr std::uint32_t data_register = 0x3c;
constexpr std::uint32_t format_register = 0x50;
constexpr std::uint32_t function_register = 20;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SPI/SHARC connection expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Action> void rejected(const Action &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) == std::string::npos) {
      throw std::runtime_error("Expected " + reason + ": " + error.what());
    }
    return;
  }
  throw std::runtime_error("Missing SPI/SHARC rejection: " + reason);
}

struct Fixture {
  lead_a1::Spi master{{.chip_selects = 1, .previous_hold = false}};
  lead_a1::Gpio gpio{lead_a1::Gpio::reset_reference(pin_count)};
  lead_a1::SharcMemory memory;
  lead_a1::SpiSharcLink link;
  lead_a1::SharcSpiBoot boot{lead_a1::SharcSpiBoot::slave_reference_214xx()};

  explicit Fixture(bool active_low = true)
      : link(master,
             gpio,
             memory,
             {.bank = 4,
              .select_pin = 7,
              .clock_pin = 9,
              .mosi_pin = 10,
              .active_low = active_low}) {
    master.write({.address = 0, .size = 4}, 1);
    master.write({.address = serial_direction, .size = 4}, both_bits);
    master.write({.address = serial_output, .size = 4}, clock_bit);
    gpio.write({.address = gpio_direction, .size = 2}, mask16 ^ select_bit);
    gpio.write({.address = gpio_set, .size = 2}, select_bit);
  }

  void levels(bool clock, bool mosi) {
    master.write({.address = serial_output, .size = 4},
                 (clock ? clock_bit : 0) | (mosi ? data_bit : 0));
  }

  void bit(bool value) {
    levels(false, value);
    link.observe(boot);
    levels(true, value);
    link.observe(boot);
  }

  void serial() {
    constexpr std::uint32_t functions = 0xe01;
    constexpr std::uint32_t format = 0x00120210;
    constexpr std::uint32_t enabled = 0x01000003;
    master.write({.address = function_register, .size = 4}, functions);
    master.write({.address = format_register, .size = 4}, format);
    master.write({.address = 4, .size = 4}, enabled);
    gpio.write({.address = gpio_clear, .size = 2}, select_bit);
  }
};

void owned_boot_and_missing_destination() {
  Fixture fixture;
  fixture.gpio.write({.address = gpio_clear, .size = 2}, select_bit);
  fixture.link.observe(fixture.boot);
  constexpr std::uint32_t prefix = 0x81a567ef;
  for (unsigned int bit = 0; bit < word_bits - 1; ++bit) {
    fixture.bit(((prefix >> bit) & 1) != 0);
  }
  fixture.levels(false, true);
  fixture.link.observe(fixture.boot);
  fixture.levels(true, true);
  const auto before = fixture.boot.state();
  const auto master = fixture.master.state();
  const auto gpio = fixture.gpio.state();
  rejected([&] { fixture.link.observe(fixture.boot); }, "word-address=0x92000");
  require(fixture.boot.state() == before && fixture.master.state() == master &&
          fixture.gpio.state() == gpio && !fixture.memory.active());
  fixture.memory.map_2148x_5mbit_ram();
  fixture.link.observe(fixture.boot); // Same retained rising boundary, no clock advance.
  require(fixture.memory.read({.address = first_word}) == prefix &&
          fixture.boot.state().remaining == boot_words - 1);
  constexpr std::uint32_t seed = 0xa5f00000;
  for (unsigned int word = 1; word < boot_words; ++word) {
    const auto expected = seed ^ word;
    for (unsigned int bit = 0; bit < word_bits; ++bit) {
      fixture.bit(((expected >> bit) & 1) != 0);
    }
    require(fixture.memory.read({.address = first_word + word}) == expected);
  }
  require(fixture.boot.entry() == reset_word && fixture.boot.state().next_word == boot_end &&
          fixture.boot.state().bits == 0 && fixture.boot.state().remaining == 0);
  const auto loaded = fixture.boot.state();
  fixture.levels(false, false);
  rejected([&] { fixture.link.observe(fixture.boot); }, "handoff required");
  require(fixture.boot.state() == loaded);
}

void ownership_and_unknown_sources() {
  Fixture fixture;
  fixture.link.observe(fixture.boot);
  const auto before = fixture.boot.state();
  for (unsigned int owner = 0; owner < 3; ++owner) {
    if (owner == 0) { fixture.master.begin_instruction(); }
    if (owner == 1) { fixture.gpio.begin_instruction(); }
    if (owner == 2) { fixture.memory.begin_instruction(); }
    rejected([&] { fixture.link.observe(fixture.boot); }, "active");
    rejected([&] { static_cast<void>(fixture.link.input(fixture.boot, true)); }, "active");
    require(fixture.boot.state() == before);
    if (owner == 0) { fixture.master.abort_instruction(); }
    if (owner == 1) { fixture.gpio.abort_instruction(); }
    if (owner == 2) { fixture.memory.abort_instruction(); }
  }
  fixture.gpio.write({.address = gpio_direction, .size = 2}, mask16);
  rejected([&] { fixture.link.observe(fixture.boot); }, "Undriven SPI/SHARC select");
  fixture.gpio.write({.address = gpio_direction, .size = 2}, mask16 ^ select_bit);
  fixture.master.write({.address = serial_direction, .size = 4}, data_bit);
  rejected([&] { fixture.link.observe(fixture.boot); }, "Undriven SPI/SHARC clock");
  fixture.master.write({.address = serial_direction, .size = 4}, clock_bit);
  fixture.gpio.write({.address = gpio_clear, .size = 2}, select_bit);
  fixture.link.observe(fixture.boot);
  fixture.levels(false, false);
  fixture.link.observe(fixture.boot); // MOSI is unneeded on this falling edge.
  fixture.levels(true, false);
  const auto missing = fixture.boot.state();
  rejected([&] { fixture.link.observe(fixture.boot); }, "Unknown SHARC boot MOSI");
  require(fixture.boot.state() == missing);
  fixture.master.write({.address = serial_direction, .size = 4}, both_bits);
  fixture.link.observe(fixture.boot);
  require(fixture.boot.state().bits == 1 && fixture.boot.state().shift == 0);
}

void selection_and_observation_order() {
  Fixture fixture(false);
  rejected([&] { static_cast<void>(fixture.link.input(fixture.boot, {})); }, "before current");
  fixture.link.observe(fixture.boot);
  require(fixture.boot.state().selected); // Explicit active-HIGH select on bank4/pin7.
  fixture.gpio.write({.address = gpio_clear, .size = 2}, select_bit);
  rejected([&] { static_cast<void>(fixture.link.input(fixture.boot, {})); }, "before current");
  fixture.link.observe(fixture.boot);
  require(!fixture.boot.state().selected);
  fixture.levels(false, true);
  rejected([&] { static_cast<void>(fixture.link.input(fixture.boot, {})); }, "before current");
  fixture.link.observe(fixture.boot);
  for (unsigned int variation = 0; variation < 6; ++variation) {
    auto wires = fixture.link.wiring();
    switch (variation) {
      case 0: wires.bank = 9; break;
      case 1: wires.select_pin = 16; break;
      case 2: wires.bank = 5; break;
      case 3: wires.clock_pin = 16; break;
      case 4: wires.mosi_pin = 16; break;
      case 5: wires.clock_pin = wires.mosi_pin; break;
      default: throw std::logic_error("Invalid owned wire variation");
    }
    rejected(
        [&] {
          const lead_a1::SpiSharcLink invalid(fixture.master, fixture.gpio, fixture.memory, wires);
        },
        "endpoints");
  }
}

void undriven_miso_and_missing_pull() {
  for (const bool pull : {false, true}) {
    Fixture fixture;
    fixture.serial();
    fixture.link.observe(fixture.boot);
    constexpr std::uint32_t buffer = 0x40;
    fixture.master.write({.address = data_register, .size = 4}, 0);
    bool missing_checked{};
    unsigned int events{};
    constexpr unsigned int limit = 40;
    while (fixture.master.next_event() && events < limit) {
      fixture.link.observe(fixture.boot);
      if (fixture.master.samples_external_input() && !missing_checked) {
        const auto before = fixture.master.state();
        rejected([&] { fixture.master.advance(fixture.link.input(fixture.boot, {})); },
                 "Unknown SPI serial");
        require(fixture.master.state() == before);
        missing_checked = true;
      }
      fixture.master.advance(fixture.link.input(fixture.boot, pull));
      fixture.link.observe(fixture.boot);
      ++events;
    }
    require(events < limit && !fixture.master.next_event() && missing_checked &&
            fixture.master.read({.address = buffer, .size = 4}) == (pull ? mask16 : 0));
    require(fixture.boot.state().bits == 16 && fixture.boot.state().shift == 0);
  }
}

void unknown_driven_miso_and_loopback() {
  for (const bool internal : {false, true}) {
    Fixture fixture;
    fixture.serial();
    constexpr std::uint32_t loopback = 0x01010003;
    if (internal) { fixture.master.write({.address = 4, .size = 4}, loopback); }
    lead_a1::SharcSpi runtime({.clock = true, .next_word = boot_end});
    lead_a1::SharcSpiPort port(runtime);
    constexpr std::uint32_t unknown_last_mode = 0x4d10;
    constexpr std::uint32_t word = 0x1234;
    runtime.write({.address = 16, .size = 4}, 0);
    port.write({.address = 0, .size = 4}, unknown_last_mode);
    fixture.link.observe(runtime);
    fixture.master.write({.address = data_register, .size = 4}, word);
    require(!fixture.master.samples_external_input());
    fixture.master.advance(fixture.link.input(runtime, true));
    fixture.link.observe(runtime);
    if (!internal) {
      require(fixture.master.samples_external_input() && runtime.state().transmit_underflow &&
              !runtime.state().transmit_shift && runtime.state().serial.bits == 0);
      const auto before = fixture.master.state();
      const auto slave = runtime.state();
      rejected([&] { static_cast<void>(fixture.link.input(runtime, true)); },
               "driven MISO data unknown");
      require(fixture.master.state() == before && runtime.state() == slave);
      continue;
    }
    unsigned int events{};
    constexpr unsigned int limit = 40;
    while (fixture.master.next_event() && events < limit) {
      require(!fixture.master.samples_external_input());
      const auto input = fixture.link.input(runtime, {});
      require(!input.received);
      fixture.master.advance(input);
      fixture.link.observe(runtime);
      ++events;
    }
    require(events < limit && !fixture.master.next_event() &&
            fixture.master.read({.address = 0x40, .size = 4}) == word &&
            !runtime.state().transmit_underflow && runtime.state().serial.bits == 0);
  }
}

void actual_slave_return_data() {
  Fixture fixture;
  fixture.serial();
  lead_a1::SharcSpi runtime({.clock = true, .next_word = boot_end});
  lead_a1::SharcSpiPort port(runtime);
  constexpr std::uint32_t core_mode = 0x4d14;
  constexpr std::uint32_t incoming = 0xd354a913;
  constexpr std::uint32_t outgoing = 0x92345679;
  runtime.write({.address = 16, .size = 4}, 0);
  port.write({.address = 0, .size = 4}, core_mode);
  port.write({.address = 12, .size = 4}, outgoing);
  fixture.link.observe(runtime);
  rejected([&] { static_cast<void>(runtime.miso()); }, "unknown");
  runtime.begin_instruction();
  rejected([&] { static_cast<void>(fixture.link.input(runtime, true)); }, "active instruction");
  runtime.abort_instruction();
  unsigned int samples{};
  for (unsigned int half = 0; half < 2; ++half) {
    fixture.master.write({.address = data_register, .size = 4}, (incoming >> (half * 16)) & mask16);
    unsigned int events{};
    constexpr unsigned int limit = 40;
    while (fixture.master.next_event() && events < limit) {
      fixture.link.observe(runtime);
      const bool consumes = fixture.master.samples_external_input();
      const auto input = fixture.link.input(runtime, true);
      if (consumes) {
        require(samples < word_bits && input.received == (((outgoing >> samples) & 1) != 0));
        ++samples;
      } else {
        require(!input.received); // The first falling edge must not query unknown driven MISO.
      }
      fixture.master.advance(input);
      fixture.link.observe(runtime);
      ++events;
    }
    require(events < limit && !fixture.master.next_event());
    require(fixture.master.read({.address = 0x40, .size = 4}) ==
            ((outgoing >> (half * 16)) & mask16));
  }
  require(samples == word_bits && runtime.state().receive == incoming &&
          runtime.state().receive_latch == incoming && runtime.interrupt_request() &&
          !runtime.state().interrupt_pending && runtime.state().occupied == 0);
}

}

int main() try {
  owned_boot_and_missing_destination();
  ownership_and_unknown_sources();
  selection_and_observation_order();
  undriven_miso_and_missing_pull();
  unknown_driven_miso_and_loopback();
  actual_slave_return_data();
  std::cout
      << "GPIO/SPI to SHARC boot/runtime wires, explicit pulls, real MISO and rollback pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
