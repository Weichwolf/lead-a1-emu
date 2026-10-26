// SPDX-License-Identifier: GPL-3.0-only
#include "arm/peripherals/gpio.hpp"
#include "arm/peripherals/spi.hpp"
#include "board/spi_sharc_bus.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/peripherals/sharc_spi_port.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr unsigned int pin_count = 80;
constexpr std::uint32_t mask16 = 0xffff;
constexpr std::uint32_t select0 = 0x80;
constexpr std::uint32_t select1 = 0x100;
constexpr std::uint32_t selects = 0x180;
constexpr std::uint32_t direction = 0x60;
constexpr std::uint32_t gpio_set = 0x68;
constexpr std::uint32_t gpio_clear = 0x6c;
constexpr std::uint32_t serial_direction = 24;
constexpr std::uint32_t serial_output = 32;
constexpr std::uint32_t clock_bit = 0x200;
constexpr std::uint32_t mosi_bit = 0x400;
constexpr std::uint32_t serial_bits = 0x600;
constexpr std::uint32_t spi_control = 4;
constexpr std::uint32_t spi_function = 20;
constexpr std::uint32_t spi_format = 0x50;
constexpr std::uint32_t spi_data = 0x3c;
constexpr std::uint32_t spi_buffer = 0x40;
constexpr std::uint32_t dma_control = 16;
constexpr std::uint32_t transmit_register = 12;
constexpr std::uint32_t core_mode = 0x4d14;
constexpr std::uint32_t unknown_last_mode = 0x4d10;
constexpr std::uint32_t destination = 0x92000;
constexpr unsigned int word_bits = 32;
constexpr unsigned int boot_words = 384;
constexpr std::array<std::uint32_t, 2> seeds{0xa5f00000, 0x12ae4000};
constexpr std::uint32_t outgoing0 = 0x92345679;
constexpr std::uint32_t outgoing1 = 0x13579bdf;
constexpr std::uint32_t incoming0 = 0xd354a913;
constexpr std::uint32_t incoming1 = 0x55aa00ff;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Shared SPI/SHARC bus expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Action> void rejected(const Action &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected " + reason + ": " + error.what());
  }
  throw std::runtime_error("Missing shared SPI rejection: " + reason);
}

struct Fixture {
  lead_a1::Spi master{{.chip_selects = 1, .previous_hold = false}};
  lead_a1::Gpio gpio{lead_a1::Gpio::reset_reference(pin_count)};
  std::array<lead_a1::SharcMemory, 2> memory;
  std::array<lead_a1::SharcSpi, 2> receivers{
      lead_a1::SharcSpi{lead_a1::SharcSpiSlaveBootReference{}},
      lead_a1::SharcSpi{lead_a1::SharcSpiSlaveBootReference{}}};
  std::array<lead_a1::SharcSpiPort, 2> ports{lead_a1::SharcSpiPort{receivers[0]},
                                             lead_a1::SharcSpiPort{receivers[1]}};
  lead_a1::SpiSharcBus bus;

  std::array<lead_a1::SpiSharcEndpoint, 2> endpoints() {
    return {
        {{.receiver = receivers.data(),
          .memory = memory.data(),
          .wiring =
              {.bank = 4, .select_pin = 7, .clock_pin = 9, .mosi_pin = 10, .active_low = true}},
         {.receiver = &receivers[1],
          .memory = &memory[1],
          .wiring = {
              .bank = 4, .select_pin = 8, .clock_pin = 9, .mosi_pin = 10, .active_low = true}}}};
  }

  Fixture() : bus(master, gpio, endpoints()) {
    master.write({.address = 0, .size = 4}, 1);
    master.write({.address = serial_direction, .size = 4}, serial_bits);
    master.write({.address = serial_output, .size = 4}, clock_bit);
    gpio.write({.address = direction, .size = 2}, mask16 ^ selects);
    gpio.write({.address = gpio_set, .size = 2}, selects);
    bus.observe();
  }

  void select(unsigned int index) {
    gpio.write({.address = gpio_set, .size = 2}, selects);
    gpio.write({.address = gpio_clear, .size = 2}, index == 0 ? select0 : select1);
    bus.observe();
  }

  void level(bool clock, bool mosi) {
    master.write({.address = serial_output, .size = 4},
                 (clock ? clock_bit : 0) | (mosi ? mosi_bit : 0));
  }

  void bit(bool value) {
    level(false, value);
    bus.observe();
    level(true, value);
    bus.observe();
  }

  void serial() {
    constexpr std::uint32_t functions = 0xe01;
    constexpr std::uint32_t format = 0x00120210;
    constexpr std::uint32_t enabled = 0x01000003;
    master.write({.address = spi_function, .size = 4}, functions);
    master.write({.address = spi_format, .size = 4}, format);
    master.write({.address = spi_control, .size = 4}, enabled);
  }

  void transmit(unsigned int index, std::uint32_t value) {
    receivers[index].write({.address = dma_control, .size = 4}, 0);
    ports[index].write({.address = 0, .size = 4}, core_mode);
    ports[index].write({.address = transmit_register, .size = 4}, value);
  }

  std::uint64_t halfword(std::uint16_t value) {
    master.write({.address = spi_data, .size = 4}, value);
    unsigned int events{};
    constexpr unsigned int event_limit = 40;
    while (master.next_event() && events < event_limit) {
      bus.observe();
      master.advance(bus.input(true));
      bus.observe();
      ++events;
    }
    require(events < event_limit && !master.next_event());
    return master.read({.address = spi_buffer, .size = 4});
  }
};

void independent_kernel_receivers() {
  Fixture fixture;
  for (unsigned int selected = 0; selected < 2; ++selected) {
    fixture.memory[selected].map_2148x_5mbit_ram();
    fixture.select(selected);
    const auto peer = fixture.receivers[1 - selected].state();
    for (unsigned int word = 0; word < boot_words; ++word) {
      const auto value = seeds[selected] ^ word;
      for (unsigned int bit = 0; bit < word_bits; ++bit) { fixture.bit(((value >> bit) & 1) != 0); }
      const auto remaining = fixture.receivers[selected].state().serial.remaining;
      require(!fixture.receivers[selected].interrupt_request());
      const auto granted = fixture.receivers[selected].service_dma(fixture.memory[selected]);
      require(granted == ((word + 1) % 4 == 0 ? 4U : 0U));
      require(fixture.receivers[selected].state().serial.remaining == remaining - granted &&
              fixture.receivers[1 - selected].state() == peer);
    }
    require(fixture.receivers[selected].state().serial.remaining == 0 &&
            fixture.receivers[selected].interrupt_request());
    for (unsigned int word = 0; word < boot_words; ++word) {
      require(fixture.memory[selected].read({.address = destination + word}) ==
              (seeds[selected] ^ word));
    }
  }
}

void late_receiver_rejection_and_retry() {
  Fixture fixture;
  fixture.gpio.write({.address = gpio_clear, .size = 2}, selects);
  fixture.bus.observe();
  for (unsigned int bit = 0; bit < word_bits - 1; ++bit) {
    fixture.bit(((incoming0 >> bit) & 1) != 0);
  }
  fixture.level(false, true);
  fixture.bus.observe();
  fixture.level(true, true);
  fixture.gpio.write({.address = gpio_set, .size = 2}, select1);
  const auto first = fixture.receivers[0].state();
  const auto second = fixture.receivers[1].state();
  rejected([&] { fixture.bus.observe(); }, "select release within runtime word");
  require(fixture.receivers[0].state() == first && fixture.receivers[1].state() == second);
  fixture.gpio.write({.address = gpio_clear, .size = 2}, select1);
  fixture.bus.observe(); // Same retained rising edge, no clock or guest instruction.
  for (const auto &receiver : fixture.receivers) {
    require(receiver.state().occupied == 1 && receiver.state().fifo[0] == incoming0 &&
            receiver.state().serial.bits == 0 && receiver.state().serial.remaining == boot_words &&
            !receiver.interrupt_request());
  }
}

void actual_selected_miso() {
  Fixture fixture;
  fixture.transmit(0, outgoing0);
  fixture.transmit(1, outgoing1);
  fixture.serial();
  constexpr std::array incoming{incoming0, incoming1};
  constexpr std::array outgoing{outgoing0, outgoing1};
  for (unsigned int selected = 0; selected < 2; ++selected) {
    fixture.select(selected);
    const auto peer = fixture.receivers[1 - selected].state();
    for (unsigned int half = 0; half < 2; ++half) {
      require(fixture.halfword(static_cast<std::uint16_t>(incoming[selected] >> (half * 16))) ==
              static_cast<std::uint16_t>(outgoing[selected] >> (half * 16)));
    }
    require(fixture.receivers[selected].state().receive == incoming[selected] &&
            fixture.receivers[1 - selected].state() == peer);
  }
}

void contention_equal_drivers_and_unknown_data() {
  constexpr std::uint32_t zero_bits = 0xaaaaaaaa;
  constexpr std::uint32_t one_bits = 0x55555555;
  for (const bool equal : {false, true}) {
    Fixture fixture;
    fixture.transmit(0, equal ? one_bits : zero_bits);
    fixture.transmit(1, one_bits);
    fixture.serial();
    fixture.gpio.write({.address = gpio_clear, .size = 2}, selects);
    fixture.bus.observe();
    fixture.master.write({.address = spi_data, .size = 4}, 0);
    require(!fixture.bus.input(true).received); // First falling launch needs no MISO sample.
    fixture.master.advance(fixture.bus.input(true));
    fixture.bus.observe();
    if (equal) {
      require(fixture.master.samples_external_input() && fixture.bus.input(false).received == true);
    } else {
      const auto master = fixture.master.state();
      const auto first = fixture.receivers[0].state();
      const auto second = fixture.receivers[1].state();
      rejected([&] { static_cast<void>(fixture.bus.input(true)); }, "MISO contention");
      require(fixture.master.state() == master && fixture.receivers[0].state() == first &&
              fixture.receivers[1].state() == second);
      fixture.gpio.write({.address = gpio_set, .size = 2}, select0);
      fixture.bus.observe();
      require(fixture.bus.input(false).received == true);
      fixture.master.advance(fixture.bus.input(false));
      fixture.bus.observe();
      require(fixture.receivers[0].state().serial.bits == 0 &&
              fixture.receivers[1].state().serial.bits == 1);
    }
  }
  Fixture fixture;
  fixture.transmit(0, outgoing0);
  fixture.receivers[1].write({.address = dma_control, .size = 4}, 0);
  fixture.ports[1].write({.address = 0, .size = 4}, unknown_last_mode);
  fixture.serial();
  fixture.gpio.write({.address = gpio_clear, .size = 2}, selects);
  fixture.bus.observe();
  fixture.master.write({.address = spi_data, .size = 4}, 0);
  fixture.master.advance(fixture.bus.input(true));
  fixture.bus.observe();
  rejected([&] { static_cast<void>(fixture.bus.input(true)); }, "driven MISO data unknown");
}

void ownership_and_topology() {
  Fixture fixture;
  fixture.select(0);
  const auto first = fixture.receivers[0].state();
  const auto second = fixture.receivers[1].state();
  fixture.receivers[1].begin_instruction();
  rejected([&] { static_cast<void>(fixture.bus.input(true)); }, "active instruction transaction");
  fixture.level(false, true);
  rejected([&] { fixture.bus.observe(); }, "active instruction transaction");
  fixture.receivers[1].abort_instruction();
  require(fixture.receivers[0].state() == first && fixture.receivers[1].state() == second);
  fixture.bus.observe();
  rejected([&] { const lead_a1::SpiSharcBus empty(fixture.master, fixture.gpio, {}); }, "Empty");
  for (unsigned int variation = 0; variation < 6; ++variation) {
    auto endpoints = fixture.endpoints();
    if (variation == 0) { endpoints[1].receiver = endpoints[0].receiver; }
    if (variation == 1) { endpoints[1].memory = nullptr; }
    if (variation == 2) { endpoints[1].wiring.clock_pin = 8; }
    if (variation == 3) { endpoints[1].wiring.mosi_pin = 11; }
    if (variation == 4) { endpoints[1].wiring.clock_pull = true; }
    if (variation == 5) { endpoints[1].receiver = nullptr; }
    rejected([&] { const lead_a1::SpiSharcBus invalid(fixture.master, fixture.gpio, endpoints); },
             "Invalid/duplicate");
  }
}

void missing_pull_and_internal_loopback() {
  constexpr std::uint16_t word = 0x1234;
  constexpr unsigned int event_limit = 40;
  for (const bool pull : {false, true}) {
    Fixture fixture;
    fixture.serial();
    fixture.select(0);
    fixture.master.write({.address = spi_data, .size = 4}, word);
    bool checked{};
    unsigned int events{};
    while (fixture.master.next_event() && events < event_limit) {
      fixture.bus.observe();
      if (fixture.master.samples_external_input() && !checked) {
        const auto before = fixture.master.state();
        rejected([&] { fixture.master.advance(fixture.bus.input({})); }, "Unknown SPI serial");
        require(fixture.master.state() == before && fixture.receivers[0].state().serial.bits == 0);
        checked = true;
      }
      fixture.master.advance(fixture.bus.input(pull));
      fixture.bus.observe();
      ++events;
    }
    require(checked && events < event_limit && !fixture.master.next_event() &&
            fixture.master.read({.address = spi_buffer, .size = 4}) == (pull ? mask16 : 0) &&
            fixture.receivers[0].state().serial.bits == 16 &&
            fixture.receivers[0].state().serial.shift == word &&
            fixture.receivers[1].state().serial.bits == 0);
  }
  Fixture fixture;
  fixture.receivers[1].write({.address = dma_control, .size = 4}, 0);
  fixture.ports[1].write({.address = 0, .size = 4}, unknown_last_mode);
  fixture.serial();
  fixture.select(1);
  constexpr std::uint32_t internal_loopback = 0x01010003;
  fixture.master.write({.address = spi_control, .size = 4}, internal_loopback);
  const auto before = fixture.receivers[1].state();
  rejected([&] { fixture.bus.observe(); }, "Undriven SPI/SHARC clock");
  auto endpoints = fixture.endpoints();
  for (auto &endpoint : endpoints) { endpoint.wiring.clock_pull = true; }
  const lead_a1::SpiSharcBus pulled(fixture.master, fixture.gpio, endpoints);
  pulled.observe();
  fixture.master.write({.address = spi_data, .size = 4}, word);
  unsigned int events{};
  while (fixture.master.next_event() && events < event_limit) {
    require(!fixture.master.samples_external_input() && !pulled.input(true).received);
    fixture.master.advance(pulled.input(true));
    pulled.observe();
    ++events;
  }
  require(events < event_limit && !fixture.master.next_event() &&
          fixture.master.read({.address = spi_buffer, .size = 4}) == word &&
          fixture.receivers[1].state() == before);
}

void driven_clock_overrides_pull() {
  for (const bool pull : {false, true}) {
    Fixture fixture;
    fixture.select(0);
    auto endpoints = fixture.endpoints();
    for (auto &endpoint : endpoints) { endpoint.wiring.clock_pull = pull; }
    const lead_a1::SpiSharcBus pulled(fixture.master, fixture.gpio, endpoints);
    pulled.observe();
    require(fixture.receivers[0].state().serial.clock == true);
    fixture.level(false, true);
    pulled.observe();
    require(fixture.receivers[0].state().serial.clock == false &&
            fixture.receivers[0].state().serial.bits == 0);
    fixture.level(true, true);
    pulled.observe();
    require(fixture.receivers[0].state().serial.clock == true &&
            fixture.receivers[0].state().serial.bits == 1 &&
            fixture.receivers[0].state().serial.shift == 1 &&
            fixture.receivers[1].state().serial.bits == 0);
  }
}

}

int main() try {
  independent_kernel_receivers();
  late_receiver_rejection_and_retry();
  actual_selected_miso();
  contention_equal_drivers_and_unknown_data();
  ownership_and_topology();
  missing_pull_and_internal_loopback();
  driven_clock_overrides_pull();
  std::cout
      << "Shared SPI selection, actual MISO resolution, atomic receivers and DMA ownership pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
