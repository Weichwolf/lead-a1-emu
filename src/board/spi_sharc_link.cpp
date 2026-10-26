// SPDX-License-Identifier: GPL-3.0-only
#include "board/spi_sharc_link.hpp"

#include "arm/peripherals/gpio.hpp"
#include "arm/peripherals/spi.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_boot.hpp"
#include "sharc/peripherals/sharc_spi.hpp"

#include <cstdint>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr unsigned int pins_per_bank = 16;

std::optional<bool> output(SpiPins pins, unsigned int pin) {
  const auto bit = std::uint32_t{1} << pin;
  if ((pins.enabled & bit) == 0) { return {}; }
  return (pins.high & bit) != 0;
}

SpiInput reply(SharcBootLines lines, std::optional<bool> driven, std::optional<bool> pull) {
  return {.received = driven ? driven : pull, .observed_output = lines.mosi};
}

void check_observed(const SharcBootState &state, SharcBootLines sample) {
  if (state.selected != sample.selected || state.clock != sample.clock) {
    throw std::logic_error("SPI/SHARC input before current select/clock observation");
  }
}

}

SpiSharcLink::SpiSharcLink(Spi &master, Gpio &gpio, SharcMemory &memory, SpiSharcWiring wiring)
    : master_(master), gpio_(gpio), memory_(memory), wiring_(wiring) {
  if (wiring.bank >= gpio_banks || wiring.select_pin >= pins_per_bank ||
      (wiring.bank * pins_per_bank) + wiring.select_pin >= gpio.state().pins ||
      wiring.clock_pin >= pins_per_bank || wiring.mosi_pin >= pins_per_bank ||
      wiring.clock_pin == wiring.mosi_pin) {
    throw std::invalid_argument("Invalid SPI/SHARC wire endpoints");
  }
}

SharcBootLines SpiSharcLink::lines() const {
  master_.check_external_event();
  gpio_.check_external_event();
  if (memory_.active()) {
    throw std::logic_error("SPI/SHARC sample during active DSP memory transaction");
  }
  const auto select = std::uint32_t{1} << wiring_.select_pin;
  const auto driver = gpio_.drives()[wiring_.bank];
  const auto serial = master_.pins();
  const auto driven_clock = output(serial, wiring_.clock_pin);
  const auto clock = driven_clock ? driven_clock : wiring_.clock_pull;
  if ((driver.enabled & select) == 0 || !clock) {
    std::ostringstream message;
    message << "Undriven SPI/SHARC " << ((driver.enabled & select) == 0 ? "select" : "clock")
            << " bank=" << wiring_.bank << " select-pin=" << wiring_.select_pin
            << " clock-pin=" << wiring_.clock_pin << " core-pc=0x" << std::hex << memory_.pc();
    throw std::runtime_error(message.str());
  }
  const bool high = (driver.high & select) != 0;
  return {.selected = high != wiring_.active_low,
          .clock = *clock,
          .mosi = output(serial, wiring_.mosi_pin)};
}

void SpiSharcLink::observe(SharcSpiBoot &boot) const {
  boot.observe(memory_, lines());
}

void SpiSharcLink::observe(SharcSpi &runtime) const {
  runtime.observe(memory_, lines());
}

void SpiSharcLink::check_observe(const SharcSpi &runtime) const {
  runtime.check_observe(memory_, lines());
}

SpiInput SpiSharcLink::input(const SharcSpiBoot &boot, std::optional<bool> miso_pull) const {
  const auto sample = lines();
  check_observed(boot.state(), sample);
  if (!master_.samples_external_input()) { return reply(sample, {}, {}); }
  return reply(sample, SharcSpiBoot::miso(), miso_pull);
}

SpiInput SpiSharcLink::input(const SharcSpi &runtime, std::optional<bool> miso_pull) const {
  const auto sample = lines();
  runtime.check_external_event(memory_);
  check_observed(runtime.state().serial, sample);
  if (!master_.samples_external_input()) { return reply(sample, {}, {}); }
  return reply(sample, runtime.miso(), miso_pull);
}

}
