// SPDX-License-Identifier: GPL-3.0-only
#include "board/spi_sharc_bus.hpp"

#include "arm/peripherals/gpio.hpp"
#include "arm/peripherals/spi.hpp"
#include "board/spi_sharc_link.hpp"

#include <algorithm>
#include <exception>
#include <ios>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {

SpiSharcBus::SpiSharcBus(Spi &master, Gpio &gpio, std::span<const SpiSharcEndpoint> endpoints)
    : master_(master) {
  if (endpoints.empty()) { throw std::invalid_argument("Empty SPI/SHARC bus"); }
  const auto common = endpoints.front().wiring;
  endpoints_.reserve(endpoints.size());
  for (const auto &endpoint : endpoints) {
    if (endpoint.receiver == nullptr || endpoint.memory == nullptr ||
        endpoint.wiring.clock_pin != common.clock_pin ||
        endpoint.wiring.clock_pull != common.clock_pull ||
        endpoint.wiring.mosi_pin != common.mosi_pin ||
        std::ranges::any_of(endpoints_, [&](const auto &existing) {
          return existing.receiver == endpoint.receiver;
        })) {
      throw std::invalid_argument("Invalid/duplicate SPI/SHARC bus endpoint or shared clock/MOSI");
    }
    endpoints_.push_back(
        Endpoint{.receiver = endpoint.receiver,
                 .memory = endpoint.memory,
                 .link = SpiSharcLink(master, gpio, *endpoint.memory, endpoint.wiring)});
  }
}

void SpiSharcBus::observe() const {
  // Each runtime observation is serial/FIFO-only. No RAM write, external event
  // or shared owner can run between these validations and publications.
  for (const auto &endpoint : endpoints_) { endpoint.link.check_observe(*endpoint.receiver); }
  for (const auto &endpoint : endpoints_) { endpoint.link.observe(*endpoint.receiver); }
}

SpiInput SpiSharcBus::input(std::optional<bool> miso_pull) const {
  SpiInput result;
  for (const auto &endpoint : endpoints_) {
    try {
      const auto input = endpoint.link.input(*endpoint.receiver, {});
      result.observed_output = input.observed_output;
      if (input.received) {
        if (result.received && result.received != input.received) {
          throw std::runtime_error("SPI/SHARC MISO contention between actual slave drivers");
        }
        result.received = input.received;
      }
    } catch (const std::exception &error) {
      std::ostringstream message;
      message << "SPI/SHARC bus input select-bank=" << endpoint.link.wiring().bank
              << " select-pin=" << endpoint.link.wiring().select_pin << " core-pc=0x" << std::hex
              << endpoint.memory->pc() << ": " << error.what();
      throw std::runtime_error(message.str());
    }
  }
  if (master_.samples_external_input() && !result.received) { result.received = miso_pull; }
  return result;
}

}
