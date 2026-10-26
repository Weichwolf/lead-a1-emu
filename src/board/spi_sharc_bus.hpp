// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "arm/peripherals/gpio.hpp"
#include "arm/peripherals/spi.hpp"
#include "board/spi_sharc_link.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_spi.hpp"

#include <optional>
#include <span>
#include <vector>

namespace lead_a1 {

struct SpiSharcEndpoint {
  SharcSpi *receiver{};
  SharcMemory *memory{};
  SpiSharcWiring wiring;
};

// Explicit shared SCLK/MOSI/MISO bus; select polarity/pads belong to each endpoint.
// The caller owns devices and event boundaries on one emulation thread. No DMA,
// IRQ, clock progression, boot activation or Nord topology is supplied here.
class SpiSharcBus final {
public:
  SpiSharcBus(Spi &master, Gpio &gpio, std::span<const SpiSharcEndpoint> endpoints);
  // Validate every receiver before publishing any serial/FIFO transition.
  void observe() const;
  // Combine actual MISO drivers only on a consuming master event. Equal drives
  // resolve; conflicting or unknown drives reject. Pull applies only if all HiZ.
  [[nodiscard]] SpiInput input(std::optional<bool> miso_pull) const;

private:
  struct Endpoint {
    SharcSpi *receiver;
    SharcMemory *memory;
    SpiSharcLink link;
  };

  Spi &master_;
  std::vector<Endpoint> endpoints_;
};

}
