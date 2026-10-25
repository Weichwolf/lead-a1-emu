// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "arm/peripherals/gpio.hpp"
#include "arm/peripherals/spi.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_boot.hpp"
#include "sharc/peripherals/sharc_spi.hpp"

#include <optional>

namespace lead_a1 {

struct SpiSharcWiring {
  unsigned int bank{};
  unsigned int select_pin{};
  unsigned int clock_pin{};
  unsigned int mosi_pin{};
  bool active_low{};
  bool operator==(const SpiSharcWiring &) const = default;
};

// Single-owner logical wires, not a clock source, DMA grant or Nord pinout.
// Devices outlive the connection. Observe committed control lines before input()
// and again after each actual master event. Failed observations require repair
// and retry at that same boundary, before another event or guest instruction.
class SpiSharcLink final {
public:
  SpiSharcLink(Spi &master, Gpio &gpio, SharcMemory &memory, SpiSharcWiring wiring);

  [[nodiscard]] SpiSharcWiring wiring() const { return wiring_; }

  void observe(SharcSpiBoot &boot) const;
  void observe(SharcSpi &runtime) const;
  // Returns samples for the master's next event; unused MISO is not queried.
  // An explicit weak MISO pull applies only when the actual slave is undriven.
  // Unknown driven data diagnoses in the slave; it never falls back to a pull.
  [[nodiscard]] SpiInput input(const SharcSpiBoot &boot, std::optional<bool> miso_pull) const;
  [[nodiscard]] SpiInput input(const SharcSpi &runtime, std::optional<bool> miso_pull) const;

private:
  [[nodiscard]] SharcBootLines lines() const;
  Spi &master_;
  Gpio &gpio_;
  SharcMemory &memory_;
  SpiSharcWiring wiring_;
};

}
