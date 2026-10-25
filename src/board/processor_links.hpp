// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "arm/peripherals/gpio.hpp"
#include "sharc/core/sharc.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace lead_a1 {

// Explicit logical endpoints. No Nord routing or physical package is implied.
struct ProcessorWire {
  unsigned int bank{};
  unsigned int pin{};
  unsigned int dsp{};
  unsigned int flag{};
  bool operator==(const ProcessorWire &) const = default;
};

// invalid is a reserved snapshot tag and is always rejected, not an unknown signal.
enum class WireDrive : std::uint8_t { floating, low, high, unresolved, invalid };

struct ProcessorWireLevel {
  // Driver identity bits: GPIO1, DSP2, external4. A missing level stays unknown.
  std::uint8_t high{};
  std::uint8_t low{};
  std::uint8_t unresolved{};
  std::optional<bool> level;
  bool operator==(const ProcessorWireLevel &) const = default;
};

struct ProcessorLinkState {
  std::vector<ProcessorWire> wires;
  std::vector<WireDrive> external;
  std::vector<ProcessorWireLevel> levels;
  bool sampled{};
  bool operator==(const ProcessorLinkState &) const = default;
};

// Single owning thread. The caller supplies synchronized sampling boundaries;
// this component invents neither propagation clocks nor unconnected pin levels.
class ProcessorLinks final {
public:
  ProcessorLinks(Gpio &gpio, std::span<Sharc *const> dsps, std::vector<ProcessorWire> wires);
  ProcessorLinks(Gpio &gpio, std::span<Sharc *const> dsps, ProcessorLinkState restored);
  [[nodiscard]] ProcessorLinkState snapshot() const;
  // Stages an explicit external driver; only sample() publishes pad changes.
  void external(unsigned int wire, WireDrive drive);
  // Validates every participant before publishing any samples or edge status.
  void sample();

private:
  void validate() const;
  void prepare();
  Gpio &gpio_;
  std::vector<Sharc *> dsps_;
  ProcessorLinkState state_;
  std::array<GpioSample, gpio_banks> gpio_samples_{};
  std::array<bool, gpio_banks> used_banks_{};
  std::vector<SharcKnown32> flag_samples_;
  std::vector<ProcessorWireLevel> next_levels_;
};

}
