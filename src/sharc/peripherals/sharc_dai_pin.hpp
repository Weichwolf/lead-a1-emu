// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

class SharcDaiEnable;
class SharcPower;
inline constexpr unsigned int sharc_dai_pins = 20;
inline constexpr unsigned int sharc_dai_pin_source_slots = 84;
using SharcDaiPinOutputs = std::array<std::optional<bool>, sharc_dai_pins>;

struct SharcDaiPinPending {
  std::uint32_t target{};
  unsigned int half_periods{};
  bool operator==(const SharcDaiPinPending &) const = default;
};

struct SharcDaiPinState {
  // HWR B-14 family reset reference; these words do not establish pad levels.
  static constexpr std::array<std::uint32_t, 5> reset_routes{
      0x04c80a94, 0x04e84b96, 0x03668c98, 0x03a714a3, 0x05694f9e};
  std::array<std::uint32_t, 5> routes = reset_routes;
  std::array<std::optional<SharcDaiPinPending>, 5> pending{};
  // Numeric sources0..83. Reserved slots must remain absent; LOW/HIGH are not inputs.
  std::array<std::optional<bool>, sharc_dai_pin_source_slots> sources{};
  bool operator==(const SharcDaiPinState &) const = default;
};

struct SharcDaiPinDrive {
  std::uint32_t high{};
  std::uint32_t low{};
  std::uint32_t high_impedance{};
  std::uint32_t unresolved{};
  bool operator==(const SharcDaiPinDrive &) const = default;
};

// Group D at IOP2460..2464, HWR A-138..143/B-14. Delivered routing/inversion
// effects take2..3 PCLK. Sources and pin-buffer enable are independent inputs;
// drive composition does not invent pad feedback or physical board wiring.
class SharcDaiPin final : public MemoryDevice {
public:
  explicit SharcDaiPin(SharcPower &power, SharcDaiPinState initial = {});
  [[nodiscard]] SharcDaiPinState state() const;
  [[nodiscard]] SharcDaiPinState snapshot() const;
  [[nodiscard]] SharcDaiPinOutputs outputs() const;
  [[nodiscard]] SharcDaiPinDrive drives(const SharcDaiEnable &enables) const;
  void source(unsigned int encoding, std::optional<bool> value);
  void advance_pclk(std::uint32_t clocks);
  void advance_half_pclk(std::uint32_t half_periods);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  void require_clock() const;
  void advance(std::uint64_t half_periods);
  [[nodiscard]] SharcDaiPinState &working();
  [[nodiscard]] const SharcDaiPinState &working() const;
  SharcPower &power_;
  SharcDaiPinState current_;
  std::optional<SharcDaiPinState> staged_;
};

}
