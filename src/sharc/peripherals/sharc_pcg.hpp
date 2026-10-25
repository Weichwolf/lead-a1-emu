// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

enum class SharcPcgInput : std::uint8_t {
  clkin,
  pclk,
  external_a,
  external_b,
  external_c,
  external_d,
  trigger_a,
  trigger_b,
  trigger_c,
  trigger_d,
  strobe_a,
  strobe_b,
  strobe_c,
  strobe_d
};

struct SharcPcgLane {
  unsigned int startup{};
  unsigned int stop_half_periods{};
  std::uint32_t position{};
  bool positioned{};
  bool lost_phase{};
  bool full_frame{};
  std::optional<bool> output{std::nullopt};
  bool operator==(const SharcPcgLane &) const = default;
};

struct SharcPcgState {
  // HWR B-19 family register reset; disabled output voltage is not inferred.
  std::array<std::uint32_t, 12> registers{};
  std::array<std::optional<bool>, 14> inputs{};
  std::array<std::array<SharcPcgLane, 2>, 4> lanes{}; // clock, frame.
  std::array<bool, 4> paired{};
  std::uint8_t active{}; // Enabled or awaiting actual source-period disable.
  bool operator==(const SharcPcgState &) const = default;
};

struct SharcPcgSignals {
  std::optional<bool> clock{std::nullopt};
  std::optional<bool> frame{std::nullopt};
  bool operator==(const SharcPcgSignals &) const = default;
};

// Source-edge reference for PCG A..D, HWR15/A-189..197. Native24c0..24cb.
// Raw register delivery supplies no input edge or source frequency. Sources
// and elapsed half-periods are independent owner events, including disable.
class SharcPcg final : public MemoryDevice {
public:
  explicit SharcPcg(SharcPower &power, SharcPcgState initial = {});
  [[nodiscard]] SharcPcgState snapshot() const;
  [[nodiscard]] SharcPcgSignals signals(unsigned int unit) const;
  void observe(SharcPcgInput input, std::optional<bool> level);
  void advance_half_periods(SharcPcgInput input, std::uint32_t half_periods);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  void idle() const;
  void require_clock() const;
  [[nodiscard]] SharcPcgState &working();
  SharcPower &power_;
  SharcPcgState current_;
  std::optional<SharcPcgState> staged_;
};

}
