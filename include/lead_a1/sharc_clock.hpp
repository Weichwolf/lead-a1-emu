// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/memory.hpp"

#include <cstdint>
#include <optional>

namespace lead_a1 {

struct SharcClockReference {
  std::optional<unsigned int> configuration; // Explicit CLK_CFG1:0; 3 is reserved.
  bool input_stable{};
  bool reset_settled{};
};

struct SharcClockElapsed {
  // Guaranteed lower bounds since the last event, not guessed CPU instruction
  // times. Full core/peripheral scheduling and analog transition phase are open.
  std::uint32_t core_cycles{};
  std::uint32_t clkin_cycles{};
};

struct SharcClockRatio {
  std::uint32_t numerator{};
  std::uint32_t denominator{1};
  bool operator==(const SharcClockRatio &) const = default;
};

struct SharcClockRates {
  std::optional<SharcClockRatio> core;
  std::optional<SharcClockRatio> peripheral;
  std::optional<SharcClockRatio> external;
};

struct SharcClockState {
  std::optional<unsigned int> configuration;
  bool input_stable{};
  bool reset_settled{};
  bool multiplier_known{};
  bool bypass_required{};
  std::uint32_t control{};
  std::uint32_t effective_dividers{};
  std::optional<std::uint32_t> pending_dividers;
  std::uint32_t divider_remaining{};
  std::uint32_t pll_remaining{};
  std::uint32_t bypass_reentry_remaining{};
  bool operator==(const SharcClockState &) const = default;
};

// Explicit 2147x/2148x PMCTL reference. Device offset0 maps to word0x2000.
// A read never advances time, clears a pending wait or invents a locked clock.
class SharcClock final : public MemoryDevice {
public:
  explicit SharcClock(SharcClockReference initial);
  [[nodiscard]] SharcClockState state() const;
  [[nodiscard]] SharcClockRates rates() const;
  void advance(SharcClockElapsed elapsed);
  void input_stable(bool stable);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  [[nodiscard]] SharcClockState &working();
  [[nodiscard]] const SharcClockState &working() const;
  SharcClockState current_;
  std::optional<SharcClockState> pending_;
};

}
