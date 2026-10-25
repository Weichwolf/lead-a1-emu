// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_power.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

enum class SharcDaiTimingGroup : std::uint8_t { clock, frame };
inline constexpr unsigned int sharc_dai_timing_dynamic_sources = 30;

struct SharcDaiTimingTap {
  unsigned int word{};  // Register index within CLK0..5 or FS0..4.
  unsigned int field{}; // Five-bit field index, not a pin number.
};

struct SharcDaiTimingPending {
  std::uint32_t target{};
  unsigned int half_periods{}; // Elapsed half-PCLK periods,0..5.
  bool operator==(const SharcDaiTimingPending &) const = default;
};

struct SharcDaiTimingState {
  std::array<std::uint32_t, 6> routes{};
  std::array<std::optional<SharcDaiTimingPending>, 6> pending{};
  // 0..29 are external/dynamic sources; frame code27 is reserved and stays absent.
  std::array<std::optional<bool>, sharc_dai_timing_dynamic_sources> sources{};
  bool operator==(const SharcDaiTimingState &) const = default;
};

// HWR A-124..138 GroupA clock and GroupC frame routing. Map separate instances
// at2430..2435 and2450..2454. Source levels are explicit inputs; this router
// does not generate clocks, infer pad voltage or select a physical phase.
class SharcDaiTiming final : public MemoryDevice {
public:
  SharcDaiTiming(SharcPower &power,
                 SharcDaiTimingGroup group,
                 std::optional<SharcDaiTimingState> initial = std::nullopt);
  [[nodiscard]] SharcDaiTimingState state() const;
  [[nodiscard]] SharcDaiTimingState snapshot() const;
  [[nodiscard]] std::optional<bool> output(SharcDaiTimingTap tap) const;
  void source(unsigned int encoding, std::optional<bool> value);
  void advance_pclk(std::uint32_t clocks);
  void advance_half_pclk(std::uint32_t half_periods);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  struct RegisterValue {
    unsigned int index;
    std::uint32_t value;
  };

  [[nodiscard]] unsigned int register_index(MemoryAccess access) const;
  [[nodiscard]] unsigned int fields(unsigned int word) const;
  void validate(const SharcDaiTimingState &state) const;
  void validate_word(RegisterValue word) const;
  void validate_change(unsigned int word, std::uint32_t old, std::uint32_t target) const;
  void advance(std::uint64_t half_periods);
  void require_clock() const;
  [[nodiscard]] SharcDaiTimingState &working();
  [[nodiscard]] const SharcDaiTimingState &working() const;
  SharcPower &power_;
  SharcDaiTimingGroup group_;
  SharcDaiTimingState current_;
  std::optional<SharcDaiTimingState> staged_;
};

}
