// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

class SharcPower;
inline constexpr unsigned int sharc_dai_data_destinations = 34;
inline constexpr unsigned int sharc_dai_data_source_slots = 49;
using SharcDaiDataOutputs = std::array<std::optional<bool>, sharc_dai_data_destinations>;

struct SharcDaiDataPending {
  std::uint32_t target{};
  unsigned int half_periods{};
  bool operator==(const SharcDaiDataPending &) const = default;
};

struct SharcDaiDataState {
  // HWR B-14 family reset reference; these words do not establish source levels.
  static constexpr std::array<std::uint32_t, 7> reset_routes{
      0x08144040, 0x0f38b289, 0x00000450, 0, 0, 0, 0x00fbefbe};
  std::array<std::uint32_t, 7> routes = reset_routes;
  std::array<std::optional<SharcDaiDataPending>, 7> pending{};
  // Numeric sources0..48. Reserved slots must remain absent; LOW/HIGH are not inputs.
  std::array<std::optional<bool>, sharc_dai_data_source_slots> sources{};
  bool operator==(const SharcDaiDataState &) const = default;
};

// Group B at IOP2440..2446, HWR A-129..134/B-14. Six words hold five
// six-bit selectors each; DAT6 holds four. Delivered effects take2..3 PCLK.
// Optional source levels do not invent SPORT samples, clocks or pad feedback.
class SharcDaiData final : public MemoryDevice {
public:
  explicit SharcDaiData(SharcPower &power, SharcDaiDataState initial = {});
  [[nodiscard]] SharcDaiDataState state() const;
  [[nodiscard]] SharcDaiDataState snapshot() const;
  [[nodiscard]] SharcDaiDataOutputs outputs() const;
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
  [[nodiscard]] SharcDaiDataState &working();
  [[nodiscard]] const SharcDaiDataState &working() const;
  SharcPower &power_;
  SharcDaiDataState current_;
  std::optional<SharcDaiDataState> staged_;
};

}
