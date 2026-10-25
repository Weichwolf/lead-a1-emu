// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

inline constexpr unsigned int sharc_dai_enable_pins = 20;
inline constexpr unsigned int sharc_dai_enable_dynamic_sources = 46;
using SharcDaiEnableOutputs = std::array<std::optional<bool>, sharc_dai_enable_pins>;

struct SharcDaiEnablePending {
  std::uint32_t target{};
  unsigned int elapsed_pclk{};
  bool half_elapsed{};
  bool operator==(const SharcDaiEnablePending &) const = default;
};

struct SharcDaiEnableState {
  // HWRrev1.1 B-15 explicit family reset; source/pad levels are separate inputs.
  static constexpr std::array<std::uint32_t, 4> reset_routes{
      0x0e2482ca, 0x1348d30f, 0x1a5545d6, 0x1d71f79b};
  std::array<std::uint32_t, 4> routes = reset_routes;
  std::array<std::optional<SharcDaiEnablePending>, 4> pending{};
  std::array<std::optional<bool>, sharc_dai_enable_dynamic_sources> sources{};
  bool operator==(const SharcDaiEnableState &) const = default;
};

// Group F pin-buffer-enable routing at IOP2478..247b. Delivered IOP writes
// affect routing after2..3 PCLK periods: before2 use the old route, at2 retain
// both possibilities, after3 use the new route. This bounded reference does
// not select an undocumented DAI/PCLK phase or implement other SRU groups.
class SharcDaiEnable final : public MemoryDevice {
public:
  explicit SharcDaiEnable(SharcPower &power, SharcDaiEnableState initial = {});
  [[nodiscard]] SharcDaiEnableState state() const;
  [[nodiscard]] SharcDaiEnableState snapshot() const;
  [[nodiscard]] SharcDaiEnableOutputs outputs() const;
  void source(unsigned int encoding, std::optional<bool> value);
  void advance_pclk(std::uint32_t clocks);
  void advance_half_pclk(std::uint32_t half_cycles);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  void require_clock() const;
  [[nodiscard]] SharcDaiEnableState &working();
  [[nodiscard]] const SharcDaiEnableState &working() const;
  SharcPower &power_;
  SharcDaiEnableState current_;
  std::optional<SharcDaiEnableState> staged_;
};

}
