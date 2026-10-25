// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

inline constexpr unsigned int sharc_picr_priorities = 19;
inline constexpr std::array<std::uint32_t, 4> sharc_picr_reset{
    0x0a418820, 0x16a4a0e6, 0x2307b9ac, 0x12};
using SharcPicrOutputs = std::array<std::optional<bool>, sharc_picr_priorities>;

struct SharcPicrState {
  // HWR B-17 family reset reference; not a physical Nord reset measurement.
  std::array<std::uint32_t, 4> control{sharc_picr_reset};
  // Dynamic source knowledge only. Disabled/software codes are constants.
  std::array<std::optional<bool>, 32> sources{};
  bool operator==(const SharcPicrState &) const = default;
};

// HWR2-3..5,A-14..18: PICR0..3 at native IOP2200..2203.
// Delivered-register logical mux. Physical effect/sampling/IRQ-entry latency
// belongs to the clock/core owner and is not inferred from a read or source.
class SharcPicr final : public MemoryDevice {
public:
  explicit SharcPicr(SharcPicrState initial = {});
  [[nodiscard]] SharcPicrState state() const;
  [[nodiscard]] SharcPicrState snapshot() const;
  [[nodiscard]] SharcPicrOutputs outputs() const;
  // Priority bits P0..18, not core IMASK/IRPTL/LIRPTL bit numbers.
  [[nodiscard]] std::uint32_t destinations(unsigned int encoding) const;
  void source(unsigned int encoding, std::optional<bool> value);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  [[nodiscard]] SharcPicrState &working();
  [[nodiscard]] const SharcPicrState &working() const;
  SharcPicrState current_;
  std::optional<SharcPicrState> staged_;
};

}
