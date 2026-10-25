// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_control.hpp"

#include <cstdint>
#include <optional>
#include <span>

namespace lead_a1 {

enum class SharcDmaGroup : std::uint8_t {
  sport01,
  sport23,
  sport45,
  idp,
  spi,
  mlb,
  spib,
  memory_write,
  memory_read,
  uart,
  link,
  sport67,
  accelerator_in,
  accelerator_out
};

struct SharcDmaCandidate {
  SharcDmaGroup group{};
  unsigned int channel{}; // HWR3-39..41 ordinal, not physical SPORT number.
  SharcMemoryRequest request;
  bool operator==(const SharcDmaCandidate &) const = default;
};

struct SharcDmaTransfer {
  SharcDmaCandidate candidate;
  unsigned int remaining_half_pclk{2}; // Explicit next-PCLK launch reference.
  bool submitted{};
  bool operator==(const SharcDmaTransfer &) const = default;
};

struct SharcDmaArbiterState {
  static constexpr std::uint16_t unknown_heads = 0x3fff;
  // Every possible next cyclic group. Reset/mode-transition pointer phase is
  // not guessed. An explicit owned reference may supply a single bit.
  std::uint16_t heads{unknown_heads};
  std::optional<bool> mode{std::nullopt};
  std::optional<SharcDmaTransfer> transfer{std::nullopt};
  bool operator==(const SharcDmaArbiterState &) const = default;
};

// HWR3-31/37..43 two-stage2148x normal32 IOD0 reference. Stage1 is fixed
// for SPORT pairs/IDP/UART. MLB/link requesters and external SPEP are guarded.
// A grant captures a real producer request. Its memory launch, peripheral
// service and retirement remain distinct owner events; no data is fabricated.
class SharcDmaArbiter {
public:
  SharcDmaArbiter(SharcControl &control, SharcMemory &memory, SharcDmaArbiterState initial = {});
  [[nodiscard]] SharcDmaArbiterState snapshot() const;
  [[nodiscard]] std::optional<SharcDmaCandidate>
  select(std::span<const SharcDmaCandidate> requests);
  void advance_half_pclk(unsigned int half_periods);
  void launch(SharcMemoryBus &bus);
  // Only after the selected real peripheral has consumed its exact IOD0 grant.
  void retire(SharcMemoryBus &bus);

private:
  void idle() const;
  SharcControl &control_;
  SharcMemory &memory_;
  SharcDmaArbiterState current_;
};
}
