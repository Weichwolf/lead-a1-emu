// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "sharc/memory/sharc_dma_arbiter.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/peripherals/sharc_pdap.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/peripherals/sharc_sport.hpp"

#include <array>

namespace lead_a1 {
struct SharcDmaFabricState {
  SharcMemoryBusState memory_bus;
  SharcDmaArbiterState arbiter;
  bool operator==(const SharcDmaFabricState &) const = default;
};

// Explicit single-owner internal DMA wiring reference. One tick is one supplied
// CCLK/half-PCLK; it does not advance the CPU, serial wires or source clocks.
class SharcDmaFabric {
public:
  SharcDmaFabric(SharcControl &control,
                 SharcMemory &memory,
                 SharcSpi &spi,
                 SharcPdap &pdap,
                 std::array<SharcSport *, 8> sports,
                 SharcDmaFabricState initial);
  [[nodiscard]] SharcDmaFabricState snapshot() const;

  [[nodiscard]] SharcMemoryBus &memory_bus() { return bus_; }

  void tick();
  void retry(); // Actual retained failed grant, with no extra clock or selection.

private:
  struct Candidates {
    static constexpr unsigned int capacity = 18;
    std::array<SharcDmaCandidate, capacity> values{};
    unsigned int count{};
  };

  [[nodiscard]] Candidates candidates() const;
  void service(const SharcDmaCandidate &candidate);
  void idle() const;
  SharcControl &control_;
  SharcMemory &memory_;
  SharcSpi &spi_;
  SharcPdap &pdap_;
  std::array<SharcSport *, 8> sports_;
  SharcMemoryBus bus_;
  SharcDmaArbiter arbiter_;
};
}
