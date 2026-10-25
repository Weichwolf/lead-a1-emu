// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"
#include "sharc/peripherals/sharc_spi.hpp"

#include <cstdint>

namespace lead_a1 {

// Map once over logical IOP1000..1084. Control/buffers and DMA share the
// existing peripheral instance and transaction. Unimplemented holes fail.
// Functional enabled LSB32/mode3 slave reference: TIMOD RX-core or RX-DMA.
// This does not qualify control-effect timing, other modes, or pad routing.
class SharcSpiPort final : public MemoryDevice {
public:
  explicit SharcSpiPort(SharcSpi &spi) : spi_(spi) {}

  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  SharcSpi &spi_;
};

}
