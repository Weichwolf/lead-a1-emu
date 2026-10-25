// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_boot.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

struct SharcSpiState {
  SharcBootState serial;
  std::uint16_t modifier{1};
  std::uint32_t control{7};
  std::array<std::uint32_t, 4> fifo{};
  unsigned int occupied{};
  std::optional<std::uint32_t> receive;
  bool overflow{};
  // Unacknowledged peripheral event, independent of the processor's mask/latch.
  bool interrupt_pending{};
  // Remaining words in a started, explicitly arbitrated RX FIFO group.
  unsigned int burst_remaining{};
  bool operator==(const SharcSpiState &) const = default;
};

// Programmable receive DMA following the explicit 214xx SPI boot reference.
// SPICTL remains 0x4d22. Map only IOP words0x1080..0x1084 to this device.
// Serial edges and memory arbitration grants are separate owner events. One
// thread; neither event may overlap an instruction transaction. No time elapses
// on MMIO reads. Other SPI modes, register blocks and chaining remain guarded.
class SharcSpi final : public MemoryDevice {
public:
  explicit SharcSpi(const SharcBootState &completed_boot);
  [[nodiscard]] const SharcSpiState &state() const;

  [[nodiscard]] static std::optional<bool> miso() { return {}; }

  void observe(SharcMemory &memory, SharcBootLines lines);
  [[nodiscard]] unsigned int dma_request() const;
  // Services exactly one requested group (up to four words), atomically.
  // Returns zero without a request; no implicit memory/clock arbitration.
  unsigned int service_dma(SharcMemory &memory);
  // HWR3-37..40 places SPI on IOD0. After its FIFO group becomes ready,
  // a memory-bus owner requests and services each normal32 word separately.
  [[nodiscard]] std::optional<SharcMemoryRequest> dma_word_request() const;
  void service_dma_word(SharcMemoryBus &bus);
  void acknowledge_interrupt();

  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  [[nodiscard]] SharcSpiState &working();
  void external_event(const SharcMemory &memory) const;
  SharcSpiState current_;
  std::optional<SharcSpiState> staged_;
};

}
