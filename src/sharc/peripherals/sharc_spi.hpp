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

inline constexpr std::uint32_t sharc_spi_boot_control = 0x4d22;

// Explicit HWR Tables24-9/10 slave-boot peripheral profile, not detected Nord straps.
struct SharcSpiSlaveBootReference {};

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
  std::uint32_t port_control{sharc_spi_boot_control};
  std::optional<std::uint32_t> receive_latch;
  std::optional<std::uint32_t> transmit;
  std::optional<std::uint32_t> transmit_latch;
  std::optional<std::uint32_t> transmit_shift;
  unsigned int transmit_bit{};
  bool transmit_underflow{};
  bool operator==(const SharcSpiState &) const = default;
};

// SPI receive DMA with an explicit slave-boot or completed-bootstrap profile.
// This facade maps DMA words0x1080..0x1084. SharcSpiPort maps the complete
// control/DMA aperture to this same state; see its functional reference contract.
// Serial edges and memory arbitration grants are separate owner events. One
// thread; neither event may overlap an instruction transaction. No time elapses
// on MMIO reads. Other SPI modes, register blocks and chaining remain guarded.
class SharcSpi final : public MemoryDevice {
public:
  explicit SharcSpi(SharcSpiSlaveBootReference reference);
  explicit SharcSpi(const SharcBootState &completed_boot);
  [[nodiscard]] const SharcSpiState &state() const;

  [[nodiscard]] std::optional<bool> miso() const;
  [[nodiscard]] bool interrupt_request() const;
  void check_external_event(const SharcMemory &memory) const;

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
  friend class SharcSpiPort;
  [[nodiscard]] std::uint64_t port_read(MemoryAccess access);
  void port_write(MemoryAccess access, std::uint64_t value);
  [[nodiscard]] SharcSpiState &working();
  SharcSpiState current_;
  std::optional<SharcSpiState> staged_;
};

}
