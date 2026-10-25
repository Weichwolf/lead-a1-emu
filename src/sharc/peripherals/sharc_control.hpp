// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "sharc/memory/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

struct SharcControlState {
  std::uint32_t control{};
  std::uint32_t known{};
  std::uint32_t effective{};
  std::uint32_t effective_known{};
  std::optional<std::uint32_t> delayed;
  std::optional<bool> dma_rotation{false};
  // HWR3-43: separate2-PCLK mode effects, sampled at actual half-PCLK events.
  std::array<std::optional<bool>, 4> dma_commands{};
  bool operator==(const SharcControlState &) const = default;
};

// SYSCTL word0x30024. Explicit boot-mode reference, not Nord reset evidence.
// The device outlives its Memory and Sharc bindings; one owning core/thread.
class SharcControl final : public MemoryDevice {
public:
  explicit SharcControl(std::optional<bool> valid_boot_mode);
  [[nodiscard]] SharcControlState state() const;
  [[nodiscard]] SharcControlState snapshot() const;
  [[nodiscard]] static SharcControl from_snapshot(SharcControlState initial);
  [[nodiscard]] SharcWidth data_width(std::uint32_t address) const;
  void begin_core_instruction();
  void commit_core_instruction() noexcept;
  void abort_core_instruction() noexcept;
  // Settles the supported one-cycle effect after a positive quiet internal
  // processor-clock interval (PGR A-32/A-45); no guest instruction retires.
  void advance_internal_clocks(unsigned int core_cycles);
  void advance_half_pclk(unsigned int half_periods);
  [[nodiscard]] std::optional<bool> dma_rotation() const;
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  enum class Restore : std::uint8_t { snapshot };
  SharcControl(SharcControlState initial, Restore restore);
  [[nodiscard]] SharcControlState &working();
  [[nodiscard]] const SharcControlState &working() const;
  SharcControlState current_;
  std::optional<SharcControlState> staged_;
  std::optional<std::uint32_t> due_;
  bool core_active_{};
  bool joined_{};
};

}
