// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_power.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

enum class SharcPdapPort : std::uint8_t { dai, ami };

struct SharcPdapInput {
  std::optional<bool> clock{std::nullopt};
  std::optional<bool> hold{std::nullopt};
  std::uint32_t data{}; // Low20 bits, DAI_PB20..1 or AMI_ADDR23..4.
  std::uint32_t known{};
  bool operator==(const SharcPdapInput &) const = default;
};

struct SharcPdapState {
  static constexpr std::uint32_t reset_channels = 0xffff;
  // IDP_CTL0,IDP_PP_CTL,IDP_CTL1,IDP_CTL2. W1S command bits always read0.
  std::array<std::uint32_t, 4> control{0, 0, reset_channels, 0};
  std::array<SharcPdapInput, 2> inputs{};
  std::array<std::uint32_t, 8> fifo{};
  unsigned int head{};
  unsigned int count{};
  std::uint32_t partial{};
  unsigned int samples{};
  unsigned int startup_sclk{};
  unsigned int flush_cclk{};
  bool overflow{};
  bool operator==(const SharcPdapState &) const = default;
};

struct SharcPdapInterrupts {
  bool threshold{}; // Level-sensitive IDP_FIFO_GTN_INT, DAI source8.
  bool overflow{};  // Sticky IDP_FIFO_OVR_INT, DAI source9.
  bool operator==(const SharcPdapInterrupts &) const = default;
};

// PDAP acquisition and core FIFO reference, HWR12-8..16/23..34,A-174..182.
// Map IOP24b0..24d0; unused holes/DAI_STAT1 still reject. DMA and SIP are
// explicitly unsupported; no successful receiver, data or IRQ is fabricated.
class SharcPdap final : public MemoryDevice {
public:
  explicit SharcPdap(SharcPower &power, SharcPdapState initial = {});
  [[nodiscard]] SharcPdapState state() const;
  [[nodiscard]] SharcPdapState snapshot() const;
  [[nodiscard]] SharcPdapInterrupts interrupts() const;
  // Explicit sampled signal levels. Missing unmasked data/HOLD/active-edge
  // history fail atomically; inactive ports cannot create received words.
  void observe(SharcPdapPort port, SharcPdapInput input);
  // Actual elapsed core clocks for the documented two-clock disable flush.
  void advance_cclk(std::uint32_t clocks);
  // Owner supplies completed selected input-clock periods, never guessed Hz.
  // HWR12-27 requires one SCLK after global enable before acquisition.
  void advance_sclk(std::uint32_t periods);
  // Only for a real completed DAI interrupt-latch acknowledgement.
  void acknowledge_overflow();
  [[nodiscard]] bool fifo_read_ready() const;
  [[nodiscard]] bool fifo_write_ready() const;
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  void idle() const;
  void require_clock() const;
  static void require_fifo(const SharcPdapState &state);
  void control_write(unsigned int index, std::uint32_t value);
  [[nodiscard]] SharcPdapState &working();
  [[nodiscard]] const SharcPdapState &working() const;
  SharcPower &power_;
  SharcPdapState current_;
  std::optional<SharcPdapState> staged_;
};

}
