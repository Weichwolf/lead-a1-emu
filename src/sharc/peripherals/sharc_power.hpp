// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"

#include <cstdint>
#include <optional>

namespace lead_a1 {

enum class SharcClockGate : std::uint32_t {
  uart = 0x1,
  twi = 0x2,
  pwm = 0x4,
  mtm = 0x8,
  dai = 0x10,
  external = 0x20,
  sport01 = 0x40,
  sport23 = 0x80,
  sport45 = 0x100,
  sport67 = 0x200,
  spi = 0x400,
  timers = 0x800,
  rtc = 0x2000,
  accelerator = 0x10000,
  mlb = 0x80000
};

enum class SharcAccelerator : std::uint8_t { fir, iir, fft };

struct SharcPowerState {
  // Unknown reset/reserved readback is not fabricated. A delivered full write
  // supplies the register value, separately from its delayed physical outputs.
  std::optional<std::uint32_t> control{std::nullopt};
  std::uint32_t shutdown{};
  std::optional<SharcAccelerator> accelerator{std::nullopt};
  unsigned int remaining_pclk{};
  bool half_elapsed{};
  bool operator==(const SharcPowerState &) const = default;
};

// HWR A-4/A-8/A-11..13, explicit 2147x/2148x PMCTL1 reference at IOP2001.
// write() is a delivered IOP access, not a model of the CCLK/PCLK bridge.
// The owner supplies actual PCLK intervals and connects each clock output to
// its peripheral. No elapsed time, DMA grant or readiness follows from reads.
class SharcPower final : public MemoryDevice {
public:
  explicit SharcPower(SharcPowerState initial = {});
  [[nodiscard]] SharcPowerState state() const;
  [[nodiscard]] SharcPowerState snapshot() const;
  [[nodiscard]] bool enabled(SharcClockGate gate) const;
  void advance_pclk(std::uint32_t clocks);
  void advance_half_pclk(std::uint32_t half_cycles);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  [[nodiscard]] SharcPowerState &working();
  [[nodiscard]] const SharcPowerState &working() const;
  SharcPowerState current_;
  std::optional<SharcPowerState> staged_;
};

}
