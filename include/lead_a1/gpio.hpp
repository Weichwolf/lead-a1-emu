// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/memory.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

inline constexpr unsigned int gpio_banks = 9;

struct GpioSample {
  std::uint16_t value{};
  std::uint16_t known{};
  bool operator==(const GpioSample &) const = default;
};

struct GpioBank {
  std::uint16_t direction{};
  std::uint16_t output{};
  std::uint16_t rising{};
  std::uint16_t falling{};
  std::uint16_t pending{};
  GpioSample input;
  bool operator==(const GpioBank &) const = default;
};

struct GpioState {
  unsigned int pins{};
  std::optional<std::uint32_t> revision;
  std::uint16_t bank_enabled{};
  std::array<GpioBank, gpio_banks> banks;
  bool operator==(const GpioState &) const = default;
};

struct GpioDrive {
  std::uint16_t enabled{};
  std::uint16_t high{};
  bool operator==(const GpioDrive &) const = default;
};

// Functional TI-family reference. The board owns pinmux, clocks and sampling.
class Gpio final : public MemoryDevice {
public:
  explicit Gpio(const GpioState &initial);
  [[nodiscard]] static GpioState reset_reference(unsigned int pins);
  [[nodiscard]] GpioState state() const;
  [[nodiscard]] std::array<GpioDrive, gpio_banks> drives() const;
  [[nodiscard]] std::array<bool, gpio_banks> interrupts() const;
  // Explicit synchronized pad sample, including output feedback if connected.
  void sample(unsigned int bank, GpioSample levels);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  [[nodiscard]] GpioState &working();
  [[nodiscard]] const GpioState &working() const;
  GpioState current_;
  std::optional<GpioState> pending_;
};

}
