// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

enum class SharcDpiGroup : std::uint8_t { pin, enable };
inline constexpr unsigned int sharc_dpi_pins = 14;
inline constexpr unsigned int sharc_dpi_dynamic_sources = 49;
using SharcDpiOutputs = std::array<std::optional<bool>, sharc_dpi_pins>;

struct SharcDpiPending {
  std::uint32_t target{};
  unsigned int half_periods{};
  bool operator==(const SharcDpiPending &) const = default;
};

struct SharcDpiState {
  std::array<std::uint32_t, 3> routes{};
  std::array<std::optional<SharcDpiPending>, 3> pending{};
  // Encodings2..50. Reserved and nonexistent inputs must remain absent.
  std::array<std::optional<bool>, sharc_dpi_dynamic_sources> sources{};
  bool operator==(const SharcDpiState &) const = default;
};

struct SharcDpiDrive {
  std::uint16_t high{};
  std::uint16_t low{};
  std::uint16_t high_impedance{};
  std::uint16_t unresolved{};
  bool operator==(const SharcDpiDrive &) const = default;
};

// HWR A-213..220/B-15: independent SRU2_PIN0..2 and SRU2_PBEN0..2 instances.
// The owner supplies live sources and PCLK intervals; HWR10-33 effects take
// 2..3 PCLK. This does not infer pad feedback, source clocks or a Nord phase.
class SharcDpiRouting final : public MemoryDevice {
public:
  explicit SharcDpiRouting(SharcDpiGroup group,
                           std::optional<SharcDpiState> initial = std::nullopt);
  [[nodiscard]] SharcDpiState state() const;
  [[nodiscard]] SharcDpiState snapshot() const;
  [[nodiscard]] SharcDpiOutputs outputs() const;
  [[nodiscard]] SharcDpiDrive drives(const SharcDpiRouting &enables) const;
  void source(unsigned int encoding, std::optional<bool> value);
  void advance_pclk(std::uint32_t clocks);
  void advance_half_pclk(std::uint32_t half_periods);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  struct RegisterValue {
    unsigned int index;
    std::uint32_t value;
  };

  [[nodiscard]] bool valid_source(unsigned int encoding) const;
  void validate_word(RegisterValue word) const;
  void validate(const SharcDpiState &state) const;
  void advance(std::uint64_t half_periods);
  [[nodiscard]] SharcDpiState &working();
  [[nodiscard]] const SharcDpiState &working() const;
  SharcDpiGroup group_;
  SharcDpiState current_;
  std::optional<SharcDpiState> staged_;
};

}
