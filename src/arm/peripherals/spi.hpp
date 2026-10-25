// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

struct SpiResetReference {
  std::uint8_t chip_selects{};
  std::optional<bool> previous_hold;
};

struct SpiWord {
  std::uint16_t data{};
  std::uint32_t control{};
  bool operator==(const SpiWord &) const = default;
};

struct SpiReceived {
  std::uint16_t data{};
  std::uint32_t errors{};
  bool operator==(const SpiReceived &) const = default;
};

enum class SpiStage : std::uint8_t { shifting, hold, gap };

struct SpiTransfer {
  SpiWord word;
  std::uint32_t format{};
  std::uint16_t received{};
  std::uint32_t errors{};
  unsigned int edges{};
  unsigned int samples{};
  SpiStage stage{SpiStage::shifting};
  bool receive_committed{};
  bool operator==(const SpiTransfer &) const = default;
};

struct SpiState {
  std::uint8_t chip_selects{};
  bool released{};
  std::uint32_t control{};
  std::uint32_t interrupt_enable{};
  std::uint32_t interrupt_level{};
  std::uint32_t flags{};
  std::uint16_t pin_function{};
  std::uint16_t pin_direction{};
  std::uint16_t pin_output{};
  std::uint16_t pin_input{};
  std::uint16_t pin_known{};
  std::uint32_t delay{};
  std::uint8_t defaults{};
  std::array<std::uint32_t, 4> formats{};
  std::uint16_t data0{};
  std::uint32_t data1{};
  std::optional<bool> programmed_hold;
  std::optional<bool> previous_hold;
  std::optional<SpiWord> transmit;
  SpiReceived visible;
  bool visible_full{};
  std::optional<SpiReceived> receive;
  std::optional<SpiTransfer> active;
  unsigned int event_delay{};
  bool clock{};
  bool data{};
  std::uint8_t selected{};
  bool held{};
  bool transfer_started{};
  bool operator==(const SpiState &) const = default;
};

struct SpiPins {
  std::uint16_t enabled{};
  std::uint16_t high{};
  bool operator==(const SpiPins &) const = default;
};

struct SpiInput {
  std::optional<bool> received;
  std::optional<bool> observed_output;
};

// Functional master reference. Explicit reset input, single owning thread.
class Spi final : public MemoryDevice {
public:
  explicit Spi(SpiResetReference initial);
  [[nodiscard]] SpiState state() const;
  [[nodiscard]] SpiPins pins() const;
  [[nodiscard]] bool interrupt() const;
  void check_external_event() const;
  // Whether the next event actually consumes external serial pad samples.
  [[nodiscard]] bool samples_external_input() const;
  // Relative event interval in half SPI-module-clock periods; idle has none.
  [[nodiscard]] std::optional<unsigned int> next_event() const;
  // Board advances emulated time by next_event(), then supplies sampled pads.
  void advance(SpiInput input = {});
  void sample_pins(SpiPins sample);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  [[nodiscard]] SpiState &working();
  [[nodiscard]] const SpiState &working() const;
  SpiState current_;
  std::optional<SpiState> pending_;
};

}
