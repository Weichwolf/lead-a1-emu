// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_power.hpp"
#include "sharc/peripherals/sharc_sport_dma.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

enum class SharcSportChannel : std::uint8_t { a, b };

struct SharcSportLines {
  std::optional<bool> clock{std::nullopt};
  std::optional<bool> frame{std::nullopt};
  std::array<std::optional<bool>, 2> data{};
  bool operator==(const SharcSportLines &) const = default;
};

struct SharcSportSignals {
  // Separate data and output-enable knowledge. Unknown is not a pull level.
  std::array<std::optional<bool>, 2> data{};
  std::array<std::optional<bool>, 2> enable{};
  bool operator==(const SharcSportSignals &) const = default;
};

struct SharcSportState {
  // HWR B-4..9 family reset reference, not identified Nord board reset pins.
  std::array<std::uint32_t, 3> control{}; // SPCTL,SPCTLN,SPMCTL raw RW bits.
  std::array<std::array<std::uint32_t, 3>, 3> sampled{};
  std::uint32_t divisor{};
  std::array<std::uint32_t, 4> select{};
  std::array<std::uint32_t, 4> compand{};
  std::array<std::array<std::uint32_t, 4>, 2> sampled_select{};
  std::array<std::array<std::uint32_t, 4>, 2> sampled_compand{};
  std::uint32_t error_control{};
  std::array<unsigned int, 2> flush{};
  std::array<std::uint32_t, 2> status{}; // One-PCLK DMA status reference.
  SharcSportLines lines{};
  SharcSportSignals signals{};
  bool core_interrupt{};
  bool frame_edge{};
  bool frame_active{};
  bool word_active{};
  bool word_selected{};
  std::optional<unsigned int> pending_frame{std::nullopt};
  unsigned int slot{};
  unsigned int bit{}; // Number of sampled bits in the current word.
  std::array<std::uint32_t, 2> shift{};
  bool operator==(const SharcSportState &) const = default;
};

struct SharcSportSnapshot {
  SharcSportState serial{};
  std::array<SharcSportDmaState, 2> dma{};
  bool operator==(const SharcSportSnapshot &) const = default;
};

// Native SPORT control and external-clock linear standard/TDM reference.
// Canonical word offsets0..12: control,secondary,multichannel,divisor,
// CS0..3,CCS0..3,error;16..23 DMA A/B;24..27 TXA/RXA/TXB/RXB.
// SharcSportPair translates the physical interleaved register aperture.
class SharcSport final : public MemoryDevice {
public:
  SharcSport(SharcPower &power,
             SharcMemory &memory,
             unsigned int sport,
             const SharcSportSnapshot &initial = {});

  [[nodiscard]] unsigned int number() const { return number_; }

  [[nodiscard]] SharcSportSnapshot snapshot() const;
  [[nodiscard]] SharcSportSignals signals() const;
  [[nodiscard]] SharcSportDma &dma(SharcSportChannel channel);
  [[nodiscard]] std::uint32_t dma_status() const;
  [[nodiscard]] bool interrupt_request() const;
  void acknowledge_interrupt();
  // Owner supplies elapsed SCLK periods separately from wire transitions.
  // No SCLK period, DMA grant or source clock follows from a register read.
  void advance_sclk(std::uint32_t periods);
  void advance_cclk(std::uint32_t clocks);
  void advance_half_pclk(std::uint32_t half_periods);
  void observe(SharcSportLines lines);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  void idle() const;
  void require_clock() const;
  [[nodiscard]] SharcSportState &working();
  [[nodiscard]] const SharcSportState &working() const;
  SharcPower &power_;
  SharcMemory &memory_;
  unsigned int number_;
  SharcClockGate gate_;
  SharcSportState current_;
  std::array<SharcSportDma, 2> dma_;
  std::optional<SharcSportState> staged_{std::nullopt};
};

// Physical pair aperture C00..C67 (0/1),400..467(2/3),800..867(4/5),
// 4800..4867(6/7). Both SPMCTL views retain the documented pair DMA bits.
class SharcSportPair final : public MemoryDevice {
public:
  SharcSportPair(SharcSport &first, SharcSport &second);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  std::array<SharcSport *, 2> ports_;
  bool active_{};
};

}
