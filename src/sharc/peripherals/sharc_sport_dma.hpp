// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

struct SharcSportDmaMode {
  bool enabled{};
  bool chained{};
  bool transmit{};
  bool external_completion{};
  bool operator==(const SharcSportDmaMode &) const = default;
};

struct SharcSportDmaWord {
  std::uint32_t value{};
  bool interrupt_after_shift{};
  bool operator==(const SharcSportDmaWord &) const = default;
};

struct SharcSportDmaLoad {
  std::uint32_t index_address{};        // CP addresses the highest TCB word, IIx.
  std::array<std::uint32_t, 4> words{}; // Captured II, IM, count, next CP.
  unsigned int received{};
  unsigned int elapsed_cclk{};
  bool operator==(const SharcSportDmaLoad &) const = default;
};

struct SharcSportDmaState {
  // HWR3-26 says parameter registers are uninitialized. Explicit reference
  // inputs may supply family reset values; the component does not guess them.
  std::optional<std::uint32_t> index{std::nullopt};
  std::optional<std::uint16_t> modifier{std::nullopt};
  std::optional<std::uint16_t> count{std::nullopt};
  std::optional<std::uint32_t> chain{std::nullopt};
  SharcSportDmaMode mode{};
  std::uint32_t remaining{};
  bool active{};
  bool insertion{};
  std::optional<SharcSportDmaLoad> load{std::nullopt};
  std::optional<SharcSportDmaWord> transmit_buffer{std::nullopt};
  std::optional<SharcSportDmaWord> transmit_shift{std::nullopt};
  std::optional<std::uint32_t> receive_buffer{std::nullopt};
  std::optional<std::uint32_t> transmit_latch{std::nullopt};
  std::optional<std::uint32_t> receive_latch{std::nullopt};
  bool interrupt_pending{};
  bool operator==(const SharcSportDmaState &) const = default;
};

// One SPORT A/B DMA channel, HWR3-4..15/28..35/45..50,11-42/51..54.
// Four relative32-bit parameter words: II,IM,count,CP. Not a SPORT-control or
// serial-clock facade. Control timing/bit framing belong to SharcSport.
// Internal transfers consume exact IO0 grants from the bound memory owner;
// chained internal TCBs need four descending reads AND26 actual CCLK. The
// owner must enforce the reported IOP-bus lock and channel arbitration.
class SharcSportDma final : public MemoryDevice {
public:
  SharcSportDma(SharcPower &power,
                SharcMemory &memory,
                unsigned int sport,
                SharcSportDmaState initial = {});
  [[nodiscard]] SharcSportDmaState state() const;
  [[nodiscard]] SharcSportDmaState snapshot() const;
  // Delivered, already synchronized SPORT control inputs, not raw SPCTL bits.
  void configure(SharcSportDmaMode mode);
  [[nodiscard]] bool locks_iop_buses() const;
  [[nodiscard]] std::optional<SharcMemoryRequest> request() const;
  void service(SharcMemoryBus &bus);
  void advance_cclk(std::uint32_t clocks);
  // Serial owner supplies complete, actually formatted32-bit buffer words.
  // Taking/finishing a TX word represents distinct shifter ownership events.
  void receive_word(std::uint32_t value);
  void transmit_word(std::uint32_t value);
  [[nodiscard]] std::uint32_t read_transmit_word() const;
  [[nodiscard]] std::uint32_t read_receive_word(bool hang_disabled = false);
  // Called only after the SPORT owner's documented six-CCLK disable flush.
  void flush_buffers();
  [[nodiscard]] std::uint32_t start_transmit();
  void finish_transmit();
  [[nodiscard]] bool interrupt_request() const;
  void acknowledge_interrupt();
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  void external_event() const;
  void require_clock() const;
  [[nodiscard]] SharcSportDmaState &working();
  [[nodiscard]] const SharcSportDmaState &working() const;
  SharcPower &power_;
  SharcMemory &memory_;
  SharcClockGate gate_;
  SharcSportDmaState current_;
  std::optional<SharcSportDmaState> staged_{std::nullopt};
};

}
