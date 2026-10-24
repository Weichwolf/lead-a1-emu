// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <cstdint>
#include <optional>

namespace lead_a1 {

struct SharcFetchWord {
  std::uint32_t address{};
  std::uint64_t opcode{};
  bool operator==(const SharcFetchWord &) const = default;
};

struct SharcFetchState {
  std::optional<std::uint32_t> fetch1;
  std::optional<SharcFetchWord> fetch2;
  std::optional<SharcFetchWord> decode;
  std::optional<SharcFetchWord> address;
  std::optional<SharcFetchWord> execute;
  bool operator==(const SharcFetchState &) const = default;
};

// Native internal instruction-bus transport, PGR4-5/8. The sequencer supplies
// F1 addresses and grants advancing clocks. Cache must explicitly be disabled.
// DAG/data arbitration, control interlocks and branch/IRQ redirects are separate.
class SharcFetch {
public:
  explicit SharcFetch(SharcMemory &memory, SharcFetchState initial = {});

  [[nodiscard]] const SharcFetchState &state() const { return state_; }

  // One granted transport clock. nullopt injects a bubble, never a guest NOP.
  // Captures the previous F1 word in F2; later phases retain its actual data.
  // An occupied Execute phase must retire before another advancing clock.
  void advance(SharcKnown32 effective_mode2, std::optional<std::uint32_t> next_address);

private:
  friend class Sharc;
  [[nodiscard]] std::uint64_t for_execution(const SharcMemory &memory, std::uint32_t pc) const;
  SharcMemory &memory_;
  SharcFetchState state_;
};

}
