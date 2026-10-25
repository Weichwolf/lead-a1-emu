// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <span>

namespace lead_a1 {

class Sharc;

// Length only, not permission to execute an instruction or its reserved fields.
// An absent result means the available prefix cannot yet determine its length.
[[nodiscard]] std::optional<unsigned int> sharc_visa_words(std::span<const std::uint16_t> prefix);

struct SharcVisaInstruction {
  std::uint32_t address{};
  std::uint64_t opcode{}; // Actual 16/32/48-bit encoding, aligned at bit47.
  unsigned int words{};   // Short words; unused opcode bits are zero.
  bool operator==(const SharcVisaInstruction &) const = default;
};

struct SharcVisaBurst {
  std::uint32_t address{};
  std::array<std::uint16_t, 3> data{};
  bool operator==(const SharcVisaBurst &) const = default;
};

struct SharcVisaState {
  std::uint32_t decode{};
  std::uint32_t fetch{};
  std::array<std::uint16_t, 5> data{};
  unsigned int count{};
  std::optional<SharcVisaBurst> delay{std::nullopt};
  bool operator==(const SharcVisaState &) const = default;
};

// PGR4-7 IAB and its delay register. The owner supplies completed F2 bursts
// and Decode boundaries. This does not clock a CPU, cache or fetch sequencer.
class SharcVisaBuffer {
public:
  SharcVisaBuffer(SharcMemory &memory, SharcVisaState state);

  [[nodiscard]] const SharcVisaState &state() const { return state_; }

  [[nodiscard]] bool can_capture() const { return !state_.delay; }

  // Capture three actual executable SW16 cells at state.fetch into delay.
  // A failed read retains both buffers. Ready data is never reread from RAM.
  void capture();
  [[nodiscard]] std::optional<SharcVisaInstruction> instruction() const;
  // Optionally consume the complete current instruction, then concatenate a
  // delayed burst if the five-word IAB has room. Returns the consumed word.
  [[nodiscard]] std::optional<SharcVisaInstruction> advance(bool accept);

private:
  friend class Sharc;
  SharcMemory &memory_;
  SharcVisaState state_;
};

}
