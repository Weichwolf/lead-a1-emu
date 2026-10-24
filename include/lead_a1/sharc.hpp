// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

class SharcControl;

struct SharcKnown32 {
  std::uint32_t value{};
  std::uint32_t known{};
  bool operator==(const SharcKnown32 &) const = default;
};

struct SharcBranch {
  std::uint32_t target{};
  unsigned int slots{};
  bool operator==(const SharcBranch &) const = default;
};

struct SharcKnown40 {
  std::uint64_t value{};
  std::uint64_t known{};
  bool operator==(const SharcKnown40 &) const = default;
};

struct SharcKnown64 {
  std::uint64_t value{};
  std::uint64_t known{};
  bool operator==(const SharcKnown64 &) const = default;
};

// Private sequencing information when architectural startup-stack timing is
// unresolved. These frames are never exposed as architectural PCSTK values.
struct SharcLoopControl {
  static constexpr unsigned int capacity = 30;
  std::array<std::optional<std::uint32_t>, capacity> frames{};
  std::optional<unsigned int> depth;
  bool operator==(const SharcLoopControl &) const = default;
};

struct SharcCountLoop {
  std::uint32_t body{};
  std::uint32_t end{};
  // Functional iterations remaining, not the pipelined CURLCNTR read view.
  std::uint64_t total{};
  std::uint64_t remaining{};
  unsigned int saved_depth{};
  bool frame_live{};
  std::optional<SharcLoopControl> control;
  bool operator==(const SharcCountLoop &) const = default;
};

struct SharcFlagDrive {
  std::uint16_t high{};
  std::uint16_t low{};
  std::uint16_t high_impedance{};
  std::uint16_t unresolved{};
  bool operator==(const SharcFlagDrive &) const = default;
};

struct SharcState {
  std::uint32_t pc{};
  // UREG 0x70..0x7f. Read views and effective controls are distinct.
  std::array<SharcKnown32, 16> system{};
  std::array<SharcKnown32, 16> effective{};
  std::array<std::optional<SharcKnown32>, 16> pending{};
  // Packed FLAG0..15 input samples, independent of the output latches in FLAGS.
  SharcKnown32 flag_inputs{};
  // PEx primary/secondary, PEy primary/secondary; all reset data stays unknown.
  std::array<std::array<SharcKnown40, 16>, 4> data{};
  // Combined bus-exchange register; PX1/PX2 alias the lower/upper32 bits.
  SharcKnown64 px{};
  // Primary/secondary, then I/M/L/B, then register number.
  std::array<std::array<std::array<SharcKnown32, 16>, 4>, 2> dag{};
  // Full 26-bit frames, including CALL/loop/interrupt metadata.
  static constexpr unsigned int pc_stack_capacity = SharcLoopControl::capacity;
  std::array<std::optional<std::uint32_t>, pc_stack_capacity> pc_stack{};
  std::optional<unsigned int> depth;
  std::optional<bool> loops_empty;
  std::optional<SharcBranch> branch;
  std::optional<SharcCountLoop> count_loop;
  bool operator==(const SharcState &) const = default;
};

struct SharcStep {
  std::uint32_t pc{};
  std::uint64_t opcode{};
  std::uint32_t next_pc{};
  // Known minimum; not a complete pipeline/cache/IOP timing model.
  unsigned int minimum_core_cycles{1};
};

// Functional native-48 instruction reference; unsupported execution fails.
// No autonomous peripheral clocks, interrupt delivery or cycle timing yet.
class Sharc {
public:
  Sharc(SharcMemory &memory, SharcState initial, SharcControl *control = nullptr);
  // Documented reset registers with an explicitly supplied execution address.
  // This does not qualify the physical post-boot-DMA processor state.
  [[nodiscard]] static SharcState reset_reference_214xx(std::uint32_t entry);

  [[nodiscard]] const SharcState &state() const { return state_; }

  // Explicit core-FLAG signals, not pin mux/wiring or automatic output feedback.
  void flag_inputs(SharcKnown32 samples);
  [[nodiscard]] SharcFlagDrive flag_drives() const;

  // Explicit externally routed IRPTL maskable request. Latches even if masked;
  // enabled delivery still fails until interrupt entry/return is implemented.
  void latch_interrupt(std::uint32_t mask);

  [[nodiscard]] SharcStep step();

private:
  SharcMemory &memory_;
  SharcState state_;
  SharcControl *control_;
};

}
