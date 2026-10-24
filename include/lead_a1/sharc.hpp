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

enum class SharcArithmeticPhase : std::uint8_t { deferred, active, draining };
enum class SharcArithmeticKind : std::uint8_t { multiple, single };

struct SharcArithmeticLoop {
  std::uint32_t body{};
  std::uint32_t end{};
  unsigned int termination{};
  unsigned int saved_depth{};
  // LADDR's type is selected at DO; a PCSTK write does not reclassify it.
  SharcArithmeticKind kind{SharcArithmeticKind::multiple};
  SharcArithmeticPhase phase{SharcArithmeticPhase::active};
  bool operator==(const SharcArithmeticLoop &) const = default;
};

struct SharcCacheState {
  // Entry bit = 2*set + way. Unknown membership is distinct from invalid.
  SharcKnown32 valid;
  SharcKnown32 lru; // Only the low16 bits exist.
  std::array<SharcKnown32, 32> addresses{};
  std::array<SharcKnown64, 32> instructions{};
  bool pending_flush{};
  bool operator==(const SharcCacheState &) const = default;
};

struct SharcFlagDrive {
  std::uint16_t high{};
  std::uint16_t low{};
  std::uint16_t high_impedance{};
  std::uint16_t unresolved{};
  bool operator==(const SharcFlagDrive &) const = default;
};

struct SharcInterruptFrame {
  std::uint32_t source{};
  std::uint32_t vector{};
  std::uint32_t return_pc{};
  unsigned int saved_depth{};
  bool operator==(const SharcInterruptFrame &) const = default;
};

struct SharcInterruptReference {
  // Explicit internal IVT base; P1I routing and the physical board remain separate.
  std::uint32_t vector_base{};
};

struct SharcInterruptStep {
  std::uint32_t source{};
  std::uint32_t return_pc{};
  std::uint32_t vector{};
  // Internal branch overhead only; no guest instruction retires here.
  unsigned int minimum_core_cycles{};
  bool operator==(const SharcInterruptStep &) const = default;
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
  std::optional<SharcArithmeticLoop> arithmetic_loop;
  SharcCacheState cache;
  std::optional<SharcInterruptFrame> interrupt;
  // Retired ordinary IDLE address; pc holds its queued resume address.
  std::optional<std::uint32_t> idle_pc;
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
// No autonomous peripheral clocks or complete physical pipeline timing.
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
  // Enabled requests require explicit qualified recognition by the owner.
  void latch_interrupt(std::uint32_t mask);

  // Explicit recognition of an already latched P1I at a qualified boundary.
  // Physical synchronization and delayed-branch/loop entry remain unqualified.
  [[nodiscard]] std::optional<SharcInterruptStep>
  service_interrupt(SharcInterruptReference reference);

  [[nodiscard]] SharcStep step();

private:
  SharcMemory &memory_;
  SharcState state_;
  SharcControl *control_;
};

}
