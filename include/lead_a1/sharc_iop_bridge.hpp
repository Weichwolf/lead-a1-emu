// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

struct SharcIopIntent {
  std::uint32_t pc{};
  std::uint32_t address{}; // Logical peripheral word address.
  AccessKind operation{AccessKind::read};
  unsigned int delivery_cycles{}; // Explicit4/5 write or7/8 read timing input.
  bool operator==(const SharcIopIntent &) const = default;
};

struct SharcIopWrite {
  SharcIopIntent intent{};
  std::uint32_t value{};
  std::uint64_t posted_at{};
  bool operator==(const SharcIopWrite &) const = default;
};

struct SharcIopRead {
  std::optional<std::uint64_t> started_at{std::nullopt};
  std::optional<std::uint32_t> response{std::nullopt};
  bool operator==(const SharcIopRead &) const = default;
};

struct SharcIopState {
  std::uint64_t clocks{};
  std::optional<std::uint64_t> last_write{std::nullopt};
  std::optional<std::uint64_t> last_read{std::nullopt};
  // Storage bound, not a claim about the physical FIFO depth.
  std::array<std::optional<SharcIopWrite>, 4> writes{};
  unsigned int write_count{};
  std::optional<SharcIopIntent> request{std::nullopt};
  std::optional<SharcIopRead> read{std::nullopt};
  bool operator==(const SharcIopState &) const = default;
};

// Peripheral control-register bridge, PGR7-7/8. Map this device into core
// memory; the separate peripheral memory maps the real target devices.
// Writes post at CPU retirement and deliver later; reads drain prior writes,
// capture real target data, and retain that response until CPU retirement.
// The owner supplies CCLK ticks and documented per-access latency inputs;
// this does not infer a physical phase, clock a CPU or emulate FIFO hangs.
class SharcIopBridge final : public MemoryDevice {
public:
  SharcIopBridge(SharcMemory &core,
                 SharcMemory &peripheral,
                 SharcIoMapping window,
                 SharcIopState initial = {});
  [[nodiscard]] SharcIopState state() const;
  [[nodiscard]] SharcIopState snapshot() const;
  void issue(SharcIopIntent intent);
  [[nodiscard]] bool ready() const;
  // One elapsed CCLK. A failed delivery retains its due request at this
  // clock; repair/retry_due supplies no extra time or second core operation.
  void tick();
  void retry_due();
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  friend class SharcIopPort;
  void idle() const;
  void validate_intent(SharcIopIntent intent) const;
  void validate_state(const SharcIopState &state) const;
  void deliver_due();
  [[nodiscard]] std::uint32_t transfer(SharcIopIntent intent, std::uint32_t value);
  [[nodiscard]] SharcIopState &retirement(MemoryAccess access, AccessKind operation);
  [[nodiscard]] SharcIopState &working();
  [[nodiscard]] const SharcIopState &working() const;
  SharcMemory &core_;
  SharcMemory &peripheral_;
  SharcIoMapping window_;
  SharcIopState current_;
  std::optional<SharcIopState> staged_;
};

// Disjoint core address windows can share the same bridge queue and clock.
// This view translates relative callback offsets only; it has no own timing,
// target data or readiness. Map each view once in the bound core memory.
class SharcIopPort final : public MemoryDevice {
public:
  SharcIopPort(SharcIopBridge &bridge, SharcIoMapping window);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  [[nodiscard]] MemoryAccess translate(MemoryAccess access) const;
  SharcIopBridge &bridge_;
  SharcIoMapping window_;
};

}
