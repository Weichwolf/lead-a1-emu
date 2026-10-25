// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "arm/core/arm_data.hpp"
#include "arm/core/arm_mmu_internal.hpp"
#include "arm/core/arm_system.hpp"
#include "common/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace lead_a1::detail {

struct DataPolicy {
  std::optional<std::uint32_t> lockdown;
  bool enabled{};
  bool round_robin{};
};

struct DataMaintenance {
  unsigned int operation{};
  unsigned int selector{};
  std::uint32_t operand{};
};

// Functional RAM-bus reference. Bus cycles, write merging and silicon replacement
// state need separate board qualification; no asynchronous device service here.
class ArmDataPath {
public:
  ArmDataPath(Memory &memory, const std::optional<ArmSystemInitial> &initial);

  [[nodiscard]] ArmDataState state() const { return state_; }

  void begin_instruction();
  void commit_instruction();
  void abort_instruction();
  void require_cache() const;
  [[nodiscard]] std::uint64_t physical_read(MemoryAccess access,
                                            AccessKind kind = AccessKind::read);
  [[nodiscard]] std::uint64_t read(const MmuTranslation &translation, const DataPolicy &policy);
  void write(const MmuTranslation &translation, const DataPolicy &policy, std::uint64_t value);
  void drain(bool explicit_request = true);
  void invalidate_all();
  void maintain(DataMaintenance operation);

private:
  static constexpr std::size_t line_bytes = 32;
  static constexpr std::size_t half_bytes = 16;
  static constexpr std::size_t ways = 4;
  static constexpr std::size_t maximum_stores = 4;
  static constexpr std::size_t maximum_changes = 32;

  struct Line {
    std::uint32_t modified{};
    std::uint32_t physical{};
    std::array<std::uint8_t, line_bytes> bytes{};
    unsigned int dirty{};
    bool valid{};
  };

  struct Change {
    std::size_t slot{};
    Line line;
  };

  struct Buffer {
    std::array<ArmBufferedWrite, maximum_stores> stores{};
    std::size_t count{};
    std::optional<Line> writeback;
  };

  [[nodiscard]] const Line &line(std::size_t slot) const;
  void change(std::size_t slot, const Line &value);
  [[nodiscard]] const Buffer &buffer() const;
  [[nodiscard]] Buffer &mutable_buffer();
  [[nodiscard]] std::size_t index(std::uint32_t modified) const;
  [[nodiscard]] std::optional<std::size_t> lookup(std::uint32_t modified) const;
  [[nodiscard]] std::size_t allocate(std::uint32_t modified, const DataPolicy &policy) const;
  [[nodiscard]] Line fill(const MmuTranslation &translation);
  void enqueue(ArmBufferedWrite write);
  void enqueue_writeback(const Line &value);
  void clean(std::size_t slot, bool invalidate);
  void require_buffers() const;
  [[nodiscard]] bool conflicts(MemoryAccess access) const;
  Memory &memory_;
  ArmDataState state_;
  ArmDataState pending_;
  std::vector<Line> lines_;
  std::array<Change, maximum_changes> changes_{};
  std::size_t changed_{};
  Buffer buffer_;
  std::optional<Buffer> pending_buffer_;
  bool invalidate_{};
  bool active_{};
};

}
