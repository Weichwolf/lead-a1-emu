// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace lead_a1 {

enum class SharcWidth : std::uint8_t {
  short16 = 16,
  normal32 = 32,
  extended40 = 40,
  wide48 = 48,
  long64 = 64
};

struct SharcAccess {
  std::uint32_t address{};
  SharcWidth width{SharcWidth::normal32};
};

struct SharcRegion {
  unsigned int bank{};
  std::uint32_t first_halfword{};
  std::uint32_t halfwords{};
  std::string name;
  bool writable{true};
  bool executable{true};
};

struct SharcIoMapping {
  std::uint32_t first_word{};
  std::uint32_t words{};
};

struct SharcCell {
  std::uint16_t value{};
  std::uint16_t known{};
  bool operator==(const SharcCell &) const = default;
};

// Logical PM/DM/IOP addresses are words. RAM views share physical 16-bit cells.
class SharcMemory {
public:
  SharcMemory() = default;
  ~SharcMemory();
  SharcMemory(const SharcMemory &) = delete;
  SharcMemory &operator=(const SharcMemory &) = delete;
  SharcMemory(SharcMemory &&) = delete;
  SharcMemory &operator=(SharcMemory &&) = delete;
  void map(SharcRegion region, std::span<const SharcCell> initial = {});
  // Explicit candidate geometry, not physical Nord identification or reset.
  void map_2148x_5mbit_ram();
  void map_io(SharcIoMapping mapping, MemoryDevice &device);
  [[nodiscard]] std::uint64_t read(SharcAccess access, AccessKind kind = AccessKind::read) const;
  void write(SharcAccess access, std::uint64_t value);

  void set_pc(std::uint32_t pc) { pc_ = pc; }

  [[nodiscard]] std::uint32_t pc() const { return pc_; }

  [[nodiscard]] bool active() const { return active_; }

  void begin_instruction();
  void commit_instruction();
  void abort_instruction() noexcept;

private:
  friend class SharcIopBridge;

  struct Region {
    SharcRegion mapping;
    std::vector<SharcCell> cells;
  };

  struct IoRegion {
    std::uint32_t first_word;
    std::uint32_t words;
    MemoryDevice *device;
    mutable bool joined{};
  };

  struct Write {
    std::size_t region;
    std::size_t cell;
    SharcCell value;
  };

  struct Located {
    std::size_t region;
    std::size_t cell;
  };

  struct PhysicalCell {
    unsigned int bank;
    std::uint32_t index;
  };

  [[nodiscard]] Located locate(PhysicalCell position) const;
  [[nodiscard]] SharcCell cell(Located location) const;
  [[nodiscard]] const IoRegion *io(SharcAccess access) const;
  void join(const IoRegion &region) const;
  std::vector<Region> regions_;
  std::vector<IoRegion> io_;
  std::array<Write, 32> writes_{};
  std::size_t write_count_{};
  std::uint32_t pc_{};
  bool active_{};
};

}
