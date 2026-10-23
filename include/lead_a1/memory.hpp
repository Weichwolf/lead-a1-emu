// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace lead_a1 {

enum class AccessKind : std::uint8_t { read, write, fetch };

struct MemoryAccess {
  std::uint32_t address{};
  unsigned int size{1};
};

struct MemoryMapping {
  std::uint32_t address{};
  std::size_t size{};
  std::string name;
  bool writable{true};
  bool executable{true};
};

class MemoryFault final : public std::runtime_error {
public:
  MemoryFault(AccessKind kind, MemoryAccess access, std::uint32_t pc, const std::string &reason);
};

// Device addresses are offsets within their explicitly mapped region. A device
// stages all read/write effects until commit; commit and rollback cannot fail.
class MemoryDevice {
public:
  MemoryDevice() = default;
  virtual ~MemoryDevice() = default;
  MemoryDevice(const MemoryDevice &) = delete;
  MemoryDevice &operator=(const MemoryDevice &) = delete;
  MemoryDevice(MemoryDevice &&) = delete;
  MemoryDevice &operator=(MemoryDevice &&) = delete;
  [[nodiscard]] virtual std::uint64_t read(MemoryAccess access) = 0;
  virtual void write(MemoryAccess access, std::uint64_t value) = 0;
  virtual void begin_instruction() = 0;
  virtual void commit_instruction() noexcept = 0;
  virtual void abort_instruction() noexcept = 0;
};

// No automatic mappings or zero-filled readback. Device lifetime exceeds Memory.
class Memory {
public:
  Memory() = default;
  ~Memory();
  Memory(const Memory &) = delete;
  Memory &operator=(const Memory &) = delete;
  Memory(Memory &&) = delete;
  Memory &operator=(Memory &&) = delete;
  void map(MemoryMapping mapping,
           std::span<const std::byte> initial = {},
           std::size_t initial_offset = 0);
  void map_device(MemoryMapping mapping, MemoryDevice &device);

  void set_pc(std::uint32_t pc) { pc_ = pc; }

  [[nodiscard]] std::uint64_t read(MemoryAccess access, AccessKind kind = AccessKind::read) const;
  void write(MemoryAccess access, std::uint64_t value);
  [[nodiscard]] std::optional<std::byte> peek(std::uint32_t address) const;
  void begin_instruction();
  void commit_instruction();
  void abort_instruction();

private:
  struct Region {
    MemoryMapping mapping;
    std::vector<std::byte> bytes;
    std::vector<bool> known;
  };

  struct PendingWrite {
    MemoryAccess access;
    std::uint64_t value{};
  };

  struct DeviceRegion {
    MemoryMapping mapping;
    MemoryDevice *device{};
    mutable bool joined{};
  };

  void validate_mapping(const MemoryMapping &mapping) const;
  void validate_access(MemoryAccess access, AccessKind kind) const;
  [[nodiscard]] const DeviceRegion *device_region(MemoryAccess access) const;
  void join(const DeviceRegion &entry) const;
  [[nodiscard]] const Region &region(MemoryAccess access, AccessKind kind) const;
  [[nodiscard]] std::uint8_t
  read_byte(const Region &entry, std::uint32_t address, MemoryAccess access, AccessKind kind) const;
  void apply_write(MemoryAccess access, std::uint64_t value);
  std::vector<Region> regions_;
  std::vector<DeviceRegion> devices_;
  std::vector<PendingWrite> writes_;
  std::uint32_t pc_{};
  bool active_{};
};

}
