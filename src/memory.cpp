// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <ios>
#include <limits>
#include <optional>
#include <ranges>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace lead_a1 {
namespace {

constexpr auto address_space_bytes = std::uint64_t{1} << 32;
constexpr std::size_t maximum_ram_region = std::size_t{64} * 1024 * 1024;
constexpr unsigned int byte_bits = 8;
constexpr auto byte_mask = std::numeric_limits<std::uint8_t>::max();

std::string
fault_message(AccessKind kind, MemoryAccess access, std::uint32_t pc, const std::string &reason) {
  std::ostringstream text;
  text << "ARM reference memory ";
  switch (kind) {
    case AccessKind::read: text << "read"; break;
    case AccessKind::write: text << "write"; break;
    case AccessKind::fetch: text << "fetch"; break;
    default: throw std::invalid_argument("Invalid memory access kind");
  }
  text << " address=0x" << std::hex << access.address << " pc=0x" << pc << std::dec
       << " width=" << access.size << ": " << reason;
  return text.str();
}

}

MemoryFault::MemoryFault(AccessKind kind,
                         MemoryAccess access,
                         std::uint32_t pc,
                         const std::string &reason)
    : std::runtime_error(fault_message(kind, access, pc, reason)) {}

Memory::~Memory() {
  abort_instruction();
}

void Memory::map(MemoryMapping mapping,
                 std::span<const std::byte> initial,
                 std::size_t initial_offset) {
  validate_mapping(mapping);
  if (initial_offset > mapping.size || initial.size() > mapping.size - initial_offset) {
    throw std::invalid_argument("Invalid RAM mapping or active instruction");
  }
  Region result{.mapping = std::move(mapping), .bytes = {}, .known = {}};
  result.bytes.resize(result.mapping.size);
  result.known.resize(result.mapping.size, false);
  const auto offset = static_cast<std::ptrdiff_t>(initial_offset);
  std::ranges::copy(initial, result.bytes.begin() + offset);
  std::fill_n(result.known.begin() + offset, initial.size(), true);
  regions_.push_back(std::move(result));
}

void Memory::validate_mapping(const MemoryMapping &mapping) const {
  if (active_ || mapping.size == 0 || mapping.size > maximum_ram_region ||
      mapping.size > address_space_bytes - mapping.address) {
    throw std::invalid_argument("Invalid RAM mapping or active instruction");
  }
  const auto end = static_cast<std::uint64_t>(mapping.address) + mapping.size;
  for (const auto &entry : regions_) {
    const auto entry_end = static_cast<std::uint64_t>(entry.mapping.address) + entry.mapping.size;
    if (mapping.address < entry_end && entry.mapping.address < end) {
      throw std::invalid_argument("Overlapping RAM mappings");
    }
  }
  for (const auto &entry : devices_) {
    const auto entry_end = static_cast<std::uint64_t>(entry.mapping.address) + entry.mapping.size;
    if (mapping.address < entry_end && entry.mapping.address < end) {
      throw std::invalid_argument("Overlapping device mappings");
    }
  }
}

void Memory::map_device(MemoryMapping mapping, MemoryDevice &device) {
  validate_mapping(mapping);
  if (mapping.executable) { throw std::invalid_argument("Executable device mapping"); }
  for (const auto &entry : devices_) {
    if (entry.device == &device) { throw std::invalid_argument("Device already mapped"); }
  }
  devices_.push_back({.mapping = std::move(mapping), .device = &device, .joined = false});
}

void Memory::validate_access(MemoryAccess access, AccessKind kind) const {
  if (access.size == 0 || access.size > sizeof(std::uint64_t) ||
      access.size > address_space_bytes - access.address) {
    throw MemoryFault(kind, access, pc_, "invalid width or address wrap");
  }
}

const Memory::DeviceRegion *Memory::device_region(MemoryAccess access) const {
  const auto end = static_cast<std::uint64_t>(access.address) + access.size;
  for (const auto &entry : devices_) {
    const auto region_end = static_cast<std::uint64_t>(entry.mapping.address) + entry.mapping.size;
    if (access.address >= entry.mapping.address && end <= region_end) { return &entry; }
  }
  return nullptr;
}

void Memory::join(const DeviceRegion &entry) const {
  if (active_ && !entry.joined) {
    entry.device->begin_instruction();
    entry.joined = true;
  }
}

const Memory::Region &Memory::region(MemoryAccess access, AccessKind kind) const {
  const auto end = static_cast<std::uint64_t>(access.address) + access.size;
  for (const auto &entry : regions_) {
    const auto region_end = static_cast<std::uint64_t>(entry.mapping.address) + entry.mapping.size;
    if (access.address >= entry.mapping.address && end <= region_end) {
      if ((kind == AccessKind::write && !entry.mapping.writable) ||
          (kind == AccessKind::fetch && !entry.mapping.executable)) {
        throw MemoryFault(
            kind, access, pc_, "RAM mapping permission denied: " + entry.mapping.name);
      }
      return entry;
    }
  }
  throw MemoryFault(kind, access, pc_, "unmapped or crossing region boundary");
}

std::uint8_t Memory::read_byte(const Region &entry,
                               std::uint32_t address,
                               MemoryAccess access,
                               AccessKind kind) const {
  for (const auto &write : std::views::reverse(writes_)) {
    if (address >= write.access.address && address - write.access.address < write.access.size) {
      const auto shift = (address - write.access.address) * byte_bits;
      return static_cast<std::uint8_t>((write.value >> shift) & byte_mask);
    }
  }
  const auto offset = address - entry.mapping.address;
  if (!entry.known[offset]) {
    throw MemoryFault(kind, access, pc_, "uninitialized RAM: " + entry.mapping.name);
  }
  return std::to_integer<std::uint8_t>(entry.bytes[offset]);
}

std::uint64_t Memory::read(MemoryAccess access, AccessKind kind) const {
  if (kind != AccessKind::read && kind != AccessKind::fetch) {
    throw std::invalid_argument("Invalid read access kind");
  }
  validate_access(access, kind);
  if (const auto *entry = device_region(access)) {
    if (kind == AccessKind::fetch) {
      throw MemoryFault(kind, access, pc_, "Device fetch denied: " + entry->mapping.name);
    }
    try {
      join(*entry);
      return entry->device->read(
          {.address = access.address - entry->mapping.address, .size = access.size});
    } catch (const std::exception &error) {
      throw MemoryFault(kind, access, pc_, entry->mapping.name + ": " + error.what());
    }
  }
  const auto &entry = region(access, kind);
  std::uint64_t value{};
  for (unsigned int index = 0; index < access.size; ++index) {
    value |= static_cast<std::uint64_t>(read_byte(entry, access.address + index, access, kind))
             << (index * byte_bits);
  }
  return value;
}

void Memory::apply_write(MemoryAccess access, std::uint64_t value) {
  for (auto &entry : regions_) {
    if (access.address >= entry.mapping.address &&
        access.address - entry.mapping.address < entry.mapping.size) {
      const auto offset = access.address - entry.mapping.address;
      for (unsigned int index = 0; index < access.size; ++index) {
        entry.bytes[offset + index] =
            static_cast<std::byte>((value >> (index * byte_bits)) & byte_mask);
        entry.known[offset + index] = true;
      }
      return;
    }
  }
  throw std::logic_error("Validated RAM mapping disappeared");
}

void Memory::write(MemoryAccess access, std::uint64_t value) {
  validate_access(access, AccessKind::write);
  if (const auto *entry = device_region(access)) {
    if (!entry->mapping.writable) {
      throw MemoryFault(AccessKind::write, access, pc_, "Device write permission denied");
    }
    try {
      join(*entry);
      entry->device->write(
          {.address = access.address - entry->mapping.address, .size = access.size}, value);
      return;
    } catch (const std::exception &error) {
      throw MemoryFault(AccessKind::write, access, pc_, entry->mapping.name + ": " + error.what());
    }
  }
  static_cast<void>(region(access, AccessKind::write));
  if (active_) {
    writes_.push_back({.access = access, .value = value});
  } else {
    apply_write(access, value);
  }
}

std::optional<std::byte> Memory::peek(std::uint32_t address) const {
  for (const auto &entry : regions_) {
    if (address >= entry.mapping.address && address - entry.mapping.address < entry.mapping.size) {
      const auto offset = address - entry.mapping.address;
      return entry.known[offset] ? std::optional{entry.bytes[offset]} : std::nullopt;
    }
  }
  return std::nullopt;
}

void Memory::begin_instruction() {
  if (active_) { throw std::logic_error("Nested memory instruction transaction"); }
  active_ = true;
}

void Memory::commit_instruction() {
  if (!active_) { throw std::logic_error("No active memory instruction transaction"); }
  for (const auto &write : writes_) { apply_write(write.access, write.value); }
  for (auto &entry : devices_) {
    if (entry.joined) {
      entry.device->commit_instruction();
      entry.joined = false;
    }
  }
  writes_.clear();
  active_ = false;
}

void Memory::abort_instruction() {
  for (auto &entry : devices_) {
    if (entry.joined) {
      entry.device->abort_instruction();
      entry.joined = false;
    }
  }
  writes_.clear();
  active_ = false;
}

}
