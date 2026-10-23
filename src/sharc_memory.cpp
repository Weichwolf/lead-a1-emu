// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_memory.hpp"

#include "lead_a1/memory.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <ios>
#include <limits>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace lead_a1 {
namespace {

constexpr std::uint32_t short_base = 0x100000;
constexpr std::uint32_t normal_base = 0x80000;
constexpr std::uint32_t long_base = 0x40000;
constexpr std::uint32_t memory_end = 0x200000;
constexpr std::uint32_t bank_cells = 0x40000;
constexpr unsigned int half_bits = 16;

unsigned int width_bits(SharcWidth width) {
  switch (width) {
    case SharcWidth::short16:
    case SharcWidth::normal32:
    case SharcWidth::extended40:
    case SharcWidth::wide48:
    case SharcWidth::long64: return static_cast<unsigned int>(width);
    default: throw std::invalid_argument("Invalid SHARC word width");
  }
}

std::string
context(std::uint32_t pc, SharcAccess access, AccessKind kind, const std::string &reason) {
  std::ostringstream text;
  const char *operation = "read";
  if (kind == AccessKind::write) { operation = "write"; }
  if (kind == AccessKind::fetch) { operation = "fetch"; }
  text << "SHARC memory " << operation << " pc=0x" << std::hex << pc << " word-address=0x"
       << access.address << std::dec << " bits=" << static_cast<unsigned int>(access.width) << ": "
       << reason;
  return text.str();
}

struct View {
  unsigned int bank;
  std::uint32_t cell;
  unsigned int cells;
  unsigned int shift;
};

View decode(SharcAccess access) {
  const auto address = access.address;
  if (address >= short_base && address < memory_end && access.width == SharcWidth::short16) {
    return {.bank = (address - short_base) / bank_cells,
            .cell = (address - short_base) % bank_cells,
            .cells = 1,
            .shift = 0};
  }
  if (address >= normal_base && address < short_base && access.width != SharcWidth::short16) {
    constexpr std::uint32_t span = 0x20000;
    const auto offset = (address - normal_base) % span;
    auto columns = access.width == SharcWidth::normal32 ? 2U : 3U;
    auto cell = offset * columns;
    if (access.width == SharcWidth::long64) {
      columns = 4;
      cell = (offset & ~1U) * 2;
    }
    return {.bank = (address - normal_base) / span,
            .cell = cell,
            .cells = columns,
            .shift = access.width == SharcWidth::extended40 ? 8U : 0U};
  }
  if (address >= long_base && address < normal_base && access.width == SharcWidth::long64) {
    constexpr std::uint32_t span = 0x10000;
    return {.bank = (address - long_base) / span,
            .cell = ((address - long_base) % span) * 4,
            .cells = 4,
            .shift = 0};
  }
  throw std::runtime_error("Unsupported SHARC address/width view or external memory");
}

std::uint64_t mask(SharcWidth width) {
  const auto bits = width_bits(width);
  return width == SharcWidth::long64 ? std::numeric_limits<std::uint64_t>::max()
                                     : (std::uint64_t{1} << bits) - 1;
}

}

SharcMemory::~SharcMemory() {
  abort_instruction();
}

void SharcMemory::map(SharcRegion region, std::span<const SharcCell> initial) {
  if (active_ || region.bank >= 4 || region.halfwords == 0 || region.first_halfword >= bank_cells ||
      region.halfwords > bank_cells - region.first_halfword || initial.size() > region.halfwords) {
    throw std::invalid_argument("Invalid SHARC physical mapping");
  }
  for (const auto &entry : regions_) {
    const auto &other = entry.mapping;
    if (other.bank == region.bank &&
        region.first_halfword < other.first_halfword + other.halfwords &&
        other.first_halfword < region.first_halfword + region.halfwords) {
      throw std::invalid_argument("Overlapping SHARC physical mapping");
    }
  }
  for (const auto &cell : initial) {
    if ((cell.value & ~cell.known) != 0) {
      throw std::invalid_argument("Invalid SHARC cell knowledge");
    }
  }
  Region entry{.mapping = std::move(region), .cells = {}};
  entry.cells.resize(entry.mapping.halfwords);
  std::ranges::copy(initial, entry.cells.begin());
  regions_.push_back(std::move(entry));
}

void SharcMemory::map_2148x_5mbit_ram() {
  constexpr std::uint32_t upper_ram = 0x24000;
  constexpr std::uint32_t large_cells = 0x18000;
  constexpr std::uint32_t small_cells = 0x10000;
  if (active_ || !regions_.empty()) {
    throw std::logic_error("SHARC geometry needs an empty memory");
  }
  for (unsigned int bank = 0; bank < 4; ++bank) {
    map({.bank = bank,
         .first_halfword = bank < 2 ? upper_ram : 0,
         .halfwords = bank < 2 ? large_cells : small_cells,
         .name = "candidate 5-Mbit SRAM bank " + std::to_string(bank)});
  }
}

void SharcMemory::map_io(SharcIoMapping mapping, MemoryDevice &device) {
  const auto [first_word, words] = mapping;
  if (active_ || words == 0 || first_word >= long_base || words > long_base - first_word) {
    throw std::invalid_argument("Invalid SHARC IOP mapping");
  }
  for (const auto &entry : io_) {
    if (entry.device == &device ||
        (first_word < entry.first_word + entry.words && entry.first_word < first_word + words)) {
      throw std::invalid_argument("Overlapping or repeated SHARC IOP mapping");
    }
  }
  io_.push_back({.first_word = first_word, .words = words, .device = &device, .joined = false});
}

const SharcMemory::IoRegion *SharcMemory::io(SharcAccess access) const {
  if (access.address >= long_base) { return nullptr; }
  if (access.width != SharcWidth::normal32) {
    throw std::runtime_error("SHARC IOP requires 32-bit words");
  }
  const auto found = std::ranges::find_if(io_, [access](const auto &entry) {
    return access.address >= entry.first_word && access.address - entry.first_word < entry.words;
  });
  if (found == io_.end()) { throw std::runtime_error("Unmapped SHARC IOP register"); }
  return &*found;
}

void SharcMemory::join(const IoRegion &region) const {
  if (active_ && !region.joined) {
    region.device->begin_instruction();
    region.joined = true;
  }
}

SharcMemory::Located SharcMemory::locate(PhysicalCell position) const {
  const auto [bank, index] = position;
  for (std::size_t entry = 0; entry < regions_.size(); ++entry) {
    const auto &region = regions_[entry].mapping;
    if (region.bank == bank && index >= region.first_halfword &&
        index - region.first_halfword < region.halfwords) {
      return {.region = entry, .cell = index - region.first_halfword};
    }
  }
  throw std::runtime_error("Unmapped SHARC physical RAM/ROM cell");
}

SharcCell SharcMemory::cell(Located location) const {
  for (std::size_t pending = write_count_; pending > 0; --pending) {
    const auto &entry = writes_[pending - 1];
    if (entry.region == location.region && entry.cell == location.cell) { return entry.value; }
  }
  return regions_[location.region].cells[location.cell];
}

std::uint64_t SharcMemory::read(SharcAccess access, AccessKind kind) const try {
  static_cast<void>(width_bits(access.width));
  if (kind != AccessKind::read && kind != AccessKind::fetch) {
    throw std::invalid_argument("Invalid SHARC read kind");
  }
  if (const auto *entry = io(access)) {
    if (kind == AccessKind::fetch) { throw std::runtime_error("SHARC instruction fetch from IOP"); }
    join(*entry);
    const auto value =
        entry->device->read({.address = (access.address - entry->first_word) * 4, .size = 4});
    if (value > std::numeric_limits<std::uint32_t>::max()) {
      throw std::runtime_error("Oversized SHARC IOP readback");
    }
    return value;
  }
  const auto view = decode(access);
  const auto required = mask(access.width) << view.shift;
  std::uint64_t value{};
  for (unsigned int index = 0; index < view.cells; ++index) {
    const auto location = locate({.bank = view.bank, .index = view.cell + index});
    if (kind == AccessKind::fetch && !regions_[location.region].mapping.executable) {
      throw std::runtime_error("Non-executable SHARC physical region");
    }
    const auto part = cell(location);
    const auto needed = static_cast<std::uint16_t>(required >> (index * half_bits));
    if ((needed & ~part.known) != 0) { throw std::runtime_error("Unknown SHARC memory bits"); }
    value |= static_cast<std::uint64_t>(part.value) << (index * half_bits);
  }
  return (value >> view.shift) & mask(access.width);
} catch (const std::exception &error) {

  throw std::runtime_error(context(pc_, access, kind, error.what()));
}

void SharcMemory::write(SharcAccess access, std::uint64_t value) try {
  if ((value & ~mask(access.width)) != 0) {
    throw std::runtime_error("Oversized SHARC word value");
  }
  if (const auto *entry = io(access)) {
    join(*entry);
    entry->device->write({.address = (access.address - entry->first_word) * 4, .size = 4}, value);
    return;
  }
  const auto view = decode(access);
  std::array<Write, 4> next{};
  const auto bits = value << view.shift;
  const auto selected = mask(access.width) << view.shift;
  if (active_ && view.cells > writes_.size() - write_count_) {
    throw std::runtime_error("SHARC instruction write journal full");
  }
  for (unsigned int index = 0; index < view.cells; ++index) {
    const auto location = locate({.bank = view.bank, .index = view.cell + index});
    if (!regions_[location.region].mapping.writable) {
      throw std::runtime_error("Read-only SHARC region");
    }
    auto part = cell(location);
    const auto written = static_cast<std::uint16_t>(selected >> (index * half_bits));
    const auto fragment = static_cast<std::uint16_t>(bits >> (index * half_bits));
    part.value = static_cast<std::uint16_t>((part.value & ~written) | (fragment & written));
    part.known |= written;
    next[index] = {.region = location.region, .cell = location.cell, .value = part};
  }
  for (unsigned int index = 0; index < view.cells; ++index) {
    const auto &entry = next[index];
    if (active_) {
      writes_[write_count_++] = entry;
    } else {
      regions_[entry.region].cells[entry.cell] = entry.value;
    }
  }
} catch (const std::exception &error) {

  throw std::runtime_error(context(pc_, access, AccessKind::write, error.what()));
}

void SharcMemory::begin_instruction() {
  if (active_) { throw std::logic_error("Nested SHARC memory transaction"); }
  active_ = true;
  write_count_ = 0;
}

void SharcMemory::commit_instruction() {
  if (!active_) { throw std::logic_error("Missing SHARC memory transaction"); }
  for (std::size_t index = 0; index < write_count_; ++index) {
    const auto &entry = writes_[index];
    regions_[entry.region].cells[entry.cell] = entry.value;
  }
  for (auto &entry : io_) {
    if (entry.joined) {
      entry.device->commit_instruction();
      entry.joined = false;
    }
  }
  write_count_ = 0;
  active_ = false;
}

void SharcMemory::abort_instruction() noexcept {
  if (!active_) { return; }
  for (auto &entry : io_) {
    if (entry.joined) {
      entry.device->abort_instruction();
      entry.joined = false;
    }
  }
  write_count_ = 0;
  active_ = false;
}

}
