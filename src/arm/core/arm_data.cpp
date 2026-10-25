// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm_data_internal.hpp"
#include "arm/core/arm_mmu_internal.hpp"
#include "arm/core/arm_system.hpp"
#include "common/memory.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int word_bytes = 4;
constexpr unsigned int byte_bits = 8;
constexpr auto byte_mask = std::numeric_limits<std::uint8_t>::max();
constexpr std::uint32_t line_mask = 31;

bool overlaps(MemoryAccess first, MemoryAccess second) {
  return first.address < static_cast<std::uint64_t>(second.address) + second.size &&
         second.address < static_cast<std::uint64_t>(first.address) + first.size;
}

void validate_access(const MmuTranslation &translation) {
  const auto access = translation.physical;
  constexpr auto address_bytes = std::uint64_t{1} << 32;
  if ((access.size != 1 && access.size != 2 && access.size != 4 && access.size != 8) ||
      access.size > address_bytes - access.address || access.address % access.size != 0 ||
      (access.address & line_mask) != (translation.modified & line_mask)) {
    throw std::runtime_error("Invalid ARM data-path access/translation alignment");
  }
}

std::size_t dirty_count(unsigned int dirty) {
  return static_cast<std::size_t>(std::popcount(dirty));
}

}

ArmDataPath::ArmDataPath(Memory &memory, const std::optional<ArmSystemInitial> &initial)
    : memory_(memory) {
  if (!initial) {
    state_.stores_known = true;
    state_.writeback_known = true;
    return;
  }
  if (const auto capacity = initial->data_cache_bytes) {
    constexpr std::size_t maximum_cache_bytes = std::size_t{128} * 1024;
    if (*capacity < line_bytes * ways || *capacity > maximum_cache_bytes ||
        !std::has_single_bit(*capacity)) {
      throw std::invalid_argument("Invalid reference ARM data cache geometry");
    }
    lines_.resize(*capacity / line_bytes);
  }
  state_.cache_known = initial->data_cache_invalid;
  state_.writeback_known = initial->writeback_buffer_empty;
  if (!initial->write_buffer) { return; }
  if (initial->write_buffer->size() > maximum_stores) {
    throw std::invalid_argument("Unqualified initial ARM write-buffer address packing");
  }
  state_.stores_known = true;
  for (const auto &store : *initial->write_buffer) {
    validate_access({.physical = store.access,
                     .modified = store.access.address,
                     .cacheable = false,
                     .bufferable = true});
    buffer_.stores[buffer_.count++] = store;
    state_.buffered_words += (store.access.size + word_bytes - 1) / word_bytes;
  }
  state_.buffered_stores = buffer_.count;
}

void ArmDataPath::begin_instruction() {
  if (active_) { throw std::logic_error("Nested ARM data-path transaction"); }
  active_ = true;
  pending_ = state_;
  pending_buffer_.reset();
  changed_ = 0;
  invalidate_ = false;
}

void ArmDataPath::commit_instruction() {
  if (!active_) { throw std::logic_error("No ARM data-path transaction"); }
  if (invalidate_) {
    for (auto &value : lines_) { value = {}; }
  }
  for (std::size_t index = 0; index < changed_; ++index) {
    const auto &update = changes_[index];
    lines_[update.slot] = update.line;
  }
  if (pending_buffer_) { buffer_ = *pending_buffer_; }
  state_ = pending_;
  pending_buffer_.reset();
  active_ = false;
}

void ArmDataPath::abort_instruction() {
  pending_buffer_.reset();
  changed_ = 0;
  active_ = false;
}

void ArmDataPath::require_buffers() const {
  if (!active_) { throw std::logic_error("ARM data access outside transaction"); }
  if (!pending_.stores_known) { throw std::runtime_error("Unknown initial ARM write buffer"); }
  if (!pending_.writeback_known) { throw std::runtime_error("Unknown ARM write-back buffer"); }
}

void ArmDataPath::require_cache() const {
  if (!active_) { throw std::logic_error("ARM data cache access outside transaction"); }
  if (!pending_.cache_known) { throw std::runtime_error("Unknown ARM data cache validity"); }
  if (lines_.empty()) { throw std::runtime_error("Unknown ARM data cache geometry"); }
}

const ArmDataPath::Buffer &ArmDataPath::buffer() const {
  return pending_buffer_ ? *pending_buffer_ : buffer_;
}

ArmDataPath::Buffer &ArmDataPath::mutable_buffer() {
  if (!pending_buffer_) { pending_buffer_ = buffer_; }
  return *pending_buffer_;
}

const ArmDataPath::Line &ArmDataPath::line(std::size_t slot) const {
  for (std::size_t index = 0; index < changed_; ++index) {
    if (changes_[index].slot == slot) { return changes_[index].line; }
  }
  static constexpr Line invalid;
  return invalidate_ ? invalid : lines_.at(slot);
}

void ArmDataPath::change(std::size_t slot, const Line &value) {
  const auto before = line(slot);
  pending_.cached_lines -= static_cast<std::size_t>(before.valid);
  pending_.cached_lines += static_cast<std::size_t>(value.valid);
  pending_.dirty_halves -= dirty_count(before.dirty);
  pending_.dirty_halves += dirty_count(value.dirty);
  for (std::size_t index = 0; index < changed_; ++index) {
    if (changes_[index].slot == slot) {
      changes_[index].line = value;
      return;
    }
  }
  if (changed_ == changes_.size()) {
    throw std::runtime_error("ARM data-cache transaction journal exhausted");
  }
  changes_[changed_++] = {.slot = slot, .line = value};
}

std::size_t ArmDataPath::index(std::uint32_t modified) const {
  require_cache();
  return (modified / line_bytes) % (lines_.size() / ways);
}

std::optional<std::size_t> ArmDataPath::lookup(std::uint32_t modified) const {
  const auto set = index(modified);
  const auto base = modified & ~line_mask;
  for (std::size_t slot = set * ways; slot < (set + 1) * ways; ++slot) {
    const auto &value = line(slot);
    if (value.valid && value.modified == base) { return slot; }
  }
  return std::nullopt;
}

std::size_t ArmDataPath::allocate(std::uint32_t modified, const DataPolicy &policy) const {
  const auto set = index(modified);
  if (!policy.lockdown) { throw std::runtime_error("Unknown ARM data-cache lockdown"); }
  constexpr std::uint32_t ways_mask = 15;
  auto available = ~*policy.lockdown & ways_mask;
  if (available == 0) { available = 8; }
  std::size_t candidate{};
  for (std::size_t way = 0; way < ways; ++way) {
    if ((available & (std::uint32_t{1} << way)) == 0) { continue; }
    candidate = (set * ways) + way;
    if (!line(candidate).valid) { return candidate; }
  }
  if (std::has_single_bit(available)) { return candidate; }
  throw std::runtime_error(policy.round_robin ? "Unqualified ARM data round-robin replacement"
                                              : "Unknown ARM data random replacement state");
}

bool ArmDataPath::conflicts(MemoryAccess access) const {
  const auto &pending = buffer();
  for (std::size_t index = 0; index < pending.count; ++index) {
    if (overlaps(access, pending.stores[index].access)) { return true; }
  }
  return pending.writeback &&
         overlaps(access, {.address = pending.writeback->physical, .size = line_bytes});
}

void ArmDataPath::drain(bool explicit_request) {
  require_buffers();
  const auto &pending = buffer();
  for (std::size_t index = 0; index < pending.count; ++index) {
    const auto &store = pending.stores[index];
    memory_.write(store.access, store.value);
  }
  if (pending.writeback) {
    const auto &value = *pending.writeback;
    for (std::size_t offset = 0; offset < line_bytes; offset += word_bytes) {
      if ((value.dirty & (1U << (offset / half_bytes))) == 0) { continue; }
      std::uint32_t word{};
      for (unsigned int byte = 0; byte < word_bytes; ++byte) {
        word |= static_cast<std::uint32_t>(value.bytes[offset + byte]) << (byte * byte_bits);
      }
      memory_.write(
          {.address = value.physical + static_cast<std::uint32_t>(offset), .size = word_bytes},
          word);
    }
  }
  mutable_buffer() = {};
  pending_.buffered_stores = 0;
  pending_.buffered_words = 0;
  pending_.writeback_words = 0;
  if (explicit_request) {
    ++pending_.explicit_drains;
  } else {
    ++pending_.bus_drains;
  }
}

std::uint64_t ArmDataPath::physical_read(MemoryAccess access, AccessKind kind) {
  require_buffers();
  if (conflicts(access)) { drain(false); }
  return memory_.read(access, kind);
}

void ArmDataPath::enqueue(ArmBufferedWrite write) {
  require_buffers();
  if (buffer().writeback || buffer().count == maximum_stores) { drain(false); }
  auto &pending = mutable_buffer();
  pending.stores[pending.count++] = write;
  pending_.buffered_stores = pending.count;
  pending_.buffered_words += (write.access.size + word_bytes - 1) / word_bytes;
}

void ArmDataPath::enqueue_writeback(const Line &value) {
  if (value.dirty == 0) { return; }
  require_buffers();
  if (buffer().writeback || buffer().count != 0) { drain(false); }
  mutable_buffer().writeback = value;
  pending_.writeback_words = dirty_count(value.dirty) * (half_bytes / word_bytes);
  ++pending_.writebacks;
}

ArmDataPath::Line ArmDataPath::fill(const MmuTranslation &translation) {
  Line value{.modified = translation.modified & ~line_mask,
             .physical = translation.physical.address & ~line_mask,
             .bytes = {},
             .dirty = 0,
             .valid = true};
  const auto critical = (translation.physical.address & line_mask) & ~(word_bytes - 1U);
  for (std::size_t progress = 0; progress < line_bytes; progress += word_bytes) {
    const auto offset = (critical + progress) & line_mask;
    const auto word = physical_read(
        {.address = value.physical + static_cast<std::uint32_t>(offset), .size = word_bytes});
    for (unsigned int byte = 0; byte < word_bytes; ++byte) {
      value.bytes[offset + byte] =
          static_cast<std::uint8_t>((word >> (byte * byte_bits)) & byte_mask);
    }
  }
  return value;
}

std::uint64_t ArmDataPath::read(const MmuTranslation &translation, const DataPolicy &policy) {
  validate_access(translation);
  if (!policy.enabled || !translation.cacheable) { return physical_read(translation.physical); }
  auto slot = lookup(translation.modified);
  if (slot) {
    ++pending_.read_hits;
  } else {
    slot = allocate(translation.modified, policy);
    const auto old = line(*slot);
    enqueue_writeback(old);
    const auto replacement = fill(translation);
    change(*slot, replacement);
    if (old.valid) { ++pending_.evictions; }
    ++pending_.read_misses;
  }
  const auto &value = line(*slot);
  const auto offset = translation.modified & line_mask;
  std::uint64_t result{};
  for (unsigned int byte = 0; byte < translation.physical.size; ++byte) {
    result |= static_cast<std::uint64_t>(value.bytes[offset + byte]) << (byte * byte_bits);
  }
  return result;
}

void ArmDataPath::write(const MmuTranslation &translation,
                        const DataPolicy &policy,
                        std::uint64_t value) {
  validate_access(translation);
  require_buffers();
  const bool cacheable = policy.enabled && translation.cacheable;
  const auto slot = cacheable ? lookup(translation.modified) : std::nullopt;
  if (slot) {
    auto updated = line(*slot);
    const auto offset = translation.modified & line_mask;
    for (unsigned int byte = 0; byte < translation.physical.size; ++byte) {
      updated.bytes[offset + byte] =
          static_cast<std::uint8_t>((value >> (byte * byte_bits)) & byte_mask);
      if (translation.bufferable) { updated.dirty |= 1U << ((offset + byte) / half_bytes); }
    }
    change(*slot, updated);
    ++pending_.write_hits;
    if (translation.bufferable) { return; }
  } else if (cacheable) {
    ++pending_.write_misses;
  }
  if (translation.bufferable || cacheable) {
    enqueue({.access = translation.physical, .value = value});
  } else {
    if (buffer().count != 0 || buffer().writeback) { drain(false); }
    memory_.write(translation.physical, value);
  }
}

void ArmDataPath::invalidate_all() {
  if (!active_) { throw std::logic_error("ARM data invalidation outside transaction"); }
  invalidate_ = true;
  changed_ = 0;
  pending_.cache_known = true;
  pending_.cached_lines = 0;
  pending_.dirty_halves = 0;
  ++pending_.invalidations;
}

void ArmDataPath::clean(std::size_t slot, bool invalidate) {
  auto value = line(slot);
  if (!value.valid) { return; }
  enqueue_writeback(value);
  value.dirty = 0;
  if (invalidate) { value = {}; }
  change(slot, value);
}

void ArmDataPath::maintain(DataMaintenance operation) {
  require_cache();
  constexpr unsigned int clean_operation = 10;
  constexpr unsigned int clean_invalidate_operation = 14;
  if (operation.operation != 6 && operation.operation != clean_operation &&
      operation.operation != clean_invalidate_operation) {
    throw std::runtime_error("Unsupported ARM data-cache maintenance operation");
  }
  if ((operation.operand & line_mask) != 0) {
    throw std::runtime_error("Invalid ARM data-cache maintenance word/SBZ bits");
  }
  std::optional<std::size_t> slot;
  if (operation.selector == 1) {
    slot = lookup(operation.operand);
  } else if (operation.selector == 2) {
    constexpr unsigned int way_shift = 30;
    const auto sets = lines_.size() / ways;
    const auto index_mask = static_cast<std::uint32_t>((sets - 1) * line_bytes);
    constexpr std::uint32_t way_mask = 0xc0000000;
    if ((operation.operand & ~(way_mask | index_mask)) != 0) {
      throw std::runtime_error("Invalid ARM data-cache maintenance Set/Way bits");
    }
    slot = (static_cast<std::size_t>((operation.operand & index_mask) / line_bytes) * ways) +
           (operation.operand >> way_shift);
  } else {
    throw std::runtime_error("Unsupported ARM data-cache maintenance selector");
  }
  if (operation.operation == 6 || operation.operation == clean_invalidate_operation) {
    ++pending_.invalidations;
  }
  if (!slot) { return; }
  if (operation.operation == 6) {
    change(*slot, {});
  } else {
    clean(*slot, operation.operation == clean_invalidate_operation);
  }
}

}
