// SPDX-License-Identifier: GPL-3.0-only
#include "arm/peripherals/aintc.hpp"

#include "common/memory.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t status_enabled_offset = 0x280;
constexpr std::uint32_t enable_bank_offset = 0x300;
constexpr std::uint32_t disable_bank_offset = 0x380;
constexpr std::uint32_t bank_offset_mask = 0x7f;
constexpr std::uint32_t global_nesting_offset = 0x1c;
constexpr std::uint32_t status_clear_offset = 0x24;
constexpr std::uint32_t enable_set_offset = 0x28;
constexpr std::uint32_t enable_clear_offset = 0x2c;
constexpr std::uint32_t host_set_offset = 0x34;
constexpr std::uint32_t host_clear_offset = 0x38;
constexpr std::uint32_t vector_base_offset = 0x50;
constexpr std::uint32_t vector_size_offset = 0x54;
constexpr std::uint32_t null_vector_offset = 0x58;
constexpr std::uint32_t global_vector_offset = 0x84;
constexpr std::uint32_t fiq_index_offset = 0x900;
constexpr std::uint32_t irq_index_offset = 0x904;
constexpr std::uint32_t fiq_nesting_offset = 0x1100;
constexpr std::uint32_t irq_nesting_offset = 0x1104;
constexpr std::uint32_t host_enable_offset = 0x1500;
constexpr std::uint32_t fiq_vector_offset = 0x1600;
constexpr std::uint32_t irq_vector_offset = 0x1604;

constexpr std::uint32_t none = 0x80000000;
constexpr std::uint32_t control_mask = 0x1c;
constexpr std::uint32_t priority_hold = 0x10;
constexpr std::uint32_t last_word_mask = 0x1f;
constexpr unsigned int source_count = 101;
constexpr unsigned int channel_count = 32;
constexpr unsigned int word_bits = 32;
constexpr unsigned int byte_bits = 8;
constexpr std::uint32_t byte_mask = 255;
constexpr std::uint32_t nesting_mask = 0x1ff;
constexpr unsigned int maximum_size = 29;
constexpr std::uint32_t channel_base = 0x400;
constexpr std::uint32_t channel_end = 0x468;

void validate_control(std::uint32_t value) {
  if ((value & ~control_mask) != 0) { throw std::runtime_error("Reserved AINTC control bits"); }
}

void validate(const AintcState &state) {
  validate_control(state.control);
  if (state.host_enabled > 3 || state.vector_size > maximum_size ||
      state.global_nesting > nesting_mask ||
      std::ranges::any_of(state.host_nesting, [](auto value) { return value > nesting_mask; }) ||
      (state.pending.back() & ~last_word_mask) != 0 ||
      (state.enabled.back() & ~last_word_mask) != 0 ||
      std::ranges::any_of(state.channels, [](auto value) { return value >= channel_count; })) {
    throw std::runtime_error("Invalid AINTC reference state");
  }
  for (const auto &held : state.held) {
    if (held && (state.control & priority_hold) == 0) {
      throw std::runtime_error("AINTC held state without priority holding mode");
    }
    if (held && held->index != none && held->index >= source_count) {
      throw std::runtime_error("Invalid AINTC held index");
    }
  }
}

bool active(const AintcState &state, unsigned int source) {
  const auto bit = std::uint32_t{1} << (source % word_bits);
  return (state.pending[source / word_bits] & state.enabled[source / word_bits] & bit) != 0;
}

bool nested_out(const AintcState &state, unsigned int channel, unsigned int host) {
  const auto mode = (state.control >> 2) & 3;
  if (mode == 1 && state.host_nesting_override[host]) {
    return channel >= state.host_nesting[host];
  }
  return mode == 2 && state.global_nesting_override && channel >= state.global_nesting;
}

std::uint32_t prioritized(const AintcState &state, std::optional<unsigned int> host) {
  std::uint32_t selected = none;
  unsigned int best_channel = channel_count;
  for (unsigned int source = 0; source < source_count; ++source) {
    const auto channel = state.channels[source];
    const auto mapped_host = channel < 2 ? 0U : 1U;
    if (host && *host != mapped_host) { continue; }
    if (nested_out(state, channel, mapped_host)) { continue; }
    if (active(state, source) && channel < best_channel) {
      selected = source;
      best_channel = channel;
    }
  }
  return selected;
}

std::uint32_t nesting_read(const AintcState &state, std::optional<unsigned int> host) {
  const auto mode = (state.control >> 2) & 3;
  const bool automatic = host ? mode == 1 && !state.host_nesting_override[*host]
                              : mode == 2 && !state.global_nesting_override;
  if (!automatic) { return host ? state.host_nesting[*host] : state.global_nesting; }
  const auto selected = prioritized(state, host);
  constexpr std::uint32_t no_channel = 0x100;
  return selected == none ? no_channel : state.channels[selected];
}

std::uint32_t vector(const AintcState &state, std::uint32_t index) {
  if (index == none) { return state.null_vector; }
  if (!state.vector_size_programmed) {
    throw std::runtime_error("AINTC vector size has not been programmed");
  }
  const auto stride = std::uint64_t{1} << (state.vector_size + 2);
  const auto address = static_cast<std::uint64_t>(state.vector_base) + (index * stride);
  return static_cast<std::uint32_t>(address & std::numeric_limits<std::uint32_t>::max());
}

std::uint32_t host_read(AintcState &state, unsigned int host, bool address) {
  const auto &held = state.held[host];
  if (held) { return address ? held->vector : held->index; }
  const auto selected = prioritized(state, host);
  if ((state.control & priority_hold) != 0) {
    const AintcPriority value{.index = selected, .vector = vector(state, selected)};
    state.held[host] = value;
    return address ? value.vector : value.index;
  }
  return address ? vector(state, selected) : selected;
}

std::uint32_t bank_read(const AintcState &state, std::uint32_t offset) {
  constexpr std::uint32_t bank_mask = 0xffffff80;
  const auto index = (offset & 15) / 4;
  switch (offset & bank_mask) {
    case 0x200: return state.pending[index];
    case status_enabled_offset: return state.pending[index] & state.enabled[index];
    case enable_bank_offset:
    case disable_bank_offset: return state.enabled[index];
    default: throw std::runtime_error("Unsupported AINTC bank");
  }
}

bool is_bank(std::uint32_t offset) {
  constexpr std::uint32_t bank_mask = 0xffffff80;
  const auto bank = offset & bank_mask;
  return (offset & bank_offset_mask) < 16 &&
         (bank == 0x200 || bank == status_enabled_offset || bank == enable_bank_offset ||
          bank == disable_bank_offset);
}

std::uint32_t read_word(AintcState &state, std::uint32_t offset) {
  if (is_bank(offset)) { return bank_read(state, offset); }
  switch (offset) {
    case 0:
      if (!state.revision) { throw std::runtime_error("Unknown AINTC silicon revision"); }
      return *state.revision;
    case 4: return state.control;
    case 0x10: return state.global_enabled ? 1 : 0;
    case global_nesting_offset: return nesting_read(state, {});
    case 0x20:
    case status_clear_offset:
    case enable_set_offset:
    case enable_clear_offset:
    case host_set_offset:
    case host_clear_offset: return 0;
    case vector_base_offset: return state.vector_base;
    case vector_size_offset: return state.vector_size;
    case null_vector_offset: return state.null_vector;
    case 0x80: return prioritized(state, {});
    case global_vector_offset: return vector(state, prioritized(state, {}));
    case fiq_index_offset: return host_read(state, 0, false);
    case irq_index_offset: return host_read(state, 1, false);
    case fiq_nesting_offset: return nesting_read(state, 0);
    case irq_nesting_offset: return nesting_read(state, 1);
    case host_enable_offset: return state.host_enabled;
    case fiq_vector_offset: return host_read(state, 0, true);
    case irq_vector_offset: return host_read(state, 1, true);
    default: throw std::runtime_error("Unsupported AINTC register read");
  }
}

void check_mask(std::uint32_t value, std::uint32_t allowed) {
  if ((value & ~allowed) != 0) { throw std::runtime_error("Reserved AINTC register bits"); }
}

void indexed(AintcState &state, std::uint32_t offset, std::uint32_t source) {
  if (source >= source_count) { throw std::runtime_error("Invalid AINTC source index"); }
  auto &word = offset < enable_set_offset ? state.pending[source / word_bits]
                                          : state.enabled[source / word_bits];
  const auto bit = std::uint32_t{1} << (source % word_bits);
  if (offset == 0x20 || offset == enable_set_offset) {
    word |= bit;
  } else {
    word &= ~bit;
  }
}

struct RegisterWrite {
  std::uint32_t offset;
  std::uint32_t value;
};

void bank_write(AintcState &state, RegisterWrite operation) {
  const auto [offset, value] = operation;
  constexpr std::uint32_t bank_mask = 0xffffff80;
  const auto index = (offset & 15) / 4;
  if (index == 3) { check_mask(value, last_word_mask); }
  switch (offset & bank_mask) {
    case 0x200: state.pending[index] |= value; break;
    case status_enabled_offset: state.pending[index] &= ~value; break;
    case enable_bank_offset: state.enabled[index] |= value; break;
    case disable_bank_offset: state.enabled[index] &= ~value; break;
    default: throw std::runtime_error("Unsupported AINTC bank write");
  }
}

void host_enable(AintcState &state, std::uint32_t value) {
  check_mask(value, 3);
  state.host_enabled = value;
  for (unsigned int host = 0; host < state.held.size(); ++host) {
    if ((value & (1U << host)) != 0) { state.held[host].reset(); }
  }
}

void write_word(AintcState &state, std::uint32_t offset, std::uint32_t value) {
  if (is_bank(offset)) {
    bank_write(state, {.offset = offset, .value = value});
    return;
  }
  if (offset >= 0x20 && offset <= enable_clear_offset) {
    indexed(state, offset, value);
    return;
  }
  switch (offset) {
    case 4:
      validate_control(value);
      state.control = value;
      if ((value & priority_hold) == 0) { state.held = {}; }
      break;
    case 0x10:
      check_mask(value, 1);
      state.global_enabled = value != 0;
      break;
    case global_nesting_offset:
      check_mask(value, none | nesting_mask);
      state.global_nesting = value & nesting_mask;
      state.global_nesting_override = (value & none) != 0;
      break;
    case host_set_offset:
    case host_clear_offset: {
      check_mask(value, 1);
      const auto bit = 1U << value;
      state.host_enabled =
          offset == host_set_offset ? state.host_enabled | bit : state.host_enabled & ~bit;
      state.held[value].reset();
      break;
    }
    case vector_base_offset: state.vector_base = value; break;
    case vector_size_offset:
      if (value > maximum_size) { throw std::runtime_error("Unqualified AINTC vector size"); }
      state.vector_size = value;
      state.vector_size_programmed = true;
      break;
    case null_vector_offset: state.null_vector = value; break;
    case fiq_index_offset:
    case irq_index_offset: state.held[(offset - fiq_index_offset) / 4].reset(); break;
    case fiq_nesting_offset:
    case irq_nesting_offset: {
      check_mask(value, none | nesting_mask);
      const auto host = (offset - fiq_nesting_offset) / 4;
      state.host_nesting[host] = value & nesting_mask;
      state.host_nesting_override[host] = (value & none) != 0;
      break;
    }
    case host_enable_offset: host_enable(state, value); break;
    case fiq_vector_offset:
    case irq_vector_offset: state.held[(offset - fiq_vector_offset) / 4].reset(); break;
    default: throw std::runtime_error("Unsupported AINTC register write");
  }
}

void validate_access(MemoryAccess access) {
  const bool channel = access.address >= channel_base && access.address < channel_end;
  if (channel) {
    if ((access.size != 1 && access.size != 2 && access.size != 4) ||
        access.address % access.size != 0 || access.size > channel_end - access.address) {
      throw std::runtime_error("Invalid AINTC channel-map access");
    }
  } else if (access.size != 4 || access.address % 4 != 0) {
    throw std::runtime_error("AINTC requires aligned word access");
  }
}

std::uint32_t channel_read(const AintcState &state, MemoryAccess access) {
  std::uint32_t value{};
  for (unsigned int byte = 0; byte < access.size; ++byte) {
    const auto index = access.address - channel_base + byte;
    if (index < source_count) { value |= state.channels[index] << (byte * byte_bits); }
  }
  return value;
}

void channel_write(AintcState &state, MemoryAccess access, std::uint32_t value) {
  for (unsigned int byte = 0; byte < access.size; ++byte) {
    const auto index = access.address - channel_base + byte;
    const auto channel = (value >> (byte * byte_bits)) & byte_mask;
    if (index >= source_count) {
      if (channel != 0) { throw std::runtime_error("Reserved AINTC channel-map byte"); }
    } else {
      if (channel >= channel_count) { throw std::runtime_error("Invalid AINTC channel index"); }
      state.channels[index] = channel;
    }
  }
}

}

Aintc::Aintc(const AintcState &initial) : current_(initial) {
  validate(initial);
}

AintcState Aintc::reset_reference() {
  AintcState result;
  constexpr std::uint32_t documented_revision = 0x4e82a900;
  result.revision = documented_revision;
  return result;
}

AintcState Aintc::state() const {
  return working();
}

std::array<bool, 2> Aintc::outputs() const {
  const auto &state = working();
  std::array<bool, 2> result{};
  for (unsigned int host = 0; host < result.size(); ++host) {
    result[host] = state.global_enabled && (state.host_enabled & (1U << host)) != 0 &&
                   prioritized(state, host) != none;
  }
  return result;
}

void Aintc::pulse(unsigned int source) {
  if (pending_) { throw std::logic_error("AINTC input during active instruction"); }
  indexed(current_, 0x20, source);
}

std::uint64_t Aintc::read(MemoryAccess access) {
  validate_access(access);
  auto &state = working();
  if (access.address >= channel_base && access.address < channel_end) {
    return channel_read(state, access);
  }
  return read_word(state, access.address);
}

void Aintc::write(MemoryAccess access, std::uint64_t value) {
  validate_access(access);
  if (value > std::numeric_limits<std::uint32_t>::max() ||
      (access.size < 4 && value >= (std::uint64_t{1} << (access.size * byte_bits)))) {
    throw std::runtime_error("Oversized AINTC write value");
  }
  auto next = working();
  const auto word = static_cast<std::uint32_t>(value);
  if (access.address >= channel_base && access.address < channel_end) {
    channel_write(next, access, word);
  } else {
    write_word(next, access.address, word);
  }
  working() = next;
}

void Aintc::begin_instruction() {
  if (pending_) { throw std::logic_error("Nested AINTC instruction transaction"); }
  pending_ = current_;
}

void Aintc::commit_instruction() noexcept {
  if (pending_) { current_ = *pending_; }
  pending_.reset();
}

void Aintc::abort_instruction() noexcept {
  pending_.reset();
}

AintcState &Aintc::working() {
  return pending_ ? *pending_ : current_;
}

const AintcState &Aintc::working() const {
  return pending_ ? *pending_ : current_;
}

}
