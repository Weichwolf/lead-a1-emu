// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_picr.hpp"

#include "common/memory.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr unsigned int encodings = 32;
constexpr unsigned int software = 31;
constexpr unsigned int fields = 6;
constexpr unsigned int field_bits = 5;
constexpr unsigned int field_mask = 31;

bool disabled(unsigned int encoding) {
  constexpr unsigned int absent0 = 0x11;
  constexpr unsigned int absent1 = 0x14;
  constexpr unsigned int absent2 = 0x16;
  constexpr unsigned int absent3 = 0x1e;
  return encoding == absent0 || encoding == absent1 || encoding == absent2 || encoding == absent3;
}

unsigned int register_index(MemoryAccess access) {
  constexpr unsigned int bytes = 4;
  constexpr unsigned int size = 16;
  if (access.size != bytes || access.address % bytes != 0 || access.address >= size) {
    throw std::runtime_error("PICR requires aligned32-bit register offsets0..12");
  }
  return access.address / bytes;
}

unsigned int selected(const SharcPicrState &state, unsigned int priority) {
  return (state.control[priority / fields] >> ((priority % fields) * field_bits)) & field_mask;
}

void validate_encoding(unsigned int encoding) {
  if (encoding >= encodings) { throw std::invalid_argument("Invalid PICR source encoding"); }
}

}

SharcPicr::SharcPicr(SharcPicrState initial) : current_(initial) {
  for (unsigned int encoding = 0; encoding < initial.sources.size(); ++encoding) {
    if ((disabled(encoding) || encoding == software) && initial.sources[encoding]) {
      throw std::invalid_argument("PICR constant source has dynamic input state");
    }
  }
}

SharcPicrState SharcPicr::state() const {
  return working();
}

SharcPicrState SharcPicr::snapshot() const {
  if (staged_) { throw std::logic_error("PICR snapshot during active instruction"); }
  return current_;
}

SharcPicrOutputs SharcPicr::outputs() const {
  SharcPicrOutputs result{};
  for (unsigned int priority = 0; priority < result.size(); ++priority) {
    const auto encoding = selected(current_, priority);
    if (disabled(encoding)) {
      result[priority] = false;
    } else if (encoding == software) {
      result[priority] = true;
    } else {
      result[priority] = current_.sources[encoding];
    }
  }
  return result;
}

std::uint32_t SharcPicr::destinations(unsigned int encoding) const {
  validate_encoding(encoding);
  std::uint32_t result{};
  for (unsigned int priority = 0; priority < sharc_picr_priorities; ++priority) {
    if (selected(current_, priority) == encoding) { result |= 1U << priority; }
  }
  return result;
}

void SharcPicr::source(unsigned int encoding, std::optional<bool> value) {
  if (staged_) { throw std::logic_error("PICR source event during active instruction"); }
  validate_encoding(encoding);
  if (disabled(encoding) || encoding == software) {
    throw std::invalid_argument("PICR source event requires a dynamic source");
  }
  current_.sources[encoding] = value;
}

std::uint64_t SharcPicr::read(MemoryAccess access) {
  return working().control[register_index(access)];
}

void SharcPicr::write(MemoryAccess access, std::uint64_t value) {
  const auto index = register_index(access);
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized PICR register write");
  }
  working().control[index] = static_cast<std::uint32_t>(value);
}

void SharcPicr::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested PICR instruction"); }
  staged_ = current_;
}

void SharcPicr::commit_instruction() noexcept {
  if (staged_) { current_ = *staged_; }
  staged_.reset();
}

void SharcPicr::abort_instruction() noexcept {
  staged_.reset();
}

SharcPicrState &SharcPicr::working() {
  return staged_ ? *staged_ : current_;
}

const SharcPicrState &SharcPicr::working() const {
  return staged_ ? *staged_ : current_;
}

}
