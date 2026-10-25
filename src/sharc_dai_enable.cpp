// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_dai_enable.hpp"

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_power.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr unsigned int fields = 5;
constexpr unsigned int field_bits = 6;
constexpr std::uint32_t field_mask = 63;
constexpr unsigned int first_dynamic = 2;
constexpr unsigned int source_count = 48;
constexpr unsigned int maximum_latency = 3;

unsigned int selector(std::uint32_t word, unsigned int field) {
  return (word >> (field * field_bits)) & field_mask;
}

void validate_word(std::uint32_t word) {
  for (unsigned int field = 0; field < fields; ++field) {
    if (selector(word, field) >= source_count) {
      throw std::invalid_argument("Reserved SHARC DAI enable source selection");
    }
  }
}

void validate(const SharcDaiEnableState &state) {
  for (unsigned int index = 0; index < state.routes.size(); ++index) {
    validate_word(state.routes[index]);
    const auto pending = state.pending[index];
    if (pending) {
      validate_word(pending->target);
      if (pending->elapsed_pclk >= maximum_latency) {
        throw std::invalid_argument("Invalid SHARC DAI enable pending interval");
      }
    }
  }
}

unsigned int register_index(MemoryAccess access) {
  constexpr unsigned int register_bytes = 4;
  constexpr unsigned int mapped_bytes = 16;
  if (access.address >= mapped_bytes || access.address % register_bytes != 0 ||
      access.size != register_bytes) {
    throw std::runtime_error("SHARC DAI enable requires aligned32-bit mapped offsets0..12");
  }
  return access.address / register_bytes;
}

std::optional<bool> level(const SharcDaiEnableState &state, unsigned int encoding) {
  if (encoding < first_dynamic) { return encoding == 1; }
  return state.sources[encoding - first_dynamic];
}

}

SharcDaiEnable::SharcDaiEnable(SharcPower &power, SharcDaiEnableState initial)
    : power_(power), current_(initial) {
  validate(current_);
}

SharcDaiEnableState SharcDaiEnable::state() const {
  return working();
}

SharcDaiEnableState SharcDaiEnable::snapshot() const {
  if (staged_) { throw std::logic_error("SHARC DAI enable snapshot during active instruction"); }
  return current_;
}

SharcDaiEnableOutputs SharcDaiEnable::outputs() const {
  require_clock();
  SharcDaiEnableOutputs result{};
  for (unsigned int pin = 0; pin < result.size(); ++pin) {
    const auto index = pin / fields;
    const auto field = pin % fields;
    auto value = level(current_, selector(current_.routes[index], field));
    const auto pending = current_.pending[index];
    if (pending && pending->elapsed_pclk >= 2) {
      const auto target = level(current_, selector(pending->target, field));
      if (value != target) { value.reset(); }
    }
    result[pin] = value;
  }
  return result;
}

void SharcDaiEnable::source(unsigned int encoding, std::optional<bool> value) {
  if (staged_) { throw std::logic_error("SHARC DAI source during active instruction"); }
  if (encoding < first_dynamic || encoding >= source_count) {
    throw std::invalid_argument("Invalid SHARC DAI dynamic source encoding");
  }
  current_.sources[encoding - first_dynamic] = value;
}

void SharcDaiEnable::advance_pclk(std::uint32_t clocks) {
  if (staged_) { throw std::logic_error("SHARC DAI PCLK during active instruction"); }
  require_clock();
  for (unsigned int index = 0; index < current_.pending.size(); ++index) {
    auto &pending = current_.pending[index];
    if (!pending) { continue; }
    if (clocks >= maximum_latency - pending->elapsed_pclk) {
      current_.routes[index] = pending->target;
      pending.reset();
    } else {
      pending->elapsed_pclk += clocks;
    }
  }
}

std::uint64_t SharcDaiEnable::read(MemoryAccess access) {
  const auto index = register_index(access);
  require_clock();
  const auto &state = working();
  if (state.pending[index]) {
    throw std::runtime_error("SHARC DAI enable unsettled readback timing not qualified");
  }
  return state.routes[index];
}

void SharcDaiEnable::write(MemoryAccess access, std::uint64_t value) {
  const auto index = register_index(access);
  require_clock();
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized SHARC DAI enable write");
  }
  const auto target = static_cast<std::uint32_t>(value);
  validate_word(target);
  auto &state = working();
  if (state.pending[index]) {
    throw std::runtime_error("SHARC DAI enable overlapping write timing not qualified");
  }
  state.pending[index] = SharcDaiEnablePending{.target = target};
}

void SharcDaiEnable::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested SHARC DAI enable instruction"); }
  staged_ = current_;
}

void SharcDaiEnable::commit_instruction() noexcept {
  if (staged_) {
    current_ = *staged_;
    staged_.reset();
  }
}

void SharcDaiEnable::abort_instruction() noexcept {
  staged_.reset();
}

void SharcDaiEnable::require_clock() const {
  if (!power_.enabled(SharcClockGate::dai)) {
    throw std::runtime_error("SHARC routing with stopped DAI clock not qualified");
  }
}

SharcDaiEnableState &SharcDaiEnable::working() {
  return staged_ ? *staged_ : current_;
}

const SharcDaiEnableState &SharcDaiEnable::working() const {
  return staged_ ? *staged_ : current_;
}

}
