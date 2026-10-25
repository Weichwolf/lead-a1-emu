// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_dai_data.hpp"

#include "common/memory.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr unsigned int fields = 5;
constexpr unsigned int field_bits = 6;
constexpr unsigned int selector_mask = 63;
constexpr unsigned int low_source = 62;
constexpr unsigned int earliest_effect = 4;
constexpr unsigned int guaranteed_effect = 6;

bool dynamic_source(unsigned int encoding) {
  constexpr unsigned int dir_data = 0x28;
  constexpr unsigned int sport6_data_a = 0x2c;
  constexpr unsigned int dit_stream = 0x30;
  return encoding <= dir_data || (encoding >= sport6_data_a && encoding <= dit_stream);
}

unsigned int selector(std::uint32_t word, unsigned int destination) {
  return (word >> ((destination % fields) * field_bits)) & selector_mask;
}

struct RouteWord {
  std::uint32_t value;
  unsigned int index;
};

void validate_word(RouteWord word) {
  constexpr unsigned int final_register = 6;
  constexpr unsigned int final_fields = 4;
  const auto count = word.index == final_register ? final_fields : fields;
  for (unsigned int destination = 0; destination < count; ++destination) {
    const auto encoding = selector(word.value, destination);
    if (!dynamic_source(encoding) && encoding < low_source) {
      throw std::invalid_argument("Reserved SHARC DAI data source selection");
    }
  }
}

void validate(const SharcDaiDataState &state) {
  for (unsigned int index = 0; index < state.routes.size(); ++index) {
    validate_word({.value = state.routes[index], .index = index});
    const auto pending = state.pending[index];
    if (pending) {
      validate_word({.value = pending->target, .index = index});
      if (pending->half_periods >= guaranteed_effect) {
        throw std::invalid_argument("Invalid SHARC DAI data pending interval");
      }
    }
  }
  for (unsigned int source = 0; source < state.sources.size(); ++source) {
    if (state.sources[source].has_value() && !dynamic_source(source)) {
      throw std::invalid_argument("Reserved SHARC DAI data source input state");
    }
  }
}

unsigned int register_index(MemoryAccess access) {
  constexpr unsigned int bytes = 4;
  constexpr unsigned int mapped_bytes = 28;
  if (access.size != bytes || access.address % bytes != 0 || access.address >= mapped_bytes) {
    throw std::runtime_error("SHARC DAI data routing requires aligned32-bit mapped offsets0..24");
  }
  return access.address / bytes;
}

std::optional<bool>
routed(const SharcDaiDataState &state, std::uint32_t word, unsigned int destination) {
  const auto encoding = selector(word, destination);
  return encoding >= low_source ? std::optional<bool>{encoding != low_source}
                                : state.sources[encoding];
}

}

SharcDaiData::SharcDaiData(SharcPower &power, SharcDaiDataState initial)
    : power_(power), current_(initial) {
  validate(current_);
}

SharcDaiDataState SharcDaiData::state() const {
  return working();
}

SharcDaiDataState SharcDaiData::snapshot() const {
  if (staged_) { throw std::logic_error("SHARC DAI data snapshot during active instruction"); }
  return current_;
}

SharcDaiDataOutputs SharcDaiData::outputs() const {
  require_clock();
  SharcDaiDataOutputs result{};
  for (unsigned int destination = 0; destination < result.size(); ++destination) {
    const auto index = destination / fields;
    auto value = routed(current_, current_.routes[index], destination);
    const auto pending = current_.pending[index];
    if (pending && pending->half_periods >= earliest_effect) {
      const auto target = routed(current_, pending->target, destination);
      if (value != target) { value.reset(); }
    }
    result[destination] = value;
  }
  return result;
}

void SharcDaiData::source(unsigned int encoding, std::optional<bool> value) {
  if (staged_) { throw std::logic_error("SHARC DAI data source event during active instruction"); }
  if (!dynamic_source(encoding)) {
    throw std::invalid_argument("Invalid SHARC DAI data dynamic source encoding");
  }
  current_.sources[encoding] = value;
}

void SharcDaiData::advance(std::uint64_t half_periods) {
  if (staged_) { throw std::logic_error("SHARC DAI data clock event during active instruction"); }
  require_clock();
  for (unsigned int index = 0; index < current_.pending.size(); ++index) {
    auto &pending = current_.pending[index];
    if (!pending) { continue; }
    if (half_periods >= guaranteed_effect - pending->half_periods) {
      current_.routes[index] = pending->target;
      pending.reset();
    } else {
      pending->half_periods += static_cast<unsigned int>(half_periods);
    }
  }
}

void SharcDaiData::advance_pclk(std::uint32_t clocks) {
  advance(std::uint64_t{clocks} * 2);
}

void SharcDaiData::advance_half_pclk(std::uint32_t half_periods) {
  advance(half_periods);
}

std::uint64_t SharcDaiData::read(MemoryAccess access) {
  const auto index = register_index(access);
  require_clock();
  const auto &state = working();
  if (state.pending[index]) {
    throw std::runtime_error("SHARC DAI data unsettled readback timing not qualified");
  }
  return state.routes[index];
}

void SharcDaiData::write(MemoryAccess access, std::uint64_t value) {
  const auto index = register_index(access);
  require_clock();
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized SHARC DAI data write");
  }
  const auto target = static_cast<std::uint32_t>(value);
  validate_word({.value = target, .index = index});
  auto &state = working();
  if (state.pending[index]) {
    throw std::runtime_error("SHARC DAI data overlapping write timing not qualified");
  }
  state.pending[index] = SharcDaiDataPending{.target = target};
}

void SharcDaiData::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested SHARC DAI data routing instruction"); }
  staged_ = current_;
}

void SharcDaiData::commit_instruction() noexcept {
  if (staged_) { current_ = *staged_; }
  staged_.reset();
}

void SharcDaiData::abort_instruction() noexcept {
  staged_.reset();
}

void SharcDaiData::require_clock() const {
  if (!power_.enabled(SharcClockGate::dai)) {
    throw std::runtime_error("SHARC data routing with stopped DAI clock not qualified");
  }
}

SharcDaiDataState &SharcDaiData::working() {
  return staged_ ? *staged_ : current_;
}

const SharcDaiDataState &SharcDaiData::working() const {
  return staged_ ? *staged_ : current_;
}

}
