// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_dai_timing.hpp"

#include "common/memory.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr unsigned int field_bits = 5;
constexpr unsigned int field_count = 6;
constexpr unsigned int field_mask = 31;
constexpr unsigned int low = 30;
constexpr unsigned int frame_reserved = 27;
constexpr unsigned int maximum_half_periods = 6;
constexpr unsigned int first_possible_effect = 4;
constexpr std::uint32_t pll_field_mask = 0x7c00;

unsigned int selector(std::uint32_t word, unsigned int field) {
  return (word >> (field * field_bits)) & field_mask;
}

SharcDaiTimingState reset(SharcDaiTimingGroup group) {
  constexpr std::array<std::uint32_t, 6> clocks{
      0x252630c2, 0x3def7bde, 0x3def7bde, 0x3def7bde, 0x3def7bde, 0x3def7bde};
  constexpr std::array<std::uint32_t, 6> frames{
      0x2736b4e3, 0x3def7bde, 0x3def7bde, 0x01ef7bde, 0x000003de, 0};
  switch (group) {
    case SharcDaiTimingGroup::clock: return {.routes = clocks};
    case SharcDaiTimingGroup::frame: return {.routes = frames};
    default: throw std::invalid_argument("Invalid SHARC DAI timing group");
  }
}

std::optional<bool> level(const SharcDaiTimingState &state, unsigned int encoding) {
  if (encoding >= low) { return encoding == field_mask; }
  return state.sources[encoding];
}

std::optional<bool> routed_level(const SharcDaiTimingState &state,
                                 SharcDaiTimingGroup group,
                                 SharcDaiTimingTap tap,
                                 std::uint32_t word) {
  const auto encoding = selector(word, tap.field);
  constexpr unsigned int pcg_a_clock = 28;
  constexpr unsigned int pcg_b_clock = 29;
  // HWR15-4: the two PCG clock outputs cannot feed their own EXT input.
  if (group == SharcDaiTimingGroup::clock && tap.word == 4 &&
      ((tap.field == 0 && encoding == pcg_a_clock) ||
       (tap.field == 1 && encoding == pcg_b_clock))) {
    return false;
  }
  return level(state, encoding);
}

}

SharcDaiTiming::SharcDaiTiming(SharcPower &power,
                               SharcDaiTimingGroup group,
                               std::optional<SharcDaiTimingState> initial)
    : power_(power), group_(group), current_(reset(group)) {
  if (initial) { current_ = *initial; }
  validate(current_);
}

unsigned int SharcDaiTiming::fields(unsigned int word) const {
  constexpr std::array<unsigned int, 6> clock_fields{63, 63, 63, 63, 59, 63};
  constexpr std::array<unsigned int, 6> frame_fields{63, 63, 63, 31, 3, 0};
  if (word >= clock_fields.size()) {
    throw std::out_of_range("Invalid SHARC DAI timing register index");
  }
  return group_ == SharcDaiTimingGroup::clock ? clock_fields[word] : frame_fields[word];
}

void SharcDaiTiming::validate_word(RegisterValue word) const {
  if (group_ != SharcDaiTimingGroup::frame) { return; }
  const auto mask = fields(word.index);
  for (unsigned int field = 0; field < field_count; ++field) {
    if ((mask & (1U << field)) != 0 && selector(word.value, field) == frame_reserved) {
      throw std::invalid_argument("Reserved SHARC DAI frame source27");
    }
  }
}

void SharcDaiTiming::validate_change(unsigned int word,
                                     std::uint32_t old,
                                     std::uint32_t target) const {
  if (group_ == SharcDaiTimingGroup::clock && word == 4 && ((old ^ target) & pll_field_mask) != 0) {
    throw std::runtime_error("SHARC CLK4 field2 PLL destination conflict not qualified");
  }
}

void SharcDaiTiming::validate(const SharcDaiTimingState &state) const {
  if (group_ == SharcDaiTimingGroup::frame && state.sources[frame_reserved]) {
    throw std::invalid_argument("Reserved SHARC DAI frame source27 input");
  }
  for (unsigned int word = 0; word < state.routes.size(); ++word) {
    const auto pending = state.pending[word];
    if (fields(word) == 0 && (state.routes[word] != 0 || pending)) {
      throw std::invalid_argument("Nonexistent SHARC DAI timing register state");
    }
    validate_word({.index = word, .value = state.routes[word]});
    if (group_ == SharcDaiTimingGroup::clock && word == 4) {
      validate_change(word, reset(group_).routes[word], state.routes[word]);
    }
    if (pending) {
      validate_word({.index = word, .value = pending->target});
      validate_change(word, state.routes[word], pending->target);
      if (pending->half_periods >= maximum_half_periods) {
        throw std::invalid_argument("Invalid SHARC DAI timing pending interval");
      }
    }
  }
}

SharcDaiTimingState SharcDaiTiming::state() const {
  return working();
}

SharcDaiTimingState SharcDaiTiming::snapshot() const {
  if (staged_) { throw std::logic_error("DAI timing snapshot during active instruction"); }
  return current_;
}

std::optional<bool> SharcDaiTiming::output(SharcDaiTimingTap tap) const {
  require_clock();
  if (tap.field >= field_count || (fields(tap.word) & (1U << tap.field)) == 0) {
    throw std::out_of_range("Unqualified/nonexistent SHARC DAI timing destination");
  }
  auto value = routed_level(current_, group_, tap, current_.routes[tap.word]);
  const auto pending = current_.pending[tap.word];
  if (pending && pending->half_periods >= first_possible_effect) {
    const auto target = routed_level(current_, group_, tap, pending->target);
    if (value != target) { value.reset(); }
  }
  return value;
}

void SharcDaiTiming::source(unsigned int encoding, std::optional<bool> value) {
  if (staged_) { throw std::logic_error("DAI timing source during active instruction"); }
  if (encoding >= low || (group_ == SharcDaiTimingGroup::frame && encoding == frame_reserved)) {
    throw std::out_of_range("Invalid/reserved SHARC DAI timing source input");
  }
  current_.sources[encoding] = value;
}

void SharcDaiTiming::advance(std::uint64_t half_periods) {
  if (staged_) { throw std::logic_error("DAI timing PCLK during active instruction"); }
  require_clock();
  for (unsigned int word = 0; word < current_.pending.size(); ++word) {
    auto &pending = current_.pending[word];
    if (!pending) { continue; }
    if (half_periods >= maximum_half_periods - pending->half_periods) {
      current_.routes[word] = pending->target;
      pending.reset();
    } else {
      pending->half_periods += static_cast<unsigned int>(half_periods);
    }
  }
}

void SharcDaiTiming::advance_pclk(std::uint32_t clocks) {
  advance(std::uint64_t{clocks} * 2);
}

void SharcDaiTiming::advance_half_pclk(std::uint32_t half_periods) {
  advance(half_periods);
}

unsigned int SharcDaiTiming::register_index(MemoryAccess access) const {
  constexpr unsigned int register_bytes = 4;
  constexpr unsigned int clock_words = 6;
  constexpr unsigned int frame_words = 5;
  const auto words = group_ == SharcDaiTimingGroup::clock ? clock_words : frame_words;
  if (access.size != register_bytes || access.address % register_bytes != 0 ||
      access.address / register_bytes >= words) {
    throw std::out_of_range("SHARC DAI timing requires aligned32-bit register access");
  }
  return access.address / register_bytes;
}

std::uint64_t SharcDaiTiming::read(MemoryAccess access) {
  const auto word = register_index(access);
  require_clock();
  const auto &state = working();
  if (state.pending[word]) {
    throw std::runtime_error("SHARC DAI timing unsettled readback not qualified");
  }
  return state.routes[word];
}

void SharcDaiTiming::write(MemoryAccess access, std::uint64_t value) {
  const auto word = register_index(access);
  require_clock();
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::out_of_range("Oversized SHARC DAI timing write");
  }
  const auto target = static_cast<std::uint32_t>(value);
  validate_word({.index = word, .value = target});
  auto &state = working();
  if (state.pending[word]) {
    throw std::runtime_error("SHARC DAI timing overlapping write not qualified");
  }
  validate_change(word, state.routes[word], target);
  state.pending[word] = SharcDaiTimingPending{.target = target};
}

void SharcDaiTiming::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested DAI timing instruction"); }
  staged_ = current_;
}

void SharcDaiTiming::commit_instruction() noexcept {
  if (staged_) { current_ = *staged_; }
  staged_.reset();
}

void SharcDaiTiming::abort_instruction() noexcept {
  staged_.reset();
}

void SharcDaiTiming::require_clock() const {
  if (!power_.enabled(SharcClockGate::dai)) {
    throw std::runtime_error("DAI timing with stopped DAI clock not qualified");
  }
}

SharcDaiTimingState &SharcDaiTiming::working() {
  return staged_ ? *staged_ : current_;
}

const SharcDaiTimingState &SharcDaiTiming::working() const {
  return staged_ ? *staged_ : current_;
}

}
