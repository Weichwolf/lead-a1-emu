// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_pcg.hpp"

#include "common/memory.hpp"
#include "sharc/peripherals/sharc_pcg_internal.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace pcg_detail {

unsigned int control_word(unsigned int unit) {
  return ((unit / 2) * 6) + ((unit % 2) * 2);
}

std::uint8_t mask(unsigned int unit, unsigned int lane) {
  return static_cast<std::uint8_t>(1U << ((unit * 2) + lane));
}

bool enabled(const SharcPcgState &state, unsigned int unit, unsigned int lane) {
  constexpr std::uint32_t clock = 0x80000000;
  constexpr std::uint32_t frame = 0x40000000;
  return (state.registers[control_word(unit)] & (lane == 0 ? clock : frame)) != 0;
}

std::uint32_t divider(const SharcPcgState &state, unsigned int unit, unsigned int lane) {
  return state.registers[control_word(unit) + (lane == 0 ? 1U : 0U)] & divider_mask;
}

std::uint32_t phase(const SharcPcgState &state, unsigned int unit) {
  constexpr unsigned int shift = 20;
  constexpr unsigned int half_bits = 10;
  const auto first = control_word(unit);
  return (((state.registers[first] >> shift) & ((1U << half_bits) - 1)) << half_bits) |
         ((state.registers[first + 1] >> shift) & ((1U << half_bits) - 1));
}

std::uint32_t pulse(const SharcPcgState &state, unsigned int unit) {
  constexpr unsigned int shift = 16;
  constexpr unsigned int field = 0xffff;
  return (state.registers[((unit / 2) * 6) + 4] >> ((unit % 2) * shift)) & field;
}

std::uint32_t sync(const SharcPcgState &state, unsigned int unit) {
  constexpr unsigned int shift = 16;
  return (state.registers[((unit / 2) * 6) + 5] >> ((unit % 2) * shift)) & 15U;
}

unsigned int source(const SharcPcgState &state, Tap tap) {
  const auto selected = sync(state, tap.unit);
  const auto override = tap.lane == 0 ? 4U : 8U;
  if ((selected & override) != 0) { return 1; } // PCLK overrides CLKIN/EXT.
  constexpr std::uint32_t clock_external = 0x80000000;
  constexpr std::uint32_t frame_external = 0x40000000;
  const auto external = tap.lane == 0 ? clock_external : frame_external;
  return (state.registers[control_word(tap.unit) + 1] & external) != 0 ? tap.unit + 2 : 0U;
}

namespace {

bool changed_parameters(const SharcPcgState &a, const SharcPcgState &b, unsigned int unit) {
  const auto first = control_word(unit);
  constexpr std::uint32_t non_enable = 0x3fffffff;
  return ((a.registers[first] ^ b.registers[first]) & non_enable) != 0 ||
         a.registers[first + 1] != b.registers[first + 1] || pulse(a, unit) != pulse(b, unit) ||
         sync(a, unit) != sync(b, unit);
}

void configure_lane(SharcPcgState &next, const SharcPcgState &before, Tap tap) {
  const auto was_enabled = enabled(before, tap.unit, tap.lane);
  const auto now_enabled = enabled(next, tap.unit, tap.lane);
  auto &runtime = next.lanes[tap.unit][tap.lane];
  if (was_enabled == now_enabled) { return; }
  if (!now_enabled) {
    runtime = {.stop_half_periods = 2};
  } else {
    if (runtime.stop_half_periods != 0) {
      throw std::runtime_error("PCG reenable before source-period disable settled");
    }
    runtime = {.startup = divider(next, tap.unit, tap.lane) > 1 ? 3U : 1U,
               .lost_phase = !next.inputs[source(next, tap)].has_value()};
  }
  next.active |= mask(tap.unit, tap.lane);
}

}

void configure(SharcPcgState &next, const SharcPcgState &before) {
  for (unsigned int unit = 0; unit < units; ++unit) {
    const bool running = (before.active & (mask(unit, 0) | mask(unit, 1))) != 0;
    if (running && changed_parameters(before, next, unit)) {
      throw std::runtime_error("PCG parameters changed before source-period disable settled");
    }
    const bool both_before = enabled(before, unit, 0) && enabled(before, unit, 1);
    const bool both_after = enabled(next, unit, 0) && enabled(next, unit, 1);
    if (before.paired[unit] && both_before && !both_after &&
        (enabled(next, unit, 0) || enabled(next, unit, 1))) {
      throw std::runtime_error("PCG paired phase disable requires both enable bits together");
    }
    if (!running) { next.paired[unit] = both_after; }
    for (unsigned int lane = 0; lane < lanes; ++lane) {
      configure_lane(next, before, {.unit = unit, .lane = lane});
    }
  }
}

namespace {
void validate_lane(const SharcPcgState &state, Tap tap) {
  const auto &runtime = state.lanes[tap.unit][tap.lane];
  const bool running = enabled(state, tap.unit, tap.lane);
  const auto div = divider(state, tap.unit, tap.lane);
  const auto maximum_startup = div > 1 ? 3U : 1U;
  if (!running) {
    if (runtime.stop_half_periods > 2 ||
        runtime != SharcPcgLane{.stop_half_periods = runtime.stop_half_periods}) {
      throw std::invalid_argument("Malformed PCG disabled lane phase");
    }
    return;
  }
  const bool unstarted = runtime.startup != 0;
  if (runtime.startup > maximum_startup || runtime.stop_half_periods != 0 ||
      (!runtime.lost_phase && !state.inputs[source(state, tap)])) {
    throw std::invalid_argument("Malformed PCG source/startup phase");
  }
  if (runtime.positioned &&
      (unstarted || div <= 1 || runtime.position >= div || runtime.lost_phase)) {
    throw std::invalid_argument("Malformed PCG divider position");
  }
  if (!runtime.lost_phase && !unstarted && div > 1 && !runtime.positioned) {
    throw std::invalid_argument("Malformed PCG missing divider position");
  }
  if (runtime.output && (unstarted || runtime.lost_phase ||
                         (div > 1 && (!runtime.positioned || !runtime.full_frame)))) {
    throw std::invalid_argument("Malformed PCG output phase");
  }
}
}

void validate(const SharcPcgState &state) {
  std::uint8_t active{};
  for (unsigned int unit = 0; unit < units; ++unit) {
    if (state.paired[unit] && enabled(state, unit, 0) != enabled(state, unit, 1)) {
      throw std::invalid_argument("Malformed PCG paired enables");
    }
    for (unsigned int lane = 0; lane < lanes; ++lane) {
      validate_lane(state, {.unit = unit, .lane = lane});
      if (enabled(state, unit, lane) || state.lanes[unit][lane].stop_half_periods != 0) {
        active |= mask(unit, lane);
      }
    }
  }
  if (active != state.active) { throw std::invalid_argument("Malformed PCG active owner mask"); }
}

}

namespace {

unsigned int input_index(SharcPcgInput input) {
  constexpr unsigned int inputs = 14;
  const auto index = static_cast<unsigned int>(input);
  if (index >= inputs) { throw std::invalid_argument("Invalid PCG input identity"); }
  return index;
}

unsigned int register_index(MemoryAccess access) {
  constexpr unsigned int bytes = 4;
  constexpr unsigned int mapped = 48;
  if (access.size != bytes || access.address % bytes != 0 || access.address >= mapped) {
    throw std::runtime_error("PCG requires aligned32-bit native register offsets0..44");
  }
  return access.address / bytes;
}

}

SharcPcg::SharcPcg(SharcPower &power, SharcPcgState initial) : power_(power), current_(initial) {
  pcg_detail::validate(current_);
}

SharcPcgState SharcPcg::snapshot() const {
  idle();
  return current_;
}

SharcPcgSignals SharcPcg::signals(unsigned int unit) const {
  require_clock();
  if (unit >= current_.lanes.size()) { throw std::invalid_argument("Invalid PCG unit"); }
  return {.clock = current_.lanes[unit][0].output, .frame = current_.lanes[unit][1].output};
}

void SharcPcg::observe(SharcPcgInput input, std::optional<bool> level) {
  idle();
  require_clock();
  const auto index = input_index(input);
  if (current_.active == 0) {
    current_.inputs[index] = level;
    return;
  }
  auto next = current_;
  const auto before = next.inputs[index];
  next.inputs[index] = level;
  for (unsigned int unit = 0; unit < pcg_detail::units; ++unit) {
    for (unsigned int lane = 0; lane < pcg_detail::lanes; ++lane) {
      if (pcg_detail::enabled(next, unit, lane) &&
          pcg_detail::source(next, {.unit = unit, .lane = lane}) == index) {
        pcg_detail::edge(next, unit, lane, before, level);
      }
    }
  }
  current_ = next;
}

void SharcPcg::advance_half_periods(SharcPcgInput input, std::uint32_t half_periods) {
  idle();
  require_clock();
  const auto index = input_index(input);
  if (current_.active == 0 || half_periods == 0) { return; }
  for (unsigned int unit = 0; unit < pcg_detail::units; ++unit) {
    for (unsigned int lane = 0; lane < pcg_detail::lanes; ++lane) {
      auto &runtime = current_.lanes[unit][lane];
      auto &remaining = runtime.stop_half_periods;
      if (remaining == 0 || pcg_detail::source(current_, {.unit = unit, .lane = lane}) != index) {
        continue;
      }
      if (half_periods >= remaining) {
        runtime = {};
        current_.active &= static_cast<std::uint8_t>(~pcg_detail::mask(unit, lane));
      } else {
        remaining -= half_periods;
      }
    }
  }
}

std::uint64_t SharcPcg::read(MemoryAccess access) {
  require_clock();
  return working().registers[register_index(access)];
}

void SharcPcg::write(MemoryAccess access, std::uint64_t value) {
  require_clock();
  const auto index = register_index(access);
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized PCG register write");
  }
  const auto before = working();
  auto next = before;
  next.registers[index] = static_cast<std::uint32_t>(value);
  pcg_detail::configure(next, before);
  working() = next;
}

void SharcPcg::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested PCG instruction"); }
  staged_ = current_;
}

void SharcPcg::commit_instruction() noexcept {
  if (staged_) { current_ = *staged_; }
  staged_.reset();
}

void SharcPcg::abort_instruction() noexcept {
  staged_.reset();
}

void SharcPcg::idle() const {
  if (staged_) { throw std::logic_error("PCG event/snapshot during active instruction"); }
}

void SharcPcg::require_clock() const {
  if (!power_.enabled(SharcClockGate::dai)) {
    throw std::runtime_error("PCG with stopped DAI peripheral clock not qualified");
  }
}

SharcPcgState &SharcPcg::working() {
  return staged_ ? *staged_ : current_;
}

}
