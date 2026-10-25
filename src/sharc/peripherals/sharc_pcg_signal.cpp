// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_pcg.hpp"
#include "sharc/peripherals/sharc_pcg_internal.hpp"

#include <optional>
#include <stdexcept>

namespace lead_a1::pcg_detail {
namespace {

void require_mode(const SharcPcgState &state, unsigned int unit, unsigned int lane) {
  const auto flags = sync(state, unit);
  if ((flags & (lane == 0 ? 2U : 1U)) != 0) {
    throw std::runtime_error("PCG external trigger phase not qualified");
  }
  if (lane == 0) { return; }
  const auto div = divider(state, unit, lane);
  if (div <= 1) {
    if ((pulse(state, unit) & 1U) != 0) {
      throw std::runtime_error("PCG one-shot strobe phase not qualified");
    }
    return;
  }
  if (pulse(state, unit) >= div || phase(state, unit) > div) {
    throw std::runtime_error("PCG frame pulse/phase exceeds divider reference");
  }
  if (phase(state, unit) != 0 &&
      (!state.paired[unit] ||
       source(state, {.unit = unit, .lane = 0}) != source(state, {.unit = unit, .lane = 1}) ||
       div % (divider(state, unit, 0) > 1 ? divider(state, unit, 0) : 1U) != 0)) {
    throw std::runtime_error("PCG nonzero phase requires qualified synchronous pair");
  }
}

void update(SharcPcgState &state, unsigned int unit, unsigned int lane) {
  auto &runtime = state.lanes[unit][lane];
  const auto div = divider(state, unit, lane);
  if (!runtime.positioned) {
    runtime.positioned = true;
    runtime.position = lane == 0 ? 0U : phase(state, unit) % div;
    // Nonzero phase can truncate the first pulse. Do not invent its waveform;
    // publish only the first complete frame in the documented steady phase.
    runtime.full_frame = lane == 0 || runtime.position == 0;
  } else {
    runtime.position = (runtime.position + 1) % div;
    if (runtime.position == 0) { runtime.full_frame = true; }
  }
  if (!runtime.full_frame) {
    runtime.output.reset();
    return;
  }
  auto high = div / 2;
  if (lane == 1 && pulse(state, unit) != 0) { high = pulse(state, unit); }
  runtime.output = runtime.position < high;
}

}

void edge(SharcPcgState &state,
          unsigned int unit,
          unsigned int lane,
          std::optional<bool> before,
          std::optional<bool> after) {
  auto &runtime = state.lanes[unit][lane];
  if (!after) {
    runtime.lost_phase = true;
    runtime.positioned = false;
    runtime.output.reset();
    return;
  }
  const auto div = divider(state, unit, lane);
  const bool rising = before == false && after == true;
  if (!before) { runtime.lost_phase = true; }
  if (runtime.lost_phase && div > 1) {
    runtime.output.reset();
    return;
  }
  require_mode(state, unit, lane);
  if (runtime.startup != 0) {
    if (!rising) { return; }
    --runtime.startup;
    if (runtime.startup != 0) { return; }
  }
  if (div <= 1) {
    runtime.lost_phase = false;
    const bool inverted = lane == 1 && (pulse(state, unit) & 2U) != 0;
    runtime.output = *after != inverted;
  } else if (rising) {
    update(state, unit, lane);
  }
}

}
