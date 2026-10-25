// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_sport.hpp"
#include "sharc/peripherals/sharc_sport_internal.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1::sport_detail {
namespace {

constexpr std::array<std::uint32_t, 2> standard_enable{1, 0x01000000};
constexpr std::array<std::uint32_t, 2> multi_enable{1, 0x00800000};
constexpr std::uint32_t frame_required = 0x2000;
constexpr std::uint32_t late_frame = 0x20000;
constexpr std::uint32_t low_frame = 0x10000;
constexpr std::uint32_t sample_rising = 0x1000;
constexpr std::uint32_t edge_frame = 4;

bool tdm(const SharcSportState &state) {
  return (effective(state)[2] & tdm_enables) != 0;
}

bool transmit(const SharcSportState &state) {
  return (effective(state)[0] & transmit_bit) != 0;
}

bool channel_ready(const SharcSportState &state, unsigned int channel) {
  return enabled(effective(state), channel) && ready(state, channel);
}

bool any_ready(const SharcSportState &state) {
  return channel_ready(state, 0) || channel_ready(state, 1);
}

bool selected(const SharcSportState &state) {
  if (!tdm(state)) { return true; }
  constexpr unsigned int register_bits = 32;
  return (state.sampled_select[1][state.slot / register_bits] &
          (1U << (state.slot % register_bits))) != 0;
}

bool frame_level(const SharcSportState &state) {
  const auto frame = state.lines.frame;
  if (!frame) { throw std::runtime_error("Unknown required SPORT frame input"); }
  return *frame != ((effective(state)[0] & low_frame) != 0);
}

unsigned int frame_delay(const SharcSportState &state) {
  constexpr unsigned int delay_mask = 15;
  if (tdm(state)) { return (effective(state)[2] >> 1) & delay_mask; }
  return (effective(state)[0] & late_frame) != 0 ? 0U : 1U;
}

void require_mode(const SharcSportState &state) {
  constexpr std::uint32_t unsupported = 0x0e00; // PACK,ICLK,OPMODE.
  constexpr std::uint32_t internal_frame = 0x4000;
  constexpr std::uint32_t loopback = 0x1000;
  const auto control = effective(state);
  if ((control[0] & (unsupported | internal_frame)) != 0 ||
      (!tdm(state) && (control[2] & loopback) != 0)) {
    throw std::runtime_error("SPORT internal clocks/I2S/packing/loopback not qualified");
  }
  if (tdm(state) && (control[0] & (standard_enable[0] | standard_enable[1])) != 0) {
    throw std::runtime_error("Mixed SPORT standard/TDM enables not qualified");
  }
  if (length(state) < 3) { throw std::runtime_error("Invalid SPORT serial word length"); }
  if (std::ranges::any_of(state.flush, [](auto remaining) { return remaining != 0; })) {
    throw std::runtime_error("SPORT wire activity during disable flush not qualified");
  }
}

void schedule_frame(SharcSportState &state) {
  if (state.pending_frame) {
    throw std::runtime_error("Overlapping SPORT frame pulses not qualified");
  }
  state.pending_frame = frame_delay(state);
  state.frame_edge = false;
}

void detect_frame(SharcSportState &state) {
  if (!tdm(state) && state.frame_active && (effective(state)[0] & frame_required) == 0) {
    state.frame_edge = false;
    return;
  }
  const bool waiting = !state.frame_active && !state.pending_frame;
  if (state.frame_edge) {
    if (!frame_level(state)) {
      throw std::runtime_error("SPORT subcycle frame pulse timing not qualified");
    }
    schedule_frame(state);
    return;
  }
  if (!waiting) { return; }
  const bool edge_only = tdm(state) && (effective(state)[1] & edge_frame) != 0;
  if (frame_level(state) && !edge_only) { schedule_frame(state); }
}

void start_frame(SharcSportState &state) {
  if (state.frame_active) {
    throw std::runtime_error("Premature SPORT frame during current transfer not qualified");
  }
  state.frame_active = true;
  state.word_active = false;
  state.word_selected = false;
  state.slot = 0;
  state.bit = 0;
  state.pending_frame.reset();
}

void check_companding(const SharcSportState &state) {
  constexpr std::uint32_t companding = 4;
  constexpr unsigned int register_bits = 32;
  if (!state.word_selected || !channel_ready(state, 0) || (effective(state)[0] & companding) == 0) {
    return;
  }
  if (!tdm(state) || (state.sampled_compand[1][state.slot / register_bits] &
                      (1U << (state.slot % register_bits))) != 0) {
    throw std::runtime_error("SPORT companding not qualified");
  }
}

void start_word(SharcSportState &state, const DmaStates &dma, Actions &actions) {
  state.word_selected = selected(state);
  check_companding(state);
  state.word_active = true;
  state.bit = 0;
  state.shift.fill(0);
  if (!transmit(state) || !state.word_selected) { return; }
  for (unsigned int channel = 0; channel < dma.size(); ++channel) {
    if (!channel_ready(state, channel)) { continue; }
    const auto buffer = dma[channel].transmit_buffer;
    if (!dma[channel].mode.transmit || !buffer || dma[channel].transmit_shift) {
      throw std::runtime_error("SPORT TX word/buffer ownership or underflow not qualified");
    }
    state.shift[channel] = buffer->value;
    actions[channel] = {.kind = ActionKind::transmit_start};
  }
}

void drive(SharcSportState &state, const DmaStates &dma, Actions &actions) {
  if (state.pending_frame == 0) { start_frame(state); }
  if (state.frame_active && !state.word_active) { start_word(state, dma, actions); }
  if (!state.word_active) {
    state.signals.enable.fill(std::nullopt);
    return;
  }
  constexpr std::uint32_t lsb_first = 8;
  const auto index =
      (effective(state)[0] & lsb_first) != 0 ? state.bit : length(state) - state.bit - 1;
  for (unsigned int channel = 0; channel < dma.size(); ++channel) {
    const bool active = channel_ready(state, channel) && state.word_selected;
    state.signals.enable[channel] = active;
    state.signals.data[channel] =
        active ? std::optional<bool>{((state.shift[channel] >> index) & 1U) != 0} : std::nullopt;
  }
}

void finish_word(SharcSportState &state) {
  state.word_active = false;
  state.word_selected = false;
  state.bit = 0;
  if (tdm(state)) {
    constexpr unsigned int slot_mask = 127;
    constexpr unsigned int slots_shift = 5;
    const auto slots = ((effective(state)[2] >> slots_shift) & slot_mask) + 1;
    ++state.slot;
    if (state.slot == slots) {
      state.slot = 0;
      state.frame_active = false;
    }
  } else if ((effective(state)[0] & frame_required) != 0) {
    state.frame_active = false;
  }
}

void receive_sample(SharcSportState &state) {
  constexpr std::uint32_t lsb_first = 8;
  for (unsigned int channel = 0; channel < state.shift.size(); ++channel) {
    if (!channel_ready(state, channel) || !state.word_selected) { continue; }
    const auto bit = state.lines.data[channel];
    if (!bit) { throw std::runtime_error("Unknown selected SPORT receive data input"); }
    if ((effective(state)[0] & lsb_first) != 0) {
      if (*bit) { state.shift[channel] |= 1U << state.bit; }
    } else {
      state.shift[channel] = static_cast<std::uint32_t>((std::uint64_t{state.shift[channel]} << 1) |
                                                        static_cast<unsigned int>(*bit));
    }
  }
}

void complete_word(SharcSportState &state, const DmaStates &dma, Actions &actions) {
  const auto bits = length(state);
  for (unsigned int channel = 0; channel < dma.size(); ++channel) {
    if (!channel_ready(state, channel) || !state.word_selected) { continue; }
    if (transmit(state)) {
      const auto shift = dma[channel].transmit_shift;
      if (!shift || shift->value != state.shift[channel]) {
        throw std::runtime_error("SPORT serial/DMA shifter ownership mismatch");
      }
      actions[channel] = {.kind = ActionKind::transmit_finish};
    } else {
      if (dma[channel].mode.transmit || dma[channel].receive_buffer) {
        throw std::runtime_error("SPORT receive overflow/buffer ownership not qualified");
      }
      auto value = state.shift[channel];
      constexpr std::uint32_t sign_extend = 2;
      constexpr unsigned int word_bits = 32;
      if ((effective(state)[0] & sign_extend) != 0 && bits < word_bits &&
          (value & (1U << (bits - 1))) != 0) {
        value |= std::numeric_limits<std::uint32_t>::max() << bits;
      }
      actions[channel] = {.kind = ActionKind::receive, .value = value};
    }
  }
  finish_word(state);
}

void sample(SharcSportState &state, const DmaStates &dma, Actions &actions) {
  if (!transmit(state)) {
    detect_frame(state);
    if (state.pending_frame == 0) { start_frame(state); }
    if (state.frame_active && !state.word_active) { start_word(state, dma, actions); }
  }
  if (state.pending_frame && *state.pending_frame != 0) { --*state.pending_frame; }
  if (!state.word_active) { return; }
  if (!transmit(state)) { receive_sample(state); }
  ++state.bit;
  if (state.bit == length(state)) { complete_word(state, dma, actions); }
}

}

std::array<std::uint32_t, 3> effective(const SharcSportState &state) {
  auto result = state.sampled[1];
  result[2] = (result[2] & ~tdm_enables) | (state.sampled[2][2] & tdm_enables);
  return result;
}

bool enabled(const std::array<std::uint32_t, 3> &control, unsigned int channel) {
  return (control[0] & standard_enable[channel]) != 0 || (control[2] & multi_enable[channel]) != 0;
}

bool ready(const SharcSportState &state, unsigned int channel) {
  return (state.sampled[2][0] & standard_enable[channel]) != 0 ||
         (state.sampled[2][2] & multi_enable[channel]) != 0;
}

unsigned int length(const SharcSportState &state) {
  constexpr unsigned int shift = 4;
  constexpr unsigned int mask = 31;
  return ((effective(state)[0] >> shift) & mask) + 1;
}

std::uint32_t buffer_status(const DmaStates &dma) {
  std::uint32_t result{};
  constexpr std::array<unsigned int, 2> shifts{30, 27};
  for (unsigned int channel = 0; channel < dma.size(); ++channel) {
    const auto &state = dma[channel];
    const unsigned int words = state.mode.transmit
                                   ? static_cast<unsigned int>(state.transmit_buffer.has_value()) +
                                         static_cast<unsigned int>(state.transmit_shift.has_value())
                                   : static_cast<unsigned int>(state.receive_buffer.has_value());
    unsigned int status{};
    if (words != 0) { status = words == 1 ? 2U : 3U; }
    result |= status << shifts[channel];
  }
  return result;
}

SharcSportSignals outputs(const SharcSportState &state) {
  auto result = state.signals;
  for (unsigned int channel = 0; channel < result.data.size(); ++channel) {
    if (!channel_ready(state, channel) || !transmit(state)) {
      result.enable[channel] = false;
      result.data[channel].reset();
    } else if (!tdm(state)) {
      result.enable[channel] = true;
    }
  }
  return result;
}

void validate(const SharcSportState &state) {
  constexpr unsigned int slots = 128;
  constexpr unsigned int bits = 32;
  if (state.slot >= slots || state.bit >= bits ||
      (state.pending_frame && *state.pending_frame > 15) ||
      (!state.word_active && (state.bit != 0 || state.word_selected)) ||
      (state.word_active && (!state.frame_active || state.bit >= length(state))) ||
      std::ranges::any_of(state.flush, [](auto clocks) { return clocks > flush_clocks; })) {
    throw std::invalid_argument("Malformed SPORT serial phase");
  }
  for (const auto &sampled : state.sampled) {
    if ((sampled[0] & ~control_mask) != 0 || (sampled[2] & ~multichannel_mask) != 0) {
      throw std::invalid_argument("Malformed SPORT sampled control bits");
    }
  }
  if ((state.control[0] & ~control_mask) != 0 || (state.control[2] & ~multichannel_mask) != 0) {
    throw std::invalid_argument("Malformed SPORT raw control bits");
  }
}

Actions wire(SharcSportState &state, const SharcSportLines &lines, const DmaStates &dma) {
  Actions actions{};
  const auto before = state.lines;
  state.lines = lines;
  if (!any_ready(state)) {
    state.frame_edge = false;
    return actions;
  }
  require_mode(state);
  const auto control = effective(state);
  for (unsigned int channel = 0; channel < state.flush.size(); ++channel) {
    if (enabled(control, channel) && !enabled(state.control, channel)) {
      throw std::runtime_error("SPORT disable synchronization interval not qualified");
    }
  }
  if (!lines.clock) { throw std::runtime_error("Unknown required SPORT clock input"); }
  // An unknown FS may contain a new frame even during the current word.
  // Only an already-running FSR0 standard stream ignores subsequent FS.
  if (tdm(state) || !state.frame_active || (control[0] & frame_required) != 0) {
    static_cast<void>(frame_level(state));
  }
  if (before.frame && lines.frame && *before.frame != *lines.frame && frame_level(state)) {
    state.frame_edge = true;
  }
  if (transmit(state)) {
    detect_frame(state);
    if (state.pending_frame == 0) { drive(state, dma, actions); }
  }
  if (!before.clock || before.clock == lines.clock) { return actions; }
  const bool sampling = *lines.clock == ((effective(state)[0] & sample_rising) != 0);
  if (sampling) {
    sample(state, dma, actions);
  } else if (transmit(state)) {
    drive(state, dma, actions);
  }
  return actions;
}

}
