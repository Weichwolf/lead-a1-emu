// SPDX-License-Identifier: GPL-3.0-only
#include "arm/peripherals/spi.hpp"
#include "arm/peripherals/spi_internal.hpp"

#include <bit>
#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t byte_mask = 0xff;
constexpr std::uint32_t character_mask = 31;
constexpr std::uint32_t bit_error = 16;
constexpr std::uint32_t parity_error = 4;
constexpr std::uint32_t timers_disabled = 0x40000;
constexpr std::uint32_t wait_enabled = 0x200000;
constexpr std::uint32_t powerdown = 0x100;
constexpr unsigned int byte_bits = 8;
constexpr unsigned int format_shift = 24;

unsigned int half_period(const SpiTransfer &transfer) {
  return ((transfer.format >> byte_bits) & byte_mask) + 1;
}

unsigned int bit_count(const SpiTransfer &transfer) {
  return (transfer.format & character_mask) + ((transfer.format & spi_parity) != 0 ? 1U : 0U);
}

bool output_bit(const SpiTransfer &transfer, unsigned int index) {
  const auto length = transfer.format & character_mask;
  if (index == length) {
    const auto mask = (1U << length) - 1;
    const bool odd = (std::popcount(transfer.word.data & mask) & 1) != 0;
    return odd != ((transfer.format & spi_odd) != 0);
  }
  const auto position = (transfer.format & spi_reverse) != 0 ? index : length - index - 1;
  return ((transfer.word.data >> position) & 1) != 0;
}

void receive_word(SpiState &state, const SpiTransfer &transfer) {
  const SpiReceived received{.data = transfer.received, .errors = transfer.errors};
  if (!state.visible_full) {
    state.visible = received;
    state.visible_full = true;
    state.flags |= spi_rx_flag;
  } else {
    if (state.receive) { state.flags |= spi_overrun; }
    state.receive = received;
  }
}

void start(SpiState &state, SpiWord word, bool consecutive) {
  const auto format = state.formats[(word.control >> format_shift) & 3];
  const auto length = format & character_mask;
  const auto divisor = (format >> byte_bits) & byte_mask;
  if (length < 2 || length > 16 || divisor < 2) {
    throw std::runtime_error("Indeterminate SPI character length or unsupported prescale");
  }
  if ((format & wait_enabled) != 0 || (state.pin_function & 0x100) != 0) {
    throw std::runtime_error("Unsupported SPI enable-pin handshaking");
  }
  if (!state.previous_hold) { throw std::runtime_error("Unknown SPI previous CSHOLD input"); }
  const bool retained = state.held || (!state.transfer_started && *state.previous_hold);
  const auto selected = static_cast<std::uint8_t>((word.control >> 16) & state.chip_selects);
  if (state.held && selected != state.selected) {
    throw std::runtime_error("Unqualified SPI held chip-select change");
  }
  state.selected = selected;
  state.clock = (format & spi_polarity) != 0;
  state.active = SpiTransfer{.word = word, .format = format};
  if ((format & spi_phase) != 0) { state.data = output_bit(*state.active, 0); }
  const auto setup = state.delay >> format_shift;
  const bool timers = (format & timers_disabled) == 0 &&
                      (state.pin_function & state.chip_selects) != 0 && !retained;
  state.event_delay = timers && setup != 0 ? (setup + 2) * 2 : 0;
  if ((format & spi_phase) != 0 || consecutive) { state.event_delay += divisor + 1; }
  state.flags |= spi_tx_flag;
  state.transfer_started = true;
}

void finish_hold(SpiState &state) {
  if (!state.active) { throw std::logic_error("Missing SPI hold transfer"); }
  auto &transfer = *state.active;
  state.held = (transfer.word.control & spi_hold) != 0;
  if (!state.held) { state.selected = state.defaults; }
  const bool word_delay = (transfer.word.control & spi_word_delay) != 0;
  const auto count = (transfer.format >> format_shift) & 0x3f;
  const auto idle_delay = state.held ? 0U : 4U;
  state.event_delay = word_delay ? (count + 2) * 2 : idle_delay;
  transfer.stage = SpiStage::gap;
}

void finish_shift(SpiState &state) {
  if (!state.active) { throw std::logic_error("Missing SPI shift transfer"); }
  auto &transfer = *state.active;
  transfer.stage = SpiStage::hold;
  const bool hold = (transfer.word.control & spi_hold) != 0;
  const auto count = (state.delay >> 16) & byte_mask;
  const bool timers = (transfer.format & timers_disabled) == 0 &&
                      (state.pin_function & state.chip_selects) != 0 && !hold;
  state.event_delay = timers && count != 0 ? (count + 1) * 2 : 0;
  if (timers && (transfer.format & spi_phase) == 0) { state.event_delay += half_period(transfer); }
  if (state.event_delay == 0) { finish_hold(state); }
}

void sample(SpiState &state, SpiInput input) {
  if (!state.active) { throw std::logic_error("Missing SPI sample transfer"); }
  auto &transfer = *state.active;
  const bool loop = (state.control & spi_loopback) != 0;
  if (!loop && (!input.received || !input.observed_output)) {
    throw std::runtime_error("Unknown SPI serial receive or output-monitor sample");
  }
  const bool incoming = loop ? state.data : *input.received;
  if (!loop && *input.observed_output != state.data) { transfer.errors |= bit_error; }
  const auto length = transfer.format & character_mask;
  if (transfer.samples == length) {
    const bool odd = (std::popcount(static_cast<unsigned int>(transfer.received)) & 1) != 0;
    if (incoming != (odd != ((transfer.format & spi_odd) != 0))) {
      transfer.errors |= parity_error;
    }
  } else if (incoming) {
    const auto index =
        (transfer.format & spi_reverse) != 0 ? transfer.samples : length - transfer.samples - 1;
    transfer.received |= static_cast<std::uint16_t>(1U << index);
  }
  ++transfer.samples;
  state.flags |= transfer.errors;
  if (transfer.samples == bit_count(transfer)) {
    receive_word(state, transfer);
    transfer.receive_committed = true;
  }
}

}

SpiState spi_reset(SpiResetReference input) {
  SpiState result;
  result.chip_selects = input.chip_selects;
  result.defaults = input.chip_selects;
  result.selected = input.chip_selects;
  result.previous_hold = input.previous_hold;
  return result;
}

void spi_reset_data(SpiState &state) {
  constexpr std::uint32_t control_half = 0xffff0000;
  state.flags = 0;
  state.data0 = 0;
  state.data1 &= control_half;
  state.transmit.reset();
  state.visible = {};
  state.visible_full = false;
  state.receive.reset();
  state.active.reset();
  state.event_delay = 0;
  state.clock = (state.formats[(state.data1 >> format_shift) & 3] & spi_polarity) != 0;
  state.data = false;
  state.selected = state.defaults;
  state.held = false;
  state.transfer_started = false;
}

void spi_submit(SpiState &state, SpiWord word) {
  if (!state.released || (state.control & spi_enable) == 0 || (state.control & 3) != 3 ||
      (state.control & powerdown) != 0) {
    throw std::runtime_error("SPI transfer requires active enabled master");
  }
  if ((state.pin_function & spi_serial_pins) != spi_serial_pins) {
    throw std::runtime_error("SPI transfer requires serial pin functions");
  }
  if (state.transmit) { throw std::runtime_error("SPI transmit buffer full"); }
  state.flags &= ~spi_tx_flag;
  if (state.active) {
    state.transmit = word;
  } else {
    start(state, word, false);
  }
}

void spi_consume(SpiState &state) {
  state.flags &= ~spi_rx_flag;
  state.visible.errors = 0;
  if (state.receive) {
    state.visible = *state.receive;
    state.receive.reset();
    state.visible_full = true;
    state.flags |= spi_rx_flag;
  } else {
    state.visible_full = false;
  }
}

std::uint32_t spi_pending(const SpiState &state) {
  const auto pending = state.flags & state.interrupt_enable & spi_flags_mask;
  if ((pending & ~state.interrupt_level) != 0) {
    throw std::runtime_error("Unqualified SPI interrupt level-zero routing");
  }
  return pending;
}

bool spi_samples_input(const SpiState &state) {
  if (!state.active || state.active->stage != SpiStage::shifting) { return false; }
  const bool leading = state.active->edges % 2 == 0;
  const bool phase = (state.active->format & spi_phase) != 0;
  return leading == phase;
}

void spi_advance(SpiState &state, SpiInput input) {
  if (!state.active) { throw std::logic_error("SPI clock event without active transfer"); }
  auto &transfer = *state.active;
  switch (transfer.stage) {
    case SpiStage::hold: finish_hold(state); return;
    case SpiStage::gap:
      state.active.reset();
      if (state.transmit) {
        const auto word = *state.transmit;
        state.transmit.reset();
        start(state, word, true);
      } else {
        state.event_delay = 0;
      }
      return;
    case SpiStage::shifting: break;
    default: throw std::logic_error("Invalid SPI transfer stage");
  }
  const bool phase = (transfer.format & spi_phase) != 0;
  state.clock = !state.clock;
  if (spi_samples_input(state)) {
    sample(state, input);
  } else {
    const auto index = (transfer.edges / 2) + (phase ? 1U : 0U);
    if (index < bit_count(transfer)) { state.data = output_bit(transfer, index); }
  }
  ++transfer.edges;
  state.event_delay = half_period(transfer);
  if (transfer.edges == bit_count(transfer) * 2) { finish_shift(state); }
}

}
