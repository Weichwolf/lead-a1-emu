// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_pdap.hpp"
#include "sharc/peripherals/sharc_pdap_internal.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

struct Packing {
  unsigned int width;
  unsigned int input_shift;
  unsigned int output_shift;
};

struct PackingPosition {
  unsigned int mode;
  unsigned int samples;
};

Packing packing(PackingPosition position) {
  const auto mode = position.mode;
  const auto samples = position.samples;
  constexpr unsigned int byte_bits = 8;
  constexpr unsigned int halfword_bits = 16;
  constexpr unsigned int pin_bits = 20;
  constexpr unsigned int tri_first_bits = 10;
  constexpr unsigned int tri_bits = 11;
  constexpr unsigned int tri_last_shift = 21;
  switch (mode) {
    case 0: return {.width = byte_bits, .input_shift = 12, .output_shift = samples * byte_bits};
    case 1:
      if (samples == 0) { return {.width = tri_first_bits, .input_shift = 10, .output_shift = 0}; }
      return {.width = tri_bits,
              .input_shift = 9,
              .output_shift = samples == 1 ? tri_first_bits : tri_last_shift};
    case 2:
      return {.width = halfword_bits, .input_shift = 4, .output_shift = samples * halfword_bits};
    case 3: return {.width = pin_bits, .input_shift = 0, .output_shift = 12};
    default: throw std::logic_error("Invalid PDAP packing mode");
  }
}

std::uint32_t partial_mask(PackingPosition position) {
  if (position.samples == 0) { return 0; }
  const auto previous = packing({.mode = position.mode, .samples = position.samples - 1});
  const auto bits = previous.width + previous.output_shift;
  return static_cast<std::uint32_t>((std::uint64_t{1} << bits) - 1);
}

void validate_fifo(const SharcPdapState &state) {
  const bool enabled = (state.control[0] & idp_enable) != 0;
  if (state.head >= state.fifo.size() || state.count > state.fifo.size() ||
      (state.count == 0 && state.head != 0) || state.startup_sclk > 1 || state.flush_cclk > 2 ||
      (!enabled && state.startup_sclk != 0) || (enabled && state.flush_cclk != 0) ||
      (!enabled && state.flush_cclk == 0 && (state.count != 0 || state.overflow))) {
    throw std::invalid_argument("Invalid PDAP FIFO/clock snapshot");
  }
  for (unsigned int index = 0; index < state.fifo.size(); ++index) {
    const auto distance = (index + state.fifo.size() - state.head) % state.fifo.size();
    if (distance >= state.count && state.fifo[index] != 0) {
      throw std::invalid_argument("Invalid PDAP unoccupied FIFO storage");
    }
  }
}

void validate_partial(const SharcPdapState &state) {
  const auto mode = pdap_mode(state);
  if (state.samples >= pdap_samples(mode) ||
      (state.partial & ~partial_mask({.mode = mode, .samples = state.samples})) != 0 ||
      (state.samples != 0 && (!pdap_active(state) || state.startup_sclk != 0))) {
    throw std::invalid_argument("Invalid PDAP partial packing snapshot");
  }
  const auto selected = (state.control[1] & pdap_select) != 0 ? 1U : 0U;
  if (state.samples != 0 && !state.inputs[selected].clock) {
    throw std::invalid_argument("Invalid PDAP partial packing clock history");
  }
}

}

unsigned int pdap_mode(const SharcPdapState &state) {
  constexpr unsigned int packing_shift = 27;
  return (state.control[1] >> packing_shift) & 3;
}

unsigned int pdap_samples(unsigned int mode) {
  constexpr std::array<unsigned int, 4> counts{4, 3, 2, 1};
  return counts.at(mode);
}

bool pdap_active(const SharcPdapState &state) {
  return (state.control[0] & idp_enable) != 0 && (state.control[1] & pdap_enable) != 0 &&
         (state.control[2] & 1) != 0;
}

void pdap_validate_input(SharcPdapInput input) {
  if ((input.data & ~input.known) != 0 || (input.known & ~pdap_pin_mask) != 0) {
    throw std::invalid_argument("Invalid PDAP twenty-bit input knowledge");
  }
}

void pdap_validate_controls(const SharcPdapState &state) {
  constexpr std::uint32_t serial_channels = 0xfe;
  if ((state.control[0] & idp_enable) != 0 &&
      ((state.control[1] & pdap_enable) == 0 || (state.control[2] & serial_channels) != 0)) {
    throw std::runtime_error("IDP active SIP channels not implemented by PDAP receiver");
  }
}

void pdap_validate(const SharcPdapState &state) {
  pdap_validate_controls(state);
  idp_dma_validate(state);
  if ((state.control[0] & idp_clear_overflow) != 0 || (state.control[1] & pdap_reset) != 0 ||
      (state.control[2] & idp_fifo_clear) != 0) {
    throw std::invalid_argument("PDAP snapshot contains self-clearing command bits");
  }
  validate_fifo(state);
  validate_partial(state);
  for (const auto &input : state.inputs) { pdap_validate_input(input); }
}

void pdap_flush(SharcPdapState &state, bool overflow) {
  state.fifo.fill(0);
  state.head = 0;
  state.count = 0;
  if (overflow) { state.overflow = false; }
}

void pdap_push(SharcPdapState &state, std::uint32_t word) {
  if (state.count == state.fifo.size()) {
    state.overflow = true;
    return;
  }
  const auto tail = (state.head + state.count) % state.fifo.size();
  state.fifo[tail] = word;
  ++state.count;
}

void pdap_sample(SharcPdapState &state, SharcPdapInput input) {
  if (!input.hold) { throw std::runtime_error("Unknown PDAP HOLD at active edge"); }
  if (*input.hold) { return; }
  const auto mode = pdap_mode(state);
  const auto format = packing({.mode = mode, .samples = state.samples});
  const auto value_mask = static_cast<std::uint32_t>((std::uint64_t{1} << format.width) - 1);
  const auto selected = state.control[1] & (value_mask << format.input_shift);
  if ((input.known & selected) != selected) {
    throw std::runtime_error("Unknown unmasked PDAP input data at active edge");
  }
  const auto value = (input.data & selected) >> format.input_shift;
  state.partial |= value << format.output_shift;
  ++state.samples;
  if (state.samples == pdap_samples(mode)) {
    pdap_push(state, state.partial);
    state.partial = 0;
    state.samples = 0;
  }
}

}
