// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_boot.hpp"

#include "lead_a1/sharc_memory.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t boot_destination = 0x92000;
constexpr std::uint32_t reset_word = 0x8c000;
constexpr unsigned int boot_words = 384;
constexpr unsigned int word_bits = 32;

void transfer(SharcMemory &memory, SharcBootState &state) {
  memory.begin_instruction();
  try {
    memory.write({.address = state.next_word}, state.shift);
    memory.commit_instruction();
  } catch (...) {
    memory.abort_instruction();
    throw;
  }
  ++state.next_word;
  --state.remaining;
  state.shift = 0;
  state.bits = 0;
}

}

SharcSpiBoot SharcSpiBoot::slave_reference_214xx() {
  return SharcSpiBoot({.clock = {},
                       .selected = false,
                       .shift = 0,
                       .bits = 0,
                       .next_word = boot_destination,
                       .remaining = boot_words});
}

std::optional<std::uint32_t> SharcSpiBoot::entry() const {
  if (state_.remaining == 0) { return reset_word; }
  return {};
}

void SharcSpiBoot::observe(SharcMemory &memory, SharcBootLines lines) try {
  if (memory.active()) { throw std::logic_error("SHARC boot edge during active core instruction"); }
  auto next = state_;
  if (!lines.selected) {
    if (next.bits != 0) {
      throw std::runtime_error("Unqualified SPI select release within boot word");
    }
    next.selected = false;
    next.clock = lines.clock;
    state_ = next;
    return;
  }
  if (!next.selected) {
    if (!lines.clock) { throw std::runtime_error("SHARC boot select requires idle-high clock"); }
    next.clock = lines.clock;
    next.selected = true;
    state_ = next;
    return;
  }
  if (!next.clock) { throw std::logic_error("Missing SHARC boot clock baseline"); }
  if (*next.clock == lines.clock) { return; }
  if (next.remaining == 0) {
    throw std::runtime_error("SHARC kernel loaded; programmable SPI/core handoff required");
  }
  next.clock = lines.clock;
  // SPICTL=0x4d22: idle high, output on falling edge, sample on rising edge,
  // 32-bit words, LSB first, receive DMA. Unneeded falling-edge MOSI may be unknown.
  if (lines.clock) {
    if (!lines.mosi) { throw std::runtime_error("Unknown SHARC boot MOSI sample"); }
    if (*lines.mosi) { next.shift |= std::uint32_t{1} << next.bits; }
    ++next.bits;
    if (next.bits == word_bits) { transfer(memory, next); }
  }
  state_ = next;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC SPI slave bootstrap core=idle word-address=0x" << std::hex << state_.next_word
          << std::dec << " remaining=" << state_.remaining << " bit=" << state_.bits << ": "
          << error.what();
  throw std::runtime_error(message.str());
}

}
