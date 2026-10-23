// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_spi.hpp"

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_boot.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <ios>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t normal_base = 0x80000;
constexpr std::uint32_t index_mask = normal_base - 1;
constexpr std::uint32_t boot_end = 0x92180;
constexpr std::uint32_t enabled = 1;
constexpr std::uint32_t receive_dma = 2;
constexpr std::uint32_t interrupt_enable = 4;
constexpr std::uint32_t error_interrupt = 0x100;
constexpr std::uint32_t supported_control =
    enabled | receive_dma | interrupt_enable | error_interrupt;
constexpr unsigned int count_modulus = 65536;
constexpr unsigned int word_bits = 32;

void validate(MemoryAccess access) {
  constexpr std::uint32_t control_offset = 16;
  if (access.size != 4 || access.address > control_offset || access.address % 4 != 0) {
    throw std::runtime_error("SHARC SPI DMA requires aligned offset0..16/read-write32");
  }
}

void pump_receive(SharcSpiState &state) {
  if ((state.control & enabled) == 0 || !state.receive || state.occupied == state.fifo.size()) {
    return;
  }
  state.fifo[state.occupied] = *state.receive;
  ++state.occupied;
  state.receive.reset();
}

void receive_word(SharcSpiState &state) {
  pump_receive(state);
  if (state.receive) {
    state.overflow = true; // SPICTL.GM=0: discard the newly completed word.
    if ((state.control & error_interrupt) != 0) { state.interrupt_pending = true; }
  } else {
    state.receive = state.serial.shift;
    pump_receive(state);
  }
  state.serial.shift = 0;
  state.serial.bits = 0;
}

void parameter_write(SharcSpiState &state, MemoryAccess access, std::uint32_t value) {
  if (state.serial.remaining != 0 && (state.control & enabled) != 0) {
    throw std::runtime_error("SHARC active SPI DMA parameter change not qualified");
  }
  if (access.address == 0) {
    state.serial.next_word = normal_base | (value & index_mask);
  } else if (access.address == 4) {
    state.modifier = static_cast<std::uint16_t>(value);
  } else if (access.address == 8) {
    state.serial.remaining = value & (count_modulus - 1);
    if (state.serial.remaining == 0) { state.serial.remaining = count_modulus; }
  } else {
    throw std::runtime_error("SHARC SPI DMA chaining register not implemented");
  }
}

std::uint32_t status(const SharcSpiState &state) {
  constexpr std::uint32_t busy = 0x8000;
  constexpr std::uint32_t fifo_partial = 0x2000;
  constexpr std::uint32_t fifo_full = 0x3000;
  constexpr std::uint32_t overflow_error = 0x4200;
  auto result = state.control;
  if ((state.control & enabled) != 0 && state.serial.remaining != 0) { result |= busy; }
  if (state.occupied != 0) {
    result |= state.occupied == state.fifo.size() ? fifo_full : fifo_partial;
  }
  if (state.overflow) { result |= overflow_error; }
  return result;
}

void write_group(SharcMemory &memory, SharcSpiState &state, unsigned int count) {
  const auto modifier = static_cast<std::uint32_t>(state.modifier);
  constexpr std::uint32_t sign = 0x8000;
  constexpr std::uint32_t extension = 0xffff0000;
  const auto step = (modifier & sign) != 0 ? modifier | extension : modifier;
  for (unsigned int index = 0; index < count; ++index) {
    memory.write({.address = state.serial.next_word}, state.fifo[index]);
    const auto sum = std::uint64_t{state.serial.next_word} + step;
    state.serial.next_word = normal_base | static_cast<std::uint32_t>(sum & index_mask);
    --state.serial.remaining;
  }
  for (unsigned int index = count; index < state.occupied; ++index) {
    state.fifo[index - count] = state.fifo[index];
  }
  state.occupied -= count;
  if (state.serial.remaining == 0 && (state.control & interrupt_enable) != 0) {
    state.interrupt_pending = true;
  }
  pump_receive(state);
}

}

SharcSpi::SharcSpi(const SharcBootState &completed_boot) {
  if (completed_boot.remaining != 0 || completed_boot.bits != 0 || completed_boot.shift != 0 ||
      completed_boot.next_word != boot_end || !completed_boot.clock) {
    throw std::runtime_error("SHARC programmable SPI requires completed initial boot reference");
  }
  current_.serial = completed_boot;
}

const SharcSpiState &SharcSpi::state() const {
  return staged_ ? *staged_ : current_;
}

SharcSpiState &SharcSpi::working() {
  return staged_ ? *staged_ : current_;
}

void SharcSpi::external_event(const SharcMemory &memory) const {
  if (staged_ || memory.active()) {
    throw std::logic_error("SHARC SPI external event during active instruction transaction");
  }
}

void SharcSpi::observe(SharcMemory &memory, SharcBootLines lines) try {
  external_event(memory);
  auto next = current_;
  if (!lines.selected) {
    if (next.serial.bits != 0) {
      throw std::runtime_error("SHARC SPI select release within runtime word not qualified");
    }
    next.serial.selected = false;
    next.serial.clock = lines.clock;
  } else if (!next.serial.selected) {
    if (!lines.clock) { throw std::runtime_error("SHARC SPI select requires idle-high clock"); }
    next.serial.selected = true;
    next.serial.clock = lines.clock;
  } else if (next.serial.clock != lines.clock) {
    next.serial.clock = lines.clock;
    if (lines.clock) {
      if (!lines.mosi) { throw std::runtime_error("Unknown SHARC runtime SPI MOSI sample"); }
      if (*lines.mosi) { next.serial.shift |= std::uint32_t{1} << next.serial.bits; }
      ++next.serial.bits;
      if (next.serial.bits == word_bits) { receive_word(next); }
    }
  }
  current_ = next;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC runtime SPI RX edge core-pc=0x" << std::hex << memory.pc() << " word-address=0x"
          << current_.serial.next_word << std::dec << " bit=" << current_.serial.bits << ": "
          << error.what();
  throw std::runtime_error(message.str());
}

unsigned int SharcSpi::dma_request() const {
  const auto &next = state();
  if ((next.control & enabled) == 0 || next.serial.remaining == 0) { return 0; }
  constexpr unsigned int group = 4;
  const auto count = std::min(group, next.serial.remaining);
  return next.occupied >= count ? count : 0;
}

unsigned int SharcSpi::service_dma(SharcMemory &memory) try {
  external_event(memory);
  const auto count = dma_request();
  if (count == 0) { return 0; }
  auto next = current_;
  memory.begin_instruction();
  try {
    write_group(memory, next, count);
    memory.commit_instruction();
  } catch (...) {
    memory.abort_instruction();
    throw;
  }
  current_ = next;
  return count;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC SPI RX DMA grant core-pc=0x" << std::hex << memory.pc() << " word-address=0x"
          << current_.serial.next_word << std::dec << " remaining=" << current_.serial.remaining
          << " fifo=" << current_.occupied << ": " << error.what();
  throw std::runtime_error(message.str());
}

void SharcSpi::acknowledge_interrupt() {
  if (staged_) { throw std::logic_error("SHARC SPI interrupt acknowledgement during transaction"); }
  current_.interrupt_pending = false;
}

std::uint64_t SharcSpi::read(MemoryAccess access) {
  validate(access);
  const auto &next = state();
  switch (access.address) {
    case 0:
      throw std::runtime_error("SHARC IISPI raw index readback/forced offset bit not qualified");
    case 4: return next.modifier;
    case 8: return next.serial.remaining & (count_modulus - 1);
    case 16: return status(next);
    default: throw std::runtime_error("SHARC SPI DMA chaining register not implemented");
  }
}

void SharcSpi::write(MemoryAccess access, std::uint64_t value) {
  validate(access);
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("SHARC SPI DMA write exceeds 32 bits");
  }
  auto next = working();
  if (access.address != 16) {
    parameter_write(next, access, static_cast<std::uint32_t>(value));
  } else {
    if ((value & ~std::uint64_t{supported_control}) != 0 ||
        ((value & enabled) != 0 && (value & receive_dma) == 0)) {
      throw std::runtime_error("SHARC SPI DMA transmit/chaining/flush/control not implemented");
    }
    next.control = static_cast<std::uint32_t>(value);
    if (next.overflow && (next.control & error_interrupt) != 0) { next.interrupt_pending = true; }
    pump_receive(next);
  }
  working() = next;
}

void SharcSpi::begin_instruction() {
  if (staged_) { throw std::logic_error("Nested SHARC SPI transaction"); }
  staged_ = current_;
}

void SharcSpi::commit_instruction() noexcept {
  if (staged_) { current_ = *staged_; }
  staged_.reset();
}

void SharcSpi::abort_instruction() noexcept {
  staged_.reset();
}

}
