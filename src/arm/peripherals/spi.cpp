// SPDX-License-Identifier: GPL-3.0-only
#include "arm/peripherals/spi.hpp"

#include "arm/peripherals/spi_internal.hpp"
#include "common/memory.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t format_base = 0x50;
constexpr std::uint32_t format_end = 0x60;
constexpr std::uint32_t buffer_offset = 0x40;
constexpr std::uint32_t vector_offset = 0x64;
constexpr std::uint32_t half_mask = 0xffff;
constexpr unsigned int byte_bits = 8;

constexpr std::uint32_t register_end = 0x68;
constexpr std::uint32_t error_vector = 0x22;
constexpr std::uint32_t overrun_vector = 0x26;
constexpr std::uint32_t receive_vector = 0x24;
constexpr std::uint32_t transmit_vector = 0x28;
constexpr std::uint32_t data0_offset = 0x38;
constexpr std::uint32_t data1_offset = 0x3c;
constexpr std::uint32_t emulation_offset = 0x44;
constexpr std::uint32_t delay_offset = 0x48;
constexpr std::uint32_t defaults_offset = 0x4c;
constexpr std::uint32_t vector_mask = 0x3e;
constexpr unsigned int format_shift = 24;
constexpr std::uint32_t function_offset = 20;
constexpr std::uint32_t direction_offset = 24;
constexpr std::uint32_t input_offset = 28;
constexpr std::uint32_t set_offset = 36;
constexpr std::uint32_t clear_offset = 40;

struct Register {
  std::uint32_t offset;
  unsigned int shift;
  std::uint32_t mask;
};

Register decode(MemoryAccess access) {
  if ((access.size != 1 && access.size != 2 && access.size != 4) ||
      access.address % access.size != 0 || access.address >= register_end) {
    throw std::runtime_error("Invalid SPI register width or alignment");
  }
  return {.offset = access.address & ~3U,
          .shift = (access.address % 4) * byte_bits,
          .mask = access.size == 4 ? std::numeric_limits<std::uint32_t>::max()
                                   : (1U << (access.size * byte_bits)) - 1};
}

std::uint32_t vector(SpiState &state, bool acknowledge) {
  const auto pending = detail::spi_pending(state);
  if ((pending & detail::spi_error_mask) != 0) { return error_vector; }
  if ((pending & detail::spi_overrun) != 0) {
    if (acknowledge) { state.flags &= ~detail::spi_overrun; }
    return overrun_vector;
  }
  if ((pending & detail::spi_rx_flag) != 0) {
    if (acknowledge) { state.flags &= ~detail::spi_rx_flag; }
    return receive_vector;
  }
  return (pending & detail::spi_tx_flag) != 0 ? transmit_vector : 0;
}

std::uint32_t buffer(const SpiState &state) {
  constexpr std::uint32_t empty = 0x80000000;
  constexpr std::uint32_t full = 0x20000000;
  constexpr std::uint32_t overrun = 0x40000000;
  constexpr unsigned int error_shift = 24;
  return state.visible.data | (state.visible.errors << error_shift) |
         (state.visible_full ? 0 : empty) | (state.transmit ? full : 0) |
         ((state.flags & detail::spi_overrun) != 0 ? overrun : 0);
}

std::uint32_t read_word(SpiState &state, Register reg) {
  const auto requested = reg.mask << reg.shift;
  if (reg.offset >= format_base && reg.offset < format_end) {
    return state.formats[(reg.offset - format_base) / 4];
  }
  switch (reg.offset) {
    case 0: return state.released ? 1 : 0;
    case 4: return state.control;
    case 8: return state.interrupt_enable;
    case 12: return state.interrupt_level;
    case 16: return state.flags | detail::spi_enable;
    case function_offset: return state.pin_function;
    case direction_offset: return state.pin_direction;
    case input_offset:
      if ((requested & (detail::spi_serial_pins | 0x100U | state.chip_selects) &
           ~static_cast<std::uint32_t>(state.pin_known)) != 0) {
        throw std::runtime_error("Unknown SPI pin input");
      }
      return state.pin_input;
    case 32:
    case set_offset:
    case clear_offset: return state.pin_output;
    case data0_offset: return state.data0;
    case data1_offset: return state.data1;
    case buffer_offset: {
      const auto value = buffer(state);
      if ((requested & half_mask) != 0) { detail::spi_consume(state); }
      return value;
    }
    case emulation_offset: return state.visible.data;
    case delay_offset: return state.delay;
    case defaults_offset: return state.defaults;
    case vector_offset: return vector(state, (requested & vector_mask) != 0);
    default: throw std::runtime_error("Unmapped SPI register read");
  }
}

struct Write {
  Register reg;
  std::uint32_t value;
};

void data_write(SpiState &state, Write operation) {
  constexpr std::uint32_t shared_control = 0x17000000;
  const auto control_mask = shared_control | (static_cast<std::uint32_t>(state.chip_selects) << 16);
  const auto mask = operation.reg.mask << operation.reg.shift;
  const auto bits = operation.value << operation.reg.shift;
  if (operation.reg.offset == data0_offset) {
    state.data0 = static_cast<std::uint16_t>((state.data0 & ~mask) | (bits & half_mask));
  } else {
    state.data1 = (state.data1 & ~mask) | (bits & (control_mask | half_mask));
    if ((mask & detail::spi_hold) != 0) {
      if (state.programmed_hold) { state.previous_hold = state.programmed_hold; }
      state.programmed_hold = (state.data1 & detail::spi_hold) != 0;
    }
    if (!state.active) {
      state.clock = (state.formats[(state.data1 >> format_shift) & 3] & detail::spi_polarity) != 0;
    }
  }
  if ((mask & half_mask) != 0) {
    const auto data = operation.reg.offset == data0_offset
                          ? state.data0
                          : static_cast<std::uint16_t>(state.data1);
    detail::spi_submit(state, {.data = data, .control = state.data1 & control_mask});
  }
}

void set_control(SpiState &state, std::uint32_t value) {
  constexpr std::uint32_t supported = 0x01010103;
  const auto next = value & supported;
  if (state.active && (next & detail::spi_enable) != 0) {
    throw std::runtime_error("SPI control change during active transfer");
  }
  state.control = next;
  if ((next & detail::spi_enable) == 0) { detail::spi_reset_data(state); }
}

void write_word(SpiState &state, Write operation) {
  const auto [reg, value] = operation;
  const auto mask = reg.mask << reg.shift;
  const auto bits = value << reg.shift;
  const auto merge = [mask, bits](std::uint32_t prior) { return (prior & ~mask) | bits; };
  const auto pin_mask = static_cast<std::uint32_t>(0xf00U | state.chip_selects);
  if (reg.offset == 0) {
    if ((mask & 1) == 0) { return; }
    if ((bits & 1) == 0) {
      const auto previous = state.previous_hold;
      state = detail::spi_reset({.chip_selects = state.chip_selects, .previous_hold = previous});
    } else {
      state.released = true;
    }
    return;
  }
  if (!state.released) { throw std::runtime_error("SPI module is held in reset"); }
  if (reg.offset >= format_base && reg.offset < format_end) {
    constexpr std::uint32_t format_mask = 0x3ff7ff1f;
    auto &format = state.formats[(reg.offset - format_base) / 4];
    if (state.active) { throw std::runtime_error("SPI format change during active transfer"); }
    format = merge(format) & format_mask;
    if ((reg.offset - format_base) / 4 == ((state.data1 >> format_shift) & 3)) {
      state.clock = (format & detail::spi_polarity) != 0;
    }
    return;
  }
  switch (reg.offset) {
    case 4: set_control(state, merge(state.control)); break;
    case 8: {
      constexpr std::uint32_t interrupt_mask = 0x0101035f;
      constexpr std::uint32_t dma_enable = 0x10000;
      const auto next = merge(state.interrupt_enable) & interrupt_mask;
      if ((next & dma_enable) != 0) {
        throw std::runtime_error("Unsupported SPI DMA request path");
      }
      state.interrupt_enable = next;
      break;
    }
    case 12: state.interrupt_level = merge(state.interrupt_level) & detail::spi_flags_mask; break;
    case 16:
      if ((bits & detail::spi_rx_flag) != 0) { detail::spi_consume(state); }
      state.flags &= ~(bits & (detail::spi_error_mask | detail::spi_overrun));
      break;
    case function_offset:
    case direction_offset:
      if (state.active) { throw std::runtime_error("SPI pin reconfiguration during transfer"); }
      if (reg.offset == function_offset) {
        state.pin_function = static_cast<std::uint16_t>(merge(state.pin_function) & pin_mask);
      } else {
        state.pin_direction = static_cast<std::uint16_t>(merge(state.pin_direction) & pin_mask);
      }
      break;
    case 32:
      state.pin_output = static_cast<std::uint16_t>(merge(state.pin_output) & pin_mask);
      break;
    case set_offset: state.pin_output |= static_cast<std::uint16_t>(bits & pin_mask); break;
    case clear_offset: state.pin_output &= static_cast<std::uint16_t>(~(bits & pin_mask)); break;
    case data0_offset:
    case data1_offset: data_write(state, operation); break;
    case delay_offset:
      if (state.active) { throw std::runtime_error("SPI delay change during transfer"); }
      state.delay = merge(state.delay);
      break;
    case defaults_offset:
      state.defaults = static_cast<std::uint8_t>(merge(state.defaults) & state.chip_selects);
      break;
    default: throw std::runtime_error("Read-only or unmapped SPI register write");
  }
}

}

Spi::Spi(SpiResetReference initial) : current_(detail::spi_reset(initial)) {}

SpiState Spi::state() const {
  return working();
}

SpiPins Spi::pins() const {
  const auto &state = working();
  const auto gpio = static_cast<std::uint16_t>(state.pin_direction & ~state.pin_function);
  SpiPins result{.enabled = gpio, .high = static_cast<std::uint16_t>(state.pin_output & gpio)};
  if (!state.released || (state.control & detail::spi_enable) == 0 || (state.control & 3) != 3) {
    return result;
  }
  const auto selects = static_cast<std::uint16_t>(state.pin_function & state.chip_selects);
  const bool loop = (state.control & detail::spi_loopback) != 0;
  result.enabled |= selects;
  result.high |= state.selected & selects;
  // TI SPRUH82C29.2.14.1: all three functional serial pads are HiZ in loopback.
  if (!loop) {
    result.enabled |= detail::spi_clock_pin;
    if (state.clock && (state.pin_function & detail::spi_clock_pin) != 0) {
      result.high |= detail::spi_clock_pin;
    }
  }
  if (!loop && (state.pin_function & detail::spi_output_pin) != 0) {
    result.enabled |= detail::spi_output_pin;
    if (state.data) { result.high |= detail::spi_output_pin; }
  }
  result.enabled &= state.pin_function | gpio;
  result.high &= result.enabled;
  return result;
}

bool Spi::interrupt() const {
  return detail::spi_pending(working()) != 0;
}

std::optional<unsigned int> Spi::next_event() const {
  return working().active ? std::optional<unsigned int>{working().event_delay} : std::nullopt;
}

void Spi::check_external_event() const {
  if (pending_) { throw std::logic_error("SPI external event during active instruction"); }
}

bool Spi::samples_external_input() const {
  return detail::spi_samples_input(working()) && (working().control & detail::spi_loopback) == 0;
}

void Spi::advance(SpiInput input) {
  check_external_event();
  auto next = current_;
  detail::spi_advance(next, input);
  current_ = next;
}

void Spi::sample_pins(SpiPins sample) {
  check_external_event();
  const auto mask = 0xf00U | current_.chip_selects;
  if ((sample.enabled & ~mask) != 0 || (sample.high & ~sample.enabled) != 0) {
    throw std::invalid_argument("Invalid SPI pin sample knowledge");
  }
  current_.pin_known = sample.enabled;
  current_.pin_input = sample.high;
}

std::uint64_t Spi::read(MemoryAccess access) {
  const auto reg = decode(access);
  return (read_word(working(), reg) >> reg.shift) & reg.mask;
}

void Spi::write(MemoryAccess access, std::uint64_t value) {
  const auto reg = decode(access);
  if (value > reg.mask) { throw std::runtime_error("Oversized SPI write value"); }
  auto next = working();
  write_word(next, {.reg = reg, .value = static_cast<std::uint32_t>(value)});
  working() = next;
}

void Spi::begin_instruction() {
  if (pending_) { throw std::logic_error("Nested SPI instruction transaction"); }
  pending_ = current_;
}

void Spi::commit_instruction() noexcept {
  if (pending_) { current_ = *pending_; }
  pending_.reset();
}

void Spi::abort_instruction() noexcept {
  pending_.reset();
}

SpiState &Spi::working() {
  return pending_ ? *pending_ : current_;
}

const SpiState &Spi::working() const {
  return pending_ ? *pending_ : current_;
}

}
