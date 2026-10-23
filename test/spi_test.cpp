// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/spi.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t control = 4;
constexpr std::uint32_t interrupt_enable = 8;
constexpr std::uint32_t level = 12;
constexpr std::uint32_t flags = 16;
constexpr std::uint32_t pin_function = 20;
constexpr std::uint32_t pin_direction = 24;
constexpr std::uint32_t pin_input = 28;
constexpr std::uint32_t pin_output = 32;
constexpr std::uint32_t pin_set = 36;
constexpr std::uint32_t pin_clear = 40;
constexpr std::uint32_t data0 = 0x38;
constexpr std::uint32_t data1 = 0x3c;
constexpr std::uint32_t buffer = 0x40;
constexpr std::uint32_t emulation = 0x44;
constexpr std::uint32_t delay = 0x48;
constexpr std::uint32_t defaults = 0x4c;
constexpr std::uint32_t format0 = 0x50;
constexpr std::uint32_t vector = 0x64;
constexpr std::uint32_t enabled = 0x01000003;
constexpr std::uint32_t loopback = 0x01010003;
constexpr std::uint32_t spi_pins = 0xe01;
constexpr std::uint32_t empty = 0x80000000;
constexpr std::uint32_t full = 0x20000000;
constexpr std::uint32_t overrun = 0x40000000;
constexpr std::uint32_t tx_flag = 0x200;
constexpr std::uint32_t rx_flag = 0x100;
constexpr std::uint32_t base_flags = 0x01000000;
constexpr std::uint32_t format16 = 0x210;
constexpr std::uint32_t hold = 0x10000000;
constexpr std::uint32_t word_delay = 0x04000000;
constexpr std::uint32_t polarity = 0x20000;
constexpr std::uint32_t phase = 0x10000;
constexpr std::uint32_t reverse = 0x100000;
constexpr std::uint32_t parity = 0x400000;
constexpr std::uint32_t odd = 0x800000;
constexpr std::uint32_t marker = 0xa54b;
constexpr std::uint32_t serial_clock = 0x200;
constexpr std::uint32_t serial_output = 0x400;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Independent SPI expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SPI rejection: " + reason);
}

void write(lead_a1::Spi &spi, std::uint32_t offset, std::uint32_t value) {
  spi.write({.address = offset, .size = 4}, value);
}

std::uint32_t read(lead_a1::Spi &spi, std::uint32_t offset) {
  return static_cast<std::uint32_t>(spi.read({.address = offset, .size = 4}));
}

void configure(lead_a1::Spi &spi, std::uint32_t format, bool loop = true) {
  write(spi, 0, 1);
  write(spi, pin_function, spi_pins);
  write(spi, format0, format);
  // Two explicit control writes qualify the otherwise unknown prior CSHOLD.
  spi.write({.address = data1 + 3, .size = 1}, 0);
  spi.write({.address = data1 + 3, .size = 1}, 0);
  write(spi, control, loop ? loopback : enabled);
}

void drain(lead_a1::Spi &spi) {
  constexpr unsigned int limit = 128;
  for (unsigned int event = 0; spi.next_event() && event < limit; ++event) { spi.advance(); }
  require(!spi.next_event());
}

constexpr std::uint32_t all_pin_mask = 0xf03;
constexpr std::uint32_t prescale7 = 0x700;
constexpr std::uint32_t all_bits = 0xffffffff;
constexpr std::uint32_t format_mask = 0x3ff7ff1f;
constexpr std::uint32_t all_interrupts = 0x35f;
constexpr std::uint32_t overrun_vector = 0x26;
constexpr std::uint32_t receive_vector = 0x24;
constexpr std::uint32_t transmit_vector = 0x28;
constexpr unsigned int parity_edges = 34;
constexpr std::uint32_t error_flags = 20;

void reset_and_registers() {
  lead_a1::Spi spi({.chip_selects = 3, .previous_hold = {}});
  require(read(spi, flags) == base_flags && read(spi, buffer) == empty);
  require(read(spi, defaults) == 3 && read(spi, format0) == 0);
  rejected([&] { write(spi, format0, format16); }, "held in reset");
  write(spi, 0, 1);
  write(spi, pin_direction, spi_pins);
  write(spi, pin_set, spi_pins);
  require(read(spi, pin_output) == spi_pins && read(spi, pin_clear) == spi_pins);
  write(spi, pin_clear, serial_clock);
  require(read(spi, pin_output) == (spi_pins & ~serial_clock));
  rejected([&] { static_cast<void>(read(spi, pin_input)); }, "Unknown SPI pin");
  spi.sample_pins({.enabled = all_pin_mask, .high = 1});
  require(read(spi, pin_input) == 1);
  spi.write({.address = format0 + 1, .size = 1}, 7);
  require(read(spi, format0) == prescale7);
  write(spi, format0, all_bits);
  require(read(spi, format0) == format_mask);
  write(spi, control, 2);
  write(spi, control, 3);
  require(read(spi, control) == 3);
  write(spi, 0, 0);
  require(read(spi, control) == 0 && read(spi, format0) == 0 && read(spi, defaults) == 3);
}

struct ClockMode {
  bool high;
  bool delayed;
  bool lsb;
};

void clock_case(ClockMode mode) {
  lead_a1::Spi spi({.chip_selects = 1, .previous_hold = {}});
  const auto format =
      format16 | (mode.high ? polarity : 0) | (mode.delayed ? phase : 0) | (mode.lsb ? reverse : 0);
  configure(spi, format, false);
  write(spi, data1, marker);
  require(spi.next_event() == (mode.delayed ? 3 : 0));
  std::vector<bool> bits;
  for (unsigned int edge = 0; edge < 32; ++edge) {
    const bool sampling = (edge % 2 == 0) == mode.delayed;
    const auto index = edge / 2;
    const bool transmitted = ((marker >> (mode.lsb ? index : 15 - index)) & 1) != 0;
    const bool incoming = !transmitted;
    spi.advance({.received = incoming, .observed_output = transmitted});
    require(((spi.pins().high & serial_clock) != 0) == (edge % 2 == 0 ? !mode.high : mode.high));
    if (sampling) {
      require(((spi.pins().high & serial_output) != 0) == transmitted);
      bits.push_back(transmitted);
    }
    if (edge + 1 < 32) { require(spi.next_event() == 3); }
  }
  require(bits.size() == 16 && spi.state().visible_full);
  constexpr std::uint32_t complement = 0x5ab4;
  require(read(spi, buffer) == complement && spi.state().flags == tx_flag);
  drain(spi);
}

void clock_modes() {
  for (const bool high : {false, true}) {
    for (const bool delayed : {false, true}) {
      for (const bool lsb : {false, true}) {
        clock_case({.high = high, .delayed = delayed, .lsb = lsb});
      }
    }
  }
}

void pipeline_and_interrupts() {
  lead_a1::Spi spi({.chip_selects = 1, .previous_hold = false});
  configure(spi, format16);
  write(spi, level, all_interrupts);
  write(spi, interrupt_enable, tx_flag | rx_flag | 0x40);
  write(spi, data0, 1);
  write(spi, data0, 2);
  require((read(spi, buffer) & full) != 0 && !spi.interrupt());
  const auto before = spi.state();
  rejected([&] { write(spi, data0, 3); }, "buffer full");
  require(spi.state() == before);
  drain(spi);
  require(spi.interrupt() && read(spi, emulation) == 1);
  require(spi.read({.address = buffer + 2, .size = 2}) == 0);
  const auto queued = spi.state().receive;
  if (!queued) { throw std::logic_error("Missing SPI receive queue"); }
  require(spi.state().visible_full && queued->data == 2);
  write(spi, data0, 3);
  drain(spi);
  require(read(spi, buffer) == (overrun | 1));
  require(read(spi, emulation) == 3);
  require(spi.read({.address = vector + 1, .size = 1}) == 0);
  require((spi.state().flags & 0x40) != 0);
  require(read(spi, vector) == overrun_vector);
  require(read(spi, vector) == receive_vector && spi.state().visible_full);
  require(read(spi, buffer) == 3);
  require(read(spi, vector) == transmit_vector);
  require(read(spi, vector) == transmit_vector);
  write(spi, data0, 4);
  write(spi, data0, 5);
  drain(spi);
  write(spi, flags, rx_flag);
  require(read(spi, emulation) == 5 && spi.state().visible_full);
  require(read(spi, buffer) == 5 && !spi.state().visible_full);
}

void parity_and_unknowns() {
  for (const bool odd_parity : {false, true}) {
    lead_a1::Spi spi({.chip_selects = 1, .previous_hold = false});
    configure(spi, format16 | parity | (odd_parity ? odd : 0));
    write(spi, data1, marker);
    drain(spi);
    require(read(spi, buffer) == marker && spi.state().flags == tx_flag);
  }
  lead_a1::Spi external({.chip_selects = 1, .previous_hold = false});
  configure(external, format16 | parity, false);
  write(external, data1, marker);
  external.advance();
  const auto before = external.state();
  rejected([&] { external.advance(); }, "Unknown SPI serial");
  require(external.state() == before);
  for (unsigned int edge = 1; edge < parity_edges; ++edge) {
    const auto index = edge / 2;
    const bool incoming = index == 16;
    const bool outgoing = index == 16 ? false : ((marker >> (15 - index)) & 1) != 0;
    external.advance({.received = incoming, .observed_output = !outgoing});
  }
  constexpr std::uint32_t errors = 0x14000000;
  require(read(external, buffer) == errors);
  require((external.state().flags & error_flags) == error_flags);
  write(external, flags, error_flags);
  require(external.state().flags == tx_flag);
}

void holds_and_reset() {
  lead_a1::Spi spi({.chip_selects = 1, .previous_hold = false});
  configure(spi, format16 | phase);
  constexpr std::uint32_t delay_fields = 0x01010000;
  write(spi, delay, delay_fields);
  write(spi, data1, hold | 1);
  require(spi.next_event() == 9);
  write(spi, data1, hold | 2);
  drain(spi);
  require(spi.state().selected == 0 && spi.state().held);
  write(spi, data1, 3);
  require(spi.next_event() == 3);
  drain(spi);
  require(spi.state().selected == 1 && !spi.state().held);
  write(spi, data1, word_delay | 4);
  require(spi.next_event() == 9);
  const auto formats = spi.state().formats;
  write(spi, control, 3);
  require(!spi.next_event() && spi.state().formats == formats && spi.state().data1 == word_delay);
  require(read(spi, buffer) == empty && read(spi, flags) == base_flags);
  write(spi, 0, 0);
  require(spi.state().formats[0] == 0 && spi.state().delay == 0);
}

void guards() {
  lead_a1::Spi spi({.chip_selects = 1, .previous_hold = {}});
  write(spi, 0, 1);
  write(spi, pin_function, spi_pins);
  write(spi, format0, format16);
  write(spi, control, loopback);
  const auto before = spi.state();
  rejected([&] { write(spi, data1, marker); }, "previous CSHOLD");
  require(spi.state() == before);
  spi.write({.address = data1 + 3, .size = 1}, 0);
  rejected([&] { write(spi, data0, marker); }, "previous CSHOLD");
  spi.write({.address = data1 + 3, .size = 1}, 0);
  write(spi, data0, marker);
  const auto active = spi.state();
  rejected([&] { write(spi, format0, 1); }, "active transfer");
  rejected([&] { write(spi, pin_function, 0); }, "during transfer");
  rejected([&] { write(spi, control, loopback); }, "during active transfer");
  require(spi.state() == active);
  drain(spi);
  rejected([&] { spi.advance(); }, "without active");
  rejected([&] { spi.write({.address = data0, .size = 1}, 0x100); }, "Oversized");
  rejected([&] { static_cast<void>(spi.read({.address = 1, .size = 2})); }, "alignment");
  rejected([&] { static_cast<void>(read(spi, format0 + 16)); }, "Unmapped");
  constexpr std::uint32_t dma = 0x10000;
  rejected([&] { write(spi, interrupt_enable, dma); }, "DMA");
  write(spi, interrupt_enable, tx_flag);
  rejected([&] { static_cast<void>(spi.interrupt()); }, "level-zero");
  write(spi, level, tx_flag);
  require(spi.interrupt());
  constexpr std::array<std::uint32_t, 3> invalid_formats{1, 0x110, 0x200210};
  for (const auto format : invalid_formats) {
    write(spi, format0, format);
    const auto prior = spi.state();
    rejected([&] { write(spi, data0, marker); }, "SPI");
    require(spi.state() == prior);
  }
}

void transactional_reads() {
  lead_a1::Spi spi({.chip_selects = 1, .previous_hold = false});
  configure(spi, format16);
  write(spi, data0, 1);
  write(spi, data0, 2);
  drain(spi);
  const auto before = spi.state();
  spi.begin_instruction();
  require(read(spi, buffer) == 1 && read(spi, emulation) == 2);
  rejected([&] { spi.begin_instruction(); }, "Nested");
  rejected([&] { spi.advance(); }, "active instruction");
  rejected([&] { spi.sample_pins({}); }, "active instruction");
  spi.abort_instruction();
  require(spi.state() == before);
  spi.begin_instruction();
  require(read(spi, buffer) == 1);
  spi.commit_instruction();
  require(read(spi, emulation) == 2 && !spi.state().receive);
  for (const unsigned int length : {2U, 7U}) {
    for (const bool lsb : {false, true}) {
      constexpr std::uint32_t prescale_maximum = 0xff00;
      write(spi, format0, prescale_maximum | phase | (lsb ? reverse : 0) | length);
      write(spi, data0, marker);
      require(spi.next_event() == 256);
      drain(spi);
      // Discard the older second word before inspecting this short character.
      static_cast<void>(read(spi, buffer));
      require(read(spi, buffer) == (marker & ((1U << length) - 1)));
      write(spi, data0, 2);
      drain(spi);
    }
  }
}

void explicit_previous_hold() {
  lead_a1::Spi spi({.chip_selects = 1, .previous_hold = true});
  write(spi, 0, 1);
  write(spi, pin_function, spi_pins);
  write(spi, format0, format16 | phase | polarity);
  constexpr std::uint32_t delay_fields = 0x01010000;
  write(spi, delay, delay_fields);
  write(spi, control, loopback);
  write(spi, data0, marker);
  require(spi.next_event() == 3);
  require((spi.pins().high & serial_clock) != 0);
  require((spi.pins().enabled & serial_output) == 0);
  drain(spi);
  require(read(spi, buffer) == marker);
}

}

int main() {
  try {
    reset_and_registers();
    clock_modes();
    pipeline_and_interrupts();
    parity_and_unknowns();
    holds_and_reset();
    guards();
    transactional_reads();
    explicit_previous_hold();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
