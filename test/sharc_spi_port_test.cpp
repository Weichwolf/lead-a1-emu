// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/peripherals/sharc_spi_port.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t first = 0x1000;
constexpr std::uint32_t transmit = 0x1003;
constexpr std::uint32_t receive = 0x1004;
constexpr std::uint32_t shadow = 0x1006;
constexpr std::uint32_t dma_control = 0x1084;
constexpr std::uint32_t boot_end = 0x92180;
constexpr std::uint32_t dma_mode = 0x4d22;
constexpr std::uint32_t core_mode = 0x4d14;
constexpr std::uint32_t overwrite_mode = 0x4d1c;
constexpr std::uint32_t unknown_last_mode = 0x4d10;
constexpr std::uint32_t incoming = 0xd354a913;
constexpr std::uint32_t reply = 0x92345679;
constexpr std::uint32_t next_reply = 0x012a9900;
constexpr unsigned int word_bits = 32;
constexpr unsigned int register_words = 0x85;

struct SpiExchange {
  std::uint32_t mosi;
  std::uint32_t miso;
};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SPI port expectation at line " + std::to_string(where.line()));
  }
}

template <typename Action> void rejected(const Action &action, const std::string &text) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(text) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SPI port rejection: " + text);
}

struct Fixture {
  lead_a1::SharcMemory memory;
  // Owned explicit completed-bootstrap inputs, not firmware or ready replies.
  lead_a1::SharcSpi spi{{.clock = true, .next_word = boot_end}};
  lead_a1::SharcSpiPort port{spi};

  Fixture() { memory.map_io({.first_word = first, .words = register_words}, port); }

  void write(std::uint32_t address, std::uint32_t value) {
    memory.write({.address = address}, value);
  }

  [[nodiscard]] std::uint32_t read(std::uint32_t address) const {
    return static_cast<std::uint32_t>(memory.read({.address = address}));
  }

  void core() {
    write(dma_control, 0);
    write(first, core_mode);
  }

  void clock_word(SpiExchange exchange) {
    spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
    for (unsigned int bit = 0; bit < word_bits; ++bit) {
      spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
      const bool expected = ((exchange.miso >> bit) & 1) != 0;
      require(spi.miso() == expected);
      spi.observe(memory,
                  {.selected = true, .clock = true, .mosi = ((exchange.mosi >> bit) & 1) != 0});
      require(spi.miso() == expected);
    }
  }
};

void shared_state_and_transactions() {
  Fixture fixture;
  require(fixture.read(first) == dma_mode && fixture.read(dma_control) == 7);
  require(fixture.spi.read({.address = 16, .size = 4}) == 7);
  const auto before = fixture.spi.state();
  fixture.memory.begin_instruction();
  fixture.core();
  fixture.write(transmit, reply);
  require(fixture.read(first) == core_mode && fixture.read(dma_control) == 0);
  fixture.memory.abort_instruction();
  require(fixture.spi.state() == before);
  fixture.memory.begin_instruction();
  fixture.core();
  fixture.write(transmit, reply);
  fixture.memory.commit_instruction();
  require(fixture.spi.state().port_control == core_mode && fixture.spi.state().control == 0);
  require(fixture.spi.state().transmit == reply && fixture.spi.state().serial == before.serial);
  require(!fixture.spi.dma_word_request() && fixture.spi.dma_request() == 0);
}

void full_duplex_and_level_interrupt() {
  Fixture fixture;
  fixture.core();
  fixture.write(transmit, reply);
  fixture.clock_word({.mosi = incoming, .miso = reply});
  require(fixture.spi.interrupt_request() && fixture.spi.state().receive == incoming);
  require(!fixture.spi.state().transmit && !fixture.spi.state().transmit_underflow);
  fixture.spi.acknowledge_interrupt();
  require(fixture.spi.interrupt_request()); // Core service is a held buffer request.
  rejected([&] { static_cast<void>(fixture.read(shadow)); }, "not implemented");
  require(fixture.spi.interrupt_request());
  const auto before = fixture.spi.state();
  fixture.memory.begin_instruction();
  require(fixture.read(receive) == incoming && !fixture.spi.interrupt_request());
  fixture.memory.abort_instruction();
  require(fixture.spi.state() == before && fixture.spi.interrupt_request());
  require(fixture.read(receive) == incoming && !fixture.spi.interrupt_request());
  require(fixture.read(receive) == incoming); // Data latch survives the full-bit clear.
  fixture.write(transmit, next_reply);
  fixture.clock_word({.mosi = reply, .miso = next_reply});
  require(fixture.read(receive) == reply && !fixture.spi.interrupt_request());
  fixture.clock_word(
      {.mosi = next_reply, .miso = 0}); // SENDZ supplies zeros with an empty TX buffer.
  require(fixture.spi.state().transmit_underflow && fixture.read(receive) == next_reply);
  constexpr std::uint32_t transmit_error_status = 0x4400;
  require(fixture.read(dma_control) == transmit_error_status);
  require(!fixture.spi.interrupt_request());
  constexpr std::uint32_t error_interrupt = 0x100;
  fixture.write(dma_control, error_interrupt);
  require(fixture.spi.interrupt_request());
  fixture.spi.acknowledge_interrupt();
  require(fixture.spi.interrupt_request()); // The uncleared error remains asserted.
  fixture.write(dma_control, 0);
  require(!fixture.spi.interrupt_request());
  fixture.spi.observe(fixture.memory, {.selected = false, .clock = true, .mosi = {}});
  require(!fixture.spi.miso());
}

void discard_overwrite_and_unknowns() {
  Fixture fixture;
  fixture.core();
  rejected([&] { static_cast<void>(fixture.read(receive)); }, "unknown");
  fixture.clock_word({.mosi = incoming, .miso = 0});
  fixture.clock_word({.mosi = reply, .miso = 0});
  require(fixture.spi.state().overflow && fixture.read(receive) == incoming);
  fixture.write(first, overwrite_mode);
  fixture.clock_word({.mosi = reply, .miso = 0});
  fixture.clock_word({.mosi = next_reply, .miso = 0});
  require(fixture.read(receive) == next_reply);
  Fixture unknown;
  unknown.write(dma_control, 0);
  unknown.write(first, unknown_last_mode);
  unknown.spi.observe(unknown.memory, {.selected = true, .clock = true, .mosi = {}});
  unknown.spi.observe(unknown.memory, {.selected = true, .clock = false, .mosi = {}});
  const auto before = unknown.spi.state();
  rejected([&] { static_cast<void>(unknown.spi.miso()); }, "unknown");
  require(unknown.spi.state() == before && unknown.spi.state().transmit_underflow);
}

void queued_transmit_and_active_guards() {
  Fixture fixture;
  fixture.core();
  fixture.write(transmit, reply);
  fixture.spi.observe(fixture.memory, {.selected = true, .clock = true, .mosi = {}});
  fixture.spi.observe(fixture.memory, {.selected = true, .clock = false, .mosi = {}});
  fixture.write(transmit, next_reply);
  const auto before = fixture.spi.state();
  rejected([&] { fixture.write(transmit, incoming); }, "full TXSPI");
  require(fixture.spi.state() == before);
  fixture.spi.observe(fixture.memory, {.selected = true, .clock = true, .mosi = true});
  rejected([&] { fixture.write(first, overwrite_mode); }, "active");
  for (unsigned int bit = 1; bit < word_bits; ++bit) {
    fixture.spi.observe(fixture.memory, {.selected = true, .clock = false, .mosi = {}});
    const bool expected = ((reply >> bit) & 1) != 0;
    require(fixture.spi.miso() == expected);
    fixture.spi.observe(fixture.memory, {.selected = true, .clock = true, .mosi = false});
    require(fixture.spi.miso() == expected);
  }
  require(fixture.read(receive) == 1 && fixture.spi.state().transmit == next_reply);
  fixture.clock_word({.mosi = incoming, .miso = next_reply});
  require(fixture.read(receive) == incoming);
}

void pending_dma_data_survives_mode_change() {
  Fixture fixture;
  // Keep DMISO disabled while genuine receive-DMA state is populated.
  const auto input_word = [&](std::uint32_t value) {
    fixture.spi.observe(fixture.memory, {.selected = true, .clock = true, .mosi = {}});
    for (unsigned int bit = 0; bit < word_bits; ++bit) {
      fixture.spi.observe(fixture.memory, {.selected = true, .clock = false, .mosi = {}});
      fixture.spi.observe(fixture.memory,
                          {.selected = true, .clock = true, .mosi = ((value >> bit) & 1) != 0});
    }
  };
  input_word(incoming);
  fixture.write(dma_control, 0);
  input_word(reply);
  const auto before = fixture.spi.state();
  require(before.occupied == 1 && before.receive == reply);
  fixture.write(first, core_mode);
  require(fixture.spi.state().fifo == before.fifo && fixture.spi.state().occupied == 1);
  require(fixture.spi.state().serial == before.serial && fixture.spi.interrupt_request());
  require(fixture.read(receive) == reply && fixture.spi.state().occupied == 1);
  require(fixture.spi.dma_request() == 0 && !fixture.spi.dma_word_request());
}

void malformed_accesses() {
  Fixture fixture;
  fixture.memory.set_pc(boot_end);
  const auto before = fixture.spi.state();
  rejected([&] { fixture.write(first, core_mode); }, "active");
  rejected([&] { static_cast<void>(fixture.read(first + 1)); },
           "read pc=0x92180 word-address=0x1001 bits=32");
  rejected([&] { fixture.write(first + 2, 0); }, "write pc=0x92180 word-address=0x1002 bits=32");
  rejected([&] { fixture.write(first, 0); }, "not qualified");
  rejected([&] { fixture.port.write({.address = 2, .size = 4}, 0); }, "aligned32");
  rejected([&] { static_cast<void>(fixture.port.read({.address = 0, .size = 8})); }, "aligned32");
  rejected([&] { fixture.port.write({.address = 0, .size = 4}, std::uint64_t{1} << 32); },
           "exceeds");
  require(fixture.spi.state() == before);
}

}

int main() {
  try {
    shared_state_and_transactions();
    full_duplex_and_level_interrupt();
    discard_overwrite_and_unknowns();
    queued_transmit_and_active_guards();
    pending_dma_data_survives_mode_change();
    malformed_accesses();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
