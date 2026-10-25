// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_boot.hpp"
#include "sharc/peripherals/sharc_spi.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t busy = 0x8000;
constexpr std::uint32_t p1i = 0x1000;
constexpr unsigned int word_bits = 32;
constexpr std::uint32_t dma_index = 0x1080;
constexpr std::uint32_t dma_count = 0x1082;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC runtime SPI expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC runtime SPI rejection: " + reason);
}

lead_a1::SharcBootState handoff() {
  // Owned explicit completed-bootstrap reference; not a fabricated ready signal.
  constexpr std::uint32_t completed_index = 0x92180;
  return {.clock = true, .next_word = completed_index};
}

void write(lead_a1::SharcSpi &spi, std::uint32_t offset, std::uint32_t value) {
  spi.write({.address = offset, .size = 4}, value);
}

std::uint32_t read(lead_a1::SharcSpi &spi, std::uint32_t offset) {
  return static_cast<std::uint32_t>(spi.read({.address = offset, .size = 4}));
}

void bit(lead_a1::SharcSpi &spi, lead_a1::SharcMemory &memory, bool value) {
  spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
  spi.observe(memory, {.selected = true, .clock = true, .mosi = value});
}

void word(lead_a1::SharcSpi &spi, lead_a1::SharcMemory &memory, std::uint32_t value) {
  spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  for (unsigned int index = 0; index < word_bits; ++index) {
    bit(spi, memory, ((value >> index) & 1) != 0);
  }
}

void header_and_retained_fifo() {
  lead_a1::SharcSpi spi(handoff());
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  write(spi, 0, base);
  write(spi, 8, 3);
  require(read(spi, 16) == (7 | busy) && read(spi, 8) == 3);
  constexpr std::array<std::uint32_t, 3> header{0x87654321, 0x2468abcd, 0xff0081fe};
  for (unsigned int index = 0; index < header.size(); ++index) {
    word(spi, memory, header[index]);
    require(spi.dma_request() == (index == 2 ? 3U : 0U));
    require((read(spi, 16) & busy) != 0 && read(spi, 8) == 3);
    rejected([&] { static_cast<void>(memory.read({.address = base})); }, "Unknown");
  }
  const auto waiting = spi.state();
  for (unsigned int index = 0; index < 8; ++index) { static_cast<void>(read(spi, 16)); }
  require(spi.state() == waiting);
  require(spi.service_dma(memory) == 3);
  for (unsigned int index = 0; index < header.size(); ++index) {
    require(memory.read({.address = base + index}) == header[index]);
  }
  require(spi.state().serial.next_word == base + 3 && spi.state().serial.remaining == 0);
  require(read(spi, 16) == 7 && read(spi, 8) == 0 && spi.state().interrupt_pending);
  spi.acknowledge_interrupt();
  require(!spi.state().interrupt_pending && spi.service_dma(memory) == 0);
  // Receive ahead while DMA count is zero; keep the words for the next block.
  word(spi, memory, header[2]);
  word(spi, memory, header[0]);
  require(spi.dma_request() == 0 && spi.state().occupied == 2);
  write(spi, 0, base + 8);
  constexpr std::uint32_t decrement = 0xffff;
  write(spi, 4, decrement); // Signed16 -1, independent of unsigned host arithmetic.
  write(spi, 8, 2);
  require(spi.dma_request() == 2 && spi.service_dma(memory) == 2);
  require(memory.read({.address = base + 8}) == header[2]);
  require(memory.read({.address = base + 7}) == header[0]);
  require(spi.state().serial.next_word == base + 6);
}

void grouped_requests() {
  lead_a1::SharcSpi spi(handoff());
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  write(spi, 0, base);
  constexpr unsigned int count = 18;
  write(spi, 8, count);
  constexpr std::array<unsigned int, 5> groups{4, 4, 4, 4, 2};
  unsigned int sent{};
  for (const auto group : groups) {
    for (unsigned int index = 0; index < group; ++index) {
      word(spi, memory, ++sent);
      require(spi.dma_request() == (index + 1 == group ? group : 0));
    }
    const auto remaining = count - sent + group;
    require(read(spi, 8) == remaining);
    require(spi.service_dma(memory) == group);
    require(spi.state().interrupt_pending == (sent == count));
  }
  for (unsigned int index = 0; index < count; ++index) {
    require(memory.read({.address = base + index}) == index + 1);
  }
  write(spi, 8, 0); // New zero count means 65536, completed zero stays idle.
  constexpr unsigned int full_count = 65536;
  require(spi.state().serial.remaining == full_count && read(spi, 8) == 0);
  require((read(spi, 16) & busy) != 0 && spi.dma_request() == 0);
}

void finite_capacity_and_disabled_dma() {
  lead_a1::SharcSpi spi(handoff());
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  write(spi, 0, base);
  constexpr std::uint32_t enabled_errors = 0x107;
  write(spi, 16, enabled_errors); // EN/RX/INTEN/INTERR; no count is active yet.
  constexpr unsigned int buffered = 5;
  for (unsigned int value = 1; value <= buffered; ++value) { word(spi, memory, value); }
  require(spi.state().occupied == 4 && spi.state().receive == buffered);
  constexpr std::uint32_t full_fifo = 0x3107;
  require(read(spi, 16) == full_fifo && !spi.state().overflow);
  constexpr std::uint32_t discarded = 0xfedcba98;
  word(spi, memory, discarded);
  require(spi.state().overflow && spi.state().receive == buffered);
  constexpr std::uint32_t full_overflow = 0x7307;
  require(read(spi, 16) == full_overflow && spi.state().interrupt_pending);
  spi.acknowledge_interrupt();
  write(spi, 8, buffered);
  require(spi.service_dma(memory) == 4);
  require(spi.state().occupied == 1 && !spi.state().receive);
  require(spi.service_dma(memory) == 1);
  for (unsigned int index = 0; index < buffered; ++index) {
    require(memory.read({.address = base + index}) == index + 1);
  }
  // Disabling DMA preserves its FIFO and the SPI port keeps receiving.
  word(spi, memory, discarded);
  write(spi, 16, 2);
  word(spi, memory, buffered);
  require(spi.state().occupied == 1 && spi.state().receive == buffered);
  write(spi, 0, base + 8);
  write(spi, 8, 2);
  require(spi.dma_request() == 0 && spi.service_dma(memory) == 0);
  write(spi, 16, 3); // Reenable with completion interrupts disabled.
  require(spi.state().occupied == 2 && spi.dma_request() == 2);
  spi.acknowledge_interrupt();
  require(spi.service_dma(memory) == 2 && !spi.state().interrupt_pending);
}

void failed_grant_and_transactions() {
  lead_a1::SharcSpi spi(handoff());
  lead_a1::SharcMemory memory;
  memory.map({.bank = 2, .halfwords = 2, .name = "owned DMA first word"});
  memory.map_io({.first_word = dma_index, .words = 5}, spi);
  write(spi, 0, base);
  write(spi, 8, 2);
  word(spi, memory, 1);
  word(spi, memory, 2);
  const auto ready = spi.state();
  rejected([&] { static_cast<void>(spi.service_dma(memory)); }, "word-address=0xc0000");
  require(spi.state() == ready && !memory.active());
  rejected([&] { static_cast<void>(memory.read({.address = base})); }, "Unknown");
  memory.map({.bank = 2, .first_halfword = 2, .halfwords = 2, .name = "owned DMA next word"});
  require(spi.service_dma(memory) == 2);
  require(memory.read({.address = base}) == 1 && memory.read({.address = base + 1}) == 2);
  const auto before = spi.state();
  memory.begin_instruction();
  memory.write({.address = dma_index}, base);
  memory.write({.address = dma_count}, 1);
  rejected([&] { spi.observe(memory, {.selected = true, .clock = true, .mosi = {}}); },
           "active instruction");
  rejected([&] { static_cast<void>(spi.service_dma(memory)); }, "active instruction");
  memory.abort_instruction();
  require(spi.state() == before);
  memory.begin_instruction();
  memory.write({.address = dma_count}, 1);
  memory.commit_instruction();
  require(spi.state().serial.remaining == 1 && !memory.active());
  rejected([&] { write(spi, 0, base); }, "active SPI DMA");
  constexpr std::uint32_t fifo_flush = 0x87;
  rejected([&] { write(spi, 16, fifo_flush); }, "flush/control");
  rejected([&] { static_cast<void>(read(spi, 0)); }, "readback");
  rejected([&] { static_cast<void>(read(spi, 12)); }, "chaining");
}

void serial_rejections() {
  auto incomplete = handoff();
  incomplete.remaining = 1;
  rejected([&] { const lead_a1::SharcSpi spi(incomplete); }, "completed initial boot");
  lead_a1::SharcSpi spi(handoff());
  lead_a1::SharcMemory memory;
  rejected([&] { spi.observe(memory, {.selected = true, .clock = false, .mosi = {}}); },
           "idle-high");
  spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
  const auto before = spi.state();
  rejected([&] { spi.observe(memory, {.selected = true, .clock = true, .mosi = {}}); },
           "Unknown SHARC runtime");
  require(spi.state() == before);
  spi.observe(memory, {.selected = true, .clock = true, .mosi = true});
  spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  require(spi.state().serial.bits == 1 && spi.state().serial.shift == 1);
  rejected([&] { spi.observe(memory, {.selected = false, .clock = true, .mosi = {}}); },
           "within runtime word");
  require(spi.state().serial.bits == 1 && !spi.miso());
}

void enable_stored_error() {
  lead_a1::SharcSpi spi(handoff());
  lead_a1::SharcMemory memory;
  for (unsigned int index = 0; index < 6; ++index) { word(spi, memory, index); }
  require(spi.state().overflow && !spi.state().interrupt_pending);
  constexpr std::uint32_t enable_errors = 0x107;
  write(spi, 16, enable_errors);
  require(spi.state().interrupt_pending);
  spi.acknowledge_interrupt();
  require(!spi.state().interrupt_pending && spi.state().overflow);
}

void cursor_wrap_and_full_zero_count() {
  constexpr std::uint32_t first = 0x80000;
  constexpr std::uint32_t last = 0xfffff;
  constexpr std::uint32_t last_cell = 0x3fffe;
  constexpr std::uint32_t backwards = 0xffff;
  lead_a1::SharcMemory memory;
  // Owned endpoint cells qualify cursor arithmetic, not the Nord RAM geometry.
  memory.map({.bank = 0, .halfwords = 2, .name = "owned low DMA wrap endpoint"});
  memory.map({.bank = 3,
              .first_halfword = last_cell,
              .halfwords = 2,
              .name = "owned high DMA wrap endpoint"});
  for (const auto start : {first, last}) {
    lead_a1::SharcSpi spi(handoff());
    write(spi, 0, start);
    write(spi, 4, start == first ? backwards : 1);
    write(spi, 8, 2);
    word(spi, memory, 1);
    word(spi, memory, 2);
    require(spi.service_dma(memory) == 2);
    require(memory.read({.address = start}) == 1);
    require(memory.read({.address = start == first ? last : first}) == 2);
    require(spi.state().serial.next_word == (start == first ? last - 1 : first + 1));
  }
  lead_a1::SharcSpi spi(handoff());
  write(spi, 0, first);
  write(spi, 4, 0); // Count every word while repeatedly writing one owned cell.
  write(spi, 8, 0);
  constexpr unsigned int words = 65536;
  for (unsigned int sent = 0; sent < words; sent += 4) {
    for (unsigned int index = 0; index < 4; ++index) { word(spi, memory, sent + index + 1); }
    require(spi.service_dma(memory) == 4);
    require(spi.state().serial.remaining == words - sent - 4);
    require(spi.state().interrupt_pending == (sent + 4 == words));
  }
  require(memory.read({.address = first}) == words && spi.state().serial.next_word == first);
  require(read(spi, 8) == 0 && read(spi, 16) == 7 && spi.service_dma(memory) == 0);
}

void masked_interrupt_latch() {
  lead_a1::SharcMemory memory;
  memory.map({.bank = 2, .halfwords = 6, .name = "owned interrupt NOPs"});
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, 0);
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, 0);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[2].value |= p1i; // MODE1.IRPTEN, independent of IMASK.P1I.
  initial.effective[2] = initial.system[2];
  lead_a1::Sharc cpu(memory, initial);
  cpu.latch_interrupt(p1i);
  require(cpu.state().system[10].value == p1i && cpu.state().effective[10].value == p1i);
  static_cast<void>(cpu.step()); // Masked event is retained across execution.
  require(cpu.state().system[10].value == p1i && cpu.state().pc == base + 1);
  const auto before = cpu.state();
  memory.begin_instruction();
  rejected([&] { cpu.latch_interrupt(p1i); }, "idle transaction");
  memory.abort_instruction();
  rejected([&] { cpu.latch_interrupt(1); }, "maskable IRPTL");
  require(cpu.state() == before);
  auto unmasked = before;
  unmasked.system[11].value |= p1i;
  unmasked.effective[11] = unmasked.system[11];
  lead_a1::Sharc other(memory, unmasked);
  rejected([&] { static_cast<void>(other.step()); }, "pending interrupt delivery");
  require(other.state() == unmasked && !memory.active());
  auto delayed = initial;
  delayed.pending[10] = delayed.system[10];
  lead_a1::Sharc pending(memory, delayed);
  pending.latch_interrupt(p1i);
  static_cast<void>(pending.step());
  require(pending.state().effective[10].value == p1i);
}

}

int main() {
  try {
    header_and_retained_fifo();
    grouped_requests();
    finite_capacity_and_disabled_dma();
    failed_grant_and_transactions();
    serial_rejections();
    enable_stored_error();
    cursor_wrap_and_full_zero_count();
    masked_interrupt_latch();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
