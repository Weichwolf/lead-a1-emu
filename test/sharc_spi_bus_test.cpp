// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
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

using lead_a1::AccessKind;
using lead_a1::SharcMemory;
using lead_a1::SharcMemoryBus;
using lead_a1::SharcMemoryRequest;
using lead_a1::SharcPeripheralPhase;
using lead_a1::SharcSpi;
constexpr auto io0 = lead_a1::SharcMemoryMaster::io0;
constexpr auto dm = lead_a1::SharcMemoryMaster::dm;
constexpr auto pm = lead_a1::SharcMemoryMaster::pm;
constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t completed_index = 0x92180;
constexpr std::uint32_t pc = 0x8c034;
constexpr std::uint32_t disabled = 6;
constexpr std::uint32_t enabled = 7;
constexpr unsigned int word_bits = 32;
constexpr std::array<std::uint32_t, 6> words{
    0x12345678, 0xfedcba98, 0x87654321, 0xabcdef01, 0x73519280, 0x2468ace0};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC SPI bus expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC SPI bus rejection: " + reason);
}

lead_a1::SharcBootState handoff() {
  // Owned explicit completed bootstrap, no fabricated physical ready line.
  return {.clock = true, .next_word = completed_index};
}

void write(SharcSpi &spi, std::uint32_t offset, std::uint32_t value) {
  spi.write({.address = offset, .size = 4}, value);
}

void word(SharcSpi &spi, SharcMemory &memory, std::uint32_t value) {
  spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  for (unsigned int index = 0; index < word_bits; ++index) {
    spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
    spi.observe(memory, {.selected = true, .clock = true, .mosi = ((value >> index) & 1) != 0});
  }
}

void configure(SharcSpi &spi, unsigned int count) {
  write(spi, 0, base);
  write(spi, 8, count);
}

SharcMemoryRequest expected_word(const SharcSpi &spi) {
  const auto request = spi.dma_word_request();
  if (!request) { throw std::runtime_error("Missing SPI word request"); }
  return *request;
}

void request_word(SharcSpi &spi, SharcMemoryBus &bus) {
  const auto request = expected_word(spi);
  bus.request(io0, request);
  bus.advance();
  if (!bus.grant(io0)) { bus.advance(); }
  require(bus.grant(io0) == request);
}

void threshold_and_each_actual_word() {
  SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  memory.set_pc(pc);
  SharcSpi spi(handoff());
  configure(spi, 6);
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  for (unsigned int index = 0; index < 5; ++index) {
    word(spi, memory, words[index]);
    require(spi.dma_word_request().has_value() == (index >= 3));
  }
  require(spi.state().occupied == 4 && spi.state().receive == words[4]);
  for (unsigned int index = 0; index < 4; ++index) {
    request_word(spi, bus);
    spi.service_dma_word(bus);
    require(memory.read({.address = base + index}) == words[index]);
    require(spi.state().serial.next_word == base + index + 1);
    require(spi.state().serial.remaining == 5 - index && spi.state().burst_remaining == 3 - index);
    require(spi.state().occupied == 4 - index && !spi.state().receive);
    require(!spi.state().interrupt_pending && spi.dma_request() == 0);
    rejected([&] { static_cast<void>(memory.read({.address = base + index + 1})); }, "Unknown");
  }
  require(!spi.dma_word_request()); // Tail of two needs one more serial word.
  word(spi, memory, words[5]);
  require(spi.dma_request() == 2 && spi.dma_word_request());
  for (unsigned int index = 4; index < words.size(); ++index) {
    request_word(spi, bus);
    spi.service_dma_word(bus);
    require(memory.read({.address = base + index}) == words[index]);
    require(spi.state().interrupt_pending == (index == 5));
  }
  require(spi.state().occupied == 0 && spi.state().burst_remaining == 0);
  require(spi.state().serial.remaining == 0 && !spi.dma_word_request());
  require(bus.state().clocks == 11); // Six IO0 words launched on 1,3,5,7,9,11.
  spi.acknowledge_interrupt();
  require(!spi.state().interrupt_pending);
}

void actual_dma_core_contention() {
  SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  SharcSpi spi(handoff());
  configure(spi, 2);
  for (unsigned int index = 0; index < 2; ++index) { word(spi, memory, words[index]); }
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  bus.request(io0, expected_word(spi));
  const SharcMemoryRequest load{.access = {.address = base}};
  bus.request(dm, load);
  bus.request(pm, {.access = {.address = base + 1}});
  bus.advance();
  require(!bus.grant(dm) && !bus.grant(pm));
  spi.service_dma_word(bus);
  require(spi.state().serial.remaining == 1 && !spi.state().interrupt_pending);
  bus.request(io0, expected_word(spi));
  bus.advance();
  require(!bus.grant(io0) && bus.grant(dm) == load && !bus.grant(pm));
  require(bus.execute(dm) == words[0]);
  bus.advance();
  spi.service_dma_word(bus);
  require(spi.state().interrupt_pending && !bus.grant(pm));
  bus.advance();
  require(bus.execute(pm) == words[1] && bus.state().clocks == 4);
}

void later_word_failure_retains_earlier_write() {
  SharcMemory memory;
  memory.map({.bank = 2, .halfwords = 2, .name = "first owned DMA word"});
  memory.set_pc(pc);
  SharcSpi spi(handoff());
  configure(spi, 2);
  word(spi, memory, words[0]);
  word(spi, memory, words[1]);
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  request_word(spi, bus);
  spi.service_dma_word(bus);
  require(memory.read({.address = base}) == words[0]);
  request_word(spi, bus);
  const auto before_spi = spi.state();
  const auto before_bus = bus.state();
  rejected([&] { spi.service_dma_word(bus); }, "pc=0x8c034");
  require(spi.state() == before_spi && bus.state() == before_bus);
  require(memory.read({.address = base}) == words[0] && !spi.state().interrupt_pending);
  memory.map({.bank = 2, .first_halfword = 2, .halfwords = 2, .name = "repair second word"});
  spi.service_dma_word(bus);
  require(memory.read({.address = base + 1}) == words[1]);
  require(spi.state().serial.remaining == 0 && spi.state().interrupt_pending);
}

void mismatched_grants() {
  SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  SharcSpi spi(handoff());
  configure(spi, 1);
  word(spi, memory, words[0]);
  const auto expected = expected_word(spi);
  std::array<SharcMemoryRequest, 4> wrong{expected, expected, expected, expected};
  ++wrong[0].access.address;
  ++wrong[1].value;
  wrong[2].kind = AccessKind::read;
  wrong[2].value = 0;
  for (unsigned int index = 0; index < wrong.size(); ++index) {
    SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
    const auto master = index == 3 ? dm : io0;
    bus.request(master, wrong[index]);
    bus.advance();
    const auto before_bus = bus.state();
    const auto before_spi = spi.state();
    rejected([&] { spi.service_dma_word(bus); }, "exact IOD0");
    require(bus.state() == before_bus && spi.state() == before_spi);
  }
  rejected([&] { static_cast<void>(memory.read({.address = base})); }, "Unknown");
  SharcMemoryBus no_grant(memory, SharcPeripheralPhase::active);
  rejected([&] { spi.service_dma_word(no_grant); }, "exact IOD0");
}

void paused_group_and_foreign_transactions() {
  SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  SharcSpi spi(handoff());
  configure(spi, 2);
  word(spi, memory, words[0]);
  word(spi, memory, words[1]);
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  request_word(spi, bus);
  spi.service_dma_word(bus);
  require(spi.state().burst_remaining == 1);
  request_word(spi, bus);
  const auto before_spi = spi.state();
  const auto before_bus = bus.state();
  rejected([&] { static_cast<void>(spi.service_dma(memory)); }, "requires word grants");
  spi.begin_instruction();
  rejected([&] { spi.service_dma_word(bus); }, "active instruction");
  spi.abort_instruction();
  memory.begin_instruction();
  rejected([&] { spi.service_dma_word(bus); }, "active instruction");
  require(memory.active());
  memory.abort_instruction();
  require(spi.state() == before_spi && bus.state() == before_bus);
  write(spi, 16, disabled);
  const auto paused = spi.state();
  require(!spi.dma_word_request());
  rejected([&] { spi.service_dma_word(bus); }, "exact IOD0");
  rejected([&] { write(spi, 0, base + 2); }, "parameter change");
  rejected([&] { write(spi, 4, 0); }, "parameter change");
  rejected([&] { write(spi, 8, 1); }, "parameter change");
  require(spi.state() == paused && bus.state() == before_bus);
  write(spi, 16, enabled);
  require(spi.state() == before_spi);
  spi.service_dma_word(bus);
  require(spi.state().interrupt_pending && spi.state().burst_remaining == 0);
  // Mixing with the historical functional API is permitted only between groups.
  write(spi, 0, base + 2);
  write(spi, 8, 1);
  word(spi, memory, words[2]);
  require(spi.service_dma(memory) == 1 && memory.read({.address = base + 2}) == words[2]);
}

void signed_modifier_wrap_and_zero_stride() {
  constexpr std::uint32_t normal_base = 0x80000;
  constexpr std::uint32_t normal_last = 0xfffff;
  constexpr std::uint32_t decrement = 0xffff;
  SharcMemory memory;
  memory.map({.bank = 0, .halfwords = 2, .name = "wrap start"});
  constexpr std::uint32_t last_cell = 0x3fffe;
  memory.map({.bank = 3, .first_halfword = last_cell, .halfwords = 2, .name = "wrap end"});
  SharcSpi spi(handoff());
  write(spi, 0, normal_base);
  write(spi, 4, decrement);
  write(spi, 8, 2);
  word(spi, memory, words[0]);
  word(spi, memory, words[1]);
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  request_word(spi, bus);
  spi.service_dma_word(bus);
  require(spi.state().serial.next_word == normal_last);
  request_word(spi, bus);
  spi.service_dma_word(bus);
  require(memory.read({.address = normal_base}) == words[0]);
  require(memory.read({.address = normal_last}) == words[1]);
  write(spi, 0, normal_base);
  write(spi, 4, 0);
  write(spi, 8, 2);
  word(spi, memory, words[2]);
  word(spi, memory, words[3]);
  for (unsigned int index = 0; index < 2; ++index) {
    request_word(spi, bus);
    spi.service_dma_word(bus);
    require(spi.state().serial.next_word == normal_base);
    require(memory.read({.address = normal_base}) == words[index + 2]);
  }
}

}

int main() try {
  threshold_and_each_actual_word();
  actual_dma_core_contention();
  later_word_failure_retains_earlier_write();
  mismatched_grants();
  paused_group_and_foreign_transactions();
  signed_modifier_wrap_and_zero_stride();
  std::cout << "SHARC SPI exact per-word RAM grants, group thresholds, contention and failure "
               "repair passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
