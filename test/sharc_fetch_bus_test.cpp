// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_boot.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr auto pm = lead_a1::SharcMemoryMaster::pm;
constexpr auto io0 = lead_a1::SharcMemoryMaster::io0;
constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t target = base + 4;
constexpr std::uint32_t dma_target = base + 6; // NW32 cell12 aliases native48 instruction4.
constexpr std::uint64_t old_instruction = 0x0f0700000007;   // R7=7.
constexpr std::uint64_t new_instruction = 0x0f070000002a;   // R7=42.
constexpr std::uint64_t later_instruction = 0x0f0700000063; // R7=99.
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t replacement = 42;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("F2 RAM grant expectation at line " + std::to_string(where.line()));
  }
}

template <typename Action> void rejected(const Action &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected " + reason + ": " + error.what());
  }
  throw std::runtime_error("Missing F2 RAM grant rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

lead_a1::SharcFetchState pending(std::uint32_t address = target) {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = address + 1};
  state.pending_fetch2 = address;
  return state;
}

lead_a1::SharcMemoryRequest request(std::uint32_t address = target) {
  return {.access = {.address = address, .width = lead_a1::SharcWidth::wide48},
          .kind = lead_a1::AccessKind::fetch};
}

struct Fixture {
  lead_a1::SharcMemory memory;
  lead_a1::SharcControl control{true};
  lead_a1::Sharc cpu{memory, lead_a1::Sharc::reset_reference_214xx(target), &control};
  lead_a1::SharcFetch fetch{memory, pending()};

  Fixture() {
    constexpr unsigned int cells = 512;
    constexpr unsigned int instructions = 64;
    memory.map({.bank = 2, .halfwords = cells, .name = "owned F2 instruction RAM"});
    memory.map({.bank = 0, .halfwords = cells, .name = "owned independent DMA RAM"});
    for (unsigned int index = 0; index < instructions; ++index) { code(memory, base + index, 0); }
    code(memory, target, old_instruction);
  }
};

void execute_captured(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch, unsigned int expected) {
  for (unsigned int clock = 0; clock < 3; ++clock) {
    require(fetch.advance_sequenced(cpu).address_accepted);
    require(fetch.state().execute.has_value() == (clock == 2));
  }
  const auto step = cpu.step(fetch);
  require(step.pc == target && cpu.state().pc == target + 1 &&
          cpu.state().data[0][7] ==
              (lead_a1::SharcKnown40{.value = std::uint64_t{expected} << 8, .known = full40}));
}

void serial_word(lead_a1::SharcSpi &spi, lead_a1::SharcMemory &memory) {
  spi.write({.address = 0, .size = 4}, dma_target);
  spi.write({.address = 8, .size = 4}, 1);
  spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  constexpr unsigned int bits = 32;
  for (unsigned int bit = 0; bit < bits; ++bit) {
    spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
    spi.observe(memory, {.selected = true, .clock = true, .mosi = ((replacement >> bit) & 1) != 0});
  }
}

void spi_fetch_order(lead_a1::SharcPeripheralPhase phase) {
  Fixture fixture;
  constexpr std::uint32_t completed_boot = 0x92180;
  lead_a1::SharcSpi spi(lead_a1::SharcBootState{.clock = true, .next_word = completed_boot});
  serial_word(spi, fixture.memory);
  const auto dma = spi.dma_word_request();
  if (!dma) { throw std::runtime_error("Missing actual serial DMA word"); }
  require(dma->access.address == dma_target && dma->value == replacement &&
          spi.state().serial.remaining == 1 && !spi.state().interrupt_pending);
  lead_a1::SharcMemoryBus bus(fixture.memory, phase);
  bus.request(pm, request());
  bus.request(io0, *dma);
  bus.advance();
  const auto native = fixture.cpu.state();
  const bool dma_first = phase == lead_a1::SharcPeripheralPhase::active;
  if (dma_first) {
    const auto before = bus.state();
    const auto transport = fixture.fetch.state();
    rejected([&] { fixture.fetch.complete_fetch2(fixture.cpu, bus); },
             "exact PM/native48 fetch grant");
    require(bus.state() == before && fixture.fetch.state() == transport &&
            fixture.cpu.state() == native);
    spi.service_dma_word(bus);
    require(fixture.memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) ==
            new_instruction);
    bus.advance();
  }
  require(bus.grant(pm) == request());
  const auto clocks = bus.state().clocks;
  fixture.fetch.complete_fetch2(fixture.cpu, bus);
  const auto word = fixture.fetch.state().fetch2;
  require(word && word->address == target &&
          word->opcode == (dma_first ? new_instruction : old_instruction));
  require(!bus.grant(pm) && bus.state().clocks == clocks && fixture.cpu.state() == native);
  if (!dma_first) {
    require(!spi.state().interrupt_pending && spi.state().serial.remaining == 1);
    bus.advance();
    spi.service_dma_word(bus);
  }
  require(spi.state().serial.remaining == 0 && spi.state().interrupt_pending &&
          !spi.dma_word_request());
  require(bus.state().clocks == 2 && fixture.memory.read({.address = dma_target}) == replacement);
  code(fixture.memory, target, later_instruction); // A later RAM change must not refresh F2.
  execute_captured(fixture.cpu, fixture.fetch, dma_first ? replacement : 7);
}

void independent_grant_is_retained() {
  Fixture fixture;
  lead_a1::SharcMemoryBus bus(fixture.memory, lead_a1::SharcPeripheralPhase::active);
  constexpr std::uint32_t independent = 0x80000;
  constexpr std::uint32_t marker = 0x12345678;
  const lead_a1::SharcMemoryRequest dma{
      .access = {.address = independent}, .kind = lead_a1::AccessKind::write, .value = marker};
  bus.request(io0, dma);
  bus.request(pm, request());
  bus.advance();
  const auto native = fixture.cpu.state();
  constexpr std::uint32_t extended_bank0 = 0x204;
  fixture.control.write({.size = 4}, extended_bank0);
  const auto controller = fixture.control.snapshot();
  fixture.fetch.complete_fetch2(fixture.cpu, bus);
  require(fixture.control.snapshot() == controller && controller.delayed == extended_bank0);
  require(bus.grant(io0) == dma && !bus.grant(pm) && bus.state().clocks == 1 &&
          fixture.cpu.state() == native);
  require(bus.execute(io0) == marker && fixture.memory.read({.address = independent}) == marker);
  bus.request(pm, request());
  bus.advance();
  const auto before = bus.state();
  const auto fetched = fixture.fetch.state();
  rejected([&] { fixture.fetch.complete_fetch2(fixture.cpu, bus); },
           "pending uncached RAM response");
  require(bus.state() == before && fixture.fetch.state() == fetched);
}

void mismatching_grants_and_owners() {
  constexpr std::array<lead_a1::SharcMemoryRequest, 4> wrong{
      {{.access = {.address = target + 1, .width = lead_a1::SharcWidth::wide48},
        .kind = lead_a1::AccessKind::fetch},
       {.access = {.address = target, .width = lead_a1::SharcWidth::wide48}},
       {.access = {.address = target}},
       {.access = {.address = target, .width = lead_a1::SharcWidth::wide48},
        .kind = lead_a1::AccessKind::write,
        .value = old_instruction}}};
  for (const auto invalid : wrong) {
    Fixture fixture;
    lead_a1::SharcMemoryBus bus(fixture.memory, lead_a1::SharcPeripheralPhase::active);
    bus.request(pm, invalid);
    bus.advance();
    const auto before = bus.state();
    const auto native = fixture.cpu.state();
    const auto transport = fixture.fetch.state();
    rejected([&] { fixture.fetch.complete_fetch2(fixture.cpu, bus); },
             "exact PM/native48 fetch grant");
    require(bus.state() == before && fixture.cpu.state() == native &&
            fixture.fetch.state() == transport);
  }
  Fixture fixture;
  Fixture other;
  lead_a1::SharcMemoryBus bus(fixture.memory, lead_a1::SharcPeripheralPhase::active);
  lead_a1::SharcMemoryBus foreign(other.memory, lead_a1::SharcPeripheralPhase::active);
  bus.request(pm, request());
  bus.advance();
  foreign.request(pm, request());
  foreign.advance();
  const auto before = bus.state();
  const auto remote = foreign.state();
  const auto transport = fixture.fetch.state();
  rejected([&] { fixture.fetch.complete_fetch2(other.cpu, bus); }, "idle bound memory");
  rejected([&] { fixture.fetch.complete_fetch2(fixture.cpu, foreign); }, "foreign RAM");
  fixture.memory.begin_instruction();
  rejected([&] { fixture.fetch.complete_fetch2(fixture.cpu, bus); }, "idle bound memory");
  fixture.memory.abort_instruction();
  fixture.control.begin_instruction();
  rejected([&] { fixture.fetch.complete_fetch2(fixture.cpu, bus); },
           "snapshot during a transaction");
  fixture.control.abort_instruction();
  require(bus.state() == before && foreign.state() == remote && fixture.fetch.state() == transport);
  fixture.fetch.complete_fetch2(fixture.cpu, bus);
  require(!bus.grant(pm));
}

void failed_read_repair_and_separate_restore() {
  Fixture fixture;
  constexpr std::uint32_t extended_bank0 = 0x204;
  fixture.control.write({.size = 4}, extended_bank0);
  constexpr std::uint32_t missing = base + 70;
  lead_a1::SharcFetch fetch(fixture.memory, pending(missing));
  lead_a1::SharcMemoryBus bus(fixture.memory, lead_a1::SharcPeripheralPhase::active);
  bus.request(pm, request(missing));
  bus.advance();
  const auto before = bus.state();
  const auto transport = fetch.state();
  const auto native = fixture.cpu.state();
  const auto control = fixture.control.snapshot();
  rejected([&] { fetch.complete_fetch2(fixture.cpu, bus); }, "Unknown SHARC memory bits");
  require(bus.state() == before && fetch.state() == transport && fixture.cpu.state() == native &&
          fixture.control.snapshot() == control && !fixture.memory.active());
  Fixture other;
  auto restored_control = lead_a1::SharcControl::from_snapshot(control);
  const lead_a1::Sharc other_cpu(other.memory, native, &restored_control);
  lead_a1::SharcFetch other_fetch(other.memory, transport);
  lead_a1::SharcMemoryBus other_bus(other.memory, before);
  code(fixture.memory, missing, new_instruction);
  code(other.memory, missing, new_instruction);
  fetch.complete_fetch2(fixture.cpu, bus);
  other_fetch.complete_fetch2(other_cpu, other_bus);
  const auto word = fetch.state().fetch2;
  require(word && word->opcode == new_instruction && word->address == missing && !bus.grant(pm) &&
          bus.state().clocks == 1 && bus.state() == other_bus.state() &&
          fetch.state() == other_fetch.state());
  require(fixture.control.snapshot() == control && restored_control.snapshot() == control &&
          control.delayed == extended_bank0);
}

}

int main() try {
  spi_fetch_order(lead_a1::SharcPeripheralPhase::active);
  spi_fetch_order(lead_a1::SharcPeripheralPhase::gap);
  independent_grant_is_retained();
  mismatching_grants_and_owners();
  failed_read_repair_and_separate_restore();
  std::cout << "Actual PM F2 grants, SPI arbitration, retained words and independent "
               "repair/restore pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
