// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_memory_bus.hpp"
#include "lead_a1/sharc_memory_view.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

using lead_a1::AccessKind;
using lead_a1::SharcMemory;
using lead_a1::SharcMemoryBus;
using lead_a1::SharcMemoryBusState;
using lead_a1::SharcMemoryMaster;
using lead_a1::SharcMemoryRequest;
using lead_a1::SharcPeripheralPhase;
using lead_a1::SharcWidth;
constexpr auto io0 = SharcMemoryMaster::io0;
constexpr auto io1 = SharcMemoryMaster::io1;
constexpr auto dm = SharcMemoryMaster::dm;
constexpr auto pm = SharcMemoryMaster::pm;
constexpr std::array masters{io0, io1, dm, pm};
constexpr std::uint32_t base = 0x80000;
constexpr std::uint32_t bank_span = 0x20000;
constexpr std::uint32_t short_base = 0x100000;
constexpr std::uint32_t cells = 0x40000;
constexpr std::uint32_t pc = 0x8c003;
constexpr std::uint32_t spi_index = 0x1080;
constexpr std::uint16_t all_known = 0xffff;
constexpr std::uint64_t first_word = 0x123456789abc;
constexpr std::uint64_t replacement = 0x01234567;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC bus expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC bus rejection: " + reason);
}

SharcMemoryRequest store(unsigned int bank = 0, std::uint64_t value = replacement) {
  return {
      .access = {.address = base + (bank * bank_span)}, .kind = AccessKind::write, .value = value};
}

void map(SharcMemory &memory) {
  for (unsigned int bank = 0; bank < 4; ++bank) {
    memory.map({.bank = bank, .halfwords = 16, .name = "owned internal RAM"});
  }
  memory.set_pc(pc);
}

void view_geometry() {
  using lead_a1::SharcAccess;
  using lead_a1::SharcMemoryView;

  struct Case {
    SharcAccess access;
    SharcMemoryView expected;
  };

  constexpr std::uint32_t long_base = 0x40000;
  constexpr std::uint32_t long_span = 0x10000;
  for (unsigned int bank = 0; bank < 4; ++bank) {
    const auto normal = base + (bank_span * bank);
    const std::array cases{
        Case{.access = {.address = short_base + (cells * bank) + 5, .width = SharcWidth::short16},
             .expected = {.bank = bank, .cell = 5, .cells = 1}},
        Case{.access = {.address = normal + 5, .width = SharcWidth::normal32},
             .expected = {.bank = bank, .cell = 10, .cells = 2}},
        Case{.access = {.address = normal + 5, .width = SharcWidth::extended40},
             .expected = {.bank = bank, .cell = 15, .cells = 3, .shift = 8}},
        Case{.access = {.address = normal + 5, .width = SharcWidth::wide48},
             .expected = {.bank = bank, .cell = 15, .cells = 3}},
        Case{.access = {.address = normal + 5, .width = SharcWidth::long64},
             .expected = {.bank = bank, .cell = 8, .cells = 4}},
        Case{.access = {.address = long_base + (long_span * bank) + 5, .width = SharcWidth::long64},
             .expected = {.bank = bank, .cell = 20, .cells = 4}}};
    for (const auto &item : cases) {
      require(lead_a1::sharc_memory_view(item.access) == item.expected);
    }
  }
  rejected([] { static_cast<void>(lead_a1::sharc_memory_view({.address = spi_index})); }, "view");
  rejected(
      [] {
        static_cast<void>(lead_a1::sharc_memory_view(
            {.address = base, .width = std::bit_cast<SharcWidth>(std::uint8_t{0})}));
      },
      "width");
}

void all_request_sets() {
  // All sixteen finite request masks, independent literal priority IO0>IO1>DM>PM.
  for (unsigned int mask = 0; mask < 16; ++mask) {
    SharcMemory memory;
    map(memory);
    SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
    unsigned int winner = 4;
    for (unsigned int port = 0; port < masters.size(); ++port) {
      if ((mask & (1U << port)) == 0) { continue; }
      bus.request(masters[port], store(0, port));
      if (winner == 4) { winner = port; }
    }
    bus.advance();
    for (unsigned int port = 0; port < masters.size(); ++port) {
      require(bus.grant(masters[port]).has_value() == (port == winner));
    }
    require(bus.state().clocks == 1 && bus.state().next_phase == SharcPeripheralPhase::gap);
    if (winner != 4) {
      require(bus.execute(masters[winner]) == winner);
      require(memory.read({.address = base}) == winner);
    }
  }
}

void carried_io_and_core_gaps() {
  SharcMemory memory;
  map(memory);
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  for (unsigned int port = 0; port < masters.size(); ++port) {
    bus.request(masters[port], store(0, port));
  }
  bus.advance();
  require(bus.grant(io0) == store(0, 0) && bus.state().carried[1]);
  require(bus.execute(io0) == 0);
  const auto retained = bus.state();
  SharcMemoryBus restored(memory, retained);
  restored.request(io0, store(0, 4)); // Newly queued IO0 must wait for the next launch clock.
  restored.advance();
  require(!restored.grant(io0) && restored.grant(io1) == store(0, 1));
  require(!restored.state().carried[1] && restored.execute(io1) == 1);
  restored.advance();
  require(restored.execute(io0) == 4);
  restored.advance();
  require(restored.grant(dm) == store(0, 2) && !restored.grant(pm));
  require(restored.execute(dm) == 2);
  restored.advance();
  require(restored.execute(pm) == 3);
  require(restored.state().clocks == 5 && memory.read({.address = base}) == 3);

  SharcMemoryBus gap(memory, SharcPeripheralPhase::gap);
  gap.request(io0, store(0, 5));
  gap.request(dm, store(0, 6));
  gap.advance();
  require(gap.grant(dm) && !gap.grant(io0) && gap.execute(dm) == 6);
  gap.advance();
  require(gap.execute(io0) == 5);
}

void independent_blocks() {
  SharcMemory memory;
  map(memory);
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  for (unsigned int bank = 0; bank < masters.size(); ++bank) {
    bus.request(masters[bank], store(bank, bank + 1));
  }
  bus.advance();
  const auto checkpoint = bus.state();
  rejected([&] { bus.advance(); }, "completed grants");
  rejected([&] { bus.request(dm, store()); }, "completed grants");
  require(bus.state() == checkpoint);
  SharcMemory other_memory;
  map(other_memory);
  SharcMemoryBus other(other_memory, checkpoint);
  // Independent block operations commute even when their grant consumption order differs.
  for (unsigned int bank = 0; bank < masters.size(); ++bank) {
    require(bus.execute(masters[bank]) == bank + 1);
    const auto reverse = 3 - bank;
    require(other.execute(masters[reverse]) == reverse + 1);
  }
  require(bus.state() == other.state());
  for (unsigned int bank = 0; bank < masters.size(); ++bank) {
    require(memory.read({.address = base + (bank * bank_span)}) == bank + 1);
    require(other_memory.read({.address = base + (bank * bank_span)}) == bank + 1);
  }
}

void actual_fetch_after_dma() {
  SharcMemory memory;
  map(memory);
  memory.write({.address = base, .width = SharcWidth::wide48}, first_word);
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  const SharcMemoryRequest fetch{.access = {.address = base, .width = SharcWidth::wide48},
                                 .kind = AccessKind::fetch};
  bus.request(pm, fetch);
  bus.request(io0, store());
  bus.advance();
  require(!bus.grant(pm) && bus.execute(io0) == replacement);
  bus.advance();
  constexpr std::uint64_t merged = 0x123401234567;
  require(bus.grant(pm) == fetch && bus.execute(pm) == merged);
  const SharcMemoryRequest alias{.access = {.address = short_base, .width = SharcWidth::short16},
                                 .kind = AccessKind::write,
                                 .value = 7};
  bus.request(dm, alias);
  bus.request(pm, fetch);
  bus.advance();
  require(bus.grant(dm) == alias && !bus.grant(pm) && bus.execute(dm) == 7);
  bus.advance();
  constexpr std::uint64_t alias_merged = 0x123401230007;
  require(bus.execute(pm) == alias_merged);
}

void actual_width_accesses() {
  SharcMemory memory;
  map(memory);
  memory.write({.address = base, .width = SharcWidth::wide48}, first_word);
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  constexpr std::uint64_t extended = 0x1122334455;
  const SharcMemoryRequest extended_store{
      .access = {.address = base, .width = SharcWidth::extended40},
      .kind = AccessKind::write,
      .value = extended};
  bus.request(dm, extended_store);
  bus.advance();
  require(bus.execute(dm) == extended);
  constexpr std::uint64_t preserved_low_byte = 0x1122334455bc;
  require(memory.read({.address = base, .width = SharcWidth::wide48}) == preserved_low_byte);
  constexpr std::uint64_t long_value = 0xaabbccddeeff0123;
  bus.request(dm,
              {.access = {.address = base + 1, .width = SharcWidth::long64},
               .kind = AccessKind::write,
               .value = long_value});
  bus.advance();
  require(bus.execute(dm) == long_value);
  constexpr std::uint32_t long_base = 0x40000;
  bus.request(pm, {.access = {.address = long_base, .width = SharcWidth::long64}});
  bus.advance();
  require(bus.execute(pm) == long_value);
  constexpr std::uint64_t upper_half = 0xaabb;
  bus.request(dm, {.access = {.address = short_base + 3, .width = SharcWidth::short16}});
  bus.advance();
  require(bus.execute(dm) == upper_half);
}

void access_failure_and_repair() {
  SharcMemory memory;
  memory.set_pc(pc);
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  bus.request(dm, store());
  bus.advance();
  const auto unmapped = bus.state();
  rejected([&] { static_cast<void>(bus.execute(dm)); }, "pc=0x8c003");
  require(bus.state() == unmapped);
  memory.map({.bank = 0, .halfwords = 2, .name = "repair"});
  require(bus.execute(dm) == replacement && memory.read({.address = base}) == replacement);
  const SharcMemoryRequest unknown{.access = {.address = base + 1}};
  memory.map({.bank = 0, .first_halfword = 2, .halfwords = 2, .name = "unknown"});
  bus.request(dm, unknown);
  bus.advance();
  const auto waiting = bus.state();
  rejected([&] { static_cast<void>(bus.execute(dm)); }, "Unknown");
  require(bus.state() == waiting);
  memory.write({.address = base + 1}, 5);
  require(bus.execute(dm) == 5);
  bus.request(dm, store());
  bus.advance();
  const auto granted = bus.state();
  memory.begin_instruction();
  memory.write({.address = base}, 6);
  rejected([&] { static_cast<void>(bus.execute(dm)); }, "another transaction");
  require(memory.active() && bus.state() == granted && memory.read({.address = base}) == 6);
  memory.abort_instruction();
  require(memory.read({.address = base}) == replacement && bus.execute(dm) == replacement);
}

void permissions() {
  SharcMemory memory;
  constexpr std::array<lead_a1::SharcCell, 3> known{{{.value = 1, .known = all_known},
                                                     {.value = 2, .known = all_known},
                                                     {.value = 3, .known = all_known}}};
  memory.map(
      {.bank = 0, .halfwords = 3, .name = "protected", .writable = false, .executable = false},
      known);
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  bus.request(dm, store());
  bus.advance();
  const auto before = bus.state();
  rejected([&] { static_cast<void>(bus.execute(dm)); }, "Read-only");
  require(bus.state() == before);
  require(memory.read({.address = short_base, .width = SharcWidth::short16}) == 1);
  SharcMemoryBus fetch(memory, SharcPeripheralPhase::active);
  fetch.request(
      pm, {.access = {.address = base, .width = SharcWidth::wide48}, .kind = AccessKind::fetch});
  fetch.advance();
  const auto no_execute = fetch.state();
  rejected([&] { static_cast<void>(fetch.execute(pm)); }, "Non-executable");
  require(fetch.state() == no_execute);
}

void malformed_requests() {
  SharcMemory memory;
  SharcMemoryBus bus(memory, SharcPeripheralPhase::active);
  constexpr auto invalid_master = std::bit_cast<SharcMemoryMaster>(std::uint8_t{4});
  const auto initial = bus.state();
  rejected([&] { bus.request(invalid_master, store()); }, "master");
  rejected([&] { static_cast<void>(bus.grant(invalid_master)); }, "master");
  rejected([&] { static_cast<void>(bus.execute(pm)); }, "no grant");
  auto request = store();
  request.kind = AccessKind::fetch;
  rejected([&] { bus.request(dm, request); }, "PM/native48");
  request = {.access = {.address = base, .width = SharcWidth::wide48}};
  rejected([&] { bus.request(io1, request); }, "normal32");
  request = {.access = {.address = base}, .value = 1};
  rejected([&] { bus.request(dm, request); }, "value");
  constexpr std::uint64_t too_wide = 0x100000000;
  rejected([&] { bus.request(dm, store(0, too_wide)); }, "value");
  request = {.access = {.address = base + bank_span - 1, .width = SharcWidth::wide48}};
  rejected([&] { bus.request(pm, request); }, "block span");
  request = {.access = {.address = spi_index}};
  rejected([&] { bus.request(dm, request); }, "view");
  request = store();
  request.kind = std::bit_cast<AccessKind>(std::uint8_t{3});
  rejected([&] { bus.request(dm, request); }, "operation");
  require(bus.state() == initial);
  bus.request(dm, store());
  const auto pending = bus.state();
  rejected([&] { bus.request(dm, store(1)); }, "already pending");
  require(bus.state() == pending);
  memory.begin_instruction();
  rejected([&] { bus.advance(); }, "idle memory");
  rejected([&] { bus.request(pm, store()); }, "idle memory");
  require(memory.active() && bus.state() == pending);
  memory.abort_instruction();
}

void invalid_snapshots() {
  SharcMemory memory;
  SharcMemoryBusState state;
  state.next_phase = std::bit_cast<SharcPeripheralPhase>(std::uint8_t{2});
  rejected([&] { const SharcMemoryBus bus(memory, state); }, "phase");
  state = {};
  state.carried[0] = true;
  rejected([&] { const SharcMemoryBus bus(memory, state); }, "carried");
  state = {.carried = {false, true}, .clocks = 1, .next_phase = SharcPeripheralPhase::gap};
  rejected([&] { const SharcMemoryBus bus(memory, state); }, "carried");
  state = {};
  state.granted[2] = store();
  rejected([&] { const SharcMemoryBus bus(memory, state); }, "inherited");
  state.clocks = 1;
  state.pending[2] = store();
  rejected([&] { const SharcMemoryBus bus(memory, state); }, "inherited");
  state.pending[2].reset();
  state.granted[3] = store();
  rejected([&] { const SharcMemoryBus bus(memory, state); }, "inherited");
  state.granted[3].reset();
  state.granted[0] = store();
  state.granted[2].reset();
  rejected([&] { const SharcMemoryBus bus(memory, state); }, "inherited");
  state = {.clocks = 1, .next_phase = SharcPeripheralPhase::gap};
  state.pending[0] = store();
  state.granted[2] = store();
  rejected([&] { const SharcMemoryBus bus(memory, state); }, "priority");
  state = {.clocks = 1};
  state.granted[1] = store();
  rejected([&] { const SharcMemoryBus bus(memory, state); }, "inherited");
  state = {.clocks = std::numeric_limits<std::uint64_t>::max()};
  SharcMemoryBus exhausted(memory, state);
  rejected([&] { exhausted.advance(); }, "overflow");
  require(exhausted.state() == state);
}

}

int main() try {
  view_geometry();
  all_request_sets();
  carried_io_and_core_gaps();
  independent_blocks();
  actual_fetch_after_dma();
  actual_width_accesses();
  access_failure_and_repair();
  permissions();
  malformed_requests();
  invalid_snapshots();
  std::cout << "SHARC internal RAM priorities, carried IO1, aliases, actual accesses and snapshots "
               "passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
