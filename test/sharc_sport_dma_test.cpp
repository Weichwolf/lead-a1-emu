// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_power.hpp"
#include "sharc/peripherals/sharc_sport_dma.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

using State = lead_a1::SharcSportDmaState;
using Mode = lead_a1::SharcSportDmaMode;
using Dma = lead_a1::SharcSportDma;
using Master = lead_a1::SharcMemoryMaster;
using Width = lead_a1::SharcWidth;
constexpr std::uint32_t first = 0x80020;
constexpr std::uint32_t second = 0x80021;
constexpr std::uint32_t tcb_index = 0x80053;
constexpr unsigned int chain_cycles = 26;
constexpr std::uint32_t first_value = 0x89abcdef;
constexpr std::uint32_t second_value = 0x12345678;
constexpr Mode receiver{.enabled = true};
constexpr Mode transmitter{.enabled = true, .transmit = true};
constexpr Mode chained_receiver{.enabled = true, .chained = true};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SPORT DMA expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing SPORT DMA rejection: " + reason);
}

template <typename Value> Value must(std::optional<Value> value) {
  if (!value) { throw std::runtime_error("Missing owned SPORT DMA test input"); }
  return *value;
}

struct Fixture {
  lead_a1::SharcPower power;
  lead_a1::SharcMemory memory;
  lead_a1::SharcMemoryBus bus{memory, lead_a1::SharcPeripheralPhase::active};
  Dma dma{power, memory, 0};

  Fixture() {
    constexpr unsigned int cells = 256;
    memory.map({.bank = 0, .halfwords = cells, .name = "owned SPORT DMA RAM"});
  }

  void tick() {
    bus.advance();
    dma.advance_cclk(1);
  }

  void transfer(Dma &channel) {
    const auto request = channel.request();
    require(request.has_value());
    bus.request(Master::io0, must(request));
    while (!bus.grant(Master::io0)) {
      bus.advance();
      channel.advance_cclk(1);
    }
    channel.service(bus);
  }

  void transfer() { transfer(dma); }

  void parameters(std::uint16_t count = 2) {
    dma.write({.size = 4}, first);
    dma.write({.address = 4, .size = 4}, 1);
    dma.write({.address = 8, .size = 4}, count);
    dma.write({.address = 12, .size = 4}, 0);
  }

  void word(std::uint32_t address, std::uint32_t value) {
    memory.write({.address = address, .width = Width::normal32}, value);
  }

  void descriptor(std::uint32_t next = 0) {
    // HWR3-34/35: CP points to II, the highest address; reads descend.
    word(tcb_index, first);
    word(tcb_index - 1, 1);
    word(tcb_index - 2, 2);
    word(tcb_index - 3, next);
  }
};

void unknown_parameters_and_transaction_ownership() {
  Fixture fixture;
  const auto before = fixture.dma.snapshot();
  for (unsigned int index = 0; index < 4; ++index) {
    rejected([&] { static_cast<void>(fixture.dma.read({.address = index * 4, .size = 4})); },
             "Unknown");
  }
  rejected([&] { fixture.dma.configure(receiver); }, "Unknown");
  require(fixture.dma.snapshot() == before && !fixture.dma.interrupt_request());
  fixture.parameters();
  const auto configured = fixture.dma.snapshot();
  fixture.dma.begin_instruction();
  fixture.dma.configure(receiver);
  require(fixture.dma.state().active && !fixture.dma.locks_iop_buses());
  rejected([&] { static_cast<void>(fixture.dma.snapshot()); }, "active instruction");
  rejected([&] { static_cast<void>(fixture.dma.request()); }, "active instruction");
  rejected([&] { fixture.dma.advance_cclk(0); }, "active instruction");
  rejected([&] { fixture.dma.receive_word(0); }, "active instruction");
  rejected([&] { fixture.dma.acknowledge_interrupt(); }, "active instruction");
  rejected([&] { fixture.dma.begin_instruction(); }, "Nested");
  fixture.dma.abort_instruction();
  require(fixture.dma.snapshot() == configured);
  fixture.dma.begin_instruction();
  fixture.dma.configure(receiver);
  fixture.dma.commit_instruction();
  require(fixture.dma.snapshot().active && !fixture.dma.request());
}

void receive_words_and_live_count_abort() {
  Fixture fixture;
  fixture.parameters(5);
  fixture.dma.configure(receiver);
  require(!fixture.dma.request() && !fixture.dma.interrupt_request());
  fixture.dma.receive_word(first_value);
  const auto before = fixture.dma.snapshot();
  rejected([&] { fixture.dma.receive_word(0); }, "empty receive buffer");
  rejected([&] { fixture.dma.write({.size = 4}, second); }, "live index/modifier");
  require(fixture.dma.snapshot() == before);
  const auto request = fixture.dma.request();
  require(request ==
          (lead_a1::SharcMemoryRequest{.access = {.address = first, .width = Width::normal32},
                                       .kind = lead_a1::AccessKind::write,
                                       .value = first_value}));
  fixture.transfer();
  require(fixture.memory.read({.address = first, .width = Width::normal32}) == first_value &&
          fixture.dma.snapshot().index == second && fixture.dma.snapshot().remaining == 4 &&
          !fixture.dma.interrupt_request());
  fixture.dma.write({.address = 8, .size = 4}, 1); // One more actual word, not an immediate IRQ.
  fixture.dma.receive_word(second_value);
  fixture.transfer();
  require(fixture.memory.read({.address = second, .width = Width::normal32}) == second_value &&
          fixture.dma.snapshot().count == 0 && !fixture.dma.snapshot().active &&
          fixture.dma.interrupt_request());
  fixture.dma.acknowledge_interrupt();
  fixture.dma.configure(receiver); // Repeating the same control does not restart a completed DMA.
  require(!fixture.dma.snapshot().active && !fixture.dma.request() &&
          !fixture.dma.interrupt_request());
}

void transmit_buffer_shifter_and_completion() {
  for (const bool external : {false, true}) {
    Fixture fixture;
    fixture.parameters();
    fixture.word(first, first_value);
    fixture.word(second, second_value);
    fixture.dma.configure({.enabled = true, .transmit = true, .external_completion = external});
    fixture.transfer();
    require(!fixture.dma.request() && !fixture.dma.interrupt_request());
    require(fixture.dma.start_transmit() == first_value);
    fixture.transfer();
    require(!fixture.dma.snapshot().active && fixture.dma.snapshot().transmit_buffer &&
            fixture.dma.snapshot().transmit_shift && fixture.dma.interrupt_request() == !external);
    const auto full = fixture.dma.snapshot();
    rejected([&] { static_cast<void>(fixture.dma.start_transmit()); }, "idle shift register");
    require(fixture.dma.snapshot() == full);
    fixture.dma.acknowledge_interrupt();
    fixture.dma.finish_transmit();
    require(!fixture.dma.interrupt_request());
    require(fixture.dma.start_transmit() == second_value);
    require(!fixture.dma.interrupt_request());
    fixture.dma.finish_transmit();
    require(fixture.dma.interrupt_request() == external);
    rejected([&] { fixture.dma.finish_transmit(); }, "no transmitted word");
  }
}

void zero_count_and_signed_stepping() {
  Fixture fixture;
  fixture.parameters(0);
  fixture.dma.write({.address = 4, .size = 4}, 0);
  fixture.dma.configure(receiver);
  constexpr unsigned int transfers = 65536;
  for (unsigned int index = 0; index < transfers; ++index) {
    require(fixture.dma.snapshot().remaining == transfers - index &&
            !fixture.dma.interrupt_request());
    fixture.dma.receive_word(index);
    fixture.transfer();
  }
  require(fixture.dma.snapshot().index == first && fixture.dma.snapshot().count == 0 &&
          fixture.dma.interrupt_request());
  constexpr std::uint32_t final_value = 65535;
  require(fixture.memory.read({.address = first, .width = Width::normal32}) == final_value);
  Fixture backwards;
  backwards.parameters();
  backwards.dma.write({.size = 4}, second);
  constexpr std::uint32_t minus_one = 0xffff;
  backwards.dma.write({.address = 4, .size = 4}, minus_one);
  backwards.dma.configure(receiver);
  backwards.dma.receive_word(first_value);
  backwards.transfer();
  require(backwards.dma.snapshot().index == first);
  backwards.dma.receive_word(second_value);
  backwards.transfer();
  require(backwards.memory.read({.address = first, .width = Width::normal32}) == second_value);
}

struct StrideCase {
  std::uint16_t modifier;
  std::array<std::uint32_t, 6> addresses;
};

void counted_stride(const StrideCase &choice, unsigned int count, Mode mode) {
  Fixture fixture;
  fixture.parameters(static_cast<std::uint16_t>(count));
  fixture.dma.write({.address = 4, .size = 4}, choice.modifier);
  const bool repeated = choice.modifier == 0;
  for (unsigned int index = 0; index < count; ++index) {
    fixture.word(choice.addresses[index], first_value + (repeated ? 0 : index));
  }
  fixture.dma.configure(mode);
  for (unsigned int index = 0; index < count; ++index) {
    const auto value = first_value + (repeated ? 0 : index);
    if (!mode.transmit) { fixture.dma.receive_word(value); }
    const auto request = must(fixture.dma.request());
    require(request.access.address == choice.addresses[index] &&
            request.kind ==
                (mode.transmit ? lead_a1::AccessKind::read : lead_a1::AccessKind::write));
    fixture.transfer();
    require(fixture.dma.snapshot().index == choice.addresses[index + 1] &&
            fixture.dma.snapshot().remaining == count - index - 1 &&
            fixture.dma.interrupt_request() == (index + 1 == count));
    if (mode.transmit) {
      require(fixture.dma.start_transmit() == value);
      fixture.dma.finish_transmit();
    }
    require(fixture.memory.read({.address = choice.addresses[index], .width = Width::normal32}) ==
            value);
  }
  require(fixture.dma.read({.address = 8, .size = 4}) == 0 && !fixture.dma.snapshot().active);
}

void literal_count_stride_direction_matrix() {
  constexpr std::array<StrideCase, 5> choices{{
      {.modifier = 0xfffe, .addresses = {0x80020, 0x8001e, 0x8001c, 0x8001a, 0x80018, 0x80016}},
      {.modifier = 0xffff, .addresses = {0x80020, 0x8001f, 0x8001e, 0x8001d, 0x8001c, 0x8001b}},
      {.modifier = 0, .addresses = {0x80020, 0x80020, 0x80020, 0x80020, 0x80020, 0x80020}},
      {.modifier = 1, .addresses = {0x80020, 0x80021, 0x80022, 0x80023, 0x80024, 0x80025}},
      {.modifier = 2, .addresses = {0x80020, 0x80022, 0x80024, 0x80026, 0x80028, 0x8002a}},
  }};
  unsigned int cases{};
  for (const auto &choice : choices) {
    for (unsigned int count = 1; count <= 5; ++count) {
      for (const auto mode : {receiver, transmitter}) {
        counted_stride(choice, count, mode);
        ++cases;
      }
    }
  }
  constexpr unsigned int expected = 50;
  require(cases == expected);
}

void descending_chain_reads_and_elapsed_time() {
  Fixture fixture;
  fixture.descriptor();
  fixture.dma.write({.address = 12, .size = 4}, tcb_index); // PCI plus internal low19 address.
  fixture.dma.configure(chained_receiver);
  require(fixture.dma.locks_iop_buses() && !fixture.dma.snapshot().active);
  for (unsigned int index = 0; index < 4; ++index) {
    const auto request = fixture.dma.request();
    require(request && request->access.address == tcb_index - index);
    fixture.transfer();
  }
  constexpr unsigned int four_read_clocks = 7;
  require(must(fixture.dma.snapshot().load).elapsed_cclk == four_read_clocks &&
          must(fixture.dma.snapshot().load).received == 4 && !fixture.dma.snapshot().index &&
          fixture.dma.locks_iop_buses() && !fixture.dma.request());
  for (unsigned int tick = four_read_clocks; tick < chain_cycles - 1; ++tick) { fixture.tick(); }
  require(!fixture.dma.snapshot().active && fixture.dma.locks_iop_buses());
  fixture.tick();
  const auto loaded = fixture.dma.snapshot();
  require(loaded.active && !fixture.dma.locks_iop_buses() && loaded.index == first &&
          loaded.modifier == 1 && loaded.count == 2 && loaded.chain == 0);
  fixture.dma.receive_word(first_value);
  fixture.transfer();
  fixture.dma.receive_word(second_value);
  fixture.transfer();
  require(fixture.dma.interrupt_request() && !fixture.dma.snapshot().active);
  Fixture slow;
  slow.descriptor();
  slow.dma.write({.address = 12, .size = 4}, tcb_index);
  slow.dma.configure(chained_receiver);
  slow.dma.advance_cclk(std::numeric_limits<std::uint32_t>::max());
  require(slow.dma.locks_iop_buses() && !slow.dma.snapshot().active);
  for (unsigned int index = 0; index < 4; ++index) { slow.transfer(); }
  require(slow.dma.snapshot().active && !slow.dma.locks_iop_buses());
}

void self_chains_and_pci() {
  for (const bool pci : {false, true}) {
    Fixture fixture;
    constexpr std::uint32_t without_pci = 0x53;
    const auto next = pci ? tcb_index : without_pci;
    fixture.descriptor(next);
    fixture.dma.write({.address = 12, .size = 4}, tcb_index);
    fixture.dma.configure(chained_receiver);
    for (unsigned int block = 0; block < 2; ++block) {
      for (unsigned int index = 0; index < 4; ++index) { fixture.transfer(); }
      fixture.dma.advance_cclk(chain_cycles);
      fixture.dma.receive_word(first_value);
      fixture.transfer();
      fixture.dma.receive_word(second_value);
      fixture.transfer();
      require(fixture.dma.locks_iop_buses() && fixture.dma.interrupt_request() == pci);
      fixture.dma.acknowledge_interrupt();
    }
  }
}

void failed_grant_and_repair() {
  Fixture fixture;
  fixture.parameters(1);
  fixture.dma.configure(transmitter);
  const auto request = fixture.dma.request();
  require(request.has_value());
  fixture.bus.request(Master::io0, must(request));
  fixture.tick();
  const auto before = fixture.dma.snapshot();
  const auto bus = fixture.bus.state();
  rejected([&] { fixture.dma.service(fixture.bus); }, "Unknown");
  require(fixture.dma.snapshot() == before && fixture.bus.state() == bus);
  fixture.word(first, first_value);
  fixture.dma.service(fixture.bus);
  require(fixture.dma.snapshot().remaining == 0 && fixture.dma.start_transmit() == first_value);
  Fixture foreign;
  fixture.dma.finish_transmit();
  fixture.dma.configure({.transmit = true});
  fixture.dma.configure(transmitter);
  const auto wanted = fixture.dma.request();
  require(wanted.has_value());
  foreign.bus.request(Master::io0, must(wanted));
  foreign.tick();
  const auto foreign_before = foreign.bus.state();
  rejected([&] { fixture.dma.service(foreign.bus); }, "exact bound IOD0");
  require(foreign.bus.state() == foreign_before && fixture.dma.request() == wanted);
}

void restore_chain_capture(unsigned int captured, const State &saved) {
  Fixture rebound;
  rebound.word(tcb_index, second);
  rebound.word(tcb_index - 1, 0);
  rebound.word(tcb_index - 2, 1);
  constexpr std::uint32_t pci_only = 0x80000;
  rebound.word(tcb_index - 3, pci_only); // End address0, PCI set.
  Dma restored(rebound.power, rebound.memory, 0, saved);
  for (unsigned int word = captured; word < 4; ++word) { rebound.transfer(restored); }
  restored.advance_cclk(chain_cycles);
  const auto loaded = restored.snapshot();
  const auto expected_index = captured >= 1 ? first : second;
  const unsigned int expected_modifier = captured >= 2 ? 1U : 0U;
  const unsigned int expected_count = captured >= 3 ? 2U : 1U;
  require(loaded.active && !loaded.load && loaded.index == expected_index &&
          loaded.modifier == expected_modifier && loaded.count == expected_count &&
          loaded.chain == (captured == 4 ? 0 : pci_only));
  for (unsigned int index = 0; index < expected_count; ++index) {
    restored.receive_word(first_value + index);
    rebound.transfer(restored);
  }
  require(restored.interrupt_request() && !restored.snapshot().active &&
          restored.snapshot().index == expected_index + (expected_modifier * expected_count));
  require(rebound.memory.read({.address = expected_index, .width = Width::normal32}) ==
          first_value + (expected_modifier == 0 ? expected_count - 1 : 0));
}

void separately_restored_chain_captures() {
  unsigned int cases{};
  for (unsigned int captured = 0; captured <= 4; ++captured) {
    for (const unsigned int age : {0U, 25U, 26U}) {
      if (captured == 4 && age == chain_cycles) { continue; }
      Fixture original;
      original.descriptor();
      original.dma.write({.address = 12, .size = 4}, tcb_index);
      original.dma.configure(chained_receiver);
      for (unsigned int word = 0; word < captured; ++word) { original.transfer(); }
      const auto actual_age = must(original.dma.snapshot().load).elapsed_cclk;
      if (age > actual_age) { original.dma.advance_cclk(age - actual_age); }
      restore_chain_capture(captured, original.dma.snapshot());
      ++cases;
    }
  }
  constexpr unsigned int expected_cases = 14;
  require(cases == expected_cases);
}

void failed_partial_chain_grant_restoration() {
  for (unsigned int captured = 0; captured < 4; ++captured) {
    Fixture original;
    // The next uncaptured word is genuinely absent RAM, not a fabricated zero.
    constexpr std::array values{first, 1U, 2U, 0U};
    for (unsigned int index = 0; index < captured; ++index) {
      original.word(tcb_index - index, values[index]);
    }
    original.dma.write({.address = 12, .size = 4}, tcb_index);
    original.dma.configure(chained_receiver);
    for (unsigned int index = 0; index < captured; ++index) { original.transfer(); }
    const auto request = must(original.dma.request());
    original.bus.request(Master::io0, request);
    while (!original.bus.grant(Master::io0)) { original.tick(); }
    const auto state = original.dma.snapshot();
    const auto bus = original.bus.state();
    rejected([&] { original.dma.service(original.bus); }, "Unknown");
    require(original.dma.snapshot() == state && original.bus.state() == bus);
    Fixture rebound;
    rebound.descriptor();
    Dma restored(rebound.power, rebound.memory, 0, state);
    lead_a1::SharcMemoryBus restored_bus(rebound.memory, bus);
    restored.service(restored_bus); // Same captured grant, no extra clock.
    require(restored_bus.state().clocks == bus.clocks &&
            must(restored.snapshot().load).received == captured + 1);
    for (unsigned int index = captured + 1; index < 4; ++index) {
      restored_bus.request(Master::io0, must(restored.request()));
      while (!restored_bus.grant(Master::io0)) {
        restored_bus.advance();
        restored.advance_cclk(1);
      }
      restored.service(restored_bus);
    }
    restored.advance_cclk(chain_cycles);
    require(restored.snapshot().index == first && restored.snapshot().modifier == 1 &&
            restored.snapshot().count == 2 && restored.snapshot().chain == 0 &&
            restored.snapshot().active);
  }
}

void insertion_pause_and_distinct_completion() {
  Fixture fixture;
  constexpr std::uint32_t next_chain = 0x53;
  fixture.descriptor(next_chain);
  fixture.dma.write({.address = 12, .size = 4}, tcb_index);
  fixture.dma.configure(chained_receiver);
  for (unsigned int index = 0; index < 4; ++index) { fixture.transfer(); }
  fixture.dma.advance_cclk(chain_cycles);
  fixture.dma.configure({.chained = true});
  require(fixture.dma.snapshot().insertion && fixture.dma.snapshot().active);
  fixture.dma.receive_word(first_value);
  fixture.transfer();
  fixture.dma.receive_word(second_value);
  fixture.transfer();
  require(fixture.dma.interrupt_request() && !fixture.dma.snapshot().active &&
          !fixture.dma.locks_iop_buses() && fixture.dma.snapshot().chain == next_chain);
  fixture.dma.configure(chained_receiver);
  require(fixture.dma.locks_iop_buses());
  Fixture paused;
  paused.parameters();
  paused.dma.configure(receiver);
  paused.dma.receive_word(first_value);
  paused.transfer();
  paused.dma.configure({});
  paused.dma.receive_word(second_value);
  require(!paused.dma.request() && !paused.dma.interrupt_request());
  paused.dma.configure(receiver);
  paused.transfer();
  require(paused.memory.read({.address = second, .width = Width::normal32}) == second_value &&
          paused.dma.interrupt_request());
}

void chained_internal_and_wire_completion() {
  for (const bool pci : {false, true}) {
    Fixture fixture;
    constexpr std::uint32_t next_descriptor = 0x80063;
    constexpr std::uint32_t no_pci = 0x63;
    fixture.word(tcb_index, first);
    fixture.word(tcb_index - 1, 1);
    fixture.word(tcb_index - 2, 1);
    fixture.word(tcb_index - 3, pci ? next_descriptor : no_pci);
    fixture.word(next_descriptor, second);
    fixture.word(next_descriptor - 1, 1);
    fixture.word(next_descriptor - 2, 1);
    fixture.word(next_descriptor - 3, 0);
    fixture.word(first, first_value);
    fixture.word(second, second_value);
    fixture.dma.write({.address = 12, .size = 4}, tcb_index);
    fixture.dma.configure(
        {.enabled = true, .chained = true, .transmit = true, .external_completion = true});
    for (unsigned int index = 0; index < 4; ++index) { fixture.transfer(); }
    fixture.dma.advance_cclk(chain_cycles);
    fixture.transfer();
    require(fixture.dma.locks_iop_buses() && fixture.dma.interrupt_request() == pci);
    fixture.dma.acknowledge_interrupt();
    require(fixture.dma.start_transmit() == first_value);
    for (unsigned int index = 0; index < 4; ++index) { fixture.transfer(); }
    fixture.dma.advance_cclk(chain_cycles);
    fixture.transfer();
    require(!fixture.dma.snapshot().active && !fixture.dma.interrupt_request());
    fixture.dma.finish_transmit();
    require(!fixture.dma.interrupt_request());
    require(fixture.dma.start_transmit() == second_value);
    require(!fixture.dma.interrupt_request());
    fixture.dma.finish_transmit();
    require(fixture.dma.interrupt_request());
  }
}

void address_widths_and_external_guards() {
  Fixture fixture;
  constexpr std::uint32_t raw_index = 0xf0080020;
  constexpr std::uint32_t raw_modifier = 0xabcdffff;
  constexpr std::uint32_t raw_count = 0x98760001;
  fixture.dma.write({.size = 4}, raw_index);
  fixture.dma.write({.address = 4, .size = 4}, raw_modifier);
  fixture.dma.write({.address = 8, .size = 4}, raw_count);
  constexpr std::uint32_t modifier = 0xffff;
  require(fixture.dma.read({.size = 4}) == first &&
          fixture.dma.read({.address = 4, .size = 4}) == modifier &&
          fixture.dma.read({.address = 8, .size = 4}) == 1);
  constexpr std::uint32_t external_chain = 0x00480053;
  constexpr std::uint32_t decoded_external = 0x00200053;
  fixture.dma.write({.address = 12, .size = 4}, external_chain);
  fixture.dma.configure(chained_receiver);
  const auto state = fixture.dma.snapshot();
  const auto request = must(fixture.dma.request());
  require(request.access.address == decoded_external);
  rejected([&] { fixture.bus.request(Master::io0, request); }, "SHARC");
  require(fixture.dma.snapshot() == state && fixture.dma.request() == request);
  Fixture no_offset;
  no_offset.parameters(1);
  constexpr std::uint32_t low_address = 0x20;
  no_offset.dma.write({.size = 4}, low_address);
  no_offset.dma.configure(transmitter);
  const auto low_before = no_offset.dma.snapshot();
  rejected([&] { static_cast<void>(no_offset.dma.request()); }, "low index interpretation");
  require(no_offset.dma.snapshot() == low_before && no_offset.dma.snapshot().index == low_address);
  Fixture low_receive;
  low_receive.parameters(1);
  low_receive.dma.write({.size = 4}, low_address);
  low_receive.dma.configure(receiver);
  require(!low_receive.dma.request()); // An empty RX buffer requests no memory access.
  low_receive.dma.receive_word(first_value);
  const auto captured = low_receive.dma.snapshot();
  rejected([&] { static_cast<void>(low_receive.dma.request()); }, "low index interpretation");
  require(low_receive.dma.snapshot() == captured);
}

void malformed_registers_states_and_power() {
  Fixture fixture;
  fixture.parameters();
  const auto before = fixture.dma.snapshot();
  constexpr std::array<lead_a1::MemoryAccess, 4> bad{
      {{.size = 2}, {.size = 8}, {.address = 1, .size = 4}, {.address = 16, .size = 4}}};
  for (const auto access : bad) {
    rejected([&] { fixture.dma.write(access, 0); }, "aligned32-bit");
    rejected([&] { static_cast<void>(fixture.dma.read(access)); }, "aligned32-bit");
  }
  rejected([&] { fixture.dma.write({.size = 4}, std::uint64_t{1} << 32); }, "Oversized");
  require(fixture.dma.snapshot() == before);
  for (unsigned int sport = 0; sport < 8; ++sport) {
    Dma channel(fixture.power, fixture.memory, sport, before);
    constexpr std::array<std::uint32_t, 4> gates{0x40, 0x80, 0x100, 0x200};
    fixture.power.write({.size = 4}, gates[sport / 2]);
    fixture.power.advance_pclk(2);
    rejected([&] { channel.configure(receiver); }, "stopped peripheral clock");
    rejected([&] { static_cast<void>(channel.request()); }, "stopped peripheral clock");
    require(channel.snapshot() == before);
    fixture.power.write({.size = 4}, 0);
    fixture.power.advance_pclk(2);
    channel.configure(receiver);
    require(channel.snapshot().active);
  }
  rejected([&] { const Dma invalid(fixture.power, fixture.memory, 8); }, "port identity");
  auto invalid = before;
  invalid.active = true;
  rejected([&] { const Dma broken(fixture.power, fixture.memory, 0, invalid); }, "active SPORT");
  invalid = before;
  invalid.mode = chained_receiver;
  invalid.chain = tcb_index;
  invalid.load = lead_a1::SharcSportDmaLoad{.index_address = tcb_index, .received = 5};
  rejected([&] { const Dma broken(fixture.power, fixture.memory, 0, invalid); }, "loading state");
  invalid.load->received = 0;
  invalid.load->words[1] = 1;
  rejected([&] { const Dma broken(fixture.power, fixture.memory, 0, invalid); }, "uncaptured");
}

}

int main() try {
  unknown_parameters_and_transaction_ownership();
  receive_words_and_live_count_abort();
  transmit_buffer_shifter_and_completion();
  zero_count_and_signed_stepping();
  literal_count_stride_direction_matrix();
  descending_chain_reads_and_elapsed_time();
  self_chains_and_pci();
  failed_grant_and_repair();
  separately_restored_chain_captures();
  failed_partial_chain_grant_restoration();
  insertion_pause_and_distinct_completion();
  address_widths_and_external_guards();
  chained_internal_and_wire_completion();
  malformed_registers_states_and_power();
  std::cout << "SPORT DMA actual RAM/chain/buffer/completion/ownership expectations pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
