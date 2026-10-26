// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/visa/sharc_visa.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t ivt = 0xc0000;
constexpr std::uint32_t handler = 0x180180;
constexpr std::uint32_t native_foreground = 0xc00e0;
constexpr std::uint32_t visa_foreground = 0x180400;
constexpr std::uint32_t parent_frame = 0x10c00a0;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t jump = 0x063e00000000;
constexpr std::uint64_t rti = 0x0b3e00000000;
constexpr std::uint16_t compact_rti = 0x0bbe;
constexpr std::uint16_t increment_r0 = 0xc500;
constexpr std::uint16_t increment_r1 = 0xc511;
constexpr std::uint64_t native_increment_r1 = 0x013e00029110;
constexpr std::uint64_t raw_compact_rti = 0x0bbe00000000;
constexpr std::uint64_t raw_conditional_rti = 0x0b8000000000;
constexpr std::uint64_t raw_increment_r2 = 0xc52200000000;
constexpr std::uint16_t native_rti_prefix = 0x0b3e;
constexpr std::uint16_t invalid_short = 0xffff;
constexpr std::uint64_t initial_r0 = 0x700;
constexpr std::uint64_t initial_r1 = 0xb00;
constexpr std::uint64_t resumed_r1 = 0xc00;
constexpr unsigned int warmup = 4;
constexpr unsigned int first_execute = 5;
constexpr unsigned int clock_limit = 32;

struct Route {
  unsigned int priority;
  std::uint32_t vector;
  std::uint32_t source;
  std::uint32_t primary_mask;
  std::uint32_t secondary_mask;
  std::uint32_t primary_pointer;
  std::uint32_t secondary_pointer;
  lead_a1::SharcInterruptBank bank;
};

constexpr std::array routes{Route{.priority = 1,
                                  .vector = 0xc0030,
                                  .source = 0x1000,
                                  .primary_mask = 0x1003,
                                  .secondary_mask = 0,
                                  .primary_pointer = 0x1000,
                                  .secondary_pointer = 0,
                                  .bank = lead_a1::SharcInterruptBank::irptl},
                            Route{.priority = 6,
                                  .vector = 0xc0044,
                                  .source = 1,
                                  .primary_mask = 3,
                                  .secondary_mask = 0x400,
                                  .primary_pointer = 0,
                                  .secondary_pointer = 0x100000,
                                  .bank = lead_a1::SharcInterruptBank::lirptl}};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("VISA interrupt expectation at line " + std::to_string(where.line()));
  }
}

template <typename Optional> auto &present(Optional &value) {
  if (!value) { throw std::runtime_error("Missing owned VISA interrupt input"); }
  return *value;
}

template <typename Action> void rejected(const Action &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected " + reason + ": " + error.what());
  }
  throw std::runtime_error("Missing VISA interrupt rejection: " + reason);
}

lead_a1::SharcState initial(Route route, std::uint32_t pc) {
  auto state = lead_a1::Sharc::reset_reference_214xx(pc);
  state.system[2].value = 0x1000;
  state.system[11].value = route.primary_mask;
  state.system[13].value = route.secondary_mask;
  state.effective = state.system;
  state.depth = 1;
  state.pc_stack[0] = parent_frame;
  for (auto &reg : state.data[0]) { reg = {.known = full40}; }
  state.data[0][0].value = initial_r0;
  state.data[0][1].value = initial_r1;
  return state;
}

lead_a1::SharcFetchState sequenced(std::uint32_t pc) {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = pc};
  return state;
}

struct Fixture {
  lead_a1::SharcMemory memory;
  lead_a1::Sharc cpu;
  lead_a1::SharcFetch fetch;
  std::optional<lead_a1::SharcVisaBuffer> input;
  unsigned int clocks{};

  explicit Fixture(Route route, std::uint32_t pc = visa_foreground, bool full_return = false)
      : cpu(memory, initial(route, pc)), fetch(memory, sequenced(pc)) {
    constexpr unsigned int cells = 0x800;
    constexpr std::uint32_t short_base = 0x180000;
    memory.map({.bank = 2, .halfwords = cells, .name = "owned ISA/VISA IRQ program"});
    for (unsigned int cell = 0; cell < cells; ++cell) { short_code(short_base + cell, 1); }
    for (unsigned int index = 0; index < 12; ++index) {
      native_code(route.vector + index, 0);
      native_code(native_foreground + index, 0);
    }
    native_code(route.vector, jump | handler);
    short_code(handler, increment_r0);
    if (full_return) {
      short_code(handler + 1, native_rti_prefix);
      short_code(handler + 2, 0);
      short_code(handler + 3, 0);
    } else {
      short_code(handler + 1, compact_rti);
    }
    if (pc >= short_base) {
      short_code(pc, increment_r1);
    } else {
      native_code(pc, native_increment_r1);
    }
  }

  void short_code(std::uint32_t pc, std::uint16_t value) {
    memory.write({.address = pc, .width = lead_a1::SharcWidth::short16}, value);
  }

  void native_code(std::uint32_t pc, std::uint64_t value) {
    memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, value);
  }

  void ready() {
    if (!input || input->state().decode != cpu.state().pc) {
      input.emplace(memory,
                    lead_a1::SharcVisaState{.decode = cpu.state().pc, .fetch = cpu.state().pc});
    }
    for (unsigned int event = 0; event < 2 && !input->instruction(); ++event) {
      if (input->can_capture()) { input->capture(); }
      static_cast<void>(input->advance(false));
    }
    require(input->instruction().has_value());
  }

  lead_a1::SharcStep retire(bool clocked) {
    if (!clocked) {
      if (cpu.state().pc < 0x100000) { return cpu.step(); }
      ready();
      return cpu.step(present(input));
    }
    for (unsigned int clock = 0; clock < clock_limit; ++clock) {
      static_cast<void>(fetch.advance_sequenced(cpu));
      ++clocks;
      if (fetch.state().execute) { return cpu.step(fetch); }
    }
    throw std::runtime_error("Missing clocked VISA interrupt retirement");
  }

  lead_a1::SharcInterruptStep accept(Route route, bool clocked) {
    if (clocked) {
      for (unsigned int clock = 0; clock < warmup; ++clock) {
        static_cast<void>(fetch.advance_sequenced(cpu));
      }
      require(fetch.state().address && !fetch.state().execute);
    }
    cpu.latch_programmable_interrupt(route.priority);
    const auto event = clocked ? cpu.service_interrupt(fetch, {.vector_base = ivt})
                               : cpu.service_interrupt({.vector_base = ivt});
    if (!event) { throw std::runtime_error("Missing owned programmable interrupt entry"); }
    return *event;
  }
};

void roundtrip(Route route, bool short_foreground, bool full_return, bool clocked) {
  const auto pc = short_foreground ? visa_foreground : native_foreground;
  const auto frame = short_foreground ? 0x2180400U : 0x20c00e0U;
  Fixture fixture(route, pc, full_return);
  const auto before = fixture.cpu.state();
  const auto entry = fixture.accept(route, clocked);
  require(entry == (lead_a1::SharcInterruptStep{.source = route.source,
                                                .return_pc = pc,
                                                .vector = route.vector,
                                                .minimum_core_cycles = 4,
                                                .bank = route.bank}));
  require(fixture.cpu.state().pc == route.vector && fixture.cpu.state().depth == 2 &&
          fixture.cpu.state().pc_stack[0] == parent_frame &&
          fixture.cpu.state().pc_stack[1] == frame && fixture.cpu.state().data == before.data &&
          fixture.cpu.state().interrupt &&
          fixture.cpu.state().system[12] ==
              (lead_a1::SharcKnown32{.value = route.primary_pointer, .known = full32}) &&
          fixture.cpu.state().system[13].value == (route.secondary_mask | route.secondary_pointer));
  if (clocked) {
    require(!fixture.fetch.state().visa && !fixture.fetch.state().decode &&
            !fixture.fetch.state().address && !fixture.fetch.state().fetch1 &&
            fixture.fetch.state().flow && present(fixture.fetch.state().flow).next == route.vector);
  }
  require(fixture.retire(clocked).pc == route.vector && fixture.cpu.state().pc == handler);
  if (clocked) { require(fixture.clocks == first_execute); }
  require(fixture.retire(clocked).pc == handler && fixture.cpu.state().data[0][0].value == 0x800);
  const auto result = fixture.retire(clocked);
  require(result.pc == handler + 1 && result.next_pc == pc &&
          result.address_units == (full_return ? 3U : 1U) &&
          result.opcode == (full_return ? rti : raw_compact_rti) &&
          fixture.cpu.state().depth == 1 && !fixture.cpu.state().interrupt &&
          fixture.cpu.state().pc_stack[1] == frame && fixture.cpu.state().system[12].value == 0 &&
          fixture.cpu.state().system[13].value == route.secondary_mask &&
          fixture.cpu.state().system[2] == before.system[2] &&
          fixture.cpu.state().system[11] == before.system[11]);
  require(fixture.retire(clocked).pc == pc && fixture.cpu.state().data[0][1].value == resumed_r1);
}

void compact_predicates() {
  constexpr std::uint32_t simd_and_global = 0x201000;
  constexpr std::uint16_t rti_eq = 0x0b80;
  for (const auto route : routes) {
    for (const bool x : {false, true}) {
      for (const bool y : {false, true}) {
        Fixture fixture(route);
        fixture.short_code(handler, rti_eq);
        auto state = fixture.cpu.state();
        state.system[2].value = simd_and_global;
        state.system[6].value = x ? 1U : 0U;
        state.system[7].value = y ? 1U : 0U;
        state.effective = state.system;
        lead_a1::Sharc cpu(fixture.memory, state);
        cpu.latch_programmable_interrupt(route.priority);
        require(cpu.service_interrupt({.vector_base = ivt}).has_value());
        static_cast<void>(cpu.step());
        lead_a1::SharcVisaBuffer input(fixture.memory, {.decode = handler, .fetch = handler});
        input.capture();
        static_cast<void>(input.advance(false));
        const auto result = cpu.step(input);
        require(result.opcode == raw_conditional_rti && result.address_units == 1 &&
                result.next_pc == (x && y ? visa_foreground : handler + 1) &&
                cpu.state().interrupt.has_value() == !(x && y));
        if (!x || !y) { require(cpu.step(input).next_pc == visa_foreground); }
        require(!cpu.state().interrupt && cpu.state().system[6] == state.system[6] &&
                cpu.state().system[7] == state.system[7] && cpu.state().data == state.data);
      }
    }
  }
}

void captured_return_and_sequential_flush() {
  Fixture fixture(routes[0], handler + 1);
  fixture.short_code(handler, compact_rti);
  static_cast<void>(fixture.accept(routes[0], false));
  static_cast<void>(fixture.cpu.step());
  fixture.ready();
  fixture.short_code(handler, invalid_short);
  constexpr std::uint16_t increment_r2 = 0xc522;
  fixture.short_code(handler + 1, increment_r2);
  require(fixture.cpu.step(present(fixture.input)).next_pc == handler + 1 &&
          present(fixture.input).state().count == 0 && !present(fixture.input).state().delay);
  require(fixture.retire(false).opcode == raw_increment_r2 &&
          fixture.cpu.state().data[0][1].value == initial_r1 &&
          fixture.cpu.state().data[0][2].value == 0x100);
}

void failed_returns_retain_state() {
  constexpr std::array<std::uint16_t, 2> unsupported{0x0bbf, 0x0bfe};
  for (const auto encoded : unsupported) {
    Fixture fixture(routes[0]);
    fixture.short_code(handler, encoded);
    static_cast<void>(fixture.accept(routes[0], false));
    static_cast<void>(fixture.cpu.step());
    fixture.ready();
    const auto before = fixture.cpu.state();
    const auto input = present(fixture.input).state();
    rejected([&] { static_cast<void>(fixture.cpu.step(present(fixture.input))); },
             encoded == unsupported[0] ? "loop reentry/reserved" : "delayed/parallel/reserved RTI");
    require(fixture.cpu.state() == before && present(fixture.input).state() == input &&
            !fixture.memory.active());
  }
  Fixture fixture(routes[1]);
  fixture.short_code(handler, compact_rti);
  static_cast<void>(fixture.accept(routes[1], true));
  static_cast<void>(fixture.retire(true));
  for (unsigned int clock = 0; clock < clock_limit && !fixture.fetch.state().execute; ++clock) {
    static_cast<void>(fixture.fetch.advance_sequenced(fixture.cpu));
  }
  auto transport = fixture.fetch.state();
  require(transport.execute && transport.execute->branch && transport.execute->branch->taken);
  present(present(transport.execute).branch).target = visa_foreground + 1;
  lead_a1::SharcFetch changed(fixture.memory, transport);
  const auto before = fixture.cpu.state();
  rejected([&] { static_cast<void>(fixture.cpu.step(changed)); }, "interrupt frame changed");
  require(fixture.cpu.state() == before && changed.state() == transport &&
          !fixture.memory.active());
  require(fixture.cpu.step(fixture.fetch).next_pc == visa_foreground);
}

void independent_interrupt_restores() {
  constexpr unsigned int snapshots = 10;
  for (const auto route : routes) {
    for (unsigned int checkpoint = 0; checkpoint < snapshots; ++checkpoint) {
      Fixture fixture(route);
      static_cast<void>(fixture.accept(route, true));
      for (unsigned int clock = 0; clock < checkpoint; ++clock) {
        static_cast<void>(fixture.fetch.advance_sequenced(fixture.cpu));
        if (fixture.fetch.state().execute) { static_cast<void>(fixture.cpu.step(fixture.fetch)); }
      }
      require(fixture.cpu.state().interrupt.has_value());
      Fixture separate(route);
      lead_a1::Sharc restored(separate.memory, fixture.cpu.state());
      lead_a1::SharcFetch replay(separate.memory, fixture.fetch.state());
      bool resumed{};
      for (unsigned int clock = 0; clock < clock_limit && !resumed; ++clock) {
        const auto event = fixture.fetch.advance_sequenced(fixture.cpu);
        require(replay.advance_sequenced(restored) == event);
        if (fixture.fetch.state().execute) {
          const auto expected = fixture.cpu.step(fixture.fetch);
          const auto actual = restored.step(replay);
          require(actual.pc == expected.pc && actual.opcode == expected.opcode &&
                  actual.next_pc == expected.next_pc &&
                  actual.address_units == expected.address_units);
          resumed = actual.pc == visa_foreground;
        }
        require(restored.state() == fixture.cpu.state() && replay.state() == fixture.fetch.state());
      }
      require(resumed && restored.state().pc == visa_foreground + 1 &&
              restored.state().data[0][0].value == 0x800 &&
              restored.state().data[0][1].value == resumed_r1 && restored.state().depth == 1 &&
              !restored.state().interrupt);
    }
  }
}

void native_rti_and_guarded_boundaries() {
  for (const auto route : routes) {
    for (const bool clocked : {false, true}) {
      Fixture fixture(route);
      fixture.native_code(route.vector, rti);
      static_cast<void>(fixture.accept(route, clocked));
      const auto result = fixture.retire(clocked);
      require(result.pc == route.vector && result.next_pc == visa_foreground &&
              result.opcode == rti && result.address_units == 1 &&
              fixture.cpu.state().data[0][0].value == initial_r0 &&
              fixture.cpu.state().depth == 1 && !fixture.cpu.state().interrupt);
      require(fixture.retire(clocked).pc == visa_foreground &&
              fixture.cpu.state().data[0][1].value == resumed_r1);
    }
  }
  Fixture fixture(routes[0]);
  constexpr std::uint32_t end_internal = 0x200000;
  for (const bool delayed : {false, true}) {
    auto state = fixture.cpu.state();
    if (delayed) {
      state.branch = lead_a1::SharcBranch{.target = visa_foreground + 4, .slots = 2};
    } else {
      state.pc = end_internal;
    }
    lead_a1::Sharc cpu(fixture.memory, state);
    cpu.latch_programmable_interrupt(routes[0].priority);
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(cpu.service_interrupt({.vector_base = ivt})); },
             delayed ? "delayed-branch/loop interrupt entry" : "internal native IVT/program PC");
    require(cpu.state() == before && !fixture.memory.active());
  }
  fixture.short_code(visa_foreground, compact_rti);
  fixture.ready();
  const auto before = fixture.cpu.state();
  const auto input = present(fixture.input).state();
  rejected([&] { static_cast<void>(fixture.cpu.step(present(fixture.input))); },
           "RTI requires a qualified programmable frame");
  require(fixture.cpu.state() == before && present(fixture.input).state() == input);
}

}

int main() try {
  for (const auto route : routes) {
    for (const bool short_foreground : {false, true}) {
      for (const bool full_return : {false, true}) {
        for (const bool clocked : {false, true}) {
          roundtrip(route, short_foreground, full_return, clocked);
        }
      }
    }
  }
  compact_predicates();
  captured_return_and_sequential_flush();
  failed_returns_retain_state();
  independent_interrupt_restores();
  native_rti_and_guarded_boundaries();
  std::cout << "ISA/VISA programmable IRQ frames, compact/full RTI and captured transport pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
