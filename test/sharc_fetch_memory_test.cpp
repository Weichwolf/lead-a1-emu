// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t target = base + 2;
constexpr std::uint64_t old_word = 0x013e0002a770; // R7=R7-1.
constexpr std::uint64_t new_word = 0x013e00029770; // R7=R7+1.
constexpr std::uint64_t store_px = 0x136b00000000;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t full64 = 0xffffffffffffffff;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("F2 memory response expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing F2 memory rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 512;
  constexpr unsigned int instructions = 64;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned pending F2 memory"});
  for (unsigned int index = 0; index < instructions; ++index) { code(memory, base + index, 0); }
  code(memory, base, store_px | target);
  code(memory, target, old_word);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.data[0][7] = {.value = std::uint64_t{7} << 8, .known = full40};
  state.px = {.value = new_word << 16, .known = full64};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

struct Fixture {
  lead_a1::SharcMemory memory;
  lead_a1::SharcControl control{true};
  lead_a1::Sharc cpu{memory, initial(), &control};
  lead_a1::SharcFetch fetch{memory, sequenced()};

  Fixture() { program(memory); }
};

lead_a1::SharcFetchEvent
advance(lead_a1::Sharc &cpu,
        lead_a1::SharcFetch &fetch,
        lead_a1::SharcFetchRead read = lead_a1::SharcFetchRead::immediate) {
  const auto bus = fetch.address_pm_request(cpu) ? lead_a1::SharcFetchBus::pm_data
                                                 : lead_a1::SharcFetchBus::available;
  return fetch.advance_sequenced(cpu, bus, read);
}

void issue(Fixture &fixture, lead_a1::SharcFetchRead read) {
  for (unsigned int clock = 0; clock < 3; ++clock) {
    require(advance(fixture.cpu, fixture.fetch).address_accepted);
    require(!fixture.fetch.state().execute);
  }
  require(fixture.fetch.state().fetch1 == target && fixture.fetch.address_pm_request(fixture.cpu));
  require(advance(fixture.cpu, fixture.fetch, read).address_accepted);
  require(!fixture.fetch.state().execute && fixture.cpu.state().pc == base);
  if (read == lead_a1::SharcFetchRead::deferred) {
    require(fixture.fetch.state().pending_fetch2 == target && !fixture.fetch.state().fetch2);
  } else {
    const auto word = fixture.fetch.state().fetch2;
    require(word && word->opcode == old_word);
  }
}

void store_clock(Fixture &fixture, bool retire = true) {
  const auto before = fixture.fetch.state();
  const auto event = advance(fixture.cpu, fixture.fetch);
  require(event.stalled && !event.address_accepted && !event.control_stall && !event.dag_stall);
  require(fixture.fetch.state().pending_fetch2 == before.pending_fetch2 &&
          fixture.fetch.state().fetch2 == before.fetch2 &&
          fixture.fetch.state().decode == before.decode);
  const auto execute = fixture.fetch.state().execute;
  require(execute && execute->address == base);
  if (retire) { require(fixture.cpu.step(fixture.fetch).opcode == (store_px | target)); }
}

void finish(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch, unsigned int retired = 1) {
  constexpr unsigned int limit = 12;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != target + 1; ++clock) {
    static_cast<void>(advance(cpu, fetch));
    if (fetch.state().execute) {
      require(cpu.step(fetch).pc == base + retired);
      ++retired;
    }
  }
  require(retired == 3 && cpu.state().pc == target + 1);
}

enum class ResponseOrder : std::uint8_t { immediate, early, late };

void response_order(ResponseOrder order) {
  Fixture fixture;
  issue(fixture,
        order == ResponseOrder::immediate ? lead_a1::SharcFetchRead::immediate
                                          : lead_a1::SharcFetchRead::deferred);
  if (order == ResponseOrder::early) { fixture.fetch.complete_fetch2(fixture.cpu); }
  store_clock(fixture);
  require(fixture.memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) ==
          new_word);
  const auto native = fixture.cpu.state();
  const auto transport = fixture.fetch.state();
  const auto controller = fixture.control.snapshot();
  if (order == ResponseOrder::late) {
    rejected([&] { static_cast<void>(advance(fixture.cpu, fixture.fetch)); },
             "memory response must arrive before advancing Decode");
    require(fixture.cpu.state() == native && fixture.fetch.state() == transport &&
            fixture.control.snapshot() == controller);
    fixture.fetch.complete_fetch2(fixture.cpu);
    require(fixture.cpu.state() == native && fixture.control.snapshot() == controller);
    const auto word = fixture.fetch.state().fetch2;
    require(!fixture.fetch.state().pending_fetch2 && word && word->opcode == new_word);
  } else {
    rejected([&] { fixture.fetch.complete_fetch2(fixture.cpu); },
             "requires a pending uncached RAM response");
    require(fixture.cpu.state() == native && fixture.fetch.state() == transport);
    const auto word = fixture.fetch.state().fetch2;
    require(word && word->opcode == old_word);
  }
  finish(fixture.cpu, fixture.fetch);
  const auto expected = order == ResponseOrder::late ? 8U : 6U;
  require(fixture.cpu.state().data[0][7] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{expected} << 8, .known = full40}));
}

void pending_restore(bool occupied) {
  Fixture fixture;
  issue(fixture, lead_a1::SharcFetchRead::deferred);
  store_clock(fixture, !occupied);
  lead_a1::SharcMemory other_memory;
  program(other_memory);
  if (!occupied) {
    lead_a1::Sharc replay(other_memory, initial());
    require(replay.step().opcode ==
            (store_px | target)); // Independent actual store, not a RAM copy.
  }
  auto other_control = lead_a1::SharcControl::from_snapshot(fixture.control.snapshot());
  lead_a1::Sharc other(other_memory, fixture.cpu.state(), &other_control);
  lead_a1::SharcFetch transport(other_memory, fixture.fetch.state());
  if (occupied) {
    require(fixture.cpu.step(fixture.fetch).opcode == (store_px | target));
    require(other.step(transport).opcode == (store_px | target));
  }
  fixture.fetch.complete_fetch2(fixture.cpu);
  transport.complete_fetch2(other);
  finish(fixture.cpu, fixture.fetch);
  finish(other, transport);
  require(fixture.cpu.state() == other.state() && fixture.fetch.state() == transport.state());
  require(fixture.cpu.state().data[0][7] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{8} << 8, .known = full40}));
  require(other_memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) == new_word);
}

void response_ownership() {
  Fixture fixture;
  Fixture wrong;
  issue(fixture, lead_a1::SharcFetchRead::deferred);
  const auto before = fixture.fetch.state();
  const auto native = fixture.cpu.state();
  rejected([&] { fixture.fetch.complete_fetch2(wrong.cpu); }, "idle bound memory");
  fixture.memory.begin_instruction();
  rejected([&] { fixture.fetch.complete_fetch2(fixture.cpu); }, "idle bound memory");
  require(fixture.memory.active() && fixture.fetch.state() == before);
  fixture.memory.abort_instruction();
  fixture.control.begin_instruction();
  rejected([&] { fixture.fetch.complete_fetch2(fixture.cpu); }, "snapshot during a transaction");
  fixture.control.abort_instruction();
  require(fixture.fetch.state() == before && fixture.cpu.state() == native);
  fixture.fetch.complete_fetch2(fixture.cpu);
  const auto word = fixture.fetch.state().fetch2;
  require(word && word->opcode == old_word);
}

void failed_response_and_malformed_state() {
  Fixture fixture;
  constexpr std::uint32_t unknown = base + 70;
  auto state = sequenced();
  state.pending_fetch2 = unknown;
  lead_a1::SharcFetch fetch(fixture.memory, state);
  const auto native = fixture.cpu.state();
  rejected([&] { fetch.complete_fetch2(fixture.cpu); }, "Unknown SHARC memory bits");
  require(fetch.state() == state && fixture.cpu.state() == native && !fixture.memory.active());
  code(fixture.memory, unknown, new_word);
  fetch.complete_fetch2(fixture.cpu);
  const auto word = fetch.state().fetch2;
  require(word && word->opcode == new_word);
  require(fixture.cpu.state() == native);
  state.fetch2 = lead_a1::SharcFetchWord{.address = unknown, .opcode = 0};
  rejected([&] { const lead_a1::SharcFetch invalid(fixture.memory, state); },
           "pending F2 memory response");
  state.fetch2.reset();
  constexpr std::uint32_t outside = 0x40000;
  state.pending_fetch2 = outside;
  rejected([&] { const lead_a1::SharcFetch invalid(fixture.memory, state); },
           "internal native address");
}

void cached_source_is_not_deferred() {
  Fixture fixture;
  code(fixture.memory, base + 1, store_px | (base + 4));
  issue(fixture, lead_a1::SharcFetchRead::immediate);
  store_clock(fixture);
  const auto native = fixture.cpu.state();
  const auto transport = fixture.fetch.state();
  require(fixture.fetch.address_pm_request(fixture.cpu) && transport.lookup);
  rejected(
      [&] {
        static_cast<void>(advance(fixture.cpu, fixture.fetch, lead_a1::SharcFetchRead::deferred));
      },
      "deferred F2 requires an uncached RAM request");
  require(fixture.cpu.state() == native && fixture.fetch.state() == transport);
}

void malformed_response_input() {
  Fixture fixture;
  const auto native = fixture.cpu.state();
  const auto transport = fixture.fetch.state();
  const auto controller = fixture.control.snapshot();
  constexpr auto invalid = std::bit_cast<lead_a1::SharcFetchRead>(std::uint8_t{2});
  rejected([&] { static_cast<void>(advance(fixture.cpu, fixture.fetch, invalid)); },
           "Invalid SHARC F2 memory response input");
  rejected(
      [&] {
        static_cast<void>(advance(fixture.cpu, fixture.fetch, lead_a1::SharcFetchRead::deferred));
      },
      "requires an Address PM request");
  require(fixture.cpu.state() == native && fixture.fetch.state() == transport &&
          fixture.control.snapshot() == controller);
}

constexpr std::uint32_t loop_body = base + 16;
constexpr std::uint32_t loop_end = loop_body + 1;
constexpr std::uint32_t loop_setup = base + 64;
constexpr std::uint32_t loop_data = 0x80040;
constexpr std::uint32_t sysctl = 0x30024;
constexpr std::uint32_t parent = 0x10c0070;
constexpr std::uint64_t payload = 0x123456789abc;
constexpr std::uint64_t replacement = 0xbb6b00000000; // PM(0,I13)=PX.

// Owned Type1 literal: DM(I1,M2)=R6; PM(I10,M15)=R8; R0=R0-R0.
constexpr std::uint64_t dual_word = 0x32acbc002000;

void loop_program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 512;
  constexpr unsigned int instructions = 128;
  constexpr std::uint64_t backward_do = 0x0e0000ffffd1;
  constexpr std::uint64_t stack_write = 0x0f6400000000;
  constexpr std::uint64_t jump_db = 0x063e04000000;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned pending loop instruction RAM"});
  memory.map({.bank = 0, .halfwords = cells, .name = "owned pending loop data RAM"});
  code(memory, loop_data, 0);
  for (unsigned int index = 0; index < instructions; ++index) { code(memory, base + index, 0); }
  code(memory, loop_setup, backward_do);
  code(memory, loop_setup + 1, stack_write | loop_body);
  code(memory, loop_setup + 8, jump_db | loop_body);
  code(memory, loop_end, dual_word);
}

lead_a1::SharcState loop_initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(loop_setup);
  constexpr std::uint32_t all_bits = 0xffffffff;
  state.depth = 1;
  state.pc_stack[0] = parent;
  constexpr std::uint64_t initial_r0 = 17;
  state.data[0][0] = {.value = initial_r0 << 8, .known = full40};
  state.data[0][6] = {.value = std::uint64_t{4} << 8, .known = full40};
  state.data[0][8] = {.value = replacement >> 8, .known = full40};
  state.px = {.value = payload << 16, .known = full64};
  state.dag[0][0][1] = {.value = sysctl, .known = all_bits};
  state.dag[0][0][10] = {.value = loop_end, .known = all_bits};
  state.dag[0][0][13] = {.value = loop_data, .known = all_bits};
  state.dag[0][1][2] = {.value = 0, .known = all_bits};
  state.dag[0][1][15] = {.value = 0, .known = all_bits};
  return state;
}

struct LoopProgress {
  unsigned int ends{};
  unsigned int firsts{};
  unsigned int stalls{};
  bool issued{};
};

void loop_retire(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch, LoopProgress &progress) {
  if (!fetch.state().execute) { return; }
  const auto step = cpu.step(fetch);
  if (step.pc == loop_body) { ++progress.firsts; }
  if (step.pc == loop_end) {
    require(step.opcode == (progress.ends == 0 ? dual_word : replacement));
    ++progress.ends;
  }
}

void loop_response(lead_a1::SharcMemory &memory,
                   lead_a1::Sharc &cpu,
                   lead_a1::SharcFetch &fetch,
                   unsigned int ends) {
  if (ends != 1 || !fetch.state().pending_fetch2) { return; }
  require(memory.read({.address = loop_end, .width = lead_a1::SharcWidth::wide48}) == replacement);
  fetch.complete_fetch2(cpu);
}

void self_modifying_loop() {
  lead_a1::SharcMemory memory;
  loop_program(memory);
  lead_a1::SharcControl control(true);
  constexpr std::uint32_t widened = 0x804;
  control.write({.address = 0, .size = 4}, widened);
  control.advance_internal_clocks(1);
  memory.map_io({.first_word = sysctl, .words = 1}, control);
  lead_a1::Sharc cpu(memory, loop_initial(), &control);
  auto transport = sequenced();
  transport.flow = lead_a1::SharcFetchFlow{.next = loop_setup};
  lead_a1::SharcFetch fetch(memory, transport);
  LoopProgress progress;
  constexpr unsigned int limit = 64;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != loop_end + 1; ++clock) {
    const bool defer = !progress.issued && fetch.address_pm_request(cpu);
    if (defer) { require(fetch.state().fetch1 == loop_end && !fetch.state().lookup); }
    const auto event = advance(
        cpu, fetch, defer ? lead_a1::SharcFetchRead::deferred : lead_a1::SharcFetchRead::immediate);
    progress.issued = progress.issued || defer;
    if (event.stalled) { ++progress.stalls; }
    loop_retire(cpu, fetch, progress);
    loop_response(memory, cpu, fetch, progress.ends);
  }
  require(progress.issued && progress.ends == 2 && progress.firsts == 2 && progress.stalls == 2 &&
          cpu.state().pc == loop_end + 1 && !cpu.state().arithmetic_loop &&
          !fetch.state().pending_fetch2);
  require(cpu.state().depth == 1 && cpu.state().pc_stack[0] == parent &&
          cpu.state().loops_empty == true && cpu.state().dag[0][0][13].value == loop_data);
  require(memory.read({.address = loop_data, .width = lead_a1::SharcWidth::wide48}) == payload);
  require(control.state().control == 4 && control.state().effective == 4 &&
          !control.state().delayed);
}

}

int main() try {
  for (const auto order : {ResponseOrder::immediate, ResponseOrder::early, ResponseOrder::late}) {
    response_order(order);
  }
  pending_restore(false);
  pending_restore(true);
  response_ownership();
  failed_response_and_malformed_state();
  cached_source_is_not_deferred();
  malformed_response_input();
  self_modifying_loop();
  std::cout
      << "Explicit pending F2 responses preserve captured words and execute actual RAM data\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
