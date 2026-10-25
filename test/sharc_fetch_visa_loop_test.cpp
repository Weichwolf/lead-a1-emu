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

constexpr std::uint32_t base = 0x180080;
constexpr unsigned int cells = 512;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t immediate_do = 0x0c0000000003;
constexpr std::uint64_t increment = 0x013e00029000;
constexpr std::uint64_t decrement = 0x013e0002a000;
using Address = std::optional<std::uint32_t>;
using Stages = std::array<Address, 5>;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("VISA loop expectation at line " + std::to_string(where.line()));
  }
}

template <typename Optional> auto &present(Optional &value) {
  if (!value) { throw std::runtime_error("Missing VISA loop test state"); }
  return *value;
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing VISA loop rejection: " + reason);
}

struct CodeWord {
  std::uint32_t address;
  std::uint64_t opcode;
};

void code(lead_a1::SharcMemory &memory, CodeWord instruction) {
  const std::array words{static_cast<std::uint16_t>(instruction.opcode >> 32),
                         static_cast<std::uint16_t>(instruction.opcode >> 16),
                         static_cast<std::uint16_t>(instruction.opcode)};
  for (unsigned int index = 0; index < words.size(); ++index) {
    memory.write({.address = instruction.address + index, .width = lead_a1::SharcWidth::short16},
                 words[index]);
  }
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  for (auto &reg : state.data[0]) { reg = {.known = full40}; }
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

struct Fixture {
  lead_a1::SharcMemory memory;
  lead_a1::Sharc cpu;
  lead_a1::SharcFetch fetch{memory, sequenced()};

  explicit Fixture(unsigned int count = 5, const lead_a1::SharcState &state = initial())
      : cpu(memory, state) {
    constexpr unsigned int counter_shift = 24;
    memory.map({.bank = 2, .halfwords = cells, .name = "owned clocked VISA counter loop"});
    constexpr std::uint32_t first_short = 0x180000;
    for (unsigned int index = 0; index < cells; ++index) {
      memory.write({.address = first_short + index, .width = lead_a1::SharcWidth::short16}, 0);
    }
    code(memory,
         {.address = base, .opcode = immediate_do | (std::uint64_t{count} << counter_shift)});
    code(memory, {.address = base + 3, .opcode = increment});
  }
};

void tick(Fixture &fixture) {
  static_cast<void>(fixture.fetch.advance_sequenced(fixture.cpu));
  if (fixture.fetch.state().execute) { static_cast<void>(fixture.cpu.step(fixture.fetch)); }
}

void warm(Fixture &fixture) {
  for (unsigned int clock = 0; clock < 3; ++clock) {
    const auto event = fixture.fetch.advance_sequenced(fixture.cpu);
    require(event.address_accepted && !event.stalled && !fixture.fetch.state().execute);
  }
}

Address offset(const std::optional<lead_a1::SharcFetchWord> &word) {
  return word ? Address{word->address - base} : Address{};
}

Stages stages(const lead_a1::SharcFetch &fetch) {
  const auto &state = fetch.state();
  return {state.fetch1 ? Address{*state.fetch1 - base} : Address{},
          state.visa && state.visa->count != 0 ? Address{state.visa->decode - base} : Address{},
          offset(state.decode),
          offset(state.address),
          offset(state.execute)};
}

struct Table {
  unsigned int count;
  unsigned int pop_clock;
  unsigned int end_clock;
  std::array<Stages, 10> clocks;
};

void pipeline_tables() {
  const Address gap;
  // PGR4-61/62 Tables4-17..20, with short-word PCs for uncompressed VISA.
  // F2 is the actual three-word IAB input, not a semantically decoded word.
  const std::array<Table, 4> tables{{
      {.count = 5,
       .pop_clock = 5,
       .end_clock = 9,
       .clocks = {{{9, 6, 3, 0, gap},
                   {3, 9, 3, 3, 0},
                   {3, 9, 3, gap, 3},
                   {3, 3, 3, 3, gap},
                   {6, 3, 3, 3, 3},
                   {9, 6, 3, 3, 3},
                   {12, 9, 6, 3, 3},
                   {15, 12, 9, 6, 3},
                   {18, 15, 12, 9, 6}}}},
      {.count = 4,
       .pop_clock = 4,
       .end_clock = 8,
       .clocks = {{{9, 6, 3, 0, gap},
                   {3, 9, 3, 3, 0},
                   {3, 9, 3, gap, 3},
                   {6, 3, 3, 3, gap},
                   {9, 6, 3, 3, 3},
                   {12, 9, 6, 3, 3},
                   {15, 12, 9, 6, 3},
                   {18, 15, 12, 9, 6}}}},
      {.count = 3,
       .pop_clock = 6,
       .end_clock = 10,
       .clocks = {{{9, 6, 3, 0, gap},
                   {3, 9, 3, 3, 0},
                   {3, 9, 3, gap, 3},
                   {3, 3, 3, 3, gap},
                   {3, 3, gap, 3, 3},
                   {6, 3, gap, gap, 3},
                   {9, 6, gap, gap, gap},
                   {12, 9, 6, gap, gap},
                   {15, 12, 9, 6, gap},
                   {18, 15, 12, 9, 6}}}},
      {.count = 2,
       .pop_clock = 5,
       .end_clock = 9,
       .clocks = {{{9, 6, 3, 0, gap},
                   {3, 9, 3, 3, 0},
                   {3, 9, 3, gap, 3},
                   {3, 3, gap, 3, gap},
                   {6, 3, gap, gap, 3},
                   {9, 6, gap, gap, gap},
                   {12, 9, 6, gap, gap},
                   {15, 12, 9, 6, gap},
                   {18, 15, 12, 9, 6}}}},
  }};
  for (const auto &table : tables) {
    Fixture fixture(table.count);
    warm(fixture);
    unsigned int retired{};
    for (unsigned int clock = 1; clock <= table.end_clock; ++clock) {
      const auto event = fixture.fetch.advance_sequenced(fixture.cpu);
      require(stages(fixture.fetch) == table.clocks.at(clock - 1));
      require(event.loop_stall == (clock == 3) && event.stalled == (clock == 3));
      if (fixture.fetch.state().execute) {
        const auto step = fixture.cpu.step(fixture.fetch);
        require(step.address_units == 3 && step.minimum_core_cycles == 1);
        if (step.pc == base + 3) {
          require(step.opcode == increment);
          ++retired;
        }
      }
      const bool live = clock >= 2 && clock < table.pop_clock;
      require(fixture.cpu.state().depth == (live ? 1U : 0U) &&
              fixture.cpu.state().loops_empty == !live);
    }
    require(retired == table.count &&
            fixture.cpu.state().data[0][0].value == (std::uint64_t{table.count} << 8) &&
            !fixture.cpu.state().count_loop && !present(fixture.fetch.state().flow).loop &&
            fixture.cpu.state().pc == base + 9);
  }
}

void captured_and_refetched_body() {
  Fixture fixture;
  warm(fixture);
  tick(fixture); // DO Address, actual body locked in Decode.
  code(fixture.memory, {.address = base + 3, .opcode = decrement});
  constexpr std::array<std::uint64_t, 5> expected{
      increment, increment, increment, decrement, decrement};
  unsigned int retired{};
  constexpr unsigned int clocks = 8;
  for (unsigned int clock = 0; clock < clocks; ++clock) {
    static_cast<void>(fixture.fetch.advance_sequenced(fixture.cpu));
    if (!fixture.fetch.state().execute) { continue; }
    const auto step = fixture.cpu.step(fixture.fetch);
    if (step.pc != base + 3) { continue; }
    require(retired < expected.size() && step.opcode == expected[retired]);
    ++retired;
  }
  require(retired == expected.size() && fixture.cpu.state().data[0][0].value == (1U << 8));
}

void compact_tail() {
  for (const auto count : {2U, 3U, 4U, 5U, 16U}) {
    Fixture fixture(count);
    constexpr unsigned int tail_words = 24;
    for (unsigned int index = 0; index < tail_words; ++index) {
      constexpr std::uint16_t short_increment = 0xc500;
      fixture.memory.write({.address = base + 6 + index, .width = lead_a1::SharcWidth::short16},
                           short_increment);
    }
    constexpr unsigned int following = 10;
    constexpr unsigned int clock_limit = 64;
    unsigned int tail{};
    for (unsigned int clock = 0; clock < clock_limit && tail < following; ++clock) {
      static_cast<void>(fixture.fetch.advance_sequenced(fixture.cpu));
      if (!fixture.fetch.state().execute) { continue; }
      const auto step = fixture.cpu.step(fixture.fetch);
      if (step.pc >= base + 6) {
        require(step.address_units == 1 && step.pc == base + 6 + tail);
        ++tail;
      }
    }
    require(tail == following &&
            fixture.cpu.state().data[0][0].value == (std::uint64_t{count + following} << 8));
  }
}

void snapshots() {
  Fixture fixture;
  warm(fixture);
  tick(fixture);
  tick(fixture);
  const auto state = fixture.fetch.state();
  lead_a1::Sharc restored(fixture.memory, fixture.cpu.state());
  lead_a1::SharcFetch replay(fixture.memory, state);
  constexpr unsigned int clocks = 7;
  for (unsigned int clock = 0; clock < clocks; ++clock) {
    require(fixture.fetch.advance_sequenced(fixture.cpu) == replay.advance_sequenced(restored));
    if (fixture.fetch.state().execute) {
      const auto actual = fixture.cpu.step(fixture.fetch);
      const auto copied = restored.step(replay);
      require(actual.pc == copied.pc && actual.opcode == copied.opcode);
    }
    require(fixture.cpu.state() == restored.state() && fixture.fetch.state() == replay.state());
  }
  require(restored.state().data[0][0].value == (5U << 8));
  auto malformed = state;
  present(present(malformed.flow).loop).units = 2;
  rejected([&fixture, malformed] { const lead_a1::SharcFetch bad(fixture.memory, malformed); },
           "inherited VISA F1/IAB cursor");
  malformed = state;
  present(present(present(malformed.flow).loop).held).visa.reset();
  rejected([&fixture, malformed] { const lead_a1::SharcFetch bad(fixture.memory, malformed); },
           "retained SHARC loop instruction span");
}

void unsupported_entry() {
  Fixture fixture(1);
  warm(fixture);
  const auto state = fixture.fetch.state();
  const auto core = fixture.cpu.state();
  rejected([&fixture] { static_cast<void>(fixture.fetch.advance_sequenced(fixture.cpu)); },
           "length/count-one timing not qualified");
  require(state == fixture.fetch.state() && core == fixture.cpu.state());
  Fixture compact;
  constexpr std::uint16_t short_increment = 0xc500;
  compact.memory.write({.address = base + 3, .width = lead_a1::SharcWidth::short16},
                       short_increment);
  warm(compact);
  const auto transport = compact.fetch.state();
  rejected([&compact] { static_cast<void>(compact.fetch.advance_sequenced(compact.cpu)); },
           "single-body input alignment/length");
  require(transport == compact.fetch.state());
}

void post_loop_return() {
  constexpr std::uint32_t target = base + 64;
  constexpr std::uint32_t call_frame = 0x1000000;
  for (const auto count : {2U, 3U, 4U, 5U, 16U}) {
    auto state = initial();
    state.depth = 1;
    state.pc_stack[0] = call_frame | target;
    Fixture fixture(count, state);
    // The actual RTS is followed by reserved bytes. Aborted input is never decoded.
    constexpr std::uint64_t tail = 0x0abeffffffff;
    code(fixture.memory, {.address = base + 6, .opcode = tail});
    constexpr unsigned int clock_limit = 64;
    unsigned int retired{};
    for (unsigned int clock = 0; clock < clock_limit && fixture.cpu.state().pc != target; ++clock) {
      static_cast<void>(fixture.fetch.advance_sequenced(fixture.cpu));
      if (!fixture.fetch.state().execute) { continue; }
      const auto step = fixture.cpu.step(fixture.fetch);
      ++retired;
      if (step.pc == base + 6) { require(step.address_units == 1 && step.next_pc == target); }
    }
    require(retired == count + 2 && fixture.cpu.state().pc == target &&
            fixture.cpu.state().depth == 0 &&
            fixture.cpu.state().data[0][0].value == (std::uint64_t{count} << 8) &&
            !fixture.cpu.state().count_loop && !present(fixture.fetch.state().flow).loop);
  }
}

}

int main() {
  try {
    pipeline_tables();
    captured_and_refetched_body();
    compact_tail();
    snapshots();
    unsupported_entry();
    post_loop_return();
    std::cout << "VISA loop phase/pop/refetch/tail/snapshot expectations passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
