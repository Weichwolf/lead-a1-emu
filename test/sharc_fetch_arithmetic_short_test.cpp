// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t body = base + 16;
constexpr std::uint32_t end = body + 1;
constexpr std::uint32_t setup = base + 64;
constexpr std::uint32_t parent = 0x10c0070;
constexpr std::uint64_t full40 = 0xffffffffff;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC short arithmetic expectation at line " +
                             std::to_string(where.line()));
  }
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, unsigned int producer) {
  constexpr unsigned int cells = 512;
  constexpr unsigned int instructions = 128;
  constexpr std::uint64_t backward_do = 0x0e0000ffffd1;
  constexpr std::uint64_t stack_write = 0x0f6400000000;
  constexpr std::uint64_t jump_db = 0x063e04000000;
  constexpr std::uint64_t decrement_r5 = 0x013e0002a550;
  constexpr std::uint64_t laddr_r1 = 0x767f00800000;
  constexpr std::uint64_t stack_r0 = 0x767e00000000;
  constexpr std::uint64_t depth_r2 = 0x767e81000000;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned reentered arithmetic loop"});
  for (unsigned int index = 0; index < instructions; ++index) { code(memory, base + index, 0); }
  code(memory, setup, backward_do);
  code(memory, setup + 1, stack_write | body);
  code(memory, setup + 8, jump_db | body);
  code(memory, body + producer, decrement_r5);
  code(memory, end + 1, laddr_r1);
  code(memory, end + 2, stack_r0);
  code(memory, end + 3, depth_r2);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(setup);
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.data[0][5] = {.value = 256, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = setup};
  return state;
}

struct Counts {
  unsigned int first{};
  unsigned int last{};
};

void retire(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu, Counts &counts) {
  if (!fetch.state().execute) { return; }
  const auto step = cpu.step(fetch);
  require(step.minimum_core_cycles == 1);
  if (step.pc == body) { ++counts.first; }
  if (step.pc == end) { ++counts.last; }
}

void tick(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu, Counts &counts) {
  const auto event = fetch.advance_sequenced(cpu);
  require(event.address_accepted && !event.stalled);
  retire(fetch, cpu, counts);
}

void finish(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu, Counts counts, unsigned int producer) {
  retire(fetch, cpu, counts);
  constexpr unsigned int limit = 128;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != end + 4; ++clock) {
    tick(fetch, cpu, counts);
  }
  // PGR4-58: first producer -> one more pass; second producer -> two more passes.
  const unsigned int passes = producer == 0 ? 2U : 3U;
  const std::uint64_t final_r5 = producer == 0 ? 0xffffffff00 : 0xfffffffe00;
  require(cpu.state().pc == end + 4 && counts.first == passes && counts.last == passes);
  require(cpu.state().data[0][5] == (lead_a1::SharcKnown40{.value = final_r5, .known = full40}));
  require(cpu.state().data[0][0] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{parent} << 8, .known = full40}));
  constexpr std::uint64_t empty_laddr = 0xffffffff00;
  require(cpu.state().data[0][1] == (lead_a1::SharcKnown40{.value = empty_laddr, .known = full40}));
  require(cpu.state().data[0][2] == (lead_a1::SharcKnown40{.value = 256, .known = full40}));
  require(cpu.state().depth == 1 && cpu.state().loops_empty == true &&
          !cpu.state().arithmetic_loop && !cpu.state().branch);
}

struct ResumeCase {
  unsigned int producer;
  unsigned int checkpoint;
  bool occupied;
};

void rebound(ResumeCase example) {
  const auto [producer, checkpoint, occupied] = example;
  lead_a1::SharcMemory memory;
  program(memory, producer);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  Counts counts;
  for (unsigned int clock = 0; clock < checkpoint && cpu.state().pc != end + 4; ++clock) {
    tick(fetch, cpu, counts);
  }
  if (occupied && cpu.state().pc != end + 4) { static_cast<void>(fetch.advance_sequenced(cpu)); }
  lead_a1::SharcMemory other_memory;
  program(other_memory, producer);
  auto other_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
  lead_a1::Sharc other(other_memory, cpu.state(), &other_control);
  lead_a1::SharcFetch transport(other_memory, fetch.state());
  finish(fetch, cpu, counts, producer);
  finish(transport, other, counts, producer);
  require(cpu.state() == other.state() && fetch.state() == transport.state());
}

void independently_rebound_short_drains() {
  constexpr unsigned int checkpoints = 30;
  for (const unsigned int producer : {0U, 1U}) {
    for (unsigned int checkpoint = 0; checkpoint < checkpoints; ++checkpoint) {
      for (const bool occupied : {false, true}) {
        rebound({.producer = producer, .checkpoint = checkpoint, .occupied = occupied});
      }
    }
  }
}

}

int main() try {
  independently_rebound_short_drains();
  std::cout << "Reentered type0 arithmetic short drains and native resource readback pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
