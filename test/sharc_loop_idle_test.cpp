// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_boot.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <array>
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
constexpr std::uint32_t vector = base + 0x30;
constexpr std::uint32_t parent = 0x10c0070;
constexpr std::uint32_t p1i = 0x1000;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t invalid = 0xffffffffffff;
constexpr std::uint64_t clear_mask = 0x0f7b00000000;
constexpr std::uint64_t rti = 0x0b3e00000000;
constexpr std::uint64_t decrement_r5 = 0x013e0002a550;
constexpr unsigned int instructions = 128;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Loop IDLE expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing loop IDLE rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 512;
  constexpr std::uint64_t backward_do = 0x0e0000ffffd1;
  constexpr std::uint64_t stack_write = 0x0f6400000000;
  constexpr std::uint64_t jump_db = 0x063e04000000;
  constexpr std::uint64_t idle = 0x008000000000;
  constexpr std::uint64_t old_decrement_r7 = 0x013e0002a770;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned loop IDLE"});
  for (unsigned int index = 0; index < instructions; ++index) { code(memory, base + index, 0); }
  code(memory, setup, backward_do);
  code(memory, setup + 1, stack_write | body);
  code(memory, setup + 8, jump_db | body);
  code(memory, setup + 9, idle);
  code(memory, setup + 10, clear_mask);
  code(memory, body, old_decrement_r7);
  code(memory, vector, invalid);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(setup);
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.system[2].value = p1i;
  state.system[11].value |= p1i;
  state.effective = state.system;
  state.data[0][5] = {.value = 256, .known = full40};
  state.data[0][7] = {.value = std::uint64_t{7} * 256, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = setup};
  return state;
}

void enter_idle(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  constexpr unsigned int limit = 32;
  for (unsigned int clock = 0; clock < limit && !cpu.state().idle_pc; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (fetch.state().execute) { require(cpu.step(fetch).minimum_core_cycles == 1); }
  }
  require(cpu.state().pc == setup + 10 && cpu.state().idle_pc == setup + 9);
  const auto branch = cpu.state().branch;
  require(branch && branch->slots == 1 && branch->target == body);
  require(cpu.state().depth == 2 && cpu.state().pc_stack[1] == body);
  require(cpu.state().idle_delay_word == clear_mask && fetch.state().fetch1 == end);
}

std::uint32_t replacement_word(unsigned int index) {
  std::array<std::uint64_t, instructions> image{};
  image[setup + 10 - base] = invalid;
  image[end - base] = decrement_r5;
  constexpr std::uint64_t laddr_r1 = 0x767f00800000;
  constexpr std::uint64_t stack_r0 = 0x767e00000000;
  constexpr std::uint64_t depth_r2 = 0x767e81000000;
  image[vector - base] = laddr_r1;
  image[vector - base + 1] = stack_r0;
  image[vector - base + 2] = depth_r2;
  image[vector - base + 3] = rti;
  constexpr unsigned int instruction_bytes = 6;
  constexpr unsigned int word_bytes = 4;
  constexpr std::uint64_t byte = 255;
  std::uint32_t value{};
  for (unsigned int offset = 0; offset < word_bytes; ++offset) {
    const auto position = (index * word_bytes) + offset;
    const auto data =
        (image.at(position / instruction_bytes) >> ((position % instruction_bytes) * 8)) & byte;
    value |= static_cast<std::uint32_t>(data) << (offset * 8);
  }
  return value;
}

void transfer(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch, lead_a1::SharcMemory &memory) {
  constexpr std::uint32_t loaded_kernel_end = 0x92180;
  constexpr unsigned int words = instructions * 3 / 2;
  lead_a1::SharcSpi spi(lead_a1::SharcBootState{.clock = true, .next_word = loaded_kernel_end});
  spi.write({.address = 0, .size = 4}, base);
  spi.write({.address = 8, .size = 4}, words);
  const auto held = cpu.state();
  const auto transport = fetch.state();
  unsigned int written{};
  for (unsigned int index = 0; index < words; ++index) {
    spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
    for (unsigned int bit = 0; bit < 32; ++bit) {
      spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
      spi.observe(
          memory,
          {.selected = true, .clock = true, .mosi = ((replacement_word(index) >> bit) & 1) != 0});
    }
    written += spi.service_dma(memory);
    require(cpu.state() == held && fetch.state() == transport && !spi.state().overflow);
    require(spi.state().interrupt_pending == (index == words - 1));
  }
  require(written == words && spi.state().serial.remaining == 0);
  for (unsigned int index = 0; index < words; ++index) {
    require(memory.read({.address = base + index}) == replacement_word(index));
  }
  require(memory.read({.address = setup + 10, .width = lead_a1::SharcWidth::wide48}) == invalid);
  cpu.latch_interrupt(p1i);
  spi.acknowledge_interrupt();
}

void resume_slot(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  require(!cpu.service_interrupt(fetch, {.vector_base = base}));
  require(cpu.wake_idle() && cpu.state().idle_delay_word == clear_mask);
  static_cast<void>(fetch.advance_sequenced(cpu));
  require(cpu.step(fetch).opcode == clear_mask);
  require(cpu.state().pc == body && cpu.state().depth == 2 && !cpu.state().branch);
}

void recognize(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  const auto event = cpu.service_interrupt(fetch, {.vector_base = base});
  require(event && event->return_pc == body && event->vector == vector);
  require(cpu.state().depth == 3 && cpu.state().pc_stack[1] == body);
  require(cpu.state().pc_stack[2] == (0x2000000 | body));
  require(cpu.state().effective[11].value == 3 && !cpu.state().pending[11]);
  const auto loop = cpu.state().arithmetic_loop;
  require(loop && loop->phase == lead_a1::SharcArithmeticPhase::fetched_suspended &&
          loop->body == body && loop->end == end && loop->saved_depth == 1 &&
          loop->kind == lead_a1::SharcArithmeticKind::multiple);
}

struct Counts {
  unsigned int first{};
  unsigned int last{};
  unsigned int returns{};
};

void retire(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch, Counts &counts) {
  if (!fetch.state().execute) { return; }
  const auto step = cpu.step(fetch);
  require(step.minimum_core_cycles == 1);
  if (step.pc == vector + 3) {
    require(step.opcode == rti);
    ++counts.returns;
  }
  if (step.pc == body) { ++counts.first; }
  if (step.pc == end) { ++counts.last; }
}

void return_and_drain(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch, Counts counts = {}) {
  retire(cpu, fetch, counts);
  constexpr unsigned int limit = 64;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != end + 1; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    retire(cpu, fetch, counts);
  }
  require(counts.returns == 1 && counts.first == 3 && counts.last == 3);
  require(cpu.state().pc == end + 1 && cpu.state().depth == 1 && cpu.state().loops_empty == true);
  require(!cpu.state().arithmetic_loop && !cpu.state().interrupt);
  constexpr std::uint64_t after_decrements = 0xfffffffe00;
  require(cpu.state().data[0][5] ==
          (lead_a1::SharcKnown40{.value = after_decrements, .known = full40}));
  require(cpu.state().data[0][7] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{7} * 256, .known = full40}));
  constexpr std::uint32_t special = 0x2000000;
  require(cpu.state().data[0][0] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{special | body} << 8, .known = full40}));
  require(cpu.state().data[0][1] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{end} << 8, .known = full40}));
  require(cpu.state().data[0][2] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{3} * 256, .known = full40}));
  require(cpu.state().pc_stack[0] == parent);
}

void independently_held_pipeline() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  enter_idle(cpu, fetch);
  lead_a1::SharcMemory other_memory;
  program(other_memory);
  auto other_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
  lead_a1::Sharc other(other_memory, cpu.state(), &other_control);
  lead_a1::SharcFetch transport(other_memory, fetch.state());
  transfer(cpu, fetch, memory);
  transfer(other, transport, other_memory);
  resume_slot(cpu, fetch);
  resume_slot(other, transport);
  recognize(cpu, fetch);
  recognize(other, transport);
  return_and_drain(cpu, fetch);
  return_and_drain(other, transport);
  require(cpu.state() == other.state() && fetch.state() == transport.state());
}

void replacement(lead_a1::SharcMemory &memory) {
  program(memory);
  lead_a1::Sharc keeper(memory, initial());
  lead_a1::SharcFetch held(memory, sequenced());
  transfer(
      keeper, held, memory); // Independent actual serial/DMA replay, before binding the snapshot.
}

struct ResumeCase {
  unsigned int checkpoint;
  bool occupied;
};

void rebound_irq_and_drain(ResumeCase example) {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  enter_idle(cpu, fetch);
  transfer(cpu, fetch, memory);
  resume_slot(cpu, fetch);
  recognize(cpu, fetch);
  Counts counts;
  for (unsigned int clock = 0; clock < example.checkpoint && cpu.state().pc != end + 1; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    retire(cpu, fetch, counts);
  }
  if (example.occupied && cpu.state().pc != end + 1) {
    static_cast<void>(fetch.advance_sequenced(cpu));
  }
  lead_a1::SharcMemory other_memory;
  replacement(other_memory);
  auto other_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
  lead_a1::Sharc other(other_memory, cpu.state(), &other_control);
  lead_a1::SharcFetch transport(other_memory, fetch.state());
  return_and_drain(cpu, fetch, counts);
  return_and_drain(other, transport, counts);
  require(cpu.state() == other.state() && fetch.state() == transport.state());
}

void raw_entry_and_failed_owner() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  enter_idle(cpu, fetch);
  transfer(cpu, fetch, memory);
  resume_slot(cpu, fetch);
  const auto before = cpu.state();
  const auto transport = fetch.state();
  const auto device = control.snapshot();
  auto raw = transport;
  raw.flow.reset();
  lead_a1::SharcFetch unsequenced(memory, raw);
  rejected([&] { static_cast<void>(cpu.service_interrupt(unsequenced, {.vector_base = base})); },
           "requires its sequenced fetch transport");
  rejected([&] { static_cast<void>(unsequenced.advance(cpu, raw.fetch1)); },
           "requires its sequenced transport");
  require(cpu.state() == before && unsequenced.state() == raw && !memory.active());
  rejected([&] { static_cast<void>(cpu.service_interrupt({.vector_base = base})); },
           "requires bound fetch");
  require(cpu.state() == before && fetch.state() == transport && control.snapshot() == device);
  control.begin_instruction();
  rejected([&] { static_cast<void>(cpu.service_interrupt(fetch, {.vector_base = base})); },
           "during a transaction");
  require(cpu.state() == before && fetch.state() == transport && !memory.active());
  control.abort_instruction();
  require(control.snapshot() == device);
  recognize(cpu, fetch);
  const auto suspended = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "requires bound retirement");
  require(cpu.state() == suspended && !memory.active());
  return_and_drain(cpu, fetch);
}

void malformed_suspension() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  enter_idle(cpu, fetch);
  transfer(cpu, fetch, memory);
  resume_slot(cpu, fetch);
  recognize(cpu, fetch);
  auto state = cpu.state();
  state.interrupt.reset();
  rejected([&] { const lead_a1::Sharc bad(memory, state); },
           "suspended arithmetic interrupt frame");
  state = cpu.state();
  if (!state.arithmetic_loop) { throw std::logic_error("Missing owned suspended loop"); }
  state.arithmetic_loop->phase = lead_a1::SharcArithmeticPhase::fetched;
  rejected([&] { const lead_a1::Sharc bad(memory, state); }, "interrupt arithmetic phase");
  state = cpu.state();
  if (!state.interrupt) { throw std::logic_error("Missing owned loop interrupt"); }
  state.interrupt->return_pc = body + 1;
  rejected([&] { const lead_a1::Sharc bad(memory, state); },
           "suspended arithmetic interrupt frame");
  state = cpu.state();
  state.pc_stack[1] = body + 1;
  rejected([&] { const lead_a1::Sharc bad(memory, state); }, "arithmetic-loop PC frame");
  state = cpu.state();
  state.depth = 2;
  rejected([&] { const lead_a1::Sharc bad(memory, state); }, "arithmetic-loop resources");
}

void masked_unknown_and_wrong_idle_context() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  enter_idle(cpu, fetch);
  transfer(cpu, fetch, memory);
  for (const unsigned int reg : {2U, 11U}) {
    auto state = cpu.state();
    state.system[reg].value &= ~p1i;
    state.effective[reg] = state.system[reg];
    lead_a1::Sharc masked(memory, state);
    const auto before = masked.state();
    require(!masked.wake_idle() && masked.state() == before);
  }
  auto state = cpu.state();
  state.system[11].value &= ~p1i;
  state.system[11].known &= ~p1i;
  state.effective[11] = state.system[11];
  lead_a1::Sharc unknown(memory, state);
  const auto before = unknown.state();
  rejected([&] { static_cast<void>(unknown.wake_idle()); }, "Unknown SHARC IDLE wake control");
  require(unknown.state() == before);
  state = cpu.state();
  if (!state.branch) { throw std::logic_error("Missing owned IDLE entry branch"); }
  state.branch->target = end;
  rejected([&] { const lead_a1::Sharc bad(memory, state); }, "retained IDLE delay word");
}

}

int main() try {
  independently_held_pipeline();
  raw_entry_and_failed_owner();
  malformed_suspension();
  masked_unknown_and_wrong_idle_context();
  constexpr unsigned int checkpoints = 24;
  for (unsigned int checkpoint = 0; checkpoint < checkpoints; ++checkpoint) {
    for (const bool occupied : {false, true}) {
      rebound_irq_and_drain({.checkpoint = checkpoint, .occupied = occupied});
    }
  }
  std::cout << "Native loop IDLE/DMA/P1I/RTI and independent drain expectations pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
