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

constexpr std::uint32_t base = 0xc0040;
constexpr std::uint32_t end = 0xc0020;
constexpr std::uint32_t body = end - 1;
constexpr std::uint32_t parent = 0x10c0070;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t deferred_do = 0x0e0000ffffe0;
constexpr std::uint64_t write_stack = 0x0f6400000000;
constexpr std::uint64_t jump_db = 0x063e04000000;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC fetched deferred expectation at line " +
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
  throw std::runtime_error("Missing SHARC fetched deferred rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 512;
  constexpr unsigned int instructions = 128;
  constexpr std::uint32_t bank = 0xc0000;
  constexpr std::uint64_t laddr_r1 = 0x767f00800000;
  constexpr std::uint64_t stack_r0 = 0x767e00000000;
  constexpr std::uint64_t depth_r2 = 0x767e81000000;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned deferred arithmetic setup"});
  for (unsigned int index = 0; index < instructions; ++index) { code(memory, bank + index, 0); }
  code(memory, base, deferred_do);
  code(memory, base + 1, write_stack | body);
  code(memory, base + 4, laddr_r1);
  code(memory, base + 5, stack_r0);
  code(memory, base + 6, depth_r2);
  code(memory, base + 8, jump_db | body);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.system[6] = {}; // A deferred DO must not test this unresolved arithmetic condition.
  state.effective[6] = {};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

void tick(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu) {
  const auto bus = fetch.address_pm_request(cpu) ? lead_a1::SharcFetchBus::pm_data
                                                 : lead_a1::SharcFetchBus::available;
  static_cast<void>(fetch.advance_sequenced(cpu, bus));
  if (fetch.state().execute) { require(cpu.step(fetch).minimum_core_cycles == 1); }
}

void prefix(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu) {
  constexpr unsigned int limit = 32;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != base + 8; ++clock) {
    tick(fetch, cpu);
  }
  require(cpu.state().pc == base + 8 && cpu.state().depth == 2 && cpu.state().loops_empty == false);
  const auto loop = cpu.state().arithmetic_loop;
  if (loop && (loop->body != body || loop->end != end || loop->termination != 0)) {
    throw std::runtime_error("Unexpected loop body/end/condition: " + std::to_string(loop->body) +
                             "/" + std::to_string(loop->end) + "/" +
                             std::to_string(loop->termination));
  }
  require(loop && loop->body == body && loop->end == end && loop->termination == 0 &&
          loop->kind == lead_a1::SharcArithmeticKind::multiple &&
          loop->phase == lead_a1::SharcArithmeticPhase::deferred);
  require(cpu.state().pc_stack[0] == parent && cpu.state().pc_stack[1] == body);
  require(cpu.state().data[0][0] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{body} << 8, .known = full40}));
  require(cpu.state().data[0][1] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{end} << 8, .known = full40}));
  require(cpu.state().data[0][2] == (lead_a1::SharcKnown40{.value = 512, .known = full40}));
  const auto address = fetch.state().address;
  const auto flow = fetch.state().flow;
  require(address && address->address == base + 8 && address->branch && address->branch->taken &&
          address->branch->target == body && flow && flow->next == body);
}

void address_and_native_frame() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int setup_clock = 4;
  for (unsigned int clock = 0; clock < setup_clock; ++clock) { tick(fetch, cpu); }
  require(!cpu.state().arithmetic_loop && cpu.state().pc == base && cpu.state().depth == 1);
  tick(fetch, cpu);
  const auto native = cpu.state().arithmetic_loop;
  require(native && native->body == base + 1 && native->end == end && native->saved_depth == 1);
  require(cpu.state().depth == 2 && cpu.state().pc_stack[1] == base + 1);
  static_cast<void>(fetch.advance_sequenced(cpu));
  const auto before = cpu.state();
  const auto transport = fetch.state();
  control.begin_instruction();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "transaction");
  require(cpu.state() == before && fetch.state() == transport && !memory.active());
  control.abort_instruction();
  require(cpu.step(fetch).minimum_core_cycles == 1);
  prefix(fetch, cpu);
  const auto completed = cpu.state();
  const auto held = fetch.state();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "end window");
  require(cpu.state() == completed && fetch.state() == held && !memory.active());
}

void independent_continuations() {
  constexpr unsigned int checkpoints = 13;
  for (unsigned int checkpoint = 0; checkpoint < checkpoints; ++checkpoint) {
    lead_a1::SharcMemory memory;
    program(memory);
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, initial(), &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    for (unsigned int clock = 0; clock < checkpoint; ++clock) { tick(fetch, cpu); }
    lead_a1::SharcMemory other_memory;
    program(other_memory);
    auto other_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
    lead_a1::Sharc other(other_memory, cpu.state(), &other_control);
    lead_a1::SharcFetch transport(other_memory, fetch.state());
    prefix(fetch, cpu);
    prefix(transport, other);
    require(cpu.state() == other.state() && fetch.state() == transport.state());
  }
}

void unsupported_setup_and_active_body() {
  constexpr std::uint64_t forward = 0x0e0000000006;
  constexpr std::uint64_t reserved = 0x0e0001ffffe0;
  constexpr std::uint64_t counter = 0x0e1e00ffffe0;
  for (const auto opcode : {forward, reserved, counter}) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, base, opcode);
    lead_a1::Sharc cpu(memory, initial());
    lead_a1::SharcFetch fetch(memory, sequenced());
    for (unsigned int clock = 0; clock < 3; ++clock) { tick(fetch, cpu); }
    const auto before = cpu.state();
    const auto transport = fetch.state();
    rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "arithmetic DO setup");
    require(cpu.state() == before && fetch.state() == transport && !memory.active());
  }
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, forward);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  auto state = sequenced();
  if (!state.flow) { throw std::logic_error("Missing owned flow"); }
  state.flow->next = base + 1;
  lead_a1::SharcFetch fetch(memory, state);
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "active/mixed arithmetic");
}

}

int main() try {
  address_and_native_frame();
  independent_continuations();
  unsupported_setup_and_active_body();
  std::cout << "Fetched deferred arithmetic setup/frame/type/readback and guarded entry pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
