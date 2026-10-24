// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t branch_pc = base + 2;
constexpr std::uint32_t destination = base + 16;
constexpr std::uint32_t tag_bits = 0xffffff;
constexpr std::uint64_t native_bits = 0xffffffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t full64 = 0xffffffffffffffff;
constexpr std::uint64_t jump = 0x063e00000000;
constexpr std::uint64_t call = 0x06be00000000;
constexpr std::uint64_t relative_jump = 0x073e00000000;
constexpr std::uint64_t relative_call = 0x07be00000000;
constexpr std::uint64_t delayed = 0x4000000;
constexpr std::uint64_t increment = 0x013e00029000;
constexpr std::uint64_t decrement = 0x013e0002a000;
constexpr std::uint64_t increment_r1 = 0x013e00029110;
constexpr std::uint64_t increment_r2 = 0x013e00029220;
constexpr std::uint64_t store_px = 0x136b00000000;
constexpr std::uint64_t invalid = 0xffffffffffff;
using Bus = lead_a1::SharcFetchBus;
using Address = std::optional<std::uint32_t>;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC fetch flow expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) == std::string::npos) {
      throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
    }
    return;
  }
  throw std::runtime_error("Missing SHARC fetch flow rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t pc, std::uint64_t opcode) {
  memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 256;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned sequenced native program"});
  constexpr unsigned int words = 48;
  for (unsigned int index = 0; index < words; ++index) { code(memory, base + index, 0); }
}

lead_a1::SharcState initial(std::uint32_t entry = base) {
  auto state = lead_a1::Sharc::reset_reference_214xx(entry);
  for (unsigned int index = 0; index < 3; ++index) {
    state.data[0][index] = {.value = std::uint64_t{10} * 256, .known = full40};
  }
  return state;
}

lead_a1::SharcFetchState sequenced(std::uint32_t entry = base) {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = entry};
  return state;
}

std::optional<lead_a1::SharcStep>
tick(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu, Bus bus = Bus::available) {
  static_cast<void>(fetch.advance_sequenced(cpu, bus));
  if (fetch.state().execute) { return cpu.step(fetch); }
  return {};
}

Address address(const std::optional<lead_a1::SharcFetchWord> &word) {
  return word ? Address{word->address} : Address{};
}

using Stages = std::array<Address, 5>; // F1, F2, Decode, Address, Execute.

Stages stages(const lead_a1::SharcFetch &fetch) {
  const auto &state = fetch.state();
  return {state.fetch1,
          address(state.fetch2),
          address(state.decode),
          address(state.address),
          address(state.execute)};
}

std::array<Stages, 7> expected(bool db) {
  const Address bubble;
  const auto n = branch_pc;
  const auto j = destination;
  return {{{n + 2, n + 1, n, n - 1, n - 2},
           {n + 3, n + 2, db ? Address{n + 1} : bubble, n, n - 1},
           {j, n + 3, db ? Address{n + 2} : bubble, db ? Address{n + 1} : bubble, n},
           {j + 1, j, bubble, db ? Address{n + 2} : bubble, db ? Address{n + 1} : bubble},
           {j + 2, j + 1, j, bubble, db ? Address{n + 2} : bubble},
           {j + 3, j + 2, j + 1, j, bubble},
           {j + 4, j + 3, j + 2, j + 1, j}}};
}

void branch_table_case(std::uint64_t instruction, bool db) {
  lead_a1::SharcMemory memory;
  program(memory);
  const bool relative = instruction == relative_jump || instruction == relative_call;
  const bool is_call = instruction == call || instruction == relative_call;
  const auto operand = relative ? destination - branch_pc : destination;
  code(memory, branch_pc, instruction | (db ? delayed : 0) | operand);
  code(memory, branch_pc + 1, db ? increment_r1 : invalid);
  code(memory, branch_pc + 2, db ? increment_r2 : invalid);
  code(memory, branch_pc + 3, invalid);
  code(memory, destination, increment);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  for (unsigned int cycle = 0; cycle < 4; ++cycle) {
    require(!tick(fetch, cpu));
    require(cpu.state().pc == base);
  }
  const auto oracle = expected(db);
  const auto return_frame = std::uint32_t{0x1000000} | (branch_pc + (db ? 3U : 1U));
  for (unsigned int cycle = 0; cycle < oracle.size(); ++cycle) {
    const auto event = fetch.advance_sequenced(cpu);
    require(event.address_accepted && !event.stalled);
    require(stages(fetch) == oracle[cycle]);
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
    if (cycle < 2) { require(cpu.state().depth == 0); }
    if (cycle >= 2 && is_call) {
      require(cpu.state().depth == 1 && cpu.state().pc_stack[0] == return_frame);
    }
  }
  require(cpu.state().pc == destination + 1 && !cpu.state().branch);
  require(cpu.state().data[0][0].value == std::uint64_t{11} * 256);
  const auto slot_value = std::uint64_t{db ? 11U : 10U} * 256;
  require(cpu.state().data[0][1].value == slot_value);
  require(cpu.state().data[0][2].value == slot_value);
}

void branch_tables() {
  for (const auto instruction : {jump, call, relative_jump, relative_call}) {
    for (const bool db : {false, true}) { branch_table_case(instruction, db); }
  }
}

void relative_backward_and_refetch() {
  for (const bool db : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, base, increment);
    code(memory, branch_pc, relative_jump | (db ? delayed : 0) | (tag_bits - 1));
    code(memory, branch_pc + 1, increment_r1);
    code(memory, branch_pc + 2, increment_r2);
    code(memory, branch_pc + 3, invalid);
    lead_a1::Sharc cpu(memory, initial());
    lead_a1::SharcFetch fetch(memory, sequenced());
    unsigned int actual = 0;
    constexpr unsigned int clocks = 40;
    for (unsigned int clock = 0; clock < clocks && actual < 3; ++clock) {
      const auto step = tick(fetch, cpu);
      if (step && step->pc == base) { ++actual; }
    }
    require(actual == 3 && cpu.state().data[0][0].value == std::uint64_t{13} * 256);
    const auto slot_value = std::uint64_t{db ? 12U : 10U} * 256;
    require(cpu.state().data[0][1].value == slot_value);
    require(cpu.state().data[0][2].value == slot_value);
  }
}

void actual_delayed_self_modification() {
  for (const bool hit : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, base, jump | delayed | destination);
    code(memory, base + 1, store_px | destination);
    code(memory, base + 3, invalid);
    code(memory, destination, increment);
    auto state = initial();
    state.px = {.value = decrement << 16, .known = full64};
    if (hit) {
      state.cache.valid.value = 1;
      state.cache.addresses[0] = {.value = destination, .known = tag_bits};
      state.cache.instructions[0] = {.value = increment, .known = native_bits};
    }
    lead_a1::Sharc cpu(memory, state);
    lead_a1::SharcFetch fetch(memory, sequenced());
    unsigned int stalls = 0;
    unsigned int retired = 0;
    constexpr unsigned int clocks = 16;
    for (unsigned int clock = 0; clock < clocks && cpu.state().pc != destination + 1; ++clock) {
      const auto decoded = fetch.state().decode;
      const bool pm = decoded && decoded->opcode == (store_px | destination);
      const auto event = fetch.advance_sequenced(cpu, pm ? Bus::pm_data : Bus::available);
      if (event.stalled) {
        ++stalls;
        require(!event.address_accepted);
      }
      if (fetch.state().execute) {
        static_cast<void>(cpu.step(fetch));
        ++retired;
      }
    }
    require(retired == 4 && stalls == (hit ? 0U : 1U));
    require(cpu.state().pc == destination + 1 && !cpu.state().branch);
    require(cpu.state().data[0][0].value == std::uint64_t{hit ? 11U : 9U} * 256);
    require(memory.read({.address = destination, .width = lead_a1::SharcWidth::wide48}) ==
            decrement);
    require(cpu.state().cache.instructions[0].value == (hit ? increment : decrement));
  }
}

void checkpoints_and_backpressure() {
  for (const bool db : {false, true}) {
    for (unsigned int checkpoint = 0; checkpoint < 7; ++checkpoint) {
      lead_a1::SharcMemory memory;
      program(memory);
      code(memory, branch_pc, jump | (db ? delayed : 0) | destination);
      code(memory, branch_pc + 1, increment_r1);
      code(memory, branch_pc + 2, increment_r2);
      code(memory, branch_pc + 3, invalid);
      code(memory, destination, increment);
      lead_a1::Sharc cpu(memory, initial());
      lead_a1::SharcFetch fetch(memory, sequenced());
      for (unsigned int clock = 0; clock < checkpoint + 4; ++clock) {
        static_cast<void>(tick(fetch, cpu));
      }
      lead_a1::Sharc restored_cpu(memory, cpu.state());
      lead_a1::SharcFetch restored(memory, fetch.state());
      constexpr unsigned int remaining = 8;
      for (unsigned int clock = 0; clock < remaining; ++clock) {
        const auto event = fetch.advance_sequenced(cpu);
        require(restored.advance_sequenced(restored_cpu) == event);
        require(restored.state() == fetch.state() && restored_cpu.state() == cpu.state());
        if (fetch.state().execute) {
          const auto before = fetch.state();
          rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "has not retired");
          require(fetch.state() == before);
          const auto left = cpu.step(fetch);
          const auto right = restored_cpu.step(restored);
          require(left.pc == right.pc && left.opcode == right.opcode &&
                  left.next_pc == right.next_pc);
        }
        require(restored.state() == fetch.state() && restored_cpu.state() == cpu.state());
      }
    }
  }
}

void guarded_control_and_clock_failures() {
  struct Boundary {
    std::uint64_t opcode;
    const char *reason;
  };

  constexpr std::array<Boundary, 8> unsupported{{
      {.opcode = 0x061200000000 | destination, .reason = "FLAG/loop condition"},
      {.opcode = jump | 0x4000000000 | destination, .reason = "parallel/LA/CI/reserved branch"},
      {.opcode = jump | 0x1000000 | destination, .reason = "parallel/LA/CI/reserved branch"},
      {.opcode = jump | 0x100000, .reason = "internal native address"},
      {.opcode = 0x083e00000000, .reason = "RTI/loop/indirect/IDLE control"},
      {.opcode = 0x0b3e00000000, .reason = "RTI requires a qualified non-loop P1I frame"},
      {.opcode = 0x0e0000000001, .reason = "arithmetic DO setup"},
      {.opcode = 0x004000000000, .reason = "RTI/loop/indirect/IDLE control"},
  }};
  for (const auto &[opcode, reason] : unsupported) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, base, opcode);
    lead_a1::Sharc cpu(memory, initial());
    lead_a1::SharcFetch fetch(memory, sequenced());
    for (unsigned int clock = 0; clock < 3; ++clock) { require(!tick(fetch, cpu)); }
    const auto before = fetch.state();
    const auto old_cpu = cpu.state();
    rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, reason);
    require(fetch.state() == before && cpu.state() == old_cpu && !memory.active());
  }
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  const auto before = fetch.state();
  rejected([&] { static_cast<void>(fetch.advance(cpu, base)); }, "sequenced clock");
  rejected([&] { fetch.advance(cpu.state().effective[4], base); }, "sequenced clock");
  memory.begin_instruction();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "idle bound memory");
  memory.abort_instruction();
  require(fetch.state() == before && cpu.state() == initial());
  lead_a1::SharcFetch unbound(memory);
  rejected([&] { static_cast<void>(unbound.advance_sequenced(cpu)); }, "requires its F1 cursor");
  auto malformed = sequenced();
  malformed.flow = lead_a1::SharcFetchFlow{.next = base, .discard_decode = 4};
  rejected([&] { const lead_a1::SharcFetch invalid_fetch(memory, malformed); },
           "Decode suppression");
}

void failed_retirement_retains_clock() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr unsigned int cells = 64;
  constexpr std::uint32_t operand = 0x80000;
  constexpr std::uint64_t load_px = 0x126b00000000;
  constexpr std::uint64_t value = 0x123456789abc;
  memory.map({.bank = 0, .halfwords = cells, .name = "owned unknown PX input"});
  code(memory, base, load_px | operand);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  for (unsigned int clock = 0; clock < 3; ++clock) { require(!tick(fetch, cpu)); }
  require(!tick(fetch, cpu, Bus::pm_data));
  const auto stall = fetch.advance_sequenced(cpu);
  require(stall.stalled && !stall.address_accepted && fetch.state().execute);
  const auto before = fetch.state();
  const auto old_cpu = cpu.state();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "Unknown SHARC memory bits");
  require(fetch.state() == before && cpu.state() == old_cpu && !memory.active());
  code(memory, operand, value);
  const auto retired = cpu.step(fetch);
  require(retired.pc == base && retired.opcode == (load_px | operand));
  require(cpu.state().px.value == value << 16 && cpu.state().px.known == full64);
  require(!fetch.state().execute && fetch.state().flow == before.flow);
  const auto continued = fetch.advance_sequenced(cpu);
  require(continued.address_accepted && !continued.stalled && !fetch.state().execute);
  require(fetch.state().fetch1 == base + 4 && fetch.state().fetch2);
}

void guarded_branch_overlap_and_fill() {
  for (const bool cache_fill : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory);
    auto state = initial();
    auto transport = sequenced();
    transport.decode = lead_a1::SharcFetchWord{.address = branch_pc, .opcode = jump | destination};
    if (cache_fill) {
      transport.fetch2 = lead_a1::SharcFetchWord{.address = base + 1, .opcode = 0};
      constexpr std::uint32_t cache_controls = 0x80010;
      transport.fill =
          lead_a1::SharcFetchFill{.mode = {.value = 0, .known = cache_controls}, .entry = 2};
      state.cache.valid.value = 4;
      state.cache.addresses[2] = {.value = base + 1, .known = tag_bits};
    } else {
      transport.flow = lead_a1::SharcFetchFlow{.next = base + 3, .discard_decode = 2};
    }
    lead_a1::Sharc cpu(memory, state);
    lead_a1::SharcFetch fetch(memory, transport);
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); },
             cache_fill ? "branch with pending cache fill" : "overlapping sequenced branches");
    require(cpu.state() == before && fetch.state() == transport && !memory.active());
  }
}

}

int main() {
  try {
    branch_tables();
    relative_backward_and_refetch();
    actual_delayed_self_modification();
    checkpoints_and_backpressure();
    guarded_control_and_clock_failures();
    failed_retirement_retains_clock();
    guarded_branch_overlap_and_fill();
    std::cout << "SHARC native sequenced branches and retained delay slots passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
