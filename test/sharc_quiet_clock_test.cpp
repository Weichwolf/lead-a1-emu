// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_control.hpp"
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

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t target = base + 16;
constexpr std::uint32_t data = 0x80000;
constexpr std::uint32_t sysctl = 0x30024;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t p1i = 0x1000;
constexpr std::uint32_t extended_bank0 = 0x204;
constexpr std::uint64_t jump_db = 0x063e04000000;
constexpr std::uint64_t increment = 0x013e00029000;
constexpr std::uint64_t write_mode = 0x0f7200000400;
constexpr std::uint64_t write_mask = 0x0f7b00001000;
constexpr std::uint64_t write_latch = 0x0f7a00001000;
constexpr std::uint64_t clear_latch = 0x0f7a00000000;
constexpr std::uint64_t set_flag0 = 0x140500000001;
constexpr std::uint64_t write_control = 0x117000030024;
constexpr std::uint64_t load_data = 0x100000080000;
constexpr std::uint64_t flush = 0x170200000000;
constexpr std::uint64_t payload = 0x123456789abc;
constexpr std::uint64_t initial_secondary = 20;
constexpr std::uint64_t expected_secondary = 21;
constexpr lead_a1::MemoryAccess word{.size = 4};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC quiet clock expectation at line " +
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
  throw std::runtime_error("Missing SHARC quiet clock rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t pc, std::uint64_t opcode) {
  memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 256;
  constexpr unsigned int words = 64;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned quiet-clock program"});
  memory.map({.bank = 0, .halfwords = cells, .name = "owned quiet-clock data"});
  for (unsigned int index = 0; index < words; ++index) { code(memory, base + index, 0); }
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.data[0][0] = {.value = std::uint64_t{10} * 256, .known = full40};
  state.data[1][0] = {.value = initial_secondary * 256, .known = full40};
  state.system[5] = {.value = 2, .known = full32}; // FLAG0 output, low.
  state.effective = state.system;
  state.flag_inputs = {.value = 0, .known = 1}; // Separate sampled input stays low.
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

void delay_program(lead_a1::SharcMemory &memory, std::uint64_t operation) {
  program(memory);
  code(memory, base, jump_db | target);
  code(memory, base + 2, operation);
  code(memory, target, increment);
}

void retire_three(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu) {
  unsigned int retired = 0;
  constexpr unsigned int clocks = 7;
  for (unsigned int clock = 0; clock < clocks; ++clock) {
    require(fetch.advance_sequenced(cpu).address_accepted);
    if (fetch.state().execute) {
      require(cpu.step(fetch).pc == base + retired);
      ++retired;
    }
  }
  require(retired == 3 && cpu.state().pc == target && !cpu.state().branch);
}

struct Effect {
  std::uint64_t opcode;
  unsigned int index;
  std::uint32_t before;
  std::uint32_t after;
};

void delay_effect(Effect effect) {
  lead_a1::SharcMemory memory;
  delay_program(memory, effect.opcode);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  retire_three(fetch, cpu);
  require(cpu.state().effective[effect.index].value == effect.before);
  require(cpu.state().system[effect.index].value == effect.after);
  require(cpu.state().pending[effect.index] == cpu.state().system[effect.index]);
  lead_a1::Sharc restored_cpu(memory, cpu.state());
  lead_a1::SharcFetch restored(memory, fetch.state());
  auto expected = cpu.state();
  expected.effective[effect.index] = {.value = effect.after, .known = full32};
  expected.pending[effect.index].reset();
  const auto event = fetch.advance_sequenced(cpu);
  require(event.address_accepted && !event.stalled && !fetch.state().execute);
  require(restored.advance_sequenced(restored_cpu) == event);
  require(cpu.state() == expected && restored_cpu.state() == expected);
  require(fetch.state() == restored.state());
  if (effect.index == 5) {
    require(cpu.flag_drives().high == 1 && cpu.flag_drives().low == 0);
    require(cpu.state().flag_inputs == (lead_a1::SharcKnown32{.value = 0, .known = 1}));
  }
  require(fetch.advance_sequenced(cpu).address_accepted);
  require(restored.advance_sequenced(restored_cpu).address_accepted);
  require(cpu.step(fetch).opcode == increment);
  require(restored_cpu.step(restored).opcode == increment);
  require(cpu.state().data[0][0].value == std::uint64_t{11} * 256);
  require(cpu.state() == restored_cpu.state() && fetch.state() == restored.state());
}

void delayed_effects() {
  constexpr std::array<Effect, 3> effects{{
      {.opcode = write_latch, .index = 10, .before = 0, .after = p1i},
      {.opcode = write_mask, .index = 11, .before = 3, .after = p1i | 3},
      {.opcode = set_flag0, .index = 5, .before = 2, .after = 3},
  }};
  for (const auto effect : effects) { delay_effect(effect); }
}

void single_instruction(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu) {
  constexpr unsigned int clocks = 5;
  unsigned int retired = 0;
  for (unsigned int clock = 0; clock < clocks; ++clock) {
    const auto address = clock == 0 ? std::optional{base} : std::nullopt;
    require(fetch.advance(cpu, address).address_accepted);
    if (fetch.state().execute) {
      require(cpu.step(fetch).pc == base);
      ++retired;
    }
  }
  require(retired == 1 && cpu.state().pc == base + 1);
}

void finish_second(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu) {
  for (unsigned int clock = 0; clock < 4; ++clock) {
    require(fetch.advance(cpu, {}).address_accepted);
    if (clock < 3) { require(!fetch.state().execute); }
  }
  require(cpu.step(fetch).pc == base + 1);
}

void mode_clock_before_decode() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, write_mode);
  code(memory, base + 1, increment);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory);
  single_instruction(fetch, cpu);
  require(cpu.state().effective[2].value == 0 && cpu.state().pending[2]);
  lead_a1::SharcFetchState decoded;
  decoded.fetch2 = lead_a1::SharcFetchWord{.address = base + 1, .opcode = increment};
  lead_a1::SharcFetch blocked(memory, decoded);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(blocked.advance(cpu, {})); }, "MODE1 pipeline");
  require(cpu.state() == before && blocked.state() == decoded);
  rejected([&] { static_cast<void>(fetch.advance(cpu, base + 1)); }, "MODE1 pipeline");
  require(cpu.state() == before && fetch.state() == lead_a1::SharcFetchState{});
  require(fetch.advance(cpu, {}).address_accepted && !fetch.state().execute);
  constexpr std::uint32_t secondary_low = 0x400;
  require(cpu.state().effective[2].value == secondary_low && !cpu.state().pending[2]);
  require(cpu.state().pc == base + 1 && cpu.state().data[0][0] == before.data[0][0]);
  require(fetch.advance(cpu, base + 1).address_accepted);
  finish_second(fetch, cpu);
  require(cpu.state().data[0][0].value == std::uint64_t{10} * 256);
  require(cpu.state().data[1][0].value == expected_secondary * 256);
}

void width_program(lead_a1::SharcMemory &memory) {
  program(memory);
  code(memory, base, write_control);
  code(memory, base + 1, load_data);
  code(memory, data, payload);
}

void sysctl_clock_and_checkpoint() {
  lead_a1::SharcControl control(true);
  lead_a1::SharcMemory memory;
  width_program(memory);
  memory.map_io({.first_word = sysctl, .words = 1}, control);
  auto state = initial();
  state.system[0].value = extended_bank0;
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory);
  single_instruction(fetch, cpu);
  require(control.state().control == extended_bank0 && control.state().effective == 4);
  require(control.state().delayed == extended_bank0);
  lead_a1::SharcFetchState decoded;
  decoded.fetch2 = lead_a1::SharcFetchWord{.address = base + 1, .opcode = load_data};
  lead_a1::SharcFetch blocked(memory, decoded);
  const auto old_cpu = cpu.state();
  const auto old_control = control.state();
  rejected([&] { static_cast<void>(blocked.advance(cpu, {})); }, "SYSCTL Decode/Address");
  require(cpu.state() == old_cpu && control.state() == old_control && blocked.state() == decoded);
  auto restored_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
  lead_a1::SharcMemory restored_memory;
  width_program(restored_memory);
  restored_memory.map_io({.first_word = sysctl, .words = 1}, restored_control);
  lead_a1::Sharc restored_cpu(restored_memory, cpu.state(), &restored_control);
  lead_a1::SharcFetch restored(restored_memory, fetch.state());
  require(fetch.advance(cpu, base + 1).address_accepted);
  require(restored.advance(restored_cpu, base + 1).address_accepted);
  require(cpu.state() == old_cpu && restored_cpu.state() == old_cpu);
  require(control.state().effective == extended_bank0 && !control.state().delayed);
  require(control.state() == restored_control.state() && fetch.state() == restored.state());
  finish_second(fetch, cpu);
  finish_second(restored, restored_cpu);
  constexpr std::uint64_t expected40 = 0x123456789a;
  require(cpu.state().data[0][0] == (lead_a1::SharcKnown40{.value = expected40, .known = full40}));
  require(cpu.state() == restored_cpu.state() && control.state() == restored_control.state());
}

void eligibility_after_quiet_clock() {
  lead_a1::SharcMemory memory;
  delay_program(memory, write_mask);
  auto state = initial();
  state.system[2].value = p1i; // Global enable; old IMASK still blocks P1I.
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  cpu.latch_interrupt(p1i);
  lead_a1::SharcFetch fetch(memory, sequenced());
  retire_three(fetch, cpu);
  require(cpu.state().effective[11].value == 3 && cpu.state().pending[11]);
  require(fetch.advance_sequenced(cpu).address_accepted && !fetch.state().execute);
  const auto before = cpu.state();
  const auto transport = fetch.state();
  require(before.effective[11].value == (p1i | 3) && !before.pending[11]);
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "pending interrupt delivery");
  require(cpu.state() == before && fetch.state() == transport && !cpu.state().interrupt);
  require(cpu.state().pc == target && cpu.state().data[0][0].value == std::uint64_t{10} * 256);
}

void routed_latch_survives_delayed_clear() {
  lead_a1::SharcMemory memory;
  delay_program(memory, clear_latch);
  lead_a1::Sharc cpu(memory, initial());
  cpu.latch_interrupt(p1i);
  lead_a1::SharcFetch fetch(memory, sequenced());
  retire_three(fetch, cpu);
  require(cpu.state().system[10].value == 0 && cpu.state().pending[10]);
  cpu.latch_interrupt(p1i); // Actual owner event merges with the still pending clear.
  require(fetch.advance_sequenced(cpu).address_accepted);
  require(cpu.state().system[10].value == p1i && cpu.state().effective[10].value == p1i);
  require(!cpu.state().pending[10] && !cpu.state().interrupt);
}

void quiet_clock_preserves_flow_and_unknown_bits() {
  lead_a1::SharcMemory memory;
  program(memory);
  auto state = initial();
  state.branch = lead_a1::SharcBranch{.target = target, .slots = 2};
  state.system[10] = {.value = p1i, .known = p1i | 3};
  state.effective[10] = {.known = 3};
  state.pending[10] = state.system[10];
  state.system[11].value = p1i | 3;
  state.pending[11] = state.system[11];
  state.system[5].value = 3;
  state.pending[5] = state.system[5];
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory);
  auto expected = cpu.state();
  for (const unsigned int index : {5U, 10U, 11U}) {
    expected.effective[index] = expected.system[index];
    expected.pending[index].reset();
  }
  require(fetch.advance(cpu, base).address_accepted && !fetch.state().execute);
  require(cpu.state() == expected && cpu.state().pc == base);
  require(cpu.state().branch == state.branch && cpu.state().effective[10].known == (p1i | 3));
}

void failed_clock_keeps_all_resources() {
  for (const bool transaction : {false, true}) {
    lead_a1::SharcControl control(true);
    lead_a1::SharcMemory memory;
    program(memory);
    control.write(word, extended_bank0);
    auto state = initial();
    state.system[11].value = p1i | 3;
    state.pending[11] = state.system[11];
    lead_a1::SharcFetchState transport;
    constexpr std::uint32_t unknown_pc = base + 70;
    if (!transaction) {
      state.pc = unknown_pc;
      transport.fetch1 = unknown_pc;
    }
    lead_a1::Sharc cpu(memory, state, &control);
    lead_a1::SharcFetch fetch(memory, transport);
    if (transaction) { control.begin_instruction(); }
    const auto before = cpu.state();
    const auto device = control.state();
    rejected([&] { static_cast<void>(fetch.advance(cpu, {})); },
             transaction ? "clock advance during a transaction" : "Unknown SHARC memory bits");
    require(cpu.state() == before && control.state() == device && fetch.state() == transport);
    require(!memory.active());
    if (transaction) {
      control.abort_instruction();
    } else {
      code(memory, unknown_pc, 0);
    }
    require(fetch.advance(cpu, {}).address_accepted && !fetch.state().execute);
    require(cpu.state().effective[11].value == (p1i | 3) && !cpu.state().pending[11]);
    require(control.state().effective == extended_bank0 && !control.state().delayed);
  }
}

void unbanked_mode_quiet_clock() {
  struct Mode {
    std::uint64_t instruction;
    std::uint32_t value;
  };

  constexpr std::array<Mode, 3> modes{{{.instruction = 0x140201000000, .value = 0x1000000},
                                       {.instruction = 0x140200010000, .value = 0x10000},
                                       {.instruction = 0x140200000400, .value = 0x400}}};
  constexpr std::uint64_t copy_mode_to_ustat = 0x773f38000000;
  constexpr std::array<std::uint64_t, 4> sensitive{
      increment, 0x0f0000000001, 0x773f00000000, 0x107000080000};
  for (const auto mode : modes) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, base, mode.instruction);
    lead_a1::Sharc cpu(memory, initial());
    lead_a1::SharcFetch writer(memory);
    single_instruction(writer, cpu);
    const auto before = cpu.state();
    require(before.effective[2].value == 0 && before.pending[2] &&
            before.system[2].value == mode.value);
    lead_a1::SharcFetchState input;
    input.flow = lead_a1::SharcFetchFlow{.next = base + 2};
    input.decode = lead_a1::SharcFetchWord{.address = base + 1, .opcode = copy_mode_to_ustat};
    for (const auto opcode : sensitive) {
      auto denied = input;
      denied.decode = lead_a1::SharcFetchWord{.address = base + 1, .opcode = opcode};
      lead_a1::SharcFetch blocked(memory, denied);
      rejected([&] { static_cast<void>(blocked.advance_sequenced(cpu)); }, "quiet MODE1 pipeline");
      require(cpu.state() == before && blocked.state() == denied);
    }
    lead_a1::SharcFetch fetch(memory, input);
    const auto event = fetch.advance_sequenced(cpu);
    auto expected = before;
    expected.effective[2] = {.value = mode.value, .known = full32};
    expected.pending[2].reset();
    require(event.address_accepted && !fetch.state().execute && cpu.state() == expected);
    require(fetch.advance_sequenced(cpu).address_accepted && fetch.state().execute);
    require(cpu.step(fetch).opcode == copy_mode_to_ustat &&
            cpu.state().system[0].value == mode.value && cpu.state().data == before.data &&
            cpu.state().pc == base + 2);
  }
}

void flush_clock_and_follower_guard() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, flush);
  auto state = initial();
  state.cache.valid.value = full32;
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory);
  single_instruction(fetch, cpu);
  const auto before = cpu.state();
  const auto transport = fetch.state();
  require(before.cache.pending_flush && !before.cache.flush_effect_applied &&
          before.cache.valid.value == full32);
  control.begin_instruction();
  rejected([&] { static_cast<void>(fetch.advance(cpu, {})); },
           "clock advance during a transaction");
  require(cpu.state() == before && fetch.state() == transport);
  control.abort_instruction();
  require(fetch.advance(cpu, {}).address_accepted && !fetch.state().execute);
  auto expected = before;
  expected.cache.valid = {.known = full32};
  expected.cache.flush_effect_applied = true;
  require(cpu.state() == expected && fetch.state() == transport && !memory.active());
  lead_a1::Sharc restored(memory, before);
  lead_a1::SharcFetch restored_fetch(memory, transport);
  require(restored_fetch.advance(restored, {}).address_accepted && restored.state() == expected);
  constexpr std::uint64_t prohibited_pm = 0x120000080000;
  code(memory, base + 1, prohibited_pm);
  require(fetch.advance(cpu, base + 1).address_accepted);
  for (unsigned int clock = 0; clock < 4; ++clock) {
    const auto bus = fetch.address_pm_request(cpu) ? lead_a1::SharcFetchBus::pm_data
                                                   : lead_a1::SharcFetchBus::available;
    static_cast<void>(fetch.advance(cpu, {}, bus));
  }
  const auto held = fetch.state();
  rejected([&] { static_cast<void>(cpu.step(fetch)); },
           "PM-data instruction immediately after FLUSH");
  require(cpu.state() == expected && fetch.state() == held && !memory.active());
}

void flush_then_actual_cache_fill() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, flush);
  auto state = initial();
  state.cache.valid.value = full32;
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch flush_fetch(memory);
  single_instruction(flush_fetch, cpu);
  constexpr std::uint32_t cache_controls = 0x80010;
  lead_a1::SharcFetchState request;
  request.fetch1 = base + 1;
  request.lookup = lead_a1::SharcFetchLookup{.mode = {.known = cache_controls}, .wait = true};
  lead_a1::SharcFetch conflict(memory, request);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(conflict.advance(cpu, {})); }, "FLUSH CACHE effect/follower");
  require(cpu.state() == before && conflict.state() == request);
  static_cast<void>(flush_fetch.advance(cpu, {}));
  require(cpu.state().cache.flush_effect_applied && cpu.state().cache.valid.value == 0);
  require(conflict.advance(cpu, {}).stalled);
  constexpr std::uint32_t entry_two = 4; // Set1,way0 for the actual next-PC request.
  constexpr unsigned int cache_entry = 2;
  constexpr std::uint64_t instruction_bits = 0xffffffffffff;
  require(cpu.state().cache.valid.value == entry_two &&
          cpu.state().cache.addresses[cache_entry].value == base + 1);
  for (unsigned int clock = 0; clock < 4; ++clock) { static_cast<void>(conflict.advance(cpu, {})); }
  require(cpu.state().cache.valid.value == entry_two &&
          cpu.state().cache.instructions[cache_entry] ==
              (lead_a1::SharcKnown64{.known = instruction_bits}));
  require(cpu.step(conflict).opcode == 0 && cpu.state().pc == base + 2 &&
          cpu.state().cache.valid.value == entry_two && !cpu.state().cache.pending_flush &&
          !cpu.state().cache.flush_effect_applied);
}

}

int main() {
  try {
    delayed_effects();
    mode_clock_before_decode();
    sysctl_clock_and_checkpoint();
    eligibility_after_quiet_clock();
    routed_latch_survives_delayed_clear();
    quiet_clock_preserves_flow_and_unknown_bits();
    failed_clock_keeps_all_resources();
    unbanked_mode_quiet_clock();
    flush_clock_and_follower_guard();
    flush_then_actual_cache_fill();
    std::cout << "SHARC quiet processor clocks and actual delayed effects passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
