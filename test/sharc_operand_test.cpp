// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_dag.hpp"
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
constexpr std::uint32_t data = 0x80000;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t payload = 0x12345678;
constexpr std::uint32_t marker = 0xa5a5a5a5;
constexpr std::uint64_t post_store = 0x64fe08800000;
constexpr std::uint64_t pre_load = 0x643ff8800000;
constexpr std::uint64_t register_store = 0x543e80800000;
constexpr std::uint64_t do_count = 0x0c0000000001;
constexpr std::uint32_t target = data + 8;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC operand expectation at line " + std::to_string(where.line()));
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
  throw std::runtime_error("Missing SHARC operand rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t pc, std::uint64_t opcode) {
  memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, std::uint64_t opcode) {
  constexpr unsigned int cells = 256;
  constexpr unsigned int words = 32;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned captured-address program"});
  for (unsigned int index = 0; index < words; ++index) { code(memory, base + index, 0); }
  code(memory, base, opcode);
}

void ram(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 256;
  constexpr unsigned int words = 64;
  memory.map({.bank = 0, .halfwords = cells, .name = "owned captured-address data"});
  for (unsigned int index = 0; index < words; ++index) {
    memory.write({.address = data + index}, marker);
  }
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.dag[0][0][2] = {.value = target, .known = full32};
  state.dag[0][1][0] = {.value = 1, .known = full32};
  state.data[0][1] = {.value = std::uint64_t{payload} << 8, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

void clock(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu, unsigned int count) {
  for (unsigned int index = 0; index < count; ++index) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
  }
}

lead_a1::SharcDagAddress operand(const std::optional<lead_a1::SharcFetchWord> &word) {
  if (!word || !word->dag) { throw std::runtime_error("Missing captured address in test"); }
  return *word->dag;
}

void retained_post_address() {
  lead_a1::SharcMemory memory;
  program(memory, post_store);
  ram(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  clock(fetch, cpu, 3);
  const lead_a1::SharcDagAddress expected{.before = target,
                                          .modifier = 1,
                                          .address = target,
                                          .after = target + 1,
                                          .index = 2,
                                          .bank = 0,
                                          .width = lead_a1::SharcWidth::normal32,
                                          .post = true};
  require(operand(fetch.state().decode) == expected);
  auto unknown = cpu.state();
  unknown.dag[0][0][2] = {};
  unknown.dag[0][1][0] = {};
  lead_a1::Sharc restored(memory, unknown, &control);
  lead_a1::SharcFetch transport(memory, fetch.state());
  code(memory, base, 0);
  require(transport.advance_sequenced(restored).address_accepted);
  require(operand(transport.state().address) == expected);
  require(transport.advance_sequenced(restored).address_accepted);
  require(operand(transport.state().execute) == expected);
  const auto step = restored.step(transport);
  require(step.opcode == post_store && step.pc == base && step.next_pc == base + 1);
  require(restored.state().dag[0][0][2] ==
          (lead_a1::SharcKnown32{.value = target + 1, .known = full32}));
  require(restored.state().dag[0][1][0].known == 0);
  require(memory.read({.address = target}) == payload);
  require(memory.read({.address = target - 1}) == marker &&
          memory.read({.address = target + 1}) == marker);
}

void pre_modify_and_bank_capture() {
  for (const bool post : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory, post ? register_store : pre_load);
    ram(memory);
    memory.write({.address = target - 1}, payload);
    auto state = initial();
    state.system[2].value = 16; // DAG0..3 secondary bank.
    state.effective = state.system;
    state.dag[1][0][2] = {.value = target, .known = full32};
    state.dag[1][1][0] = {.value = full32 - 1, .known = full32}; // -2
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, state, &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    clock(fetch, cpu, 3);
    const auto captured = operand(fetch.state().decode);
    require(captured.bank == 1 && captured.before == target);
    require(captured.address == (post ? target : target - 1));
    require(captured.after == (post ? target - 2 : target));
    auto changed = cpu.state();
    changed.dag[1][0][2] = {};
    changed.dag[1][1][0] = {};
    constexpr std::uint32_t dag_selection = 16;
    changed.system[2].value &= ~dag_selection;
    changed.system[2].known &= ~dag_selection;
    changed.effective[2] = changed.system[2];
    lead_a1::Sharc restored(memory, changed, &control);
    lead_a1::SharcFetch transport(memory, fetch.state());
    constexpr std::uint32_t replacement = 0x87654321;
    if (!post) { memory.write({.address = target - 1}, replacement); }
    clock(transport, restored, 2);
    require(restored.state().dag[0] == state.dag[0]);
    require(restored.state().dag[1][1][0].known == 0);
    if (post) {
      require(restored.state().dag[1][0][2].value == target - 2 &&
              restored.state().dag[1][0][2].known == full32);
      require(memory.read({.address = target}) == payload);
    } else {
      require(restored.state().dag[1][0][2].known == 0);
      require(restored.state().data[0][1].value == std::uint64_t{replacement} << 8);
    }
  }
}

void in_flight_forwarding() {
  lead_a1::SharcMemory memory;
  program(memory, post_store);
  ram(memory);
  code(memory, base + 1, post_store);
  code(memory, base + 2, post_store);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  clock(fetch, cpu, 4);
  require(operand(fetch.state().address).address == target);
  require(operand(fetch.state().decode).address == target + 1);
  require(cpu.state().dag[0][0][2].value == target);
  require(fetch.advance_sequenced(cpu).address_accepted);
  require(operand(fetch.state().execute).address == target);
  require(operand(fetch.state().address).address == target + 1);
  require(operand(fetch.state().decode).address == target + 2);
  require(cpu.state().dag[0][0][2].value == target);
  static_cast<void>(cpu.step(fetch));
  clock(fetch, cpu, 2);
  require(cpu.state().dag[0][0][2].value == target + 3);
  for (unsigned int index = 0; index < 3; ++index) {
    require(memory.read({.address = target + index}) == payload);
  }
  require(memory.read({.address = target + 3}) == marker);
}

void real_loop_stores() {
  for (const unsigned int count : {2U, 3U, 4U, 5U, 16U}) {
    lead_a1::SharcMemory memory;
    constexpr unsigned int counter_shift = 24;
    program(memory, do_count | (std::uint64_t{count} << counter_shift));
    ram(memory);
    code(memory, base + 1, post_store);
    auto state = initial();
    state.depth = 1;
    constexpr std::uint32_t parent = 0x10c0030;
    state.pc_stack[0] = parent;
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, state, &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    unsigned int retired{};
    constexpr unsigned int limit = 32;
    for (unsigned int index = 0; cpu.state().pc != base + 2 && index < limit; ++index) {
      static_cast<void>(fetch.advance_sequenced(cpu));
      const auto executing = fetch.state().execute;
      if (executing) {
        if (executing->address == base + 1) {
          require(operand(executing).address == target + retired);
          ++retired;
        }
        static_cast<void>(cpu.step(fetch));
      }
      const auto flow = fetch.state().flow;
      if (flow && flow->loop && flow->loop->held) { require(!flow->loop->held->dag); }
    }
    require(retired == count && cpu.state().pc == base + 2);
    require(cpu.state().depth == 1 && cpu.state().pc_stack[0] == parent && !cpu.state().count_loop);
    require(cpu.state().dag[0][0][2].value == target + count);
    for (unsigned int index = 0; index < count; ++index) {
      require(memory.read({.address = target + index}) == payload);
    }
    require(memory.read({.address = target + count}) == marker);
  }
}

void held_opcode_new_addresses() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t loop5 = 0x0c0005000001;
  constexpr std::uint64_t post_two = 0x64fe10800000;
  program(memory, loop5);
  ram(memory);
  code(memory, base + 1, post_store);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  clock(fetch, cpu, 4);
  code(memory, base + 1, post_two);
  constexpr std::array<std::uint32_t, 5> addresses{
      target, target + 1, target + 2, target + 3, target + 5};
  unsigned int retired{};
  constexpr unsigned int limit = 16;
  for (unsigned int index = 0; cpu.state().pc != base + 2 && index < limit; ++index) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    const auto word = fetch.state().execute;
    if (word) {
      if (word->address == base + 1) {
        require(operand(word).address == addresses.at(retired));
        require(word->opcode == (retired < 3 ? post_store : post_two));
        ++retired;
      }
      static_cast<void>(cpu.step(fetch));
    }
  }
  require(retired == addresses.size() && cpu.state().dag[0][0][2].value == target + 7);
  for (const auto address : addresses) { require(memory.read({.address = address}) == payload); }
  require(memory.read({.address = target + 4}) == marker);
}

void failed_pop_and_repair() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t loop5 = 0x0c0005000001;
  program(memory, loop5);
  memory.map({.bank = 0, .halfwords = 2, .name = "owned first loop store"});
  code(memory, base + 1, post_store);
  auto state = initial();
  state.dag[0][0][2].value = data;
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  clock(fetch, cpu, 7); // DO, first store and startup bubble have completed.
  require(fetch.advance_sequenced(cpu).address_accepted);
  const auto word = fetch.state().execute;
  require(word && word->loop_pop && operand(word).address == data + 1);
  const auto before = cpu.state();
  const auto transport = fetch.state();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "Unmapped SHARC");
  require(cpu.state() == before && fetch.state() == transport && !memory.active());
  require(before.depth == 1 && before.count_loop && before.count_loop->frame_live);
  constexpr unsigned int repair_cells = 30;
  memory.map({.bank = 0,
              .first_halfword = 2,
              .halfwords = repair_cells,
              .name = "owned repaired loop target"});
  static_cast<void>(cpu.step(fetch));
  const auto active = cpu.state().count_loop;
  require(cpu.state().depth == 0 && active && !active->frame_live);
  clock(fetch, cpu, 3);
  require(cpu.state().pc == base + 2 && !cpu.state().count_loop);
  for (unsigned int index = 0; index < 5; ++index) {
    require(memory.read({.address = data + index}) == payload);
  }
}

template <typename Mutate>
void invalid_capture(lead_a1::SharcMemory &memory,
                     lead_a1::SharcFetchState state,
                     const Mutate &mutate) {
  if (!state.decode || !state.decode->dag) {
    throw std::runtime_error("Missing corruptible operand");
  }
  mutate(*state.decode->dag);
  rejected([&] { const lead_a1::SharcFetch invalid(memory, state); }, "Invalid SHARC captured DAG");
}

void malformed_and_missing() {
  lead_a1::SharcMemory memory;
  program(memory, post_store);
  ram(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  clock(fetch, cpu, 3);
  const auto before = fetch.state();
  invalid_capture(memory, before, [](auto &item) { item.bank = 2; });
  invalid_capture(memory, before, [](auto &item) { item.index = 16; });
  invalid_capture(memory, before, [](auto &item) { ++item.address; });
  invalid_capture(memory, before, [](auto &item) { ++item.after; });
  invalid_capture(memory, before, [](auto &item) { ++item.modifier; });
  invalid_capture(memory, before, [](auto &item) { item.width = lead_a1::SharcWidth::wide48; });
  invalid_capture(memory, before, [](auto &item) { item.post = false; });
  auto missing = before;
  if (!missing.decode) { throw std::runtime_error("Missing Decode"); }
  missing.decode->dag.reset();
  rejected([&] { const lead_a1::SharcFetch invalid(memory, missing); }, "Missing or unexpected");
  auto early = before;
  early.fetch2 = before.decode;
  rejected([&] { const lead_a1::SharcFetch invalid(memory, early); }, "before Decode");
  auto crossing = before;
  if (!crossing.decode) { throw std::runtime_error("Missing crossing Decode"); }
  constexpr std::uint64_t update = std::uint64_t{1} << 44;
  crossing.decode->opcode = register_store & ~update;
  constexpr std::uint32_t iop = 0x1000;
  crossing.decode->dag = lead_a1::SharcDagAddress{.before = iop,
                                                  .modifier = target - iop,
                                                  .address = target,
                                                  .after = iop,
                                                  .index = 2,
                                                  .bank = 0,
                                                  .width = lead_a1::SharcWidth::normal32,
                                                  .post = false};
  rejected([&] { const lead_a1::SharcFetch invalid(memory, crossing); }, "space crossing");
}

void set_extended(lead_a1::SharcControl &control) {
  constexpr std::uint32_t extended_bank0 = 0x204;
  control.begin_instruction();
  control.write({.size = 4}, extended_bank0);
  control.commit_instruction();
  control.advance_internal_clocks(1);
}

void widths_and_stale_width() {
  for (const auto width : {lead_a1::SharcWidth::normal32,
                           lead_a1::SharcWidth::extended40,
                           lead_a1::SharcWidth::short16}) {
    lead_a1::SharcMemory memory;
    program(memory, post_store);
    ram(memory);
    lead_a1::SharcControl control(true);
    if (width == lead_a1::SharcWidth::extended40) { set_extended(control); }
    auto state = initial();
    constexpr std::uint32_t short_target = 0x100008;
    const auto address = width == lead_a1::SharcWidth::short16 ? short_target : target;
    state.dag[0][0][2].value = address;
    constexpr std::uint64_t extension = 0xab;
    state.data[0][1].value |= extension;
    lead_a1::Sharc cpu(memory, state, width == lead_a1::SharcWidth::short16 ? nullptr : &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    clock(fetch, cpu, 3);
    require(operand(fetch.state().decode).width == width);
    clock(fetch, cpu, 2);
    std::uint64_t expected = payload;
    constexpr std::uint32_t short_mask = 0xffff;
    if (width == lead_a1::SharcWidth::short16) { expected = payload & short_mask; }
    if (width == lead_a1::SharcWidth::extended40) { expected = state.data[0][1].value; }
    require(memory.read({.address = address, .width = width}) == expected);
    require(cpu.state().dag[0][0][2].value == address + 1);
  }
  lead_a1::SharcMemory memory;
  program(memory, post_store);
  ram(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  clock(fetch, cpu, 3);
  set_extended(control);
  require(fetch.advance_sequenced(cpu).address_accepted);
  require(fetch.advance_sequenced(cpu).address_accepted);
  const auto before = cpu.state();
  const auto transport = fetch.state();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "width changed before retirement");
  require(cpu.state() == before && fetch.state() == transport && !memory.active());
  require(memory.read({.address = target}) == marker);
  lead_a1::SharcControl normal(true);
  lead_a1::Sharc restored(memory, before, &normal);
  static_cast<void>(restored.step(fetch));
  require(memory.read({.address = target}) == payload);
}

void checkpoint_continuation() {
  for (unsigned int checkpoint = 3; checkpoint <= 7; ++checkpoint) {
    lead_a1::SharcMemory memory;
    program(memory, post_store);
    ram(memory);
    code(memory, base + 1, post_store);
    code(memory, base + 2, post_store);
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, initial(), &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    clock(fetch, cpu, checkpoint - 1);
    static_cast<void>(fetch.advance_sequenced(cpu));
    lead_a1::SharcMemory other_memory;
    program(other_memory, post_store);
    ram(other_memory);
    code(other_memory, base + 1, post_store);
    code(other_memory, base + 2, post_store);
    constexpr unsigned int words = 64;
    for (unsigned int index = 0; index < words; ++index) {
      other_memory.write({.address = data + index}, memory.read({.address = data + index}));
    }
    auto other_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
    lead_a1::Sharc restored(other_memory, cpu.state(), &other_control);
    lead_a1::SharcFetch other(other_memory, fetch.state());
    constexpr unsigned int limit = 10;
    for (unsigned int time = checkpoint; time <= limit; ++time) {
      if (fetch.state().execute) {
        const auto actual = cpu.step(fetch);
        const auto replay = restored.step(other);
        require(actual.pc == replay.pc && actual.opcode == replay.opcode);
      }
      require(cpu.state() == restored.state() && fetch.state() == other.state());
      if (time != limit) {
        require(fetch.advance_sequenced(cpu) == other.advance_sequenced(restored));
      }
    }
    require(cpu.state().dag[0][0][2].value == target + 3);
    for (unsigned int index = 0; index < 3; ++index) {
      require(memory.read({.address = target + index}) == payload);
      require(other_memory.read({.address = target + index}) == payload);
    }
  }
}

void guarded_contexts() {
  constexpr std::uint64_t condition_bits = std::uint64_t{31} << 33;
  const std::array<std::uint64_t, 4> unsupported{
      0x0f7200000010, 0x117000030024, 0x240200000000, post_store & ~condition_bits};
  for (const auto opcode : unsupported) {
    lead_a1::SharcMemory memory;
    program(memory, opcode);
    ram(memory);
    code(memory, base + 1, post_store);
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, initial(), &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    clock(fetch, cpu, 3);
    const auto before = cpu.state();
    const auto transport = fetch.state();
    rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "not qualified");
    require(cpu.state() == before && fetch.state() == transport && !memory.active());
  }
}

}

int main() {
  try {
    retained_post_address();
    pre_modify_and_bank_capture();
    in_flight_forwarding();
    real_loop_stores();
    held_opcode_new_addresses();
    failed_pop_and_repair();
    malformed_and_missing();
    widths_and_stale_width();
    checkpoint_continuation();
    guarded_contexts();
    std::cout << "SHARC captured DAG addresses and real loop RAM operations passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
