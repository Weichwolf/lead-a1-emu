// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
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
constexpr std::uint32_t body = base + 1;
constexpr std::uint32_t store = body + 3;
constexpr std::uint32_t target = base + 64;
constexpr std::uint64_t payload = 0x123456789abc;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Arithmetic cache expectation at line " +
                             std::to_string(where.line()));
  }
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 512;
  constexpr unsigned int instructions = 128;
  constexpr std::uint64_t forever_do = 0x0e3e00000006;
  constexpr std::uint64_t store_px = 0x136b00000000;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned arithmetic cache composition"});
  for (unsigned int index = 0; index < instructions; ++index) { code(memory, base + index, 0); }
  code(memory, base, forever_do);
  code(memory, store, store_px | target);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t parent = 0x10c0070;
  constexpr std::uint64_t full64 = 0xffffffffffffffff;
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.px = {.value = payload << 16, .known = full64};
  state.system[6] = {};
  state.system[7] = {};
  state.effective = state.system;
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

std::optional<std::uint32_t> address(const std::optional<lead_a1::SharcFetchWord> &word) {
  return word ? std::optional{word->address} : std::nullopt;
}

lead_a1::SharcFetchEvent advance(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  const auto bus = fetch.address_pm_request(cpu) ? lead_a1::SharcFetchBus::pm_data
                                                 : lead_a1::SharcFetchBus::available;
  return fetch.advance_sequenced(cpu, bus);
}

void retire(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch, unsigned int &stores) {
  if (!fetch.state().execute) { return; }
  const auto step = cpu.step(fetch);
  require(step.minimum_core_cycles == 1);
  if (step.pc == store) { ++stores; }
}

void miss_columns_and_real_stores() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int stores{};
  constexpr unsigned int warmup = 16;
  for (unsigned int clock = 0; clock < warmup && address(fetch.state().decode) != store; ++clock) {
    static_cast<void>(advance(cpu, fetch));
    retire(cpu, fetch, stores);
  }
  require(address(fetch.state().decode) == store && cpu.state().pc == body + 2);
  // PGR4-84 Table4-31 columns2..5, with n=body+3 and the actual loop wrap.
  constexpr std::array<std::array<int, 4>, 5> columns{{
      {2, 3, -1, 4},
      {3, -1, 4, 5},
      {4, 4, 5, 0},
      {5, 5, 0, 1},
      {0, 0, 1, 2},
  }};
  for (unsigned int cycle = 0; cycle < 4; ++cycle) {
    const auto event = advance(cpu, fetch);
    require(event.stalled == (cycle == 1) && event.address_accepted == (cycle != 1));
    const auto &state = fetch.state();
    const std::array stages{address(state.execute),
                            address(state.address),
                            address(state.decode),
                            address(state.fetch2),
                            state.fetch1};
    for (unsigned int stage = 0; stage < stages.size(); ++stage) {
      const int offset = columns[stage][cycle];
      const auto expected =
          offset < 0 ? std::nullopt : std::optional{body + static_cast<unsigned int>(offset)};
      require(stages[stage] == expected);
    }
    require(cpu.state().depth == 2 && cpu.state().loops_empty == false);
    retire(cpu, fetch, stores);
    if (cycle >= 1) {
      require(memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) == payload);
    }
  }
  require(stores == 1);
  constexpr unsigned int tail = 32;
  unsigned int hits{};
  for (unsigned int clock = 0; clock < tail; ++clock) {
    const auto event = advance(cpu, fetch);
    require(!event.stalled);
    if (event.cache_hit) { ++hits; }
    retire(cpu, fetch, stores);
  }
  require(stores > 3 && hits > 2 && cpu.state().depth == 2 && cpu.state().loops_empty == false);
  require(cpu.state().system[6].known == 0 && cpu.state().system[7].known == 0);
}

void held_condition_is_not_resampled() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t until_eq = 0x0e0000000006;
  constexpr std::uint64_t decrement_r5 = 0x013e0002a550;
  constexpr std::uint64_t store_px = 0x136b00000000;
  constexpr std::uint32_t second_store = body + 4;
  constexpr std::uint32_t end = body + 5;
  code(memory, base, until_eq);
  code(memory, body + 3, decrement_r5);
  code(memory, second_store, store_px | target);
  auto state = initial();
  constexpr std::uint64_t full40 = 0xffffffffff;
  constexpr std::uint32_t all_bits = 0xffffffff;
  state.data[0][5] = {.value = 256, .known = full40};
  state.system[6] = {.value = 0, .known = all_bits};
  state.system[7] = state.system[6];
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int firsts{};
  unsigned int lasts{};
  unsigned int stores{};
  unsigned int holds{};
  constexpr unsigned int limit = 64;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != end + 1; ++clock) {
    const auto before = cpu.state();
    const auto event = advance(cpu, fetch);
    if (event.stalled) {
      require((before.system[6].value & 1U) != 0 && fetch.state().fetch1 == body + 1);
      require(cpu.state().loops_empty == false && cpu.state().depth == 2);
      ++holds;
    }
    if (!fetch.state().execute) { continue; }
    const auto step = cpu.step(fetch);
    if (step.pc == body) { ++firsts; }
    if (step.pc == end) { ++lasts; }
    if (step.pc == second_store) { ++stores; }
  }
  // Table4-31 holds F1 on the partial stall. The first producer's new AZ must wait
  // for the next real start+1 fetch, leaving two complete passes and stores.
  require(cpu.state().pc == end + 1 && firsts == 2 && lasts == 2 && stores == 2 && holds == 1);
  constexpr std::uint64_t negative_one = 0xffffffff00;
  require(cpu.state().data[0][5] ==
          (lead_a1::SharcKnown40{.value = negative_one, .known = full40}));
  require(cpu.state().loops_empty == true && cpu.state().depth == 1 &&
          !cpu.state().arithmetic_loop);
  require(memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) == payload);
}

void unfilled_cache_modes() {
  constexpr std::uint32_t disabled = 0x10;
  constexpr std::uint32_t frozen = 0x80000;
  constexpr unsigned int limit = 64;
  for (const auto mode : {disabled, frozen}) {
    lead_a1::SharcMemory memory;
    program(memory);
    auto state = initial();
    state.system[4].value |= mode;
    state.effective = state.system;
    lead_a1::Sharc cpu(memory, state);
    lead_a1::SharcFetch fetch(memory, sequenced());
    unsigned int stores{};
    unsigned int stalls{};
    for (unsigned int clock = 0; clock < limit; ++clock) {
      const auto event = advance(cpu, fetch);
      require(!event.cache_hit);
      if (event.stalled) { ++stalls; }
      retire(cpu, fetch, stores);
    }
    require(stores > 3 && stalls == stores && cpu.state().cache == state.cache);
    require(cpu.state().depth == 2 && cpu.state().loops_empty == false);
    require(cpu.state().system[6].known == 0 && cpu.state().system[7].known == 0);
    require(memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) == payload);
  }
}

void frozen_hit_executes_cached_word() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t decrement_r7 = 0x013e0002a770;
  code(memory, body, decrement_r7);
  auto state = initial();
  constexpr std::uint32_t frozen = 0x80000;
  constexpr std::uint32_t pc_mask = 0xffffff;
  constexpr std::uint64_t full40 = 0xffffffffff;
  constexpr std::uint64_t native_bits = 0xffffffffffff;
  constexpr unsigned int entry = 2;
  state.system[4].value |= frozen;
  state.effective = state.system;
  state.data[0][7] = {.value = std::uint64_t{7} << 8, .known = full40};
  state.cache.valid.value |= std::uint32_t{1} << entry;
  state.cache.addresses[entry] = {.value = body, .known = pc_mask};
  state.cache.instructions[entry] = {.value = 0, .known = native_bits};
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int stores{};
  unsigned int hits{};
  constexpr unsigned int limit = 40;
  for (unsigned int clock = 0; clock < limit; ++clock) {
    const auto event = advance(cpu, fetch);
    require(!event.stalled);
    if (event.cache_hit) { ++hits; }
    retire(cpu, fetch, stores);
  }
  require(stores > 3 && hits > 3);
  // The first, unconflicted body fetch uses RAM. Later conflicts use the cached
  // NOP, so exactly one real decrement executes despite unchanged RAM code.
  require(cpu.state().data[0][7] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{6} << 8, .known = full40}));
  require(cpu.state().cache.instructions[entry] == state.cache.instructions[entry]);
  require(memory.read({.address = body, .width = lead_a1::SharcWidth::wide48}) == decrement_r7);
  require(memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) == payload);
}

void pending_miss_termination_preserves_state() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t until_eq = 0x0e0000000006;
  constexpr std::uint64_t decrement_r5 = 0x013e0002a550;
  constexpr std::uint64_t store_px = 0x136b00000000;
  code(memory, base, until_eq);
  code(memory, body + 3, decrement_r5);
  code(memory, body + 4, store_px | target);
  auto state = initial();
  constexpr std::uint64_t full40 = 0xffffffffff;
  constexpr std::uint32_t all_bits = 0xffffffff;
  constexpr std::uint32_t disabled = 0x10;
  state.system[4].value |= disabled;
  state.system[6] = {.value = 0, .known = all_bits};
  state.system[7] = state.system[6];
  state.effective = state.system;
  state.data[0][5] = {.value = 256, .known = full40};
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int limit = 32;
  for (unsigned int clock = 0; clock < limit; ++clock) {
    const auto before = cpu.state();
    const auto transport = fetch.state();
    try {
      static_cast<void>(advance(cpu, fetch));
    } catch (const std::exception &error) {
      require(std::string(error.what()).find("termination with pending cache transfer") !=
              std::string::npos);
      require(before.pc == body + 3 && transport.fetch1 == body && transport.flow &&
              transport.flow->next == body + 1 && (before.system[6].value & 1U) != 0);
      require(cpu.state() == before && fetch.state() == transport && !memory.active());
      require(memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) == payload);
      return;
    }
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
  }
  throw std::runtime_error("Missing pending-miss arithmetic termination guard");
}

void rebound_cache_transfer(unsigned int checkpoint, bool occupied) {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int stores{};
  for (unsigned int clock = 0; clock < checkpoint; ++clock) {
    static_cast<void>(advance(cpu, fetch));
    retire(cpu, fetch, stores);
  }
  if (occupied) { static_cast<void>(advance(cpu, fetch)); }
  lead_a1::SharcMemory other_memory;
  program(other_memory);
  if (stores != 0) { code(other_memory, target, payload); }
  lead_a1::Sharc other(other_memory, cpu.state());
  lead_a1::SharcFetch transport(other_memory, fetch.state());
  auto other_stores = stores;
  retire(cpu, fetch, stores);
  retire(other, transport, other_stores);
  constexpr unsigned int tail = 24;
  for (unsigned int clock = 0; clock < tail; ++clock) {
    require(advance(cpu, fetch) == advance(other, transport));
    retire(cpu, fetch, stores);
    retire(other, transport, other_stores);
  }
  require(stores == other_stores && stores >= 3);
  require(cpu.state() == other.state() && fetch.state() == transport.state());
  require(cpu.state().depth == 2 && cpu.state().loops_empty == false);
  require(memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) == payload);
  require(other_memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) == payload);
}

constexpr std::uint32_t tail_data = 0x80040;
constexpr std::uint32_t tail_end = body + 5;

void tail_program(lead_a1::SharcMemory &memory, bool writable) {
  program(memory);
  constexpr unsigned int cells = 512;
  constexpr std::uint16_t known_half = 0xffff;
  std::array<lead_a1::SharcCell, cells> zero{};
  for (auto &cell : zero) { cell.known = known_half; }
  memory.map(
      {.bank = 0, .halfwords = cells, .name = "owned arithmetic tail data", .writable = writable},
      zero);
  constexpr std::uint64_t until_eq = 0x0e0000000006;
  constexpr std::uint64_t decrement_r5 = 0x013e0002a550;
  constexpr std::uint64_t store_px = 0x136b00000000;
  code(memory, base, until_eq);
  code(memory, body, decrement_r5);
  code(memory, store, 0);
  code(memory, tail_end, store_px | tail_data);
}

lead_a1::SharcState tail_initial() {
  auto state = initial();
  constexpr std::uint32_t all_bits = 0xffffffff;
  constexpr std::uint64_t full40 = 0xffffffffff;
  state.system[6] = {.known = all_bits};
  state.system[7] = state.system[6];
  state.effective = state.system;
  state.data[0][5] = {.value = 256, .known = full40};
  return state;
}

void issue_marked_tail(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  unsigned int firsts{};
  constexpr unsigned int limit = 32;
  for (unsigned int clock = 0; clock < limit; ++clock) {
    const auto event = advance(cpu, fetch);
    if (event.stalled) {
      const auto &state = fetch.state();
      require(!event.address_accepted && !event.control_stall && !event.dag_stall &&
              state.execute && state.execute->arithmetic_exit &&
              state.execute->address == tail_end && !state.decode && !state.address && state.flow &&
              state.flow->discard_decode == 1 && state.fetch1 == tail_end + 1 && state.fetch2 &&
              state.fetch2->address == body + 1);
      require(firsts == 1 && cpu.state().pc == tail_end && cpu.state().depth == 1 &&
              cpu.state().loops_empty == true && cpu.state().arithmetic_loop);
      return;
    }
    if (fetch.state().execute) {
      const auto step = cpu.step(fetch);
      if (step.pc == body) { ++firsts; }
      require(step.pc != tail_end);
    }
  }
  throw std::runtime_error("Missing real marked-end cache partial stall");
}

void finish_marked_tail(lead_a1::Sharc &cpu,
                        lead_a1::SharcFetch &fetch,
                        lead_a1::SharcMemory &memory) {
  require(cpu.step(fetch).pc == tail_end && !cpu.state().arithmetic_loop);
  const auto flow = fetch.state().flow;
  require(cpu.state().pc == tail_end + 1 && flow && flow->discard_decode == 1);
  unsigned int post{};
  constexpr unsigned int limit = 8;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != tail_end + 2; ++clock) {
    static_cast<void>(advance(cpu, fetch));
    if (fetch.state().execute) {
      const auto step = cpu.step(fetch);
      require(step.pc == tail_end + 1 && step.opcode == 0);
      ++post;
    }
  }
  constexpr std::uint32_t parent = 0x10c0070;
  require(post == 1 && cpu.state().pc == tail_end + 2 && cpu.state().depth == 1 &&
          cpu.state().loops_empty == true && cpu.state().pc_stack[0] == parent &&
          cpu.state().data[0][5].value == 0);
  require(memory.read({.address = tail_data, .width = lead_a1::SharcWidth::wide48}) == payload);
}

void marked_tail_miss_and_failure(bool read_only) {
  lead_a1::SharcMemory memory;
  tail_program(memory, !read_only);
  lead_a1::Sharc cpu(memory, tail_initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  issue_marked_tail(cpu, fetch);
  if (!read_only) {
    finish_marked_tail(cpu, fetch, memory);
    return;
  }
  const auto native = cpu.state();
  const auto transport = fetch.state();
  try {
    static_cast<void>(cpu.step(fetch));
  } catch (const std::exception &error) {
    require(std::string(error.what()).find("Read-only SHARC region") != std::string::npos);
    require(cpu.state() == native && fetch.state() == transport && !memory.active());
    require(memory.read({.address = tail_data, .width = lead_a1::SharcWidth::wide48}) == 0);
    lead_a1::SharcMemory repaired_memory;
    tail_program(repaired_memory, true);
    lead_a1::Sharc repaired(repaired_memory, native);
    lead_a1::SharcFetch repaired_fetch(repaired_memory, transport);
    finish_marked_tail(repaired, repaired_fetch, repaired_memory);
    return;
  }
  throw std::runtime_error("Missing marked-end read-only store failure");
}

}

int main() try {
  miss_columns_and_real_stores();
  held_condition_is_not_resampled();
  unfilled_cache_modes();
  frozen_hit_executes_cached_word();
  pending_miss_termination_preserves_state();
  marked_tail_miss_and_failure(false);
  marked_tail_miss_and_failure(true);
  constexpr unsigned int checkpoints = 24;
  for (unsigned int checkpoint = 0; checkpoint < checkpoints; ++checkpoint) {
    for (const bool occupied : {false, true}) { rebound_cache_transfer(checkpoint, occupied); }
  }
  std::cout << "Arithmetic loop and actual PM cache-miss columns/stores pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
