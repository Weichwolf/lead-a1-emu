// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_boot.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_spi.hpp"

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
constexpr std::uint32_t data = 0x80010;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint32_t tag_bits = 0xffffff;
constexpr std::uint64_t native_bits = 0xffffffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t full64 = 0xffffffffffffffff;
constexpr std::uint64_t store_px = 0x136b00000000;
constexpr std::uint64_t increment = 0x013e00029000;
constexpr std::uint64_t decrement = 0x013e0002a000;
constexpr std::uint64_t increment_r1 = 0x013e00029110;
constexpr std::uint64_t payload = 0x123456789abc;
constexpr std::uint32_t disabled = 0x10;
constexpr std::uint32_t frozen = 0x80000;
constexpr unsigned int target_entry = 6;
constexpr unsigned int target_set = 3;
constexpr std::uint64_t initial_r1 = 20;
using Bus = lead_a1::SharcFetchBus;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC fetch/cache expectation at line " +
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
  throw std::runtime_error("Missing SHARC fetch/cache rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t pc, std::uint64_t opcode) {
  memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, std::uint32_t start = base) {
  constexpr unsigned int cells = 512;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned cached instruction program"});
  memory.map({.bank = 0, .halfwords = cells, .name = "owned actual PM data target"});
  for (unsigned int word = 0; word < 16; ++word) { code(memory, start + word, 0); }
  code(memory, start, store_px | data);
  code(memory, start + 3, increment);
}

lead_a1::SharcState initial(std::uint32_t start = base) {
  auto state = lead_a1::Sharc::reset_reference_214xx(start);
  state.data[0][0] = {.value = std::uint64_t{10} * 256, .known = full40};
  state.data[0][1] = {.value = initial_r1 * 256, .known = full40};
  state.px = {.value = payload << 16, .known = full64};
  return state;
}

lead_a1::SharcFetchEvent tick(lead_a1::SharcFetch &fetch,
                              lead_a1::Sharc &cpu,
                              std::optional<std::uint32_t> next,
                              Bus bus = Bus::available) {
  const auto event = fetch.advance(cpu, next, bus);
  if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
  return event;
}

void prime(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu, std::uint32_t start = base) {
  const auto before = cpu.state();
  for (unsigned int word = 0; word < 3; ++word) {
    require(tick(fetch, cpu, start + word).address_accepted);
  }
  require(cpu.state() == before);
}

const lead_a1::SharcFetchLookup &lookup(const lead_a1::SharcFetch &fetch) {
  const auto &value = fetch.state().lookup;
  if (!value) { throw std::runtime_error("Missing owned cache lookup"); }
  return *value;
}

const lead_a1::SharcFetchWord &fetch2(const lead_a1::SharcFetch &fetch) {
  const auto &value = fetch.state().fetch2;
  if (!value) { throw std::runtime_error("Missing owned F2 data"); }
  return *value;
}

void tag(unsigned int entry, lead_a1::SharcState &state, std::uint32_t address) {
  state.cache.valid.value |= std::uint32_t{1} << entry;
  state.cache.addresses[entry] = {.value = address, .known = tag_bits};
  state.cache.instructions[entry] = {.value = decrement, .known = native_bits};
}

void miss_phases_and_modes() {
  for (const auto mode : {0U, disabled, frozen, disabled | frozen}) {
    lead_a1::SharcMemory memory;
    program(memory);
    auto state = initial();
    state.system[4].value |= mode;
    state.effective = state.system;
    lead_a1::Sharc cpu(memory, state);
    lead_a1::SharcFetch fetch(memory);
    prime(fetch, cpu);
    const auto miss = tick(fetch, cpu, base + 3, Bus::pm_data);
    require(miss.address_accepted && !miss.cache_hit && !miss.stalled);
    require(cpu.state().cache.valid.value == 0 && lookup(fetch).wait);
    const auto stall = tick(fetch, cpu, base + 4);
    require(stall.stalled && !stall.address_accepted && fetch.state().fetch1 == base + 3);
    require(cpu.state().pc == base + 1); // The real PM instruction retires during the stall.
    require(memory.read({.address = data, .width = lead_a1::SharcWidth::wide48}) == payload);
    const bool fills = mode == 0;
    require(cpu.state().cache.valid.value == (fills ? 1U << target_entry : 0U));
    require(cpu.state().cache.valid.known == full32);
    require(cpu.state().cache.instructions[target_entry].known == 0);
    if (fills) { require(cpu.state().cache.addresses[target_entry].value == base + 3); }
    const auto held = cpu.state();
    require(tick(fetch, cpu, base + 4).address_accepted); // Retry the unaccepted address.
    require(fetch2(fetch).opcode == increment && cpu.state() == held);
    require(cpu.state().cache.instructions[target_entry].known == 0); // F2, before Decode.
    static_cast<void>(tick(fetch, cpu, base + 5));
    if (fills) {
      require(cpu.state().cache.instructions[target_entry] ==
              (lead_a1::SharcKnown64{.value = increment, .known = native_bits}));
    }
    static_cast<void>(tick(fetch, cpu, base + 6));
    static_cast<void>(tick(fetch, cpu, base + 7));
    require(cpu.state().data[0][0].value == std::uint64_t{11} * 256 && cpu.state().pc == base + 4);
    require(cpu.state().cache.lru.value == (fills ? 1U << target_set : 0U));
  }
}

void send_word(lead_a1::SharcSpi &spi, lead_a1::SharcMemory &memory, std::uint32_t value) {
  spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  for (unsigned int bit = 0; bit < 32; ++bit) {
    spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
    spi.observe(memory, {.selected = true, .clock = true, .mosi = ((value >> bit) & 1) != 0});
  }
}

void replace_ram(lead_a1::SharcMemory &memory) {
  constexpr std::uint32_t completed_kernel = 0x92180;
  lead_a1::SharcSpi spi(lead_a1::SharcBootState{.clock = true, .next_word = completed_kernel});
  spi.write({.address = 0, .size = 4}, base + 3); // normal32 cells6..11 = native words2/3.
  spi.write({.address = 8, .size = 4}, 3);
  constexpr std::uint64_t low16 = 0xffff;
  constexpr std::array<std::uint32_t, 3> words{
      0,
      static_cast<std::uint32_t>((increment_r1 & low16) << 16),
      static_cast<std::uint32_t>(increment_r1 >> 16)};
  for (const auto word : words) { send_word(spi, memory, word); }
  require(spi.service_dma(memory) == words.size() && spi.state().interrupt_pending);
  require(memory.read({.address = base + 3, .width = lead_a1::SharcWidth::wide48}) == increment_r1);
}

void cache_hit_survives_dma() {
  for (const bool freeze : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory);
    auto state = initial();
    state.system[4].value |= freeze ? frozen : 0;
    state.effective = state.system;
    tag(target_entry, state, base + 3);
    state.cache.lru.known &= ~(1U << target_set); // A hit establishes this set's recency.
    lead_a1::Sharc cpu(memory, state);
    lead_a1::SharcFetch fetch(memory);
    prime(fetch, cpu);
    const auto hit = tick(fetch, cpu, base + 3, Bus::pm_data);
    require(hit.cache_hit && !hit.stalled && lookup(fetch).hit);
    const auto captured = fetch.state();
    const auto core = cpu.state();
    replace_ram(memory);
    require(fetch.state() == captured && cpu.state() == core);
    lead_a1::Sharc restored(memory, core);
    lead_a1::SharcFetch continued(memory, captured);
    for (unsigned int word = 4; word < 8; ++word) {
      require(tick(fetch, cpu, base + word) == tick(continued, restored, base + word));
      require(cpu.state() == restored.state() && fetch.state() == continued.state());
    }
    require(cpu.state().data[0][0].value == std::uint64_t{9} * 256 &&
            cpu.state().data[0][1].value == initial_r1 * 256);
    require(cpu.state().cache.instructions[target_entry].value == decrement);
    require(cpu.state().cache.lru.value == 1U << target_set);
    constexpr std::uint32_t lru_bits = 0xffff;
    require(cpu.state().cache.lru.known == lru_bits);
  }
}

void miss_hit_load_bus_forwarding() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base + 1, store_px | data); // Real second Address-stage PM request.
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory);
  prime(fetch, cpu);
  require(!tick(fetch, cpu, base + 3, Bus::pm_data).cache_hit);
  require(tick(fetch, cpu, base + 3).stalled);
  require(cpu.state().cache.instructions[target_entry].known == 0);
  // Table4-33: same F1 address hits before its instruction reaches the cache data array.
  require(tick(fetch, cpu, base + 3, Bus::pm_data).cache_hit);
  require(cpu.state().cache.instructions[target_entry].known == 0 && fetch.state().fill);
  require(fetch2(fetch).opcode == increment);
  replace_ram(memory);
  const auto saved_cpu = cpu.state();
  const auto saved_fetch = fetch.state();
  lead_a1::Sharc restored(memory, saved_cpu);
  lead_a1::SharcFetch continued(memory, saved_fetch);
  const auto forwarded = tick(fetch, cpu, base + 4);
  require(forwarded.forwarded && !forwarded.stalled);
  require(tick(continued, restored, base + 4) == forwarded);
  require(cpu.state() == restored.state() && fetch.state() == continued.state());
  require(fetch2(fetch).opcode == increment);
  require(cpu.state().cache.instructions[target_entry].value == increment);
  static_cast<void>(tick(fetch, cpu, base + 5));
  static_cast<void>(tick(fetch, cpu, base + 6));
  require(cpu.state().data[0][0].value == std::uint64_t{11} * 256 && cpu.state().pc == base + 4);
  // The duplicated fetch tests the bus forwarding, not an invented architectural loop.
}

void native_self_modification_uses_selected_source() {
  for (const bool hit : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, base, store_px | (base + 3)); // The actual guest overwrites its later instruction.
    auto state = initial();
    state.px = {.value = decrement << 16, .known = full64};
    if (hit) {
      tag(target_entry, state, base + 3);
      state.cache.instructions[target_entry] = {.value = increment, .known = native_bits};
    }
    lead_a1::Sharc cpu(memory, state);
    lead_a1::SharcFetch fetch(memory);
    prime(fetch, cpu);
    require(tick(fetch, cpu, base + 3, Bus::pm_data).cache_hit == hit);
    if (!hit) { require(tick(fetch, cpu, base + 4).stalled); }
    for (unsigned int word = 4; word < 8; ++word) {
      static_cast<void>(tick(fetch, cpu, base + word));
    }
    require(memory.read({.address = base + 3, .width = lead_a1::SharcWidth::wide48}) == decrement);
    const auto expected = hit ? std::uint64_t{11} : std::uint64_t{9};
    require(cpu.state().data[0][0].value == expected * 256 && cpu.state().pc == base + 4);
    require(cpu.state().cache.instructions[target_entry].value == (hit ? increment : decrement));
  }
}

void replacement_all_sets_and_ways() {
  constexpr unsigned int sets = 16;
  for (unsigned int offset = 0; offset < sets; ++offset) {
    for (unsigned int way = 0; way < 2; ++way) {
      lead_a1::SharcMemory memory;
      const auto start = base + offset;
      program(memory, start);
      auto state = initial(start);
      const auto set = (offset + 3) % sets;
      const auto first = set * 2;
      const auto mask = std::uint32_t{1} << set;
      state.cache.lru.value = way == 0 ? 0 : mask;
      constexpr std::uint32_t other_tag = 0x100;
      tag(first, state, start + 3 + other_tag);
      tag(first + 1, state, start + 3 + (2 * other_tag));
      const auto untouched = state.cache.addresses[first + 1 - way];
      lead_a1::Sharc cpu(memory, state);
      lead_a1::SharcFetch fetch(memory);
      prime(fetch, cpu, start);
      static_cast<void>(tick(fetch, cpu, start + 3, Bus::pm_data));
      require(tick(fetch, cpu, start + 4).stalled);
      require(cpu.state().cache.addresses[first + way].value == start + 3);
      require(cpu.state().cache.addresses[first + 1 - way] == untouched);
      require(cpu.state().cache.instructions[first + way].value == decrement); // Old data until D.
      static_cast<void>(tick(fetch, cpu, start + 4));
      static_cast<void>(tick(fetch, cpu, start + 5));
      require(cpu.state().cache.instructions[first + way].value == increment);
      require(cpu.state().cache.instructions[first + 1 - way].value == decrement);
      require(cpu.state().cache.lru.value == (way == 0 ? mask : 0));
    }
  }
}

enum class Unknown : std::uint8_t { membership, duplicate, data, replacement };

void unknown_cache_preserves_state() {
  for (const auto problem :
       {Unknown::membership, Unknown::duplicate, Unknown::data, Unknown::replacement}) {
    lead_a1::SharcMemory memory;
    program(memory);
    auto state = initial();
    switch (problem) {
      case Unknown::membership: state.cache.valid.known &= ~(1U << target_entry); break;
      case Unknown::duplicate:
        tag(target_entry + 1, state, base + 3);
        tag(target_entry, state, base + 3);
        break;
      case Unknown::data:
        tag(target_entry, state, base + 3);
        state.cache.instructions[target_entry] = {};
        break;
      case Unknown::replacement: state.cache.lru.known &= ~(1U << target_set); break;
      default: throw std::logic_error("Invalid owned unknown cache case");
    }
    lead_a1::Sharc cpu(memory, state);
    lead_a1::SharcFetch fetch(memory);
    prime(fetch, cpu);
    const bool late = problem == Unknown::data || problem == Unknown::replacement;
    if (late) { static_cast<void>(tick(fetch, cpu, base + 3, Bus::pm_data)); }
    const auto core = cpu.state();
    const auto held = fetch.state();
    rejected(
        [&] {
          static_cast<void>(
              fetch.advance(cpu, late ? base + 4 : base + 3, late ? Bus::available : Bus::pm_data));
        },
        "SHARC cache clock pc=");
    require(cpu.state() == core && fetch.state() == held && !memory.active());
  }
}

void ownership_and_effect_guards() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory);
  const auto core = cpu.state();
  rejected([&] { static_cast<void>(fetch.advance(cpu, base, Bus::pm_data)); }, "Address-stage PM");
  rejected([&] { static_cast<void>(fetch.advance(cpu, base, Bus::block_conflict)); },
           "block-conflict arbitration");
  require(cpu.state() == core && fetch.state() == lead_a1::SharcFetchState{});
  auto pending = initial();
  pending.pending[2] = pending.system[2];
  lead_a1::Sharc delayed(memory, pending);
  const auto before = delayed.state();
  rejected([&] { static_cast<void>(fetch.advance(delayed, base)); }, "bubble control-effect clock");
  require(delayed.state() == before && fetch.state() == lead_a1::SharcFetchState{});
  prime(fetch, cpu);
  static_cast<void>(tick(fetch, cpu, base + 3)); // Owner omitted the actual PM request.
  static_cast<void>(fetch.advance(cpu, base + 4));
  const auto held = fetch.state();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "PM request disagrees with retirement");
  require(cpu.state() == core && fetch.state() == held && !memory.active());
}

void nonconflicting_and_disabled_bypass() {
  for (const bool disable : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory);
    auto state = initial();
    if (disable) {
      state.system[4].value |= disabled;
      state.effective = state.system;
      state.cache.valid = {};
      state.cache.lru = {};
    } else {
      code(memory, base, 0); // No actual PM data request; internal cache is not consulted.
      tag(target_entry, state, base + 3);
    }
    lead_a1::Sharc cpu(memory, state);
    lead_a1::SharcFetch fetch(memory);
    const auto untouched = cpu.state().cache;
    prime(fetch, cpu);
    require(!tick(fetch, cpu, base + 3, disable ? Bus::pm_data : Bus::available).cache_hit);
    if (disable) { require(tick(fetch, cpu, base + 4).stalled); }
    for (unsigned int word = 4; word < 8; ++word) {
      require(tick(fetch, cpu, base + word).address_accepted);
    }
    require(cpu.state().data[0][0].value == std::uint64_t{11} * 256);
    require(cpu.state().cache == untouched);
  }
}

void late_ram_failure_preserves_early_tag() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory);
  prime(fetch, cpu);
  constexpr std::uint32_t unknown = base + 16;
  static_cast<void>(tick(fetch, cpu, unknown, Bus::pm_data));
  require(tick(fetch, cpu, base + 4).stalled);
  const auto held = fetch.state();
  const auto core = cpu.state();
  rejected([&] { static_cast<void>(fetch.advance(cpu, base + 4)); }, "Unknown SHARC memory bits");
  require(cpu.state() == core && fetch.state() == held && !memory.active());
  require(cpu.state().cache.addresses[0].value == unknown && cpu.state().cache.valid.value == 1);
  require(memory.read({.address = data, .width = lead_a1::SharcWidth::wide48}) == payload);
  code(memory, unknown, increment);
  require(tick(fetch, cpu, base + 4).address_accepted);
  require(fetch2(fetch).opcode == increment && cpu.state() == core);
  static_cast<void>(tick(fetch, cpu, base + 5));
  require(cpu.state().cache.instructions[0].value == increment);
}

void pending_modes_and_invalid_restore() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory);
  prime(fetch, cpu);
  static_cast<void>(tick(fetch, cpu, base + 3, Bus::pm_data));
  const auto held = fetch.state();
  auto changed = cpu.state();
  changed.system[4].value |= disabled;
  changed.effective[4] = changed.system[4];
  lead_a1::Sharc other_mode(memory, changed);
  const auto before = other_mode.state();
  rejected([&] { static_cast<void>(fetch.advance(other_mode, base + 4)); },
           "control change during pending fetch");
  require(other_mode.state() == before && fetch.state() == held);
  rejected([&] { fetch.advance({.value = disabled, .known = full32}, base + 4); },
           "requires its bound owner");
  require(fetch.state() == held);
  auto invalid = held;
  constexpr unsigned int outside = 32;
  constexpr auto controls = disabled | frozen;
  invalid.lookup =
      lead_a1::SharcFetchLookup{.mode = {.known = controls}, .entry = outside, .hit = true};
  rejected([&] { const lead_a1::SharcFetch bad(memory, invalid); }, "cache lookup entry");
  invalid = held;
  invalid.fetch2 = lead_a1::SharcFetchWord{.address = base + 2, .opcode = 0, .pm_data = false};
  rejected([&] { const lead_a1::SharcFetch bad(memory, invalid); }, "PM request before Address");
}

}

int main() {
  try {
    miss_phases_and_modes();
    cache_hit_survives_dma();
    miss_hit_load_bus_forwarding();
    native_self_modification_uses_selected_source();
    replacement_all_sets_and_ways();
    unknown_cache_preserves_state();
    ownership_and_effect_guards();
    nonconflicting_and_disabled_bypass();
    late_ram_failure_preserves_early_tag();
    pending_modes_and_invalid_restore();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
