// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t target = base + 96;
constexpr std::uint32_t full = 0xffffffff;
constexpr std::uint32_t marker = 0x12345678;
constexpr std::uint64_t flush = 0x170200000000;
constexpr std::uint64_t pm_read = 0x120000000000 | target;
constexpr std::uint32_t disabled = 0x10;
constexpr std::uint32_t frozen = 0x80000;
constexpr std::uint32_t external_disabled = 0x40;
constexpr std::uint32_t mode_mask = disabled | frozen | external_disabled;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC cache expectation at line " + std::to_string(where.line()));
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
  throw std::runtime_error("Missing SHARC cache rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, unsigned int offset, std::uint64_t opcode) {
  memory.write({.address = base + offset, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 384;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned cache program and data"});
  code(memory, 0, flush);
  code(memory, 1, 0);
  code(memory, 2, 0);
  memory.write({.address = target}, marker);
}

lead_a1::SharcCacheState populated() {
  lead_a1::SharcCacheState cache;
  constexpr std::uint32_t valid = 0xa55a3cc3;
  constexpr std::uint32_t replacement = 0xa5a5;
  constexpr std::uint32_t lru_mask = 0xffff;
  cache.valid = {.value = valid, .known = full};
  cache.lru = {.value = replacement, .known = lru_mask};
  for (unsigned int entry = 0; entry < cache.addresses.size(); ++entry) {
    constexpr std::uint32_t address_knowledge = 0xffff0f;
    const auto address = base + (entry / 2) + ((entry & 1) * 16);
    cache.addresses[entry] = {.value = address & address_knowledge, .known = address_knowledge};
    constexpr std::uint64_t instruction_knowledge = 0xffff00ffff;
    constexpr std::uint64_t opaque_word = 0x123456789abc;
    cache.instructions[entry] = {.value = (opaque_word + entry) & instruction_knowledge,
                                 .known = instruction_knowledge};
  }
  return cache;
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.cache = populated();
  return state;
}

void validity_and_latency() {
  for (unsigned int controls = 0; controls < 8; ++controls) {
    for (const bool unknown : {false, true}) {
      lead_a1::SharcMemory memory;
      program(memory);
      auto state = initial();
      state.system[4].value |= ((controls & 1) != 0 ? disabled : 0) |
                               ((controls & 2) != 0 ? frozen : 0) |
                               ((controls & 4) != 0 ? external_disabled : 0);
      if (unknown) {
        state.system[4].known &= ~mode_mask;
        state.system[4].value &= state.system[4].known;
        state.cache.valid = {}; // Unknown membership must become known invalid.
      }
      state.effective = state.system;
      lead_a1::Sharc cpu(memory, state);
      require(cpu.step().minimum_core_cycles == 1);
      auto pending = state.cache;
      pending.pending_flush = true;
      require(cpu.state().cache == pending && cpu.state().system == state.system);
      lead_a1::Sharc restored(memory, cpu.state());
      require(restored.step().minimum_core_cycles == 1);
      auto invalidated = state.cache;
      invalidated.valid = {.known = full};
      require(restored.state().cache == invalidated);
      require(restored.state().data == state.data && restored.state().dag == state.dag);
      static_cast<void>(cpu.step());
      require(cpu.state() == restored.state());
    }
  }
}

void repeated_flush_and_simd() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, 1, flush);
  auto state = initial();
  constexpr std::uint32_t simd = 0x200000;
  state.system[2].value = simd;
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(cpu.state().cache.valid == (lead_a1::SharcKnown32{.known = full}));
  require(cpu.state().cache.pending_flush);
  static_cast<void>(cpu.step());
  require(!cpu.state().cache.pending_flush && cpu.state().cache.valid.known == full);
  require(cpu.state().cache.addresses == state.cache.addresses &&
          cpu.state().cache.instructions == state.cache.instructions &&
          cpu.state().cache.lru == state.cache.lru);
  require(cpu.state().data == state.data && cpu.state().system == state.system);
}

void failed_follower_and_repair() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t unmapped_dm = 0x100000002004;
  code(memory, 1, unmapped_dm);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  static_cast<void>(cpu.step());
  const auto before = cpu.state();
  const auto before_control = control.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Unmapped SHARC IOP");
  require(cpu.state() == before && control.state() == before_control && !memory.active());
  code(memory, 1, 0);
  static_cast<void>(cpu.step());
  require(cpu.state().cache.valid.known == full && cpu.state().cache.valid.value == 0 &&
          !cpu.state().cache.pending_flush);
}

void prohibited_pm_followers() {
  constexpr std::array<std::uint64_t, 5> instructions{
      pm_read,
      0x130000000000 | target,
      0x613e00000000,
      0x403f00000000,
      0x400100000000}; // Last is a false EQ PM encoding; the restriction still applies.
  for (const auto opcode : instructions) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, 1, opcode);
    lead_a1::Sharc cpu(memory, initial());
    static_cast<void>(cpu.step());
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(cpu.step()); }, "immediately after FLUSH CACHE prohibited");
    require(cpu.state() == before && !memory.active());
    require(memory.read({.address = target}) == marker);
  }
}

void program_access_knowledge() {
  for (const auto mode : {0U, disabled, frozen}) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, 0, pm_read);
    auto state = initial();
    state.system[4].value |= mode;
    state.effective = state.system;
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, state, &control);
    static_cast<void>(cpu.step());
    require(cpu.state().data[0][0].value == std::uint64_t{marker} * 256);
    auto expected = state.cache;
    if (mode == 0) { expected = {}; }
    if (mode == frozen) { expected.lru = {}; }
    require(cpu.state().cache == expected);
    if (mode == 0) {
      require(cpu.state().cache.valid.known == 0); // Not a falsely known empty cache.
    }
  }
}

void reset_and_invalid_state() {
  const auto reset = lead_a1::Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t lru_mask = 0xffff;
  require(reset.cache.valid == (lead_a1::SharcKnown32{.known = full}));
  require(reset.cache.lru == (lead_a1::SharcKnown32{.known = lru_mask}));
  lead_a1::SharcMemory memory;
  program(memory);
  auto state = initial();
  state.cache.valid = {.value = 1};
  rejected([&] { const lead_a1::Sharc cpu(memory, state); }, "cache validity/LRU knowledge");
  state = initial();
  state.cache.addresses[0].value |= 1;
  rejected([&] { const lead_a1::Sharc cpu(memory, state); }, "cache address/instruction knowledge");
  state = initial();
  constexpr std::uint64_t reserved_bit = std::uint64_t{1} << 63;
  state.cache.instructions[0].known |= reserved_bit;
  rejected([&] { const lead_a1::Sharc cpu(memory, state); }, "cache address/instruction knowledge");
}

void stack_and_reserved_guards() {
  constexpr std::array<std::uint64_t, 3> unsupported{
      0x170200000001, 0x170600000000, 0x178200000000};
  for (const auto opcode : unsupported) {
    lead_a1::SharcMemory memory;
    program(memory);
    code(memory, 0, opcode);
    lead_a1::Sharc cpu(memory, initial());
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(cpu.step()); }, "cache/stack push-pop/reserved");
    require(cpu.state() == before && !memory.active());
  }
}

}

int main() {
  try {
    validity_and_latency();
    repeated_flush_and_simd();
    failed_follower_and_repair();
    prohibited_pm_followers();
    program_access_knowledge();
    reset_and_invalid_state();
    stack_and_reserved_guards();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
