// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_boot.hpp"
#include "lead_a1/sharc_control.hpp"
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
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t increment_r0 = 0x013e00029000;
constexpr std::uint64_t increment_r1 = 0x013e00029110;
constexpr std::uint64_t decrement_r0 = 0x013e0002a000;
constexpr std::uint64_t decrement_r1 = 0x013e0002a110;
constexpr std::uint32_t cadis = 0x10;
constexpr lead_a1::SharcKnown32 mode2{.value = cadis, .known = full32};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC fetch expectation at line " + std::to_string(where.line()));
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
  throw std::runtime_error("Missing SHARC fetch rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t pc, std::uint64_t opcode) {
  memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, std::uint64_t first = increment_r0) {
  constexpr unsigned int cells = 256;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned native fetch program"});
  code(memory, base, first);
  code(memory, base + 1, increment_r1);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.system[4].value |= cadis;
  state.effective = state.system;
  constexpr std::uint64_t ten = 10;
  constexpr std::uint64_t twenty = 20;
  state.data[0][0] = {.value = ten * 256, .known = full40};
  state.data[0][1] = {.value = twenty * 256, .known = full40};
  return state;
}

std::uint64_t value(const lead_a1::Sharc &cpu, unsigned int reg) {
  require(cpu.state().data[0][reg].known == full40);
  return cpu.state().data[0][reg].value >> 8;
}

void to_execute(lead_a1::SharcFetch &fetch) {
  fetch.advance(mode2, base);
  for (unsigned int clock = 0; clock < 4; ++clock) { fetch.advance(mode2, {}); }
  require(fetch.state().execute.has_value());
}

void linear_pipeline_table() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::array<std::uint64_t, 6> words{
      increment_r0, increment_r1, 0, increment_r0, 0, increment_r1};
  for (unsigned int word = 0; word < words.size(); ++word) {
    code(memory, base + word, words[word]);
  }
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory);
  unsigned int retired{};
  for (unsigned int clock = 0; clock < words.size() + 4; ++clock) {
    const auto before = cpu.state();
    const std::optional<std::uint32_t> next =
        clock < words.size() ? std::optional{base + clock} : std::nullopt;
    fetch.advance(cpu.state().effective[4], next);
    const auto &state = fetch.state();
    require(state.fetch1 == next);
    const std::array stages{state.fetch2, state.decode, state.address, state.execute};
    for (unsigned int index = 0; index < stages.size(); ++index) {
      const auto distance = index + 1;
      const bool occupied = clock >= distance && clock - distance < words.size();
      const auto phase = stages[index];
      require(phase.has_value() == occupied);
      if (phase) {
        require(phase->address == base + clock - distance);
        require(phase->opcode == words[clock - distance]);
      }
    }
    if (state.execute) {
      const auto expected = *state.execute;
      const auto event = cpu.step(fetch);
      require(event.pc == expected.address && event.opcode == expected.opcode);
      require(event.next_pc == expected.address + 1 && !fetch.state().execute);
      ++retired;
    } else {
      require(cpu.state() == before);
    }
  }
  require(retired == words.size() && fetch.state() == lead_a1::SharcFetchState{});
  constexpr std::uint64_t twelve = 12;
  constexpr std::uint64_t twenty_two = 22;
  require(value(cpu, 0) == twelve && value(cpu, 1) == twenty_two);
}

void send_word(lead_a1::SharcSpi &spi, lead_a1::SharcMemory &memory, std::uint32_t word) {
  spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  for (unsigned int bit = 0; bit < 32; ++bit) {
    spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
    spi.observe(memory, {.selected = true, .clock = true, .mosi = ((word >> bit) & 1) != 0});
  }
}

void overwrite_through_spi(lead_a1::SharcMemory &memory) {
  constexpr std::uint32_t completed_kernel = 0x92180;
  lead_a1::SharcSpi spi(lead_a1::SharcBootState{.clock = true, .next_word = completed_kernel});
  spi.write({.address = 0, .size = 4}, base);
  spi.write({.address = 8, .size = 4}, 3);
  // Three real normal32 DMA writes alias two adjacent wide48 instruction words.
  constexpr std::uint64_t low16 = 0xffff;
  constexpr std::array<std::uint32_t, 3> packed{
      static_cast<std::uint32_t>(decrement_r0 & full32),
      static_cast<std::uint32_t>((decrement_r0 >> 32) | ((decrement_r1 & low16) << 16)),
      static_cast<std::uint32_t>(decrement_r1 >> 16)};
  for (const auto word : packed) { send_word(spi, memory, word); }
  require(spi.dma_request() == packed.size());
  require(spi.service_dma(memory) == packed.size() && spi.state().interrupt_pending);
  require(memory.read({.address = base, .width = lead_a1::SharcWidth::wide48}) == decrement_r0);
  require(memory.read({.address = base + 1, .width = lead_a1::SharcWidth::wide48}) == decrement_r1);
}

void dma_capture_boundary_and_checkpoint() {
  // F1 alone has not read data. F2/Decode/Address/Execute retain the old data.
  for (unsigned int clocks_before_dma = 1; clocks_before_dma <= 5; ++clocks_before_dma) {
    lead_a1::SharcMemory memory;
    program(memory);
    lead_a1::Sharc cpu(memory, initial());
    lead_a1::SharcFetch fetch(memory);
    fetch.advance(mode2, base);
    for (unsigned int clock = 1; clock < clocks_before_dma; ++clock) { fetch.advance(mode2, {}); }
    const auto held = fetch.state();
    const auto core = cpu.state();
    overwrite_through_spi(memory);
    require(fetch.state() == held && cpu.state() == core);
    lead_a1::SharcFetch restored(memory, held);
    lead_a1::Sharc continued(memory, core);
    for (unsigned int clock = clocks_before_dma; clock < 5; ++clock) {
      fetch.advance(mode2, {});
      restored.advance(mode2, {});
    }
    const auto expected = clocks_before_dma == 1 ? decrement_r0 : increment_r0;
    require(cpu.step(fetch).opcode == expected && continued.step(restored).opcode == expected);
    require(cpu.state() == continued.state() && fetch.state() == restored.state());
    constexpr std::uint64_t nine = 9;
    constexpr std::uint64_t eleven = 11;
    require(value(cpu, 0) == (clocks_before_dma == 1 ? nine : eleven));

    lead_a1::Sharc fresh(memory, initial());
    lead_a1::SharcFetch refetched(memory);
    to_execute(refetched);
    require(fresh.step(refetched).opcode == decrement_r0 && value(fresh, 0) == nine);
  }
}

void failed_instruction_keeps_captured_word() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t read_iop = 0x100000002004;
  program(memory, read_iop);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory);
  to_execute(fetch);
  const auto core = cpu.state();
  const auto held = fetch.state();
  const auto device = control.state();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "Unmapped SHARC IOP");
  require(cpu.state() == core && fetch.state() == held && control.state() == device);
  require(!memory.active());
  code(memory, base, 0); // A RAM repair cannot change the instruction already in Execute.
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "opcode=0x100000002004");
  require(cpu.state() == core && fetch.state() == held && !memory.active());
  // Explicit owned mapping, not a claim about physical IOP0x2004.
  constexpr std::uint32_t owned_iop = 0x2004;
  memory.map_io({.first_word = owned_iop, .words = 1}, control);
  require(cpu.step(fetch).opcode == read_iop && value(cpu, 0) == 4);
  require(!fetch.state().execute && cpu.state().pc == base + 1);
}

void two_inflight_words_capture_separately() {
  for (const bool second_captured : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory);
    lead_a1::Sharc cpu(memory, initial());
    lead_a1::SharcFetch fetch(memory);
    fetch.advance(mode2, base);
    fetch.advance(mode2, base + 1); // First data already in F2; second address only in F1.
    if (second_captured) { fetch.advance(mode2, {}); }
    const auto before = fetch.state();
    overwrite_through_spi(memory);
    require(fetch.state() == before);
    for (unsigned int clock = second_captured ? 3U : 2U; clock < 5; ++clock) {
      fetch.advance(mode2, {});
    }
    require(cpu.step(fetch).opcode == increment_r0);
    fetch.advance(mode2, {});
    const auto expected_second = second_captured ? increment_r1 : decrement_r1;
    require(cpu.step(fetch).opcode == expected_second);
    constexpr std::uint64_t eleven = 11;
    constexpr std::uint64_t nineteen = 19;
    constexpr std::uint64_t twenty_one = 21;
    require(value(cpu, 0) == eleven);
    require(value(cpu, 1) == (second_captured ? twenty_one : nineteen));
    require(cpu.state().pc == base + 2 && fetch.state() == lead_a1::SharcFetchState{});
  }
}

void retirement_identity_and_backpressure() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory);
  const auto core = cpu.state();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "missing fetched word");
  require(cpu.state() == core && fetch.state() == lead_a1::SharcFetchState{});
  to_execute(fetch);
  const auto held = fetch.state();
  rejected([&] { fetch.advance(mode2, {}); }, "Execute word has not retired");
  require(fetch.state() == held);
  auto other_pc = initial();
  ++other_pc.pc;
  lead_a1::Sharc wrong_pc(memory, other_pc);
  const auto wrong = wrong_pc.state();
  rejected([&] { static_cast<void>(wrong_pc.step(fetch)); }, "wrong memory binding, PC");
  require(wrong_pc.state() == wrong && fetch.state() == held);
  lead_a1::SharcMemory other;
  program(other);
  lead_a1::Sharc wrong_memory(other, initial());
  rejected([&] { static_cast<void>(wrong_memory.step(fetch)); }, "wrong memory binding, PC");
  require(wrong_memory.state() == core && fetch.state() == held);
  memory.begin_instruction();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "another memory transaction");
  require(memory.active() && cpu.state() == core && fetch.state() == held);
  memory.abort_instruction();
  require(cpu.step(fetch).opcode == increment_r0 && !fetch.state().execute);
}

void failed_fetch_and_profile_guards() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcFetch fetch(memory);
  constexpr std::uint32_t unknown = base + 8;
  fetch.advance(mode2, unknown);
  const auto held = fetch.state();
  rejected([&] { fetch.advance({}, {}); }, "cache lookup/unknown CADIS");
  rejected([&] { fetch.advance({.value = cadis | 1, .known = cadis}, {}); },
           "Invalid SHARC fetch MODE2 knowledge");
  rejected([&] { fetch.advance(mode2, 0x100000); }, "internal native address");
  rejected([&] { fetch.advance(mode2, {}); }, "Unknown SHARC memory bits");
  require(fetch.state() == held && !memory.active());
  memory.begin_instruction();
  rejected([&] { fetch.advance(mode2, {}); }, "during a memory transaction");
  require(memory.active() && fetch.state() == held);
  memory.abort_instruction();
  code(memory, unknown, 0);
  fetch.advance(mode2, {});
  require(fetch.state().fetch2 == (lead_a1::SharcFetchWord{.address = unknown, .opcode = 0}));
  auto invalid = fetch.state();
  constexpr std::uint64_t beyond_native = std::uint64_t{1} << 48;
  invalid.fetch2 = lead_a1::SharcFetchWord{.address = unknown, .opcode = beyond_native};
  rejected([&] { const lead_a1::SharcFetch bad(memory, invalid); }, "Invalid SHARC native");
  invalid = fetch.state();
  invalid.fetch2 = lead_a1::SharcFetchWord{.address = 0x100000, .opcode = 0};
  rejected([&] { const lead_a1::SharcFetch bad(memory, invalid); }, "VISA word outside its IAB");
}

void executable_permission_and_unmapped_bus() {
  lead_a1::SharcMemory memory;
  constexpr std::array<lead_a1::SharcCell, 3> cells{lead_a1::SharcCell{.known = 0xffff},
                                                    lead_a1::SharcCell{.known = 0xffff},
                                                    lead_a1::SharcCell{.known = 0xffff}};
  memory.map({.bank = 2,
              .halfwords = cells.size(),
              .name = "owned non-executable data",
              .executable = false},
             cells);
  lead_a1::SharcFetch fetch(memory);
  fetch.advance(mode2, base);
  const auto held = fetch.state();
  rejected([&] { fetch.advance(mode2, {}); }, "Non-executable SHARC physical region");
  require(fetch.state() == held && !memory.active());
  lead_a1::SharcMemory unmapped;
  lead_a1::SharcFetch missing(unmapped);
  missing.advance(mode2, base);
  const auto before = missing.state();
  rejected([&] { missing.advance(mode2, {}); }, "word-address=0xc0000 bits=48");
  require(missing.state() == before && !unmapped.active());
}

}

int main() {
  try {
    linear_pipeline_table();
    dma_capture_boundary_and_checkpoint();
    failed_instruction_keeps_captured_word();
    two_inflight_words_capture_separately();
    retirement_identity_and_backpressure();
    failed_fetch_and_profile_guards();
    executable_permission_and_unmapped_bus();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
