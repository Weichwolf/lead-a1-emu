// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_boot.hpp"
#include "sharc/peripherals/sharc_spi.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t vector = base + 0x30;
constexpr std::uint32_t foreground = base + 96;
constexpr std::uint32_t data_address = base + 160;
constexpr std::uint32_t p1i = 0x1000;
constexpr std::uint32_t global = 0x1000;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t idle = 0x008000000000;
constexpr std::uint64_t rti = 0x0b3e00000000;
constexpr std::uint64_t increment_r0 = 0x013e00029000;
constexpr std::uint64_t enable_mode1 = 0x0f7200001000;
constexpr std::uint64_t initial_counter = 7;
constexpr std::uint64_t resumed_counter = 8;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC IDLE expectation at line " + std::to_string(where.line()));
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
  throw std::runtime_error("Missing SHARC IDLE rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t instruction) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, instruction);
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 1024;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned idle/DMA/vector/continuation"});
  code(memory, foreground, idle);
  code(memory, foreground + 1, increment_r0);
  code(memory, vector, rti);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(foreground);
  state.system[2].value |= global;
  state.system[11].value = p1i | 3;
  state.effective = state.system;
  state.data[0][0] = {.value = initial_counter * 256, .known = full40};
  return state;
}

void send_word(lead_a1::SharcSpi &spi, lead_a1::SharcMemory &memory, std::uint32_t value) {
  spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  for (unsigned int bit = 0; bit < 32; ++bit) {
    spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
    spi.observe(memory, {.selected = true, .clock = true, .mosi = ((value >> bit) & 1) != 0});
  }
}

void no_fictional_retirement_and_real_dma() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  require(cpu.step().opcode == idle);
  const auto held = cpu.state();
  require(held.idle_pc == foreground && held.pc == foreground + 1);
  for (unsigned int attempt = 0; attempt < 3; ++attempt) {
    rejected([&] { static_cast<void>(cpu.step()); }, "IDLE core requires an owner event");
    require(cpu.state() == held && !memory.active());
    require(!cpu.service_interrupt({.vector_base = base}) && cpu.state() == held);
  }
  lead_a1::Sharc restored(memory, held);
  // Explicit completed-bootstrap reference; actual serial/DMA transactions follow.
  constexpr std::uint32_t loaded_kernel_end = 0x92180;
  lead_a1::SharcSpi spi(lead_a1::SharcBootState{.clock = true, .next_word = loaded_kernel_end});
  spi.write({.address = 0, .size = 4}, data_address);
  spi.write({.address = 8, .size = 4}, 2);
  constexpr std::array<std::uint32_t, 2> payload{0x89abcdef, 0x01020304};
  send_word(spi, memory, payload[0]);
  require(spi.service_dma(memory) == 0 && !spi.state().interrupt_pending);
  require(restored.state() == held);
  send_word(spi, memory, payload[1]);
  require(spi.dma_request() == 2 && spi.service_dma(memory) == 2);
  require(spi.state().interrupt_pending && restored.state() == held);
  require(memory.read({.address = data_address}) == payload[0] &&
          memory.read({.address = data_address + 1}) == payload[1]);
  restored.latch_interrupt(p1i); // Owner routes the real DMA-completion event.
  spi.acknowledge_interrupt();   // Peripheral acknowledgement is separate from core recognition.
  const auto event = restored.service_interrupt({.vector_base = base});
  require(event && event->return_pc == foreground + 1);
  require(!restored.state().idle_pc && restored.state().pc == vector);
  require(restored.state().data[0][0].value == initial_counter * 256);
  static_cast<void>(restored.step());
  require(restored.state().pc == foreground + 1 && !restored.state().interrupt);
  static_cast<void>(restored.step());
  require(restored.state().data[0][0].value == resumed_counter * 256);
}

void masked_request_stays_latched() {
  lead_a1::SharcMemory memory;
  program(memory);
  auto state = initial();
  state.system[11].value = 3;
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  cpu.latch_interrupt(p1i);
  const auto before = cpu.state();
  require(!cpu.service_interrupt({.vector_base = base}) && cpu.state() == before);
  require(cpu.state().idle_pc == foreground && (cpu.state().system[10].value & p1i) != 0);
}

void committed_controls_and_cache_follower() {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, foreground, enable_mode1);
  code(memory, foreground + 1, idle);
  auto state = initial();
  state.system[2].value = 0;
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  cpu.latch_interrupt(p1i);
  static_cast<void>(cpu.step());
  require(cpu.state().pending[2].has_value() && cpu.state().effective[2].value == 0);
  static_cast<void>(cpu.step());
  require(cpu.state().idle_pc == foreground + 1 && !cpu.state().pending[2] &&
          cpu.state().effective[2].value == global);
  require(cpu.service_interrupt({.vector_base = base}).has_value());

  constexpr std::uint64_t flush = 0x170200000000;
  code(memory, foreground, flush);
  lead_a1::Sharc flushed(memory, initial());
  static_cast<void>(flushed.step());
  require(flushed.state().cache.pending_flush);
  static_cast<void>(flushed.step());
  require(flushed.state().idle_pc == foreground + 1 && !flushed.state().cache.pending_flush &&
          flushed.state().cache.valid == (lead_a1::SharcKnown32{.known = full32}));
  const lead_a1::Sharc checkpoint(memory, flushed.state());
  require(checkpoint.state() == flushed.state());
}

void delayed_and_reserved_guards() {
  lead_a1::SharcMemory memory;
  program(memory);
  auto state = initial();
  state.branch = lead_a1::SharcBranch{.target = base, .slots = 2};
  lead_a1::Sharc delayed(memory, state);
  const auto before = delayed.state();
  rejected([&] { static_cast<void>(delayed.step()); }, "delayed-branch/loop IDLE not qualified");
  require(delayed.state() == before && !memory.active());
  constexpr std::array<std::uint64_t, 3> unsupported{0x004000000000, 0x00c000000000, idle | 1};
  for (const auto opcode : unsupported) {
    code(memory, foreground, opcode);
    lead_a1::Sharc cpu(memory, initial());
    const auto original = cpu.state();
    rejected([&] { static_cast<void>(cpu.step()); }, "emuidle/VISA/reserved");
    require(cpu.state() == original && !memory.active());
  }
}

void invalid_idle_restoration() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  static_cast<void>(cpu.step());
  auto state = cpu.state();
  ++state.pc;
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "IDLE/resume state");
  state = cpu.state();
  state.pending[11] = lead_a1::SharcKnown32{.known = full32};
  rejected([&] { const lead_a1::Sharc invalid(memory, state); },
           "pending effect after retired IDLE");
  state = cpu.state();
  state.cache.pending_flush = true;
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "IDLE/resume state");
}

}

int main() {
  try {
    no_fictional_retirement_and_real_dma();
    masked_request_stays_latched();
    committed_controls_and_cache_follower();
    delayed_and_reserved_guards();
    invalid_idle_restoration();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
