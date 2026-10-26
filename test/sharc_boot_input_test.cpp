// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0x8c000;
constexpr std::uint32_t reset = base + 4;
constexpr std::uint32_t kernel_start = base + 5;
constexpr std::uint32_t vector = base + 0x30;
constexpr std::uint32_t destination = 0x92000;
constexpr unsigned int words = 384;
constexpr std::uint64_t idle = 0x008000000000;
constexpr std::uint64_t rti = 0x0b3e00000000;
constexpr std::uint64_t move_r7 = 0x0f070000002a;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t partial_mask = 0xffffefff;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint32_t core_directions = 0xaa;
constexpr std::uint64_t expected_r7 = 42;
constexpr std::uint8_t invalid_phase = 255;
constexpr std::uint32_t frame = 0x0208c005;
constexpr auto io0 = lead_a1::SharcMemoryMaster::io0;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SPI boot input expectation at line " + std::to_string(where.line()));
  }
}

template <typename Action> void rejected(const Action &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected " + reason + ": " + error.what());
  }
  throw std::runtime_error("Missing SPI boot input rejection: " + reason);
}

std::uint64_t instruction(unsigned int index) {
  if (index == 5) { return move_r7; }
  constexpr unsigned int spi_offset = 0x30;
  return index == spi_offset ? rti : 0;
}

std::uint32_t packed_word(unsigned int index) {
  constexpr std::uint64_t half = 0xffff;
  const auto first = index * 2;
  const auto low = (instruction(first / 3) >> ((first % 3) * 16)) & half;
  const auto high = (instruction((first + 1) / 3) >> (((first + 1) % 3) * 16)) & half;
  return static_cast<std::uint32_t>(low | (high << 16));
}

void map_owned_kernel(lead_a1::SharcMemory &memory) {
  memory.map_2148x_5mbit_ram();
  for (unsigned int index = 0; index < words; ++index) {
    memory.write({.address = destination + index}, packed_word(index));
  }
}

void serial_word(lead_a1::SharcSpi &spi, lead_a1::SharcMemory &memory, std::uint32_t word) {
  spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  constexpr unsigned int bits = 32;
  for (unsigned int bit = 0; bit < bits; ++bit) {
    spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
    spi.observe(memory, {.selected = true, .clock = true, .mosi = ((word >> bit) & 1) != 0});
  }
}

void grant(lead_a1::SharcSpi &spi, lead_a1::SharcMemoryBus &bus) {
  const auto request = spi.dma_word_request();
  if (!request) { throw std::runtime_error("Missing actual SPI boot word request"); }
  bus.request(io0, *request);
  bus.advance();
  if (!bus.grant(io0)) { bus.advance(); }
  require(bus.grant(io0) == request);
}

void no_program_fetch_while_booting() {
  lead_a1::SharcMemory memory; // No instruction RAM at all: boot IDLE must use its own input.
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::spi_slave_boot_reference_214xx(), &control);
  const auto before = cpu.state();
  require(before.pc == reset &&
          before.system[2] == (lead_a1::SharcKnown32{.known = partial_mask}) &&
          before.system[11] == (lead_a1::SharcKnown32{.value = 3, .known = partial_mask}) &&
          before.system[5] == (lead_a1::SharcKnown32{.known = core_directions}));
  rejected([&] { static_cast<void>(cpu.step()); }, "boot-channel IDLE, not program execution");
  lead_a1::SharcFetchState pipeline;
  pipeline.flow = lead_a1::SharcFetchFlow{.next = reset};
  lead_a1::SharcFetch fetch(memory, pipeline);
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); },
           "boot-channel input, not RAM fetch");
  require(cpu.state() == before && fetch.state() == pipeline);
  const auto step = cpu.step_boot_input();
  require(step.pc == reset && step.opcode == idle && step.next_pc == kernel_start &&
          cpu.state().spi_boot == lead_a1::SharcSpiBootPhase::idle && cpu.state().depth == 0 &&
          cpu.state().idle_pc == reset && cpu.state().data == before.data &&
          cpu.state().dag == before.dag);
  const auto held = cpu.state();
  rejected([&] { static_cast<void>(cpu.step_boot_input()); }, "selected initial channel");
  rejected([&] { static_cast<void>(cpu.wake_idle()); }, "boot-channel recognition");
  rejected([&] { static_cast<void>(cpu.service_interrupt({.vector_base = base})); },
           "boot-channel recognition");
  require(!cpu.service_boot_interrupt() && cpu.state() == held && !memory.active());
}

void check_entry(const lead_a1::Sharc &cpu) {
  const auto &state = cpu.state();
  require(state.spi_boot == lead_a1::SharcSpiBootPhase::interrupt && state.pc == vector &&
          !state.idle_pc && state.depth == 1 && state.pc_stack[0] == frame && state.interrupt &&
          state.interrupt->return_pc == kernel_start && state.interrupt->source == 0x1000 &&
          state.system[12] == (lead_a1::SharcKnown32{.value = 0x1000, .known = full32}) &&
          state.system[2].known == partial_mask && state.system[11].known == partial_mask);
}

void finish_original_shape(lead_a1::Sharc &cpu) {
  require(cpu.step().pc == vector && cpu.state().pc == kernel_start && !cpu.state().spi_boot &&
          !cpu.state().interrupt && cpu.state().depth == 0 && cpu.state().pc_stack[0] == frame);
  require(cpu.step().opcode == move_r7 && cpu.state().pc == kernel_start + 1 &&
          cpu.state().data[0][7] ==
              (lead_a1::SharcKnown40{.value = expected_r7 << 8, .known = full40}) &&
          cpu.state().system[2].known == partial_mask &&
          cpu.state().system[11].known == partial_mask);
}

void real_dma_completion_and_boot_rti() {
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::spi_slave_boot_reference_214xx(), &control);
  require(cpu.step_boot_input().opcode == idle);
  lead_a1::SharcSpi spi(lead_a1::SharcSpiSlaveBootReference{});
  lead_a1::SharcMemoryBus bus(memory, lead_a1::SharcPeripheralPhase::active);
  unsigned int granted{};
  for (unsigned int index = 0; index < words; ++index) {
    serial_word(spi, memory, packed_word(index));
    while (spi.dma_word_request() && granted < words - 1) {
      grant(spi, bus);
      spi.service_dma_word(bus);
      ++granted;
    }
  }
  require(granted == words - 1 && spi.state().serial.remaining == 1 && !spi.interrupt_request() &&
          !cpu.service_boot_interrupt());
  grant(spi, bus); // A granted but uncommitted last word must not activate the core.
  require(!spi.interrupt_request() && !cpu.service_boot_interrupt());
  spi.service_dma_word(bus);
  constexpr std::uint64_t actual_clocks = 767;
  require(bus.state().clocks == actual_clocks && spi.state().serial.remaining == 0 &&
          spi.interrupt_request());
  const auto received = spi.state();
  for (unsigned int index = 0; index < words; ++index) {
    require(memory.read({.address = destination + index}) == packed_word(index));
  }
  cpu.latch_programmable_interrupt(
      1); // Actual selected peripheral completion, not a software reply.
  const auto event = cpu.service_boot_interrupt();
  require(event && event->source == 0x1000 && event->return_pc == kernel_start &&
          event->vector == vector && event->minimum_core_cycles == 4 && spi.state() == received &&
          spi.interrupt_request());
  check_entry(cpu);
  lead_a1::SharcMemory other_memory;
  map_owned_kernel(other_memory);
  auto other_control = lead_a1::SharcControl::from_snapshot(control.snapshot());
  lead_a1::Sharc other(other_memory, cpu.state(), &other_control);
  spi.acknowledge_interrupt();
  require(!spi.interrupt_request() && !spi.dma_word_request() && spi.state().occupied == 0 &&
          !spi.state().receive);
  finish_original_shape(cpu);
  finish_original_shape(other);
  require(cpu.state() == other.state() && control.snapshot() == other_control.snapshot() &&
          bus.state().clocks == actual_clocks);
}

void failed_input_and_initial_vector_repair() {
  lead_a1::SharcMemory memory;
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::spi_slave_boot_reference_214xx(), &control);
  const auto input = cpu.state();
  control.begin_instruction();
  rejected([&] { static_cast<void>(cpu.step_boot_input()); }, "already owned");
  require(cpu.state() == input && !memory.active());
  control.abort_instruction();
  require(cpu.step_boot_input().opcode == idle);
  cpu.latch_programmable_interrupt(1); // Explicit owned malformed-input path, no reception claim.
  const auto held = cpu.state();
  control.begin_instruction();
  rejected([&] { static_cast<void>(cpu.service_boot_interrupt()); },
           "clock advance during a transaction");
  require(cpu.state() == held && !memory.active());
  control.abort_instruction();
  require(cpu.service_boot_interrupt().has_value());
  const auto entered = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Unmapped SHARC physical RAM/ROM cell");
  require(cpu.state() == entered && !memory.active());
  map_owned_kernel(memory);
  memory.write({.address = vector, .width = lead_a1::SharcWidth::wide48}, move_r7);
  rejected([&] { static_cast<void>(cpu.step()); }, "actual IDLE/RTI instruction");
  require(cpu.state() == entered && !memory.active());
  memory.write({.address = vector, .width = lead_a1::SharcWidth::wide48}, rti);
  finish_original_shape(cpu);
}

void malformed_boot_snapshots() {
  lead_a1::SharcMemory memory;
  auto state = lead_a1::Sharc::spi_slave_boot_reference_214xx();
  state.pc = kernel_start;
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "boot input/IDLE/interrupt phase");
  state.pc = reset;
  state.spi_boot = std::bit_cast<lead_a1::SharcSpiBootPhase>(invalid_phase);
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "boot input/IDLE/interrupt phase");
  state.spi_boot = lead_a1::SharcSpiBootPhase::interrupt;
  state.pc = vector;
  rejected([&] { const lead_a1::Sharc invalid(memory, state); }, "boot input/IDLE/interrupt phase");
  lead_a1::Sharc ordinary(memory, lead_a1::Sharc::reset_reference_214xx(reset));
  rejected([&] { static_cast<void>(ordinary.step_boot_input()); }, "selected initial channel");
  rejected([&] { static_cast<void>(ordinary.service_boot_interrupt()); },
           "retired boot-channel IDLE");
}

}

int main() try {
  no_program_fetch_while_booting();
  real_dma_completion_and_boot_rti();
  failed_input_and_initial_vector_repair();
  malformed_boot_snapshots();
  std::cout << "Boot-channel IDLE, actual SPI completion, initial P1I/RTI and retained unknown "
               "controls pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
