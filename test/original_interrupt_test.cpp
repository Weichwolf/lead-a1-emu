// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/aintc.hpp"
#include "lead_a1/arm.hpp"
#include "lead_a1/arm_system.hpp"
#include "lead_a1/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t code_base = 0x1000;
constexpr std::uint32_t callback = 0x1100;
constexpr std::uint32_t local_base = 0xffff0000;
constexpr std::uint32_t original_entry = 0xffff0020;
constexpr std::uint32_t controller_base = 0xfffee000;
constexpr std::uint32_t table_base = 0x4000;
constexpr std::uint32_t marker = 0x5000;
constexpr std::uint32_t initial_marker = 0x10203040;
constexpr std::uint32_t user_stack = 0x9000;
constexpr std::uint32_t irq_stack = 0x8000;
constexpr std::uint32_t fiq_stack = 0x7000;
constexpr std::uint32_t user_link = 0x11223344;
constexpr std::uint32_t caller_status = 0xb8000010;
constexpr unsigned int pc_index = 15;

struct Inputs {
  std::filesystem::path spec;
  std::filesystem::path images;
  std::filesystem::path owned;
};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Original interrupt expectation at line " +
                             std::to_string(where.line()));
  }
}

std::vector<std::byte> read(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) {
    throw std::runtime_error("Missing original/owned interrupt input");
  }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

lead_a1::ArmSystemInitial profile() {
  constexpr std::uint32_t high_control = 0x52078;
  return {.control = high_control,
          .instruction_cache_bytes = {},
          .data_cache_bytes = {},
          .instruction_cache_invalid = true,
          .data_cache_invalid = true,
          .instruction_lockdown = {},
          .data_lockdown = {},
          .write_buffer = std::vector<lead_a1::ArmBufferedWrite>{},
          .writeback_buffer_empty = true,
          .mmu = {}};
}

lead_a1::AintcState controller(unsigned int source, bool fiq) {
  auto state = lead_a1::Aintc::reset_reference();
  state.control = 0x10;
  state.global_enabled = true;
  state.host_enabled = 3;
  state.vector_base = table_base;
  state.channels[source] = fiq ? 0 : 2;
  state.enabled[source / 32] = std::uint32_t{1} << (source % 32);
  return state;
}

void map(lead_a1::Memory &memory,
         std::span<const std::byte> original,
         std::span<const std::byte> caller,
         lead_a1::Aintc &device) {
  constexpr unsigned int prefix = 32;
  memory.map({.address = local_base, .size = original.size() + prefix, .name = "original ARM0"},
             original,
             prefix);
  memory.map({.address = code_base,
              .size = caller.size(),
              .name = "owned interrupt caller",
              .writable = false},
             caller);
  memory.map_device(
      {.address = controller_base, .size = 0x2000, .name = "candidate AINTC", .executable = false},
      device);
  constexpr unsigned int stack_extent = 128;
  const std::vector<std::byte> pattern(stack_extent, std::byte{0xa5});
  for (const auto top : {user_stack, irq_stack, fiq_stack}) {
    memory.map({.address = top - stack_extent, .size = pattern.size(), .name = "owned stack"},
               pattern);
  }
  constexpr unsigned int table_extent = 512;
  memory.map({.address = table_base, .size = table_extent, .name = "owned pointer table"});
  memory.map({.address = marker, .size = 4, .name = "owned counter"});
  memory.write({.address = marker, .size = 4}, initial_marker);
}

lead_a1::ArmState initial() {
  lead_a1::ArmState state;
  state.cpsr = caller_status;
  constexpr std::uint32_t first = 0x10203000;
  constexpr std::uint32_t increment = 0x101;
  for (unsigned int reg = 0; reg < 13; ++reg) { state.registers[reg] = first + (reg * increment); }
  state.registers[6] = marker;
  state.registers[13] = user_stack;
  state.registers[14] = user_link;
  state.registers[pc_index] = code_base;
  state.banks.modes[1].sp = fiq_stack;
  state.banks.modes[2].sp = irq_stack;
  return state;
}

lead_a1::ArmStep step(lead_a1::Arm &cpu, const lead_a1::Aintc &device) {
  const auto lines = device.outputs();
  cpu.set_interrupts({.irq = lines[1], .fiq = lines[0]});
  return cpu.step();
}

void check_frame(const lead_a1::Memory &memory, const lead_a1::ArmState &caller, bool fiq) {
  const auto top = fiq ? fiq_stack : irq_stack;
  const unsigned int count = fiq ? 4 : 13;
  const unsigned int bytes = (count + 1) * 4;
  for (unsigned int reg = 0; reg < count; ++reg) {
    require(memory.read({.address = top - bytes + (reg * 4), .size = 4}) == caller.registers[reg]);
  }
  require(memory.read({.address = top - 4, .size = 4}) == code_base + 4);
  if (!fiq) {
    constexpr unsigned int frame_bytes = 60;
    require(memory.read({.address = top - frame_bytes, .size = 4}) == caller_status);
    constexpr std::array<std::uint32_t, 5> system_frame{
        0xb800009f, controller_base, 0xfffee900, 0x100, user_link};
    constexpr unsigned int system_bytes = 20;
    for (unsigned int index = 0; index < system_frame.size(); ++index) {
      require(memory.read({.address = user_stack - system_bytes + (index * 4), .size = 4}) ==
              system_frame[index]);
    }
  }
}

unsigned int run_handler(lead_a1::Arm &cpu, const lead_a1::Aintc &device, bool fiq) {
  constexpr std::uint32_t fiq_callback_status = 0xb80000d1;
  constexpr std::uint32_t irq_callback_status = 0xb800001f;
  unsigned int count{};
  unsigned int calls{};
  constexpr unsigned int limit = 100;
  while (cpu.state().registers[pc_index] != code_base && count < limit) {
    if (cpu.state().registers[pc_index] == callback) {
      require(cpu.state().cpsr == (fiq ? fiq_callback_status : irq_callback_status));
      require(!device.outputs()[fiq ? 0 : 1]);
      ++calls;
    }
    require(!step(cpu, device).exception);
    ++count;
  }
  constexpr unsigned int fiq_instructions = 15;
  constexpr unsigned int irq_instructions = 43;
  require(count == (fiq ? fiq_instructions : irq_instructions) && calls == 1);
  return count;
}

void execute(const Inputs &inputs, unsigned int part, bool fiq) {
  const auto original = read(inputs.images / (std::to_string(part) + "-ARM0.bin"));
  const auto owned = read(inputs.owned);
  constexpr std::size_t original_bytes = 616;
  require(original.size() == original_bytes);
  constexpr unsigned int fiq_source = 40;
  constexpr unsigned int irq_source = 41;
  const auto source = fiq ? fiq_source : irq_source;
  lead_a1::Aintc device(controller(source, fiq));
  lead_a1::Memory memory;
  map(memory, original, owned, device);
  const auto vector = local_base + (fiq ? 0x1cU : 0x18U);
  const auto handler = local_base + (fiq ? 0x60U : 0x8cU);
  constexpr std::uint32_t branch_opcode = 0xea000000;
  memory.write({.address = vector, .size = 4}, branch_opcode | ((handler - vector - 8) / 4));
  memory.write({.address = table_base + (source * 4), .size = 4}, callback);
  constexpr std::uint32_t vector_size_offset = 0x54;
  device.write({.address = vector_size_offset, .size = 4}, 0);
  const auto caller = initial();
  lead_a1::Arm cpu(memory, inputs.spec, caller, profile());
  device.pulse(source);
  const auto event = step(cpu, device);
  require(event.exception == (fiq ? lead_a1::ArmException::fiq : lead_a1::ArmException::irq));
  require(event.after == vector);
  require(!step(cpu, device).exception && cpu.state().registers[pc_index] == handler);
  const auto count = run_handler(cpu, device, fiq);
  require(cpu.state().cpsr == caller_status && cpu.state().registers[pc_index] == code_base);
  for (unsigned int reg = 0; reg < pc_index; ++reg) {
    require(cpu.state().registers[reg] == caller.registers[reg]);
  }
  require(cpu.state().banks.modes[fiq ? 1 : 2].sp == (fiq ? fiq_stack : irq_stack));
  require(cpu.state().banks.modes[fiq ? 1 : 2].saved.bits == caller_status);
  require(device.state().pending == (std::array<std::uint32_t, 4>{}));
  require(device.state().host_nesting[1] == 0x100);
  require(!device.state().held[fiq ? 0 : 1]);
  constexpr std::uint32_t updated_marker = 0x10203041;
  require(memory.read({.address = marker, .size = 4}) == updated_marker);
  check_frame(memory, caller, fiq);
  for (unsigned int byte = 0; byte < original.size(); ++byte) {
    require(memory.peek(original_entry + byte) == original[byte]);
  }
  require(!step(cpu, device).exception && cpu.state().registers[0] == 7);
  std::cout << "Part " << part << ": original " << (fiq ? "FIQ" : "IRQ")
            << " wrapper acknowledges AINTC, calls owned callback and restores caller (" << count
            << " instructions including four owned callback instructions)\n";
}

}

int main(int argc, char **argv) {
  if (argc != 4) { return 1; }
  try {
    const Inputs inputs{.spec = argv[1], .images = argv[2], .owned = argv[3]};
    for (const unsigned int part : {0U, 1U}) {
      for (const bool fiq : {false, true}) { execute(inputs, part, fiq); }
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
