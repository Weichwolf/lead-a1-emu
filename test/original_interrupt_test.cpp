// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm.hpp"
#include "arm/core/arm_system.hpp"
#include "arm/peripherals/aintc.hpp"
#include "common/memory.hpp"

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

struct NestedObservations {
  unsigned int irq_events{};
  unsigned int fiq_events{};
  std::vector<std::uint32_t> callbacks;
  std::vector<std::uint32_t> counter;
  bool injected{};
};

constexpr std::uint32_t higher_callback = 0x1400;
constexpr std::uint32_t equal_callback = 0x1500;
constexpr std::uint32_t fiq_callback = 0x1600;
constexpr unsigned int parent_source = 41;
constexpr unsigned int equal_source = 42;
constexpr unsigned int higher_source = 43;
constexpr unsigned int nested_fiq_source = 44;
constexpr std::uint32_t board_limit = 100;

struct ControllerPlacement {
  std::uint32_t installer;
  std::uint32_t constructor;
  std::uint32_t table;
};

constexpr std::array<ControllerPlacement, 2> controller_placements{
    {{.installer = 0xc0172238, .constructor = 0xc0172504, .table = 0xc0767e6c},
     {.installer = 0xc002d83c, .constructor = 0xc002e258, .table = 0xc00a8004}}};

void original_call(lead_a1::Memory &memory,
                   const std::filesystem::path &spec,
                   std::uint32_t entry,
                   std::array<std::uint32_t, 4> arguments) {
  auto state = initial();
  constexpr std::uint32_t masked_system = 0xb80000df;
  state.cpsr = masked_system;
  state.registers[pc_index] = entry;
  state.registers[14] = code_base;
  for (unsigned int index = 0; index < arguments.size(); ++index) {
    state.registers[index] = arguments[index];
  }
  lead_a1::Arm cpu(memory, spec, state, profile());
  constexpr unsigned int limit = 512;
  unsigned int count{};
  while (cpu.state().registers[pc_index] != code_base && count < limit) {
    require(!cpu.step().exception);
    ++count;
  }
  require(cpu.state().registers[pc_index] == code_base && cpu.state().registers[13] == user_stack);
}

void check_original_vectors(const lead_a1::Memory &memory) {
  constexpr std::array<std::uint32_t, 8> targets{0xffff0020,
                                                 0xffff0128,
                                                 0xffff002c,
                                                 0xffff0130,
                                                 0xffff012c,
                                                 0xffff0014,
                                                 0xffff008c,
                                                 0xffff0060};
  constexpr std::uint32_t branch = 0xea000000;
  constexpr std::uint32_t displacement_mask = 0x00ffffff;
  constexpr auto address_bytes = std::uint64_t{1} << 32;
  for (unsigned int index = 0; index < targets.size(); ++index) {
    const auto address = local_base + (index * 4);
    const auto displacement = (address_bytes + targets[index] - address - 8) / 4;
    require(memory.read({.address = address, .size = 4}) ==
            (branch | (displacement & displacement_mask)));
  }
}

void initialize_original_controller(lead_a1::Memory &memory,
                                    lead_a1::Aintc &device,
                                    const Inputs &inputs,
                                    unsigned int part) {
  const auto code = read(inputs.images / (std::to_string(part) + "-CODE.bin"));
  const auto where = controller_placements[part];
  constexpr std::uint32_t code_image_base = 0xc0000000;
  constexpr std::uint32_t object = 0x6000;
  constexpr unsigned int table_extent = 512;
  memory.map(
      {.address = code_image_base, .size = code.size(), .name = "original CODE", .writable = false},
      code);
  memory.map({.address = where.table, .size = table_extent, .name = "owned original dispatch RAM"});
  memory.map({.address = object, .size = 4, .name = "owned original controller object"});
  original_call(memory, inputs.spec, where.installer, {});
  check_original_vectors(memory);
  original_call(memory, inputs.spec, where.constructor, {object, 0, 0, 0});
  constexpr std::uint32_t bind_offset = 0xf8;
  constexpr std::uint32_t source_offset = 0x13c;
  constexpr std::uint32_t nesting_offset = 0x18c;
  constexpr std::uint32_t global_offset = 0x80;
  constexpr std::array<unsigned int, 4> sources{
      parent_source, equal_source, higher_source, nested_fiq_source};
  constexpr std::array<std::uint32_t, 4> callbacks{
      callback, equal_callback, higher_callback, fiq_callback};
  constexpr std::array<unsigned int, 4> channels{5, 5, 2, 0};
  for (unsigned int index = 0; index < sources.size(); ++index) {
    original_call(memory,
                  inputs.spec,
                  where.constructor + bind_offset,
                  {object, sources[index], callbacks[index], channels[index]});
    original_call(
        memory, inputs.spec, where.constructor + source_offset, {object, sources[index], 1, 0});
  }
  original_call(memory, inputs.spec, where.constructor + nesting_offset, {board_limit, 0, 0, 0});
  original_call(memory, inputs.spec, where.constructor + global_offset, {object, 1, 0, 0});
  constexpr std::uint32_t individual_hold = 0x14;
  const auto state = device.state();
  require(state.control == individual_hold && state.vector_base == where.table + 4);
  require(state.host_nesting[1] == board_limit && state.host_nesting_override[1]);
  require(state.global_enabled && state.host_enabled == 3 && state.vector_size_programmed);
}

void observe_callback(const lead_a1::ArmState &before,
                      std::uint32_t pc,
                      const lead_a1::Aintc &device,
                      NestedObservations &observed) {
  if (pc != callback && pc != higher_callback && pc != equal_callback && pc != fiq_callback) {
    return;
  }
  constexpr std::uint32_t system_status = 0xb800001f;
  constexpr std::uint32_t fiq_status = 0xb80000d1;
  const auto expected_status = pc == fiq_callback ? fiq_status : system_status;
  require(before.cpsr == expected_status);
  if (pc == higher_callback) {
    constexpr unsigned int two_system_frames = 40;
    constexpr unsigned int two_irq_frames = 120;
    require(before.registers[13] == user_stack - two_system_frames);
    require(before.banks.modes[2].sp == irq_stack - two_irq_frames);
    require(device.state().host_nesting[1] == 2);
  } else {
    require(device.state().host_nesting[1] == 5);
  }
  observed.callbacks.push_back(pc);
}

void nested_step(lead_a1::Arm &cpu,
                 lead_a1::Aintc &device,
                 lead_a1::Memory &memory,
                 NestedObservations &observed) {
  const auto before = cpu.state();
  const auto pc = before.registers[pc_index];
  if (!pc) { throw std::runtime_error("Unknown nested test PC"); }
  if (*pc == callback && !observed.injected) {
    device.pulse(equal_source);
    device.pulse(higher_source);
    device.pulse(nested_fiq_source);
    observed.injected = true;
  }
  const auto event = step(cpu, device);
  if (event.exception == lead_a1::ArmException::irq) { ++observed.irq_events; }
  if (event.exception == lead_a1::ArmException::fiq) { ++observed.fiq_events; }
  if (event.exception) { return; }
  observe_callback(before, *pc, device, observed);
  constexpr unsigned int store_offset = 8;
  if (*pc == callback + store_offset || *pc == higher_callback + store_offset ||
      *pc == equal_callback + store_offset || *pc == fiq_callback + store_offset) {
    observed.counter.push_back(
        static_cast<std::uint32_t>(memory.read({.address = marker, .size = 4})));
  }
}

void execute_nested(const Inputs &inputs, unsigned int part) {
  const auto original = read(inputs.images / (std::to_string(part) + "-ARM0.bin"));
  const auto owned = read(inputs.owned);
  lead_a1::Aintc device(lead_a1::Aintc::reset_reference());
  lead_a1::Memory memory;
  map(memory, original, owned, device);
  initialize_original_controller(memory, device, inputs, part);
  const auto caller = initial();
  lead_a1::Arm cpu(memory, inputs.spec, caller, profile());
  device.pulse(parent_source);
  NestedObservations observed;
  constexpr unsigned int limit = 256;
  unsigned int count{};
  while (cpu.state().registers[pc_index] != code_base + 4 && count < limit) {
    nested_step(cpu, device, memory, observed);
    ++count;
  }
  require(cpu.state().registers[pc_index] == code_base + 4 && count < limit);
  require(observed.injected && observed.irq_events == 3 && observed.fiq_events == 1);
  const std::vector<std::uint32_t> expected_calls{
      fiq_callback, higher_callback, callback, equal_callback};
  const std::vector<std::uint32_t> expected_counters{
      0x10204040, 0x10204050, 0x10204051, 0x10204151};
  require(observed.callbacks == expected_calls && observed.counter == expected_counters);
  require(cpu.state().cpsr == caller_status && cpu.state().registers[0] == 7);
  for (unsigned int reg = 1; reg < pc_index; ++reg) {
    require(cpu.state().registers[reg] == caller.registers[reg]);
  }
  require(cpu.state().banks.modes[2].sp == irq_stack && cpu.state().banks.modes[1].sp == fiq_stack);
  require(cpu.state().banks.modes[2].saved.bits == caller_status);
  require(device.state().host_nesting[1] == board_limit && device.state().host_nesting_override[1]);
  require(device.state().pending == (std::array<std::uint32_t, 4>{}));
  require(device.state().held == (std::array<std::optional<lead_a1::AintcPriority>, 2>{}));
  for (unsigned int byte = 0; byte < original.size(); ++byte) {
    require(memory.peek(original_entry + byte) == original[byte]);
  }
  std::cout
      << "Part " << part
      << ": original vector/controller initialization and nested IRQ/FIQ defer equal priority, "
         "preempt in FIQ/higher-IRQ/parent/equal order and restore caller/levels/stacks\n";
}

}

int main(int argc, char **argv) {
  if (argc != 4) { return 1; }
  try {
    const Inputs inputs{.spec = argv[1], .images = argv[2], .owned = argv[3]};
    for (const unsigned int part : {0U, 1U}) {
      for (const bool fiq : {false, true}) { execute(inputs, part, fiq); }
      execute_nested(inputs, part);
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
