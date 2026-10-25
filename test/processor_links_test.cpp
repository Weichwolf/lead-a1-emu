// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm.hpp"
#include "arm/peripherals/gpio.hpp"
#include "board/processor_links.hpp"
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"

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

using Drive = lead_a1::WireDrive;
constexpr unsigned int pins = 144;
constexpr unsigned int bank = 7;
constexpr unsigned int pin = 12;
constexpr unsigned int flag = 13;
constexpr std::uint16_t pin_bit = 0x1000;
constexpr std::uint32_t flag_bit = 0x2000;
constexpr std::uint16_t all_pins = 0xffff;
constexpr std::uint32_t native = 0xc0000;
constexpr std::uint32_t gpio_base = 0x01e26000;
constexpr std::uint32_t arm_program = 0x1000;
constexpr std::uint32_t btf = 0x40000;
constexpr std::uint16_t unconnected_pads = 0xfffc;
constexpr std::uint16_t output_direction = 0xfffe;
constexpr std::uint32_t arm_status = 0xdf;
constexpr std::uint64_t test_flag0 = 0x148500000001;
constexpr std::uint64_t select_flag1_output = 0x140500000008;
constexpr std::uint64_t set_flag1 = 0x140500000004;
constexpr std::uint64_t clear_flag1 = 0x142500000004;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Processor link expectation at line " + std::to_string(where.line()));
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
  throw std::runtime_error("Missing processor link rejection: " + reason);
}

lead_a1::SharcState signal(Drive drive, unsigned int output_flag) {
  auto state = lead_a1::Sharc::reset_reference_214xx(native);
  const auto level = std::uint32_t{1} << (output_flag * 2);
  const auto direction = level << 1;
  state.system[5].known |= direction;
  state.system[5].value &= ~(direction | level);
  state.system[5].known &= ~level;
  if (drive != Drive::floating) { state.system[5].value |= direction; }
  if (drive == Drive::low || drive == Drive::high) { state.system[5].known |= level; }
  if (drive == Drive::high) { state.system[5].value |= level; }
  state.effective = state.system;
  return state;
}

void program(lead_a1::SharcMemory &memory) {
  constexpr unsigned int cells = 64;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned FLAG program"});
  constexpr std::array<std::uint64_t, 8> words{
      test_flag0, select_flag1_output, 0, set_flag1, 0, clear_flag1, 0, test_flag0};
  for (unsigned int index = 0; index < words.size(); ++index) {
    memory.write({.address = native + index, .width = lead_a1::SharcWidth::wide48}, words[index]);
  }
}

std::optional<bool> level(Drive drive) {
  if (drive == Drive::low) { return false; }
  if (drive == Drive::high) { return true; }
  return {};
}

void check_resolution(unsigned int gpio_mode, unsigned int dsp_mode, unsigned int external) {
  constexpr std::array<Drive, 4> types{Drive::floating, Drive::low, Drive::high, Drive::unresolved};
  constexpr std::array<std::array<Drive, 4>, 4> pair{
      {{Drive::floating, Drive::low, Drive::high, Drive::unresolved},
       {Drive::low, Drive::low, Drive::unresolved, Drive::unresolved},
       {Drive::high, Drive::unresolved, Drive::high, Drive::unresolved},
       {Drive::unresolved, Drive::unresolved, Drive::unresolved, Drive::unresolved}}};
  constexpr std::uint16_t previous = 0xa5a5;
  auto initial = lead_a1::Gpio::reset_reference(pins);
  initial.banks[bank].input = {.value = previous, .known = all_pins};
  if (gpio_mode != 0) { initial.banks[bank].direction &= ~pin_bit; }
  if (gpio_mode == 2) { initial.banks[bank].output = pin_bit; }
  lead_a1::Gpio gpio(initial);
  lead_a1::SharcMemory memory;
  auto state = signal(types[dsp_mode], flag);
  state.flag_inputs = {.value = previous, .known = all_pins};
  lead_a1::Sharc dsp(memory, state);
  const std::array dsps{&dsp};
  lead_a1::ProcessorLinks links(
      gpio,
      dsps,
      std::vector<lead_a1::ProcessorWire>{{.bank = bank, .pin = pin, .dsp = 0, .flag = flag}});
  links.external(0, types[external]);
  const auto before = gpio.state();
  links.sample();
  const auto expected = level(pair[static_cast<unsigned int>(pair[gpio_mode][dsp_mode])][external]);
  const auto saved = links.snapshot();
  require(saved.sampled && saved.levels[0].level == expected);
  const auto sample = gpio.state().banks[bank].input;
  require((sample.known & pin_bit) == (expected ? pin_bit : 0) &&
          (sample.value & pin_bit) == (expected.value_or(false) ? pin_bit : 0));
  require((sample.known & ~pin_bit) == (before.banks[bank].input.known & ~pin_bit) &&
          (sample.value & ~pin_bit) == (previous & ~pin_bit));
  require((dsp.state().flag_inputs.known & flag_bit) == (expected ? flag_bit : 0) &&
          (dsp.state().flag_inputs.value & flag_bit) == (expected.value_or(false) ? flag_bit : 0));
  require((dsp.state().flag_inputs.known & ~flag_bit) == (state.flag_inputs.known & ~flag_bit) &&
          (dsp.state().flag_inputs.value & ~flag_bit) == (previous & ~flag_bit));
}

void resolution_truth_table() {
  for (unsigned int gpio_mode = 0; gpio_mode < 3; ++gpio_mode) {
    for (unsigned int dsp_mode = 0; dsp_mode < 4; ++dsp_mode) {
      for (unsigned int external = 0; external < 4; ++external) {
        check_resolution(gpio_mode, dsp_mode, external);
      }
    }
  }
}

void atomic_multi_processor_samples(bool acknowledge) {
  auto initial = lead_a1::Gpio::reset_reference(pins);
  initial.banks[0].input = {.known = all_pins};
  initial.banks[1].input = {.known = all_pins};
  initial.banks[0].rising = 2;
  initial.banks[1].rising = 0x20;
  lead_a1::Gpio gpio(initial);
  lead_a1::SharcMemory first_memory;
  lead_a1::SharcMemory second_memory;
  lead_a1::Sharc first(first_memory, signal(Drive::high, 1));
  lead_a1::Sharc second(second_memory, signal(Drive::high, 1));
  const std::array dsps{&first, &second};
  const std::vector<lead_a1::ProcessorWire> wires{{.bank = 0, .pin = 1, .dsp = 0, .flag = 1},
                                                  {.bank = 1, .pin = 5, .dsp = 1, .flag = 1}};
  lead_a1::ProcessorLinks links(gpio, dsps, wires);
  const auto before = links.snapshot();
  const auto first_before = first.state();
  const auto second_before = second.state();
  second_memory.begin_instruction();
  rejected([&] { links.sample(); }, "during a memory transaction");
  require(first.state() == first_before && second.state() == second_before &&
          gpio.state() == initial && links.snapshot() == before && second_memory.active());
  second_memory.abort_instruction();
  gpio.begin_instruction();
  rejected([&] { links.sample(); }, "during active instruction");
  require(first.state() == first_before && second.state() == second_before &&
          gpio.state() == initial && links.snapshot() == before);
  gpio.abort_instruction();
  links.sample();
  require(gpio.state().banks[0].pending == 2 && gpio.state().banks[1].pending == 0x20 &&
          gpio.state().banks[0].input.value == 2 && gpio.state().banks[1].input.value == 0x20);
  require(first.state().flag_inputs == (lead_a1::SharcKnown32{.value = 2, .known = 2}) &&
          second.state().flag_inputs == (lead_a1::SharcKnown32{.value = 2, .known = 2}));
  if (acknowledge) {
    constexpr std::uint32_t status_pair = 0x34;
    constexpr std::uint32_t edge_bits = 0x00200002;
    gpio.write({.address = status_pair, .size = 4}, edge_bits);
    require(gpio.state().banks[0].pending == 0 && gpio.state().banks[1].pending == 0);
  }
  const auto gpio_saved = gpio.state();
  lead_a1::Gpio restored_gpio(gpio_saved);
  lead_a1::SharcMemory restored_first_memory;
  lead_a1::SharcMemory restored_second_memory;
  lead_a1::Sharc restored_first(restored_first_memory, first.state());
  lead_a1::Sharc restored_second(restored_second_memory, second.state());
  const std::array restored_dsps{&restored_first, &restored_second};
  lead_a1::ProcessorLinks restored(restored_gpio, restored_dsps, links.snapshot());
  for (const auto drive : {Drive::floating, Drive::high}) {
    links.external(0, drive);
    restored.external(0, drive);
    links.sample();
    restored.sample();
    require(gpio.state() == restored_gpio.state() && first.state() == restored_first.state() &&
            second.state() == restored_second.state() && links.snapshot() == restored.snapshot());
    require(gpio.state() == gpio_saved); // Agreed HIGH creates no second rising event.
  }
}

void late_sample_failure_and_guest_repair() {
  auto initial = lead_a1::Gpio::reset_reference(pins);
  initial.banks[0].input = {.known = all_pins};
  initial.banks[1].input = {.known = all_pins};
  initial.banks[0].rising = 2;
  initial.banks[1].rising = 0x20;
  lead_a1::Gpio gpio(initial);
  lead_a1::SharcMemory first_memory;
  lead_a1::SharcMemory second_memory;
  program(second_memory);
  second_memory.write({.address = native, .width = lead_a1::SharcWidth::wide48}, set_flag1);
  second_memory.write({.address = native + 1, .width = lead_a1::SharcWidth::wide48}, 0);
  lead_a1::Sharc first(first_memory, signal(Drive::high, 1));
  lead_a1::Sharc second(second_memory, signal(Drive::unresolved, 1));
  const std::array dsps{&first, &second};
  lead_a1::ProcessorLinks links(
      gpio,
      dsps,
      std::vector<lead_a1::ProcessorWire>{{.bank = 0, .pin = 1, .dsp = 0, .flag = 1},
                                          {.bank = 1, .pin = 5, .dsp = 1, .flag = 1}});
  const auto links_before = links.snapshot();
  const auto first_before = first.state();
  for (unsigned int attempt = 0; attempt < 2; ++attempt) {
    const auto second_before = second.state();
    rejected([&] { links.sample(); }, "Unknown GPIO edge baseline or new sample");
    require(gpio.state() == initial && links.snapshot() == links_before &&
            first.state() == first_before && second.state() == second_before);
    static_cast<void>(second.step()); // Real SET followed by its effect-latency NOP.
  }
  links.sample();
  require(gpio.state().banks[0].pending == 2 && gpio.state().banks[1].pending == 0x20 &&
          first.state().flag_inputs == (lead_a1::SharcKnown32{.value = 2, .known = 2}) &&
          second.state().flag_inputs == (lead_a1::SharcKnown32{.value = 2, .known = 2}));
}

void malformed_topology_and_snapshots() {
  lead_a1::Gpio gpio(lead_a1::Gpio::reset_reference(pins));
  lead_a1::SharcMemory memory;
  lead_a1::Sharc dsp(memory, signal(Drive::floating, 0));
  const std::array dsps{&dsp};
  const std::vector<lead_a1::ProcessorWire> wires{{.bank = 0, .pin = 0, .dsp = 0, .flag = 0}};
  lead_a1::ProcessorLinks good(gpio, dsps, wires);
  for (unsigned int variation = 0; variation < 9; ++variation) {
    auto changed = good.snapshot();
    switch (variation) {
      case 0: changed.wires[0].bank = 9; break;
      case 1: changed.wires[0].pin = 16; break;
      case 2: changed.wires[0].dsp = 1; break;
      case 3: changed.wires[0].flag = 16; break;
      case 4: changed.external.clear(); break;
      case 5: changed.external[0] = Drive::invalid; break;
      case 6: changed.levels[0].level = true; break;
      case 7: changed.levels[0] = {.high = 1, .level = true}; break;
      case 8:
        changed.sampled = true;
        changed.levels[0] = {.high = 1, .low = 1, .level = std::nullopt};
        break;
      default: throw std::logic_error("Invalid owned topology variation");
    }
    rejected([&] { const lead_a1::ProcessorLinks invalid(gpio, dsps, changed); },
             variation == 4 ? "matching state" : "processor");
  }
  const auto before = good.snapshot();
  rejected([&] { good.external(1, Drive::high); }, "wire index");
  rejected([&] { good.external(0, Drive::invalid); }, "external drive");
  require(good.snapshot() == before);
  const std::vector<lead_a1::ProcessorWire> duplicate{wires[0],
                                                      {.bank = 0, .pin = 1, .dsp = 0, .flag = 0}};
  rejected([&] { const lead_a1::ProcessorLinks invalid(gpio, dsps, duplicate); }, "Duplicate");
  const std::array<lead_a1::Sharc *, 1> missing{nullptr};
  rejected([&] { const lead_a1::ProcessorLinks invalid(gpio, missing, wires); }, "Null");
  const std::array aliased{&dsp, &dsp};
  rejected([&] { const lead_a1::ProcessorLinks invalid(gpio, aliased, wires); }, "Aliased");
}

std::vector<std::byte> read(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input), {}};
  if (!input || bytes.empty()) { throw std::runtime_error("Missing owned ARM wire program"); }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

void actual_arm_and_sharc_handshake(const std::filesystem::path &spec,
                                    std::span<const std::byte> bytes) {
  auto initial = lead_a1::Gpio::reset_reference(pins);
  initial.banks[0].output = 1;
  initial.banks[0].input = {.known = unconnected_pads}; // Explicit owned external pad inputs.
  lead_a1::Gpio gpio(initial);
  lead_a1::SharcMemory dsp_memory;
  program(dsp_memory);
  lead_a1::Sharc dsp(dsp_memory, lead_a1::Sharc::reset_reference_214xx(native));
  const std::array dsps{&dsp};
  lead_a1::ProcessorLinks links(
      gpio,
      dsps,
      std::vector<lead_a1::ProcessorWire>{{.bank = 0, .pin = 0, .dsp = 0, .flag = 0},
                                          {.bank = 0, .pin = 1, .dsp = 0, .flag = 1}});
  lead_a1::Memory memory;
  memory.map({.address = arm_program, .size = bytes.size(), .name = "owned ARM handshake"}, bytes);
  memory.map_device({.address = gpio_base, .size = 0x1000, .name = "GPIO", .executable = false},
                    gpio);
  lead_a1::ArmState state;
  state.cpsr = arm_status;
  state.registers[0] = gpio_base + 16;
  state.registers[1] = 1;
  state.registers[2] = output_direction;
  state.registers[15] = arm_program;
  lead_a1::Arm arm(memory, spec, state);
  links.sample();
  require((dsp.state().flag_inputs.known & 1) == 0);
  require(!arm.step().exception && gpio.state().banks[0].output == 0);
  links.sample();
  require((dsp.state().flag_inputs.known & 1) == 0); // Latch preload is not enabled drive.
  require(!arm.step().exception);
  links.sample();
  require(dsp.state().flag_inputs == (lead_a1::SharcKnown32{.known = 1}));
  static_cast<void>(dsp.step());
  require((dsp.state().system[6].known & btf) != 0 && (dsp.state().system[6].value & btf) == 0);
  const auto waiting = arm.state();
  rejected([&] { static_cast<void>(arm.step()); }, "Unknown GPIO synchronized input pins");
  require(arm.state() == waiting);
  for (unsigned int instruction = 0; instruction < 4; ++instruction) {
    static_cast<void>(dsp.step());
    links.sample();
    if (instruction < 3) { require((gpio.state().banks[0].input.known & 2) == 0); }
  }
  require(gpio.state().banks[0].input == (lead_a1::GpioSample{.value = 2, .known = all_pins}));
  require(!arm.step().exception && arm.state().registers[3] == 2);
  require(!arm.step().exception && arm.state().registers[3] == 2);
  static_cast<void>(dsp.step());
  links.sample();
  require((gpio.state().banks[0].input.value & 2) != 0); // Delayed FLAGS clear not visible yet.
  static_cast<void>(dsp.step());
  links.sample();
  require(gpio.state().banks[0].input == (lead_a1::GpioSample{.known = all_pins}));
  require(!arm.step().exception);
  links.sample();
  static_cast<void>(dsp.step());
  require((dsp.state().system[6].known & dsp.state().system[6].value & btf) != 0);
  require(!arm.step().exception && arm.state().registers[3] == 1);
}

}

int main(int argc, char **argv) try {
  if (argc != 3) { throw std::runtime_error("processor-links-test sleigh-spec owned-arm-program"); }
  resolution_truth_table();
  atomic_multi_processor_samples(false);
  atomic_multi_processor_samples(true);
  late_sample_failure_and_guest_repair();
  malformed_topology_and_snapshots();
  actual_arm_and_sharc_handshake(argv[1], read(argv[2]));
  std::cout
      << "Explicit GPIO/SHARC wires, unknown/contention, atomic samples and guest handshake pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
