// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm.hpp"
#include "arm/peripherals/gpio.hpp"
#include "common/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr unsigned int pins = 144;
constexpr std::uint32_t base = 0x01e26000;
constexpr std::uint32_t program_base = 0x1000;
constexpr std::uint32_t second_pair = 0x38;
constexpr std::uint32_t last_pair = 0xb0;
constexpr std::uint16_t all_pins = 0xffff;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Independent GPIO expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing GPIO rejection: " + reason);
}

constexpr std::uint32_t revision_high = 0x4483;
constexpr std::uint32_t preload_high = 0x11;
constexpr std::uint32_t preload_low = 0x22;
constexpr std::uint32_t replacement_byte = 0xab;
constexpr std::uint32_t replacement_word = 0xab22;
constexpr std::uint32_t odd_set = 0x42;
constexpr std::uint32_t odd_preload = 0x8010;
constexpr std::uint32_t paired_output = 0x3c;
constexpr std::uint32_t paired_value = 0x8010ab22;
constexpr std::uint32_t odd_direction = 0x3a;
constexpr std::uint32_t input2 = 0x48;
constexpr std::uint32_t byte_known = 0xff;
constexpr std::uint32_t rise2 = 0x4c;
constexpr std::uint32_t fall2 = 0x54;
constexpr std::uint32_t rise_clear2 = 0x50;
constexpr std::uint32_t fall_clear2 = 0x58;
constexpr std::uint32_t both_edges = 0xc0;
constexpr std::uint32_t status2 = 0x5c;
constexpr std::uint32_t set0 = 0x18;
constexpr std::uint32_t last_set = 0xb8;
constexpr std::uint32_t last_clear = 0xbc;
constexpr std::uint32_t partial_directions = 0x1ffff;
constexpr unsigned int partial_pins = 17;

void aliases_and_widths() {
  lead_a1::Gpio gpio(lead_a1::Gpio::reset_reference(pins));
  constexpr std::uint32_t revision = 0x44830105;
  require(gpio.read({.address = 0, .size = 4}) == revision);
  require(gpio.read({.address = 2, .size = 2}) == revision_high);
  for (const auto pair : {0x10U, 0x38U, 0x60U, 0x88U, 0xb0U}) {
    const auto mask = pair == last_pair ? 0xffffU : 0xffffffffU;
    require(gpio.read({.address = pair, .size = 4}) == mask);
    constexpr std::uint32_t preload = 0x1122;
    gpio.write({.address = pair + 8, .size = 2}, preload);
    require(gpio.read({.address = pair + 4, .size = 2}) == preload);
    require(gpio.read({.address = pair + 12, .size = 2}) == preload);
    const auto bank = ((pair - 16) / 40) * 2;
    require(gpio.drives()[bank].enabled == 0);
    gpio.write({.address = pair, .size = 2}, 0);
    require(gpio.drives()[bank] == (lead_a1::GpioDrive{.enabled = all_pins, .high = preload}));
    gpio.write({.address = pair + 13, .size = 1}, preload_high);
    require(gpio.read({.address = pair + 8, .size = 2}) == preload_low);
    gpio.write({.address = pair + 5, .size = 1}, replacement_byte);
    require(gpio.read({.address = pair + 4, .size = 2}) == replacement_word);
    require(gpio.read({.address = pair + 6, .size = 2}) == 0);
  }
  gpio.write({.address = odd_set, .size = 2}, odd_preload);
  require(gpio.read({.address = paired_output, .size = 4}) == paired_value);
  require(gpio.read({.address = 0x40, .size = 4}) == paired_value);
  require(gpio.state().banks[2].output == replacement_word &&
          gpio.state().banks[3].output == odd_preload);
  require(gpio.drives()[3].enabled == 0);
  gpio.write({.address = odd_direction, .size = 2}, 0);
  require(gpio.drives()[3] == (lead_a1::GpioDrive{.enabled = all_pins, .high = odd_preload}));
}

void samples_and_interrupts() {
  lead_a1::Gpio gpio(lead_a1::Gpio::reset_reference(pins));
  rejected([&] { static_cast<void>(gpio.read({.address = input2, .size = 2})); }, "Unknown GPIO");
  gpio.sample(2, {.value = 0x80, .known = byte_known});
  require(gpio.read({.address = input2, .size = 1}) == 0x80);
  rejected([&] { static_cast<void>(gpio.read({.address = input2, .size = 2})); }, "Unknown GPIO");
  gpio.sample(2, {.value = 0, .known = all_pins});
  gpio.write({.address = rise2, .size = 2}, 0x40);
  gpio.write({.address = fall2, .size = 2}, 0x80);
  require(gpio.read({.address = rise_clear2, .size = 2}) == 0x40);
  require(gpio.read({.address = fall_clear2, .size = 2}) == 0x80);
  gpio.sample(2, {.value = both_edges, .known = all_pins});
  require(gpio.read({.address = status2, .size = 2}) == 0x40 && !gpio.interrupts()[2]);
  gpio.write({.address = 8, .size = 1}, 4);
  require(gpio.interrupts()[2]);
  gpio.write({.address = status2, .size = 2}, 0);
  require(gpio.interrupts()[2]);
  gpio.write({.address = status2, .size = 2}, 0x40);
  require(!gpio.interrupts()[2]);
  gpio.sample(2, {.value = 0, .known = all_pins});
  require(gpio.state().banks[2].pending == 0x80 && gpio.interrupts()[2]);
  gpio.write({.address = rise_clear2, .size = 2}, 0x40);
  gpio.write({.address = fall_clear2, .size = 2}, 0x80);
  gpio.write({.address = status2, .size = 2}, 0x80);
  gpio.sample(2, {.value = both_edges, .known = all_pins});
  require(gpio.state().banks[2].pending == 0);
  gpio.write({.address = second_pair, .size = 2}, 0);
  gpio.write({.address = rise2, .size = 2}, 1);
  gpio.write({.address = 0x40, .size = 2}, 1);
  require(gpio.drives()[2].high == 1 && gpio.read({.address = input2, .size = 2}) == both_edges);
  gpio.sample(2, {.value = 1, .known = all_pins});
  require(gpio.interrupts()[2] && gpio.state().banks[2].pending == 1);
  const auto before = gpio.state();
  rejected([&] { gpio.sample(2, {}); }, "Unknown GPIO edge");
  require(gpio.state() == before);
}

void invalid_inputs() {
  rejected([] { static_cast<void>(lead_a1::Gpio::reset_reference(0)); }, "pin count");
  rejected([] { static_cast<void>(lead_a1::Gpio::reset_reference(pins + 1)); }, "pin count");
  auto initial = lead_a1::Gpio::reset_reference(pins);
  initial.banks[0].input = {.value = 1, .known = 0};
  rejected([&] { const lead_a1::Gpio invalid(initial); }, "sample knowledge");
  lead_a1::Gpio gpio(lead_a1::Gpio::reset_reference(pins));
  for (const auto address : {4U, 12U, 0xd8U}) {
    rejected([&] { static_cast<void>(gpio.read({.address = address, .size = 4})); }, "GPIO");
  }
  rejected([&] { static_cast<void>(gpio.read({.address = 16 + 1, .size = 2})); }, "alignment");
  rejected([&] { static_cast<void>(gpio.read({.address = 16, .size = 8})); }, "width");
  const auto before = gpio.state();
  rejected([&] { gpio.write({.address = 0, .size = 4}, 0); }, "Read-only");
  rejected([&] { gpio.write({.address = 0x20, .size = 4}, 0); }, "Read-only");
  rejected([&] { gpio.write({.address = set0, .size = 1}, 0x100); }, "Oversized");
  rejected([&] { gpio.write({.address = 8, .size = 4}, 0x200); }, "Reserved");
  rejected([&] { gpio.write({.address = last_set, .size = 4}, 0x10000); }, "Reserved");
  require(gpio.state() == before);
  rejected([&] { gpio.sample(9, {}); }, "bank");
  gpio.begin_instruction();
  rejected([&] { gpio.begin_instruction(); }, "Nested");
  rejected([&] { gpio.sample(0, {}); }, "active instruction");
  gpio.write({.address = set0, .size = 2}, 3);
  require(gpio.state().banks[0].output == 3);
  gpio.abort_instruction();
  require(gpio.state() == before);
  auto unknown = lead_a1::Gpio::reset_reference(pins);
  unknown.revision.reset();
  lead_a1::Gpio no_revision(unknown);
  rejected([&] { static_cast<void>(no_revision.read({.address = 0, .size = 4})); }, "Unknown");
  lead_a1::Gpio partial(lead_a1::Gpio::reset_reference(partial_pins));
  require(partial.read({.address = 0x10, .size = 4}) == partial_directions);
  require(partial.read({.address = last_pair, .size = 4}) == 0);
}

std::vector<std::byte> program(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Missing owned GPIO program"); }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

void cpu_transactions(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  for (const unsigned int instruction : {0U, 2U, 3U}) {
    lead_a1::Gpio gpio(lead_a1::Gpio::reset_reference(pins));
    lead_a1::Memory memory;
    memory.map_device({.address = base, .size = 0x1000, .name = "GPIO", .executable = false}, gpio);
    memory.map({.address = program_base, .size = bytes.size(), .name = "owned GPIO program"},
               bytes);
    lead_a1::ArmState state;
    constexpr std::uint32_t status = 0xb80000df;
    state.cpsr = status;
    const auto failing_offset = instruction == 2 ? last_set : last_clear;
    state.registers[0] = base + (instruction == 0 ? odd_set : failing_offset);
    state.registers[1] = 1;
    state.registers[2] = 2;
    state.registers[3] = 4;
    state.registers[15] = program_base + (instruction * 4);
    lead_a1::Arm cpu(memory, spec, state);
    const auto before = gpio.state();
    if (instruction == 0) {
      require(!cpu.step().exception);
      require(gpio.state().banks[3].output == 1);
      require(!cpu.step().exception && cpu.state().registers[2] == 1);
    } else {
      rejected([&] { static_cast<void>(cpu.step()); }, instruction == 2 ? "Read-only" : "Unknown");
      require(cpu.state() == state && gpio.state() == before);
    }
  }
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    aliases_and_widths();
    samples_and_interrupts();
    invalid_inputs();
    const auto bytes = program(argv[2]);
    cpu_transactions(argv[1], bytes);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
