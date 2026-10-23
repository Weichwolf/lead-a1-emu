// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/arm.hpp"
#include "lead_a1/memory.hpp"
#include "lead_a1/spi.hpp"

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

constexpr std::uint32_t base = 0x01c41000;
constexpr std::uint32_t code = 0x1000;
constexpr std::uint32_t status = 0xb80000df;
constexpr std::uint32_t data0 = 0x38;
constexpr std::uint32_t buffer = 0x40;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SPI CPU expectation at line " + std::to_string(where.line()));
  }
}

std::vector<std::byte> program(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Missing owned SPI CPU program"); }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

struct Fixture {
  lead_a1::Spi spi{{.chip_selects = 1, .previous_hold = false}};
  lead_a1::Memory memory;

  explicit Fixture(std::span<const std::byte> bytes) {
    memory.map({.address = code, .size = bytes.size(), .name = "owned SPI program"}, bytes);
    memory.map_device({.address = base, .size = 0x1000, .name = "SPI", .executable = false}, spi);
    spi.write({.address = 0, .size = 4}, 1);
    constexpr std::uint32_t serial_pins = 0xe01;
    constexpr std::uint32_t format0 = 0x50;
    constexpr std::uint32_t format = 0x210;
    constexpr std::uint32_t loopback = 0x01010003;
    constexpr std::uint32_t pin_function = 20;
    spi.write({.address = pin_function, .size = 4}, serial_pins);
    spi.write({.address = format0, .size = 4}, format);
    spi.write({.address = 4, .size = 4}, loopback);
  }

  void seed() {
    spi.write({.address = data0, .size = 4}, 1);
    spi.write({.address = data0, .size = 4}, 2);
    constexpr unsigned int limit = 128;
    for (unsigned int event = 0; spi.next_event() && event < limit; ++event) { spi.advance(); }
    require(!spi.next_event());
  }
};

lead_a1::ArmState initial(unsigned int index) {
  lead_a1::ArmState state;
  state.cpsr = status;
  for (unsigned int reg = 0; reg < 15; ++reg) { state.registers[reg] = reg; }
  state.registers[0] = base + (index == 0 ? data0 : buffer);
  state.registers[15] = code + (index * 4);
  return state;
}

void execute(const std::filesystem::path &spec, std::span<const std::byte> bytes) {
  for (const unsigned int index : {0U, 1U}) {
    Fixture fixture(bytes);
    if (index == 1) { fixture.seed(); }
    const auto state = initial(index);
    const auto peripheral = fixture.spi.state();
    lead_a1::Arm cpu(fixture.memory, spec, state);
    const auto before = cpu.state();
    require(before.registers == state.registers && before.cpsr == state.cpsr);
    bool rejected{};
    try {
      static_cast<void>(cpu.step());
    } catch (const std::runtime_error &error) {
      const std::string text = error.what();
      require(text.find(index == 0 ? "Read-only" : "Unmapped SPI") != std::string::npos);
      rejected = true;
    }
    require(rejected);
    require(cpu.state() == before);
    require(fixture.spi.state() == peripheral);
  }
  Fixture fixture(bytes);
  fixture.seed();
  lead_a1::Arm cpu(fixture.memory, spec, initial(2));
  require(!cpu.step().exception && cpu.state().registers[2] == 0);
  require(fixture.spi.state().visible.data == 1 && fixture.spi.state().visible_full);
  require(!cpu.step().exception && cpu.state().registers[2] == 1);
  require(fixture.spi.state().visible.data == 2 && !fixture.spi.state().receive);
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    const auto bytes = program(argv[2]);
    execute(argv[1], bytes);
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
