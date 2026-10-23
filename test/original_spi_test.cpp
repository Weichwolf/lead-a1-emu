// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/arm.hpp"
#include "lead_a1/arm_system.hpp"
#include "lead_a1/gpio.hpp"
#include "lead_a1/memory.hpp"
#include "lead_a1/spi.hpp"

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

constexpr std::uint32_t code = 0xc0000000;
constexpr std::uint32_t spi0_base = 0x01c41000;
constexpr std::uint32_t spi1_base = 0x01e12000;
constexpr std::uint32_t gpio_base = 0x01e26000;
constexpr std::uint32_t object = 0x6000;
constexpr std::uint32_t buffer = 0x6800;
constexpr std::uint32_t stack = 0x8000;
constexpr std::uint32_t sentinel = 0x1000;
constexpr std::uint32_t clock_input = 200000000;
constexpr std::uint32_t requested_frequency = 44444000;
constexpr std::uint32_t expected_format = 0x14120210;
constexpr std::uint32_t template1 = 0x01010000;
constexpr std::uint32_t enabled = 0x01000003;
constexpr std::uint32_t loopback = 0x01010003;
constexpr std::uint32_t data0 = 0x38;
constexpr std::uint32_t data1 = 0x3c;
constexpr std::uint32_t receive_data = 0x40;
constexpr std::uint32_t format1 = 0x54;
constexpr std::uint32_t gpio_mask = 0x7800;
constexpr std::uint32_t backend_member = 0x4c;
constexpr std::uint32_t endpoint_member = 0x7c;
constexpr unsigned int constructor_entry = 192;
constexpr unsigned int enable_entry = 292;
constexpr unsigned int configure_entry = 472;
constexpr unsigned int write_block_entry = 292;
constexpr unsigned int read_block_entry = 380;
constexpr unsigned int write_halfwords_entry = 468;
constexpr unsigned int gpio_pins = 144;
constexpr std::uint32_t serial_pins = 0xe01;
constexpr std::uint32_t delay_fields = 0x01010000;
constexpr std::uint32_t gpio_clear = 0x44;

struct Placement {
  std::uint32_t controller;
  std::uint32_t factory;
  std::uint32_t endpoint;
  std::uint32_t clock;
  std::uint32_t bins;
  std::uint32_t bss;
  std::uint32_t heap;
  std::uint32_t globals;
  std::uint32_t scratch;
};

constexpr std::array<Placement, 2> placements{{{.controller = 0xc018c998,
                                                .factory = 0xc0173494,
                                                .endpoint = 0xc01758a4,
                                                .clock = 0xc02f7e38,
                                                .bins = 0xc030012c,
                                                .bss = 0xc0300550,
                                                .heap = 0xc0774ac8,
                                                .globals = 0xc0767fdc,
                                                .scratch = 0xc076807c},
                                               {.controller = 0xc0042aec,
                                                .factory = 0xc002ef88,
                                                .endpoint = 0xc0031398,
                                                .clock = 0xc0087948,
                                                .bins = 0xc008a014,
                                                .bss = 0xc008a438,
                                                .heap = 0xc00a9690,
                                                .globals = 0xc00a8174,
                                                .scratch = 0xc00a8214}}};

struct Event {
  std::uint32_t offset;
  unsigned int width;
  std::uint64_t value;
  bool write;
  std::uint64_t instruction;
};

template <typename Device> class Recorded final : public lead_a1::MemoryDevice {
public:
  template <typename Initial>
  Recorded(const Initial &initial, const std::uint64_t &clock) : device(initial), clock_(clock) {}

  std::uint64_t read(lead_a1::MemoryAccess access) override {
    const auto value = device.read(access);
    events.push_back({.offset = access.address,
                      .width = access.size,
                      .value = value,
                      .write = false,
                      .instruction = clock_});
    return value;
  }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    device.write(access, value);
    events.push_back({.offset = access.address,
                      .width = access.size,
                      .value = value,
                      .write = true,
                      .instruction = clock_});
  }

  void begin_instruction() override {
    checkpoint = events.size();
    device.begin_instruction();
  }

  void commit_instruction() noexcept override { device.commit_instruction(); }

  void abort_instruction() noexcept override {
    events.resize(checkpoint);
    device.abort_instruction();
  }

  Device device;
  std::vector<Event> events;
  std::size_t checkpoint{};

  const std::uint64_t &clock_;
};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Original SPI expectation at line " + std::to_string(where.line()));
  }
}

std::vector<std::byte> read_image(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Missing original SPI image"); }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

struct Fixture {
  std::uint64_t ticks{};
  Recorded<lead_a1::Spi> spi0{lead_a1::SpiResetReference{.chip_selects = 3, .previous_hold = {}},
                              ticks};
  Recorded<lead_a1::Spi> spi1{lead_a1::SpiResetReference{.chip_selects = 3, .previous_hold = {}},
                              ticks};
  Recorded<lead_a1::Gpio> gpio{lead_a1::Gpio::reset_reference(gpio_pins), ticks};
  lead_a1::Memory memory;
  std::optional<std::uint64_t> due;
  std::vector<std::uint16_t> transmitted;

  Fixture(std::span<const std::byte> image, Placement placement) {
    constexpr std::uint32_t bin_bytes = 1040;
    const std::array<lead_a1::MemoryAccess, 2> data{
        {{.address = placement.clock, .size = 4}, {.address = placement.bins, .size = bin_bytes}}};
    std::uint32_t cursor = code;
    for (const auto &region : data) {
      const auto prefix = image.subspan(cursor - code, region.address - cursor);
      memory.map({.address = cursor,
                  .size = prefix.size(),
                  .name = "original executable CODE",
                  .writable = false},
                 prefix);
      memory.map({.address = region.address,
                  .size = region.size,
                  .name = "original mutable data",
                  .executable = false},
                 image.subspan(region.address - code, region.size));
      cursor = region.address + region.size;
    }
    const auto suffix = image.subspan(cursor - code);
    memory.map({.address = cursor,
                .size = suffix.size(),
                .name = "original CODE suffix",
                .writable = false},
               suffix);
    constexpr unsigned int heap_bytes = 0x8000;
    const std::vector<std::byte> zeros(placement.heap - placement.bss + heap_bytes);
    memory.map({.address = placement.bss,
                .size = zeros.size(),
                .name = "owned zero BSS/heap",
                .executable = false},
               zeros);
    constexpr std::uint32_t caller = 0x4000;
    constexpr std::size_t caller_bytes = 0x5000;
    const std::vector<std::byte> pattern(caller_bytes, std::byte{0xa5});
    memory.map({.address = caller, .size = pattern.size(), .name = "owned caller RAM"}, pattern);
    memory.map_device({.address = spi0_base, .size = 0x1000, .name = "SPI0", .executable = false},
                      spi0);
    memory.map_device({.address = spi1_base, .size = 0x1000, .name = "SPI1", .executable = false},
                      spi1);
    memory.map_device({.address = gpio_base, .size = 0x1000, .name = "GPIO", .executable = false},
                      gpio);
    memory.write({.address = placement.clock, .size = 4}, clock_input);
  }

  void progress() {
    ++ticks;
    const auto interval = spi0.device.next_event();
    if (!interval) {
      due.reset();
      return;
    }
    if (!due) { due = ticks + *interval; }
    while (due && *due <= ticks) {
      const auto before = spi0.device.state().active;
      spi0.device.advance();
      const auto after = spi0.device.state().active;
      if (before && after && !before->receive_committed && after->receive_committed) {
        transmitted.push_back(before->word.data);
      }
      const auto next = spi0.device.next_event();
      if (next) {
        *due += *next;
      } else {
        due.reset();
      }
    }
  }
};

lead_a1::ArmSystemInitial profile() {
  constexpr std::uint32_t control = 0x50078;
  return {.control = control,
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

lead_a1::ArmState call(Fixture &fixture,
                       const std::filesystem::path &spec,
                       std::uint32_t entry,
                       std::array<std::uint32_t, 4> args) {
  lead_a1::ArmState initial;
  constexpr std::uint32_t status = 0xb80000df;
  constexpr std::uint32_t marker = 0x11223300;
  initial.cpsr = status;
  for (unsigned int reg = 0; reg < 13; ++reg) { initial.registers[reg] = marker + reg; }
  for (unsigned int reg = 0; reg < args.size(); ++reg) { initial.registers[reg] = args[reg]; }
  initial.registers[13] = stack;
  initial.registers[14] = sentinel;
  initial.registers[15] = entry;
  lead_a1::Arm cpu(fixture.memory, spec, initial, profile());
  constexpr unsigned int limit = 20000;
  unsigned int count{};
  while (cpu.state().registers[15] != sentinel && count < limit) {
    require(!cpu.step().exception);
    fixture.progress();
    ++count;
  }
  const auto result = cpu.state();
  require(result.registers[15] == sentinel && count < limit && result.registers[13] == stack);
  for (unsigned int reg = 4; reg < 12; ++reg) { require(result.registers[reg] == marker + reg); }
  return result;
}

void controllers(Fixture &fixture, const std::filesystem::path &spec, Placement placement) {
  for (const unsigned int index : {0U, 1U}) {
    static_cast<void>(
        call(fixture, spec, placement.controller + constructor_entry, {object, index, 0, 0}));
    require(fixture.memory.read({.address = object, .size = 4}) ==
            (index == 0 ? spi0_base : spi1_base));
    const auto state = index == 0 ? fixture.spi0.device.state() : fixture.spi1.device.state();
    require(state.control == enabled && state.pin_function == serial_pins &&
            state.delay == delay_fields);
    require(state.defaults == 3 && !state.previous_hold && !state.active);
    static_cast<void>(call(fixture, spec, placement.controller + enable_entry, {object, 2, 0, 0}));
    const auto disabled = index == 0 ? fixture.spi0.device.state() : fixture.spi1.device.state();
    require(disabled.control == 3);
    fixture.memory.write({.address = stack, .size = 4}, 1);
    fixture.memory.write({.address = stack + 4, .size = 4}, 0);
    fixture.memory.write({.address = stack + 8, .size = 4}, 16);
    const auto &events = index == 0 ? fixture.spi0.events : fixture.spi1.events;
    const auto checkpoint = events.size();
    const auto result = call(
        fixture, spec, placement.controller + configure_entry, {object, 1, requested_frequency, 1});
    constexpr std::uint32_t frequency = 33333333;
    require(result.registers[0] == frequency);
    const auto formatted = index == 0 ? fixture.spi0.device.state() : fixture.spi1.device.state();
    require(formatted.formats[1] == expected_format);
    require(fixture.memory.read({.address = object + 8, .size = 4}) == template1);
    constexpr std::array<std::uint32_t, 10> expected{0x14000000,
                                                     0x14000000,
                                                     0x14000000,
                                                     0x14000000,
                                                     0x14100000,
                                                     0x14100000,
                                                     0x14120000,
                                                     0x14120000,
                                                     0x14120200,
                                                     0x14120210};
    require(events.size() == checkpoint + (expected.size() * 2));
    for (std::size_t step = 0; step < expected.size(); ++step) {
      const auto &load = events[checkpoint + (step * 2)];
      const auto &store = events[checkpoint + (step * 2) + 1];
      require(!load.write && load.offset == format1 && load.width == 4);
      require(load.value == (step == 0 ? 0 : expected[step - 1]));
      require(store.write && store.offset == format1 && store.width == 4 &&
              store.value == expected[step]);
    }
  }
}

std::uint32_t
factory(Fixture &fixture, const std::filesystem::path &spec, Placement placement, bool reused) {
  const auto result = call(fixture, spec, placement.factory, {0, 0, 0, 0});
  const auto pointer = result.registers[0];
  if (!pointer) { throw std::logic_error("Unknown original endpoint pointer"); }
  const auto endpoint = *pointer;
  const auto expected = placement.heap + (reused ? 80 : 32);
  require(endpoint == expected);
  require(fixture.memory.read({.address = placement.globals + endpoint_member, .size = 4}) ==
          endpoint);
  const auto backend =
      fixture.memory.read({.address = placement.globals + backend_member, .size = 4});
  require(backend == placement.heap + 8);
  require(fixture.memory.read({.address = endpoint + 4, .size = 4}) == backend);
  require(fixture.memory.read({.address = endpoint + 8, .size = 1}) == 1);
  const auto state = fixture.spi0.device.state();
  require(state.formats[1] == expected_format && state.control == enabled && !state.previous_hold);
  require(fixture.gpio.device.state().banks[2].output == gpio_mask);
  require(fixture.gpio.device.drives()[2].high == gpio_mask);
  return endpoint;
}

void check_transfers(Fixture &fixture, unsigned int count, bool zeros) {
  require(fixture.transmitted.size() == count);
  if (!zeros) {
    for (unsigned int index = 0; index < count; ++index) {
      require(fixture.transmitted[index] == 0x100 + index);
    }
  } else {
    for (const auto word : fixture.transmitted) { require(word == 0); }
  }
  unsigned int writes{};
  unsigned int reads{};
  for (const auto &event : fixture.spi0.events) {
    if (event.write && (event.offset == data0 || event.offset == data1)) {
      require(event.width == 4);
      ++writes;
    }
    if (!event.write && event.offset == receive_data && event.width == 4) { ++reads; }
  }
  require(writes == count && reads == count + (zeros ? 0 : 1));
}

void transport(Fixture &fixture,
               const std::filesystem::path &spec,
               Placement placement,
               std::uint32_t endpoint) {
  fixture.spi0.device.write({.address = 4, .size = 4}, loopback);
  for (unsigned int index = 0; index < 8; ++index) {
    fixture.memory.write({.address = buffer + (index * 2), .size = 2}, 0x100 + index);
  }
  for (const unsigned int words : {1U, 3U}) {
    fixture.spi0.events.clear();
    fixture.gpio.events.clear();
    fixture.transmitted.clear();
    static_cast<void>(
        call(fixture, spec, placement.endpoint + write_block_entry, {endpoint, 1, buffer, words}));
    check_transfers(fixture, words * 2, false);
    require(fixture.gpio.events.size() == 2 && fixture.gpio.events[0].width == 2);
    require(fixture.gpio.events[0].offset == gpio_clear && fixture.gpio.events[0].value == 0x1000);
    require(fixture.gpio.events[1].offset == 0x40 && fixture.gpio.events[1].value == 0x1000);
    require(fixture.spi0.events.front().offset == data1 + 3 &&
            fixture.spi0.events.front().width == 1);
    require(fixture.spi0.events.front().instruction < fixture.gpio.events[0].instruction);
    require(fixture.gpio.events[1].instruction > fixture.spi0.events.back().instruction);
    require(fixture.memory.read({.address = placement.scratch, .size = 2}) ==
            0x100 + (words * 2) - 1);
  }
  for (const unsigned int count : {0U, 1U}) {
    fixture.spi0.events.clear();
    fixture.transmitted.clear();
    static_cast<void>(call(
        fixture, spec, placement.endpoint + write_halfwords_entry, {endpoint, buffer, count, 0}));
    check_transfers(fixture, 2, false);
  }
  fixture.spi0.events.clear();
  fixture.gpio.events.clear();
  fixture.transmitted.clear();
  constexpr std::uint32_t guard = 0xa5a5;
  fixture.memory.write({.address = buffer + 8, .size = 2}, guard);
  static_cast<void>(
      call(fixture, spec, placement.endpoint + read_block_entry, {endpoint, 0, buffer, 2}));
  check_transfers(fixture, 4, true);
  for (unsigned int offset = 0; offset < 8; offset += 2) {
    require(fixture.memory.read({.address = buffer + offset, .size = 2}) == 0);
  }
  require(fixture.memory.read({.address = buffer + 8, .size = 2}) == guard);
  require(fixture.gpio.events[0].value == 0x800 && fixture.gpio.events[1].value == 0x800);
  require(!fixture.spi0.device.state().visible_full && !fixture.spi0.device.state().receive);
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    for (unsigned int part = 0; part < placements.size(); ++part) {
      const auto placement = placements[part];
      const auto image =
          read_image(std::filesystem::path(argv[2]) / (std::to_string(part) + "-CODE.bin"));
      Fixture fixture(image, placement);
      controllers(fixture, argv[1], placement);
      static_cast<void>(factory(fixture, argv[1], placement, false));
      const auto endpoint = factory(fixture, argv[1], placement, true);
      transport(fixture, argv[1], placement, endpoint);
      std::cout << "Part " << part
                << ": original SPI controllers/format/factory/heap/GPIO and transport"
                   " execute against clocked serial loopback and real buffer backpressure\n";
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
