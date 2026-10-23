// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/arm.hpp"
#include "lead_a1/arm_system.hpp"
#include "lead_a1/gpio.hpp"
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_boot.hpp"
#include "lead_a1/sharc_clock.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_spi.hpp"
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
  std::optional<lead_a1::SharcSpiBoot> dsp;
  std::uint64_t dsp_steps{};
  Recorded<lead_a1::SharcClock> clock{
      lead_a1::SharcClockReference{.configuration = 0, .input_stable = true, .reset_settled = true},
      dsp_steps};
  Recorded<lead_a1::SharcControl> control{std::optional<bool>{true}, dsp_steps};
  std::optional<Recorded<lead_a1::SharcSpi>> runtime;
  std::vector<unsigned int> dma_grants;
  lead_a1::SharcMemory dsp_memory;
  unsigned int dsp_channel{};

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

  void observe_dsp() {
    if (!dsp) { return; }
    const auto select = 1U << (11 + dsp_channel);
    const auto bank = gpio.device.drives()[2];
    const auto serial = spi0.device.pins();
    constexpr std::uint16_t clock_pin = 0x200;
    constexpr std::uint16_t data_pin = 0x400;
    require((bank.enabled & select) != 0);
    require((serial.enabled & (clock_pin | data_pin)) == (clock_pin | data_pin));
    const lead_a1::SharcBootLines lines{.selected = (bank.high & select) == 0,
                                        .clock = (serial.high & clock_pin) != 0,
                                        .mosi = (serial.high & data_pin) != 0};
    if (runtime) {
      runtime->device.observe(dsp_memory, lines);
      // Explicit functional owner grant between ARM instructions/serial edges.
      // No physical IOP bus timing or concurrent DSP execution is asserted.
      const auto count = runtime->device.service_dma(dsp_memory);
      if (count != 0) { dma_grants.push_back(count); }
    } else {
      dsp->observe(dsp_memory, lines);
    }
  }

  void progress() {
    ++ticks;
    observe_dsp();
    const auto interval = spi0.device.next_event();
    if (!interval) {
      due.reset();
      return;
    }
    if (!due) { due = ticks + *interval; }
    while (due && *due <= ticks) {
      const auto before = spi0.device.state().active;
      if (dsp) {
        constexpr std::uint16_t data_pin = 0x400;
        const bool outgoing = (spi0.device.pins().high & data_pin) != 0;
        // Owned external pull-up at the disabled DSP MISO output.
        spi0.device.advance({.received = true, .observed_output = outgoing});
        observe_dsp();
      } else {
        spi0.device.advance();
      }
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
                       std::array<std::uint32_t, 4> args,
                       unsigned int limit = 20000) {
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

void kernel_prefix(lead_a1::SharcMemory &memory, std::uint32_t entry) {
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(entry));
  constexpr unsigned int prefix_instructions = 8;
  for (unsigned int step = 0; step < prefix_instructions; ++step) {
    require(cpu.state().pc == entry + step);
    static_cast<void>(cpu.step());
  }
  constexpr std::uint32_t clock_setup = 0x8c091;
  constexpr std::uint32_t return_frame = 0x108c008;
  constexpr unsigned int imask = 11;
  constexpr unsigned int irptl = 10;
  require(cpu.state().pc == clock_setup && cpu.state().depth == 1);
  require(cpu.state().pc_stack[0] == return_frame && !cpu.state().branch);
  // EMUI/RSTI remain unmaskable (RO bits0/1) despite the original IMASK=0.
  require(cpu.state().system[imask].value == 3 && cpu.state().system[irptl].value == 0);
  const auto before = cpu.state();
  bool stopped{};
  try {
    static_cast<void>(cpu.step());
  } catch (const std::exception &error) {
    const std::string message(error.what());
    require(message.find("native48 pc=0x8c091") != std::string::npos);
    require(message.find("read pc=0x8c091 word-address=0x2000 bits=32") != std::string::npos);
    require(message.find("Unmapped SHARC IOP register") != std::string::npos);
    stopped = true;
  }
  require(stopped && cpu.state() == before && !memory.active());
}

void check_clock_transactions(const Fixture &fixture) {
  constexpr std::array<std::uint32_t, 6> writes{0x248, 0x8148, 0x148, 0x8052, 0x52, 0x40212};
  constexpr std::array<std::uint32_t, 5> reads{8, 0x8148, 0x148, 0x8052, 0x52};
  unsigned int written{};
  unsigned int read{};
  for (const auto &event : fixture.clock.events) {
    require(event.width == 4 && event.offset == 0);
    if (event.write) {
      require(written < writes.size() && event.value == writes[written]);
      ++written;
    } else {
      require(read < reads.size() && event.value == reads[read]);
      ++read;
    }
  }
  require(written == writes.size() && read == reads.size());
}

void flag_setup(Fixture &fixture, lead_a1::Sharc &cpu) {
  constexpr std::uint32_t dag_entry = 0x8c0b4;
  constexpr unsigned int instruction_count = 8293;
  constexpr std::uint32_t flag1_output_select = 8;
  constexpr std::uint32_t core_directions = 0xaa;
  constexpr unsigned int flags = 5;
  for (unsigned int index = 0; index < 2; ++index) {
    static_cast<void>(cpu.step());
    ++fixture.dsp_steps;
  }
  require(cpu.state().pc == dag_entry && fixture.dsp_steps == instruction_count);
  require(cpu.state().system[flags].known == core_directions &&
          cpu.state().system[flags].value == flag1_output_select);
  require(cpu.state().pending[flags] == cpu.state().system[flags]);
  require(cpu.state().effective[flags].known == core_directions &&
          cpu.state().effective[flags].value == 0);
  constexpr std::uint16_t other_flags = 0xfff0;
  require((cpu.flag_drives() ==
           lead_a1::SharcFlagDrive{.high_impedance = 15, .unresolved = other_flags}));
  require(cpu.state().flag_inputs.known == 0); // No ARM connection/pull-up invented.
}

void clear_ram(Fixture &fixture, lead_a1::Sharc &cpu) {
  constexpr std::uint32_t kernel = 0x92000;
  constexpr unsigned int kernel_words = 384;
  std::array<std::uint32_t, kernel_words> preserved{};
  for (unsigned int index = 0; index < preserved.size(); ++index) {
    preserved[index] =
        static_cast<std::uint32_t>(fixture.dsp_memory.read({.address = kernel + index}));
  }

  struct ClearSpan {
    std::uint32_t address;
    unsigned int words;
  };

  constexpr std::array<ClearSpan, 4> spans{{{.address = 0x92180, .words = 0xbe80},
                                            {.address = 0xb2000, .words = 0xbfff},
                                            {.address = 0xc0000, .words = 0x7fff},
                                            {.address = 0xe0000, .words = 0x7fff}}};
  constexpr std::uint32_t tail_marker = 0x6e5d4c3b;
  for (unsigned int index = 1; index < spans.size(); ++index) {
    fixture.dsp_memory.write({.address = spans[index].address + spans[index].words}, tail_marker);
  }
  constexpr std::uint32_t caller = 0x8c008;
  constexpr unsigned int limit = 200000;
  while (cpu.state().pc != caller && fixture.dsp_steps < limit) {
    static_cast<void>(cpu.step());
    ++fixture.dsp_steps;
  }
  constexpr unsigned int expected_instructions = 171758;
  require(cpu.state().pc == caller && fixture.dsp_steps == expected_instructions);
  require(cpu.state().depth == 0 && cpu.state().loops_empty == true && !cpu.state().count_loop);
  constexpr std::uint32_t parent = 0x108c008;
  require(cpu.state().pc_stack[0] == parent); // Physical popped frame retained.
  for (const auto span : spans) {
    for (unsigned int index = 0; index < span.words; ++index) {
      require(fixture.dsp_memory.read({.address = span.address + index}) == 0);
    }
  }
  for (unsigned int index = 1; index < spans.size(); ++index) {
    require(fixture.dsp_memory.read({.address = spans[index].address + spans[index].words}) ==
            tail_marker);
  }
  for (unsigned int index = 0; index < preserved.size(); ++index) {
    require(fixture.dsp_memory.read({.address = kernel + index}) == preserved[index]);
  }
  constexpr std::uint32_t final_index = 0xe7fff;
  constexpr std::uint64_t all_data_bits = 0xffffffffff;
  require(cpu.state().dag[0][0][0].value == final_index);
  require(cpu.state().data[0][0].value == 0 && cpu.state().data[0][0].known == all_data_bits);
  constexpr std::uint16_t remaining = 0xfff0;
  require((cpu.flag_drives() ==
           lead_a1::SharcFlagDrive{.high = 2, .high_impedance = 13, .unresolved = remaining}));
  require(cpu.state().flag_inputs.known == 0 && !cpu.state().pending[5]);
  require(fixture.control.events.empty() && fixture.control.device.state().effective == 4);
}

void loader_frontier(Fixture &fixture, lead_a1::Sharc &cpu) {
  constexpr unsigned int register_instructions = 11;
  for (unsigned int index = 0; index < register_instructions; ++index) {
    static_cast<void>(cpu.step());
    ++fixture.dsp_steps;
  }
  constexpr std::uint32_t compute_pc = 0x8c013;
  constexpr unsigned int instructions = 171769;
  require(cpu.state().pc == compute_pc && fixture.dsp_steps == instructions);
  constexpr std::uint32_t interrupt_enable = 0x1000;
  require(cpu.state().effective[2].value == interrupt_enable);
  for (const unsigned int reg : {0U, 4U, 8U, 12U, 15U}) {
    require(cpu.state().dag[0][2][reg].value == 0);
  }
  require(cpu.state().dag[0][1][5].value == 0 && cpu.state().dag[0][1][13].value == 0);
  require(cpu.state().dag[0][1][6].value == 1 && cpu.state().dag[0][1][14].value == 1);
  constexpr std::uint64_t sysctl_left_justified = 0x400;
  require(cpu.state().data[0][11].value == sysctl_left_justified);
  require(fixture.control.events.size() == 1);
  const auto &read = fixture.control.events.front();
  require(!read.write && read.offset == 0 && read.width == 4 && read.value == 4 &&
          read.instruction == instructions - 1);
  constexpr unsigned int setup_instructions = 5;
  for (unsigned int index = 0; index < setup_instructions; ++index) {
    static_cast<void>(cpu.step());
    ++fixture.dsp_steps;
  }
  constexpr std::uint32_t spi_pc = 0x8c034;
  constexpr unsigned int spi_instructions = 171774;
  require(cpu.state().pc == spi_pc && fixture.dsp_steps == spi_instructions);
  constexpr std::uint32_t saved_caller = 0x108c016;
  require(cpu.state().depth == 1 && cpu.state().pc_stack[0] == saved_caller);
  require(cpu.state().data[0][12] == cpu.state().data[0][11]);
  constexpr std::uint32_t alu_status_mask = 0x43f;
  require((cpu.state().system[6].value & alu_status_mask) == 0);
  constexpr std::uint32_t sysctl_word = 0x30024;
  require(cpu.state().dag[0][0][4].value == sysctl_word && cpu.state().system[15].value == 3);
  const auto before = cpu.state();
  const auto control = fixture.control.device.state();
  bool stopped{};
  try {
    static_cast<void>(cpu.step());
  } catch (const std::exception &error) {
    const std::string message(error.what());
    require(message.find("native48 pc=0x8c034") != std::string::npos);
    require(message.find("write pc=0x8c034 word-address=0x1081 bits=32") != std::string::npos);
    stopped = true;
  }
  require(stopped && cpu.state() == before && fixture.control.device.state() == control &&
          !fixture.dsp_memory.active());
}

struct HeaderInput {
  std::uint32_t routine;
  std::uint32_t endpoint;
  std::uint32_t source;
};

void integer_dispatch(Fixture &fixture, lead_a1::Sharc &cpu) {
  constexpr unsigned int dispatch_steps = 10;
  for (unsigned int index = 0; index < dispatch_steps; ++index) {
    static_cast<void>(cpu.step());
    ++fixture.dsp_steps;
  }
  constexpr std::uint32_t shift_pc = 0x8c04c;
  constexpr unsigned int total = 171801;
  require(cpu.state().pc == shift_pc && fixture.dsp_steps == total);
  require(cpu.state().data[0][0].value == 0 && cpu.state().depth == 0);
  const auto count = static_cast<std::uint32_t>(cpu.state().data[0][2].value >> 8);
  require(count > 1 && cpu.state().system[1].value == count);
  require(cpu.state().data[0][6].value == (std::uint64_t{count - 1} << 8));
  require(cpu.state().dag[0][0][0].value == (cpu.state().data[0][3].value >> 8));
  constexpr std::uint32_t alu_status = 0x43f;
  require((cpu.state().system[6].value & alu_status) == 8);
  const auto before = cpu.state();
  bool stopped{};
  try {
    static_cast<void>(cpu.step());
  } catch (const std::exception &error) {
    const std::string message(error.what());
    require(message.find("native48 pc=0x8c04c") != std::string::npos);
    require(message.find("instruction class not implemented") != std::string::npos);
    stopped = true;
  }
  require(stopped && cpu.state() == before && !fixture.dsp_memory.active());
}

void runtime_header(Fixture &fixture,
                    lead_a1::Sharc &cpu,
                    const std::filesystem::path &spec,
                    HeaderInput input) {
  const auto &boot = fixture.dsp;
  if (!boot) { throw std::logic_error("Missing completed SHARC boot fixture"); }
  fixture.runtime.emplace(boot->state(), fixture.dsp_steps);
  constexpr std::uint32_t dma_index = 0x1080;
  constexpr std::uint32_t header_address = 0x92003;
  constexpr std::uint32_t receiving = 0x8007;
  fixture.dsp_memory.map_io({.first_word = dma_index, .words = 5}, *fixture.runtime);
  constexpr std::uint32_t poll = 0x8c038;
  constexpr unsigned int poll_instructions = 171781;
  for (unsigned int index = 0; index < 7; ++index) {
    static_cast<void>(cpu.step());
    ++fixture.dsp_steps;
  }
  require(cpu.state().pc == poll && fixture.dsp_steps == poll_instructions);
  auto &runtime = fixture.runtime->device;
  require(runtime.state().serial.next_word == header_address &&
          runtime.state().serial.remaining == 3);
  require(runtime.state().modifier == 1 && runtime.dma_request() == 0);
  const auto &events = fixture.runtime->events;
  require(events.size() == 4 && events[0].write && events[0].offset == 4 && events[0].value == 1);
  require(events[1].write && events[1].offset == 0 && events[1].value == header_address);
  require(events[2].write && events[2].offset == 8 && events[2].value == 3);
  require(!events[3].write && events[3].offset == 16 && events[3].value == receiving);
  const auto transmitted = fixture.transmitted.size();
  constexpr unsigned int call_limit = 20000;
  static_cast<void>(call(fixture,
                         spec,
                         input.routine,
                         {input.endpoint, fixture.dsp_channel, input.source, 3},
                         call_limit));
  require(fixture.transmitted.size() == transmitted + 6);
  require(fixture.dma_grants.size() == 1 && fixture.dma_grants.front() == 3);
  require(runtime.state().serial.remaining == 0 &&
          runtime.state().serial.next_word == header_address + 3);
  require(runtime.state().occupied == 0 && !runtime.state().receive && !runtime.state().overflow);
  require(runtime.state().interrupt_pending && !runtime.state().serial.selected);
  constexpr std::uint32_t primary_spi = 0x1000;
  cpu.latch_interrupt(primary_spi); // Explicit HWR default P1I route; IMASK still3.
  runtime.acknowledge_interrupt();
  std::array<std::uint32_t, 3> expected{};
  constexpr std::uint32_t halfword_mask = 0xffff;
  for (unsigned int index = 0; index < expected.size(); ++index) {
    expected[index] = static_cast<std::uint32_t>(
        fixture.memory.read({.address = input.source + (index * 4), .size = 4}));
    require(fixture.dsp_memory.read({.address = header_address + index}) == expected[index]);
    const auto halfword = transmitted + (std::size_t{index} * 2);
    require(fixture.transmitted[halfword] == (expected[index] & halfword_mask));
    require(fixture.transmitted[halfword + 1] == (expected[index] >> 16));
  }
  // Genuine polling, header reads, return and the first tag-dispatch instruction.
  require(expected[0] != 0);
  constexpr unsigned int dispatch_instructions = 10;
  for (unsigned int index = 0; index < dispatch_instructions; ++index) {
    static_cast<void>(cpu.step());
    ++fixture.dsp_steps;
  }
  constexpr std::uint32_t arithmetic_pc = 0x8c01b;
  constexpr unsigned int total = 171791;
  require(cpu.state().pc == arithmetic_pc && fixture.dsp_steps == total);
  require(cpu.state().data[0][8].value == (std::uint64_t{expected[0]} << 8));
  require(cpu.state().data[0][2].value == (std::uint64_t{expected[1]} << 8));
  require(cpu.state().data[0][3].value == (std::uint64_t{expected[2]} << 8));
  require(cpu.state().data[0][0].value == (std::uint64_t{expected[0]} << 8));
  require(cpu.state().depth == 0 && cpu.state().system[10].value == primary_spi);
  require(cpu.state().effective[11].value == 3 && events.size() == 5);
  require(!events[4].write && events[4].value == 7 && events[4].offset == 16);
  integer_dispatch(fixture, cpu);
}

void clock_setup(Fixture &fixture, const std::filesystem::path &spec, HeaderInput input) {
  constexpr std::uint32_t pmctl = 0x2000;
  constexpr std::uint32_t entry = 0x8c000;
  constexpr std::uint32_t flags_entry = 0x8c0b2;
  constexpr unsigned int instruction_limit = 20000;
  constexpr unsigned int instruction_count = 8291;
  constexpr std::uint32_t bypass = 0x8000;
  fixture.dsp_memory.map_io({.first_word = pmctl, .words = 1}, fixture.clock);
  constexpr std::uint32_t sysctl = 0x30024;
  fixture.dsp_memory.map_io({.first_word = sysctl, .words = 1}, fixture.control);
  lead_a1::Sharc cpu(
      fixture.dsp_memory, lead_a1::Sharc::reset_reference_214xx(entry), &fixture.control.device);
  while (cpu.state().pc != flags_entry && fixture.dsp_steps < instruction_limit) {
    const auto events = fixture.clock.events.size();
    const auto step = cpu.step();
    ++fixture.dsp_steps;
    if (fixture.clock.events.size() == events) {
      const bool bypassed = (fixture.clock.device.state().control & bypass) != 0;
      // Owned lower-bound clock reference: internal instructions retire at least
      // one core clock; pure short-loop bubbles are included. In settled bypass
      // those clocks also guarantee CLKIN time. Skip the PMCTL access itself.
      fixture.clock.device.advance({.core_cycles = step.minimum_core_cycles,
                                    .clkin_cycles = bypassed ? step.minimum_core_cycles : 0});
    }
  }
  require(cpu.state().pc == flags_entry && fixture.dsp_steps == instruction_count);
  require(cpu.state().depth == 1 && cpu.state().loops_empty == true && !cpu.state().count_loop);
  constexpr std::uint32_t call_frame = 0x108c008;
  require(cpu.state().pc_stack[0] == call_frame);
  check_clock_transactions(fixture);
  const auto clock = fixture.clock.device.state();
  constexpr std::uint32_t expected_control = 0x40012;
  require(clock.control == expected_control && !clock.pending_dividers && clock.pll_remaining == 0);
  constexpr std::uint32_t final_multiplier = 18;
  const auto rates = fixture.clock.device.rates();
  require((rates.core == lead_a1::SharcClockRatio{.numerator = final_multiplier}));
  require((rates.peripheral == lead_a1::SharcClockRatio{.numerator = 9}));
  constexpr std::uint32_t ep_numerator = 36;
  require(
      (rates.external == lead_a1::SharcClockRatio{.numerator = ep_numerator, .denominator = 5}));
  flag_setup(fixture, cpu);
  clear_ram(fixture, cpu);
  loader_frontier(fixture, cpu);
  runtime_header(fixture, cpu, spec, input);
  require(fixture.clock.device.state() == clock);
}

void kernel_transfer(std::span<const std::byte> image,
                     const std::filesystem::path &spec,
                     unsigned int part) {
  constexpr std::array<std::array<std::uint32_t, 2>, 2> offsets{
      {{0x2c6d78, 0x2ab0f8}, {0x71bc8, 0x701d8}}};
  constexpr unsigned int kernel_bytes = 1536;
  constexpr unsigned int kernel_words = 384;
  constexpr unsigned int instruction_bytes = 6;
  constexpr std::uint32_t reset_word = 0x8c000;
  constexpr std::uint32_t destination = 0x92000;
  for (unsigned int channel = 0; channel < 2; ++channel) {
    Fixture fixture(image, placements[part]);
    const auto endpoint = factory(fixture, spec, placements[part], false);
    fixture.dsp.emplace(lead_a1::SharcSpiBoot::slave_reference_214xx());
    fixture.dsp_memory.map_2148x_5mbit_ram();
    fixture.dsp_channel = channel;
    fixture.spi0.events.clear();
    fixture.gpio.events.clear();
    fixture.transmitted.clear();
    constexpr unsigned int limit = 200000;
    static_cast<void>(call(fixture,
                           spec,
                           placements[part].endpoint + write_block_entry,
                           {endpoint, channel, code + offsets[part][channel], kernel_words},
                           limit));
    require(fixture.dsp->entry() == reset_word && fixture.dsp->state().remaining == 0);
    require(!fixture.dsp->state().selected && fixture.dsp->state().bits == 0);
    require(fixture.transmitted.size() == kernel_bytes / 2);
    const auto kernel = image.subspan(offsets[part][channel], kernel_bytes);
    for (unsigned int offset = 0; offset < kernel_bytes; offset += 2) {
      const auto expected = std::to_integer<std::uint32_t>(kernel[offset]) |
                            (std::to_integer<std::uint32_t>(kernel[offset + 1]) << 8);
      require(fixture.transmitted[offset / 2] == expected);
    }
    for (unsigned int offset = 0; offset < kernel_bytes; offset += instruction_bytes) {
      std::uint64_t expected{};
      for (unsigned int index = 0; index < instruction_bytes; ++index) {
        expected |= std::to_integer<std::uint64_t>(kernel[offset + index]) << (index * 8);
      }
      require(fixture.dsp_memory.read({.address = reset_word + (offset / instruction_bytes),
                                       .width = lead_a1::SharcWidth::wide48},
                                      lead_a1::AccessKind::fetch) == expected);
    }
    require(fixture.dsp->state().next_word == destination + kernel_words);
    const auto select = 1U << (11 + channel);
    require(fixture.gpio.events.size() == 2 && fixture.gpio.events[0].value == select &&
            fixture.gpio.events[1].value == select);
    require(fixture.gpio.events.front().offset == gpio_clear &&
            fixture.gpio.events.back().offset == 0x40);
    constexpr std::uint16_t pullup_halfword = 0xffff;
    require(fixture.memory.read({.address = placements[part].scratch, .size = 2}) ==
            pullup_halfword);
    kernel_prefix(fixture.dsp_memory, reset_word);
    clock_setup(fixture,
                spec,
                {.routine = placements[part].endpoint + write_block_entry,
                 .endpoint = endpoint,
                 .source = code + offsets[part][channel] + kernel_bytes});
    std::cout << "Part " << part << " transport " << channel
              << ": original ARM/SPI kernel+header loading and 171801 SHARC instructions through "
                 "tag arithmetic, register transfers and code-record selection; immediate shift "
                 "remains unsupported\n";
  }
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
      kernel_transfer(image, argv[1], part);
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
