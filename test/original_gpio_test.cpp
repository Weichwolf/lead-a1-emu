// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/arm.hpp"
#include "lead_a1/arm_system.hpp"
#include "lead_a1/gpio.hpp"
#include "lead_a1/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t code = 0xc0000000;
constexpr std::uint32_t base = 0x01e26000;
constexpr std::uint32_t object = 0x6000;
constexpr std::uint32_t board = 0x5000;
constexpr std::uint32_t stack = 0x8000;
constexpr std::uint32_t sentinel = 0x1000;
constexpr std::uint32_t status = 0xb80000df;
constexpr std::uint32_t marker = 0x11223300;
constexpr unsigned int pins = 144;
constexpr std::uint16_t pattern = 0xa5a5;
constexpr std::uint32_t pair_stride = 40;
constexpr unsigned int direction_entry = 104;
constexpr unsigned int direction_zero_flags = 0x680000df;
constexpr unsigned int direction_nonzero_flags = 0x280000df;
constexpr unsigned int object_bytes = 20;
constexpr std::byte guard_byte{0xa5};
constexpr std::uint16_t all_pins = 0xffff;

struct Placement {
  std::uint32_t gpio;
  std::uint32_t board;
};

struct HeapInput {
  std::uint32_t bins;
  std::uint32_t bss;
  std::uint32_t start;
  std::uint32_t break_word;
};

constexpr std::array<HeapInput, 2> heap_inputs{
    {{.bins = 0xc030012c, .bss = 0xc0300550, .start = 0xc0774ac8, .break_word = 0xc0767e64},
     {.bins = 0xc008a014, .bss = 0xc008a438, .start = 0xc00a9690, .break_word = 0xc009cafc}}};

constexpr std::array<Placement, 2> placements{
    {{.gpio = 0xc0172460, .board = 0xc00065cc}, {.gpio = 0xc002e1b4, .board = 0xc000189c}}};

struct Event {
  std::uint32_t offset;
  unsigned int width;
  bool write;
  std::uint64_t value;
  bool operator==(const Event &) const = default;
};

class RecordedGpio final : public lead_a1::MemoryDevice {
public:
  std::uint64_t read(lead_a1::MemoryAccess access) override {
    const auto value = device.read(access);
    events.push_back(
        {.offset = access.address, .width = access.size, .write = false, .value = value});
    return value;
  }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    device.write(access, value);
    events.push_back(
        {.offset = access.address, .width = access.size, .write = true, .value = value});
  }

  void begin_instruction() override {
    checkpoint = events.size();
    device.begin_instruction();
  }

  void commit_instruction() noexcept override { device.commit_instruction(); }

  void abort_instruction() noexcept override {
    device.abort_instruction();
    events.resize(checkpoint);
  }

  lead_a1::Gpio device{lead_a1::Gpio::reset_reference(pins)};
  std::vector<Event> events;
  std::size_t checkpoint{};
};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Original GPIO expectation at line " + std::to_string(where.line()));
  }
}

std::vector<std::byte> read(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Missing original GPIO input"); }
  const auto view = std::as_bytes(std::span(bytes));
  return {view.begin(), view.end()};
}

struct Fixture {
  RecordedGpio gpio;
  lead_a1::Memory memory;

  explicit Fixture(std::span<const std::byte> image, std::optional<HeapInput> heap = {}) {
    if (heap) {
      constexpr std::uint32_t bin_bytes = 1040;
      const auto offset = heap->bins - code;
      require(offset <= image.size() && image.size() - offset >= bin_bytes);
      const auto prefix = image.first(offset);
      const auto suffix = image.subspan(offset + bin_bytes);
      memory.map({.address = code, .size = prefix.size(), .name = "CODE prefix", .writable = false},
                 prefix);
      memory.map({.address = heap->bins,
                  .size = bin_bytes,
                  .name = "original heap bins",
                  .executable = false},
                 image.subspan(offset, bin_bytes));
      memory.map({.address = heap->bins + bin_bytes,
                  .size = suffix.size(),
                  .name = "CODE suffix",
                  .writable = false},
                 suffix);
      constexpr unsigned int heap_bytes = 0x4000;
      const std::vector<std::byte> zeros(heap->start - heap->bss + heap_bytes);
      memory.map({.address = heap->bss,
                  .size = zeros.size(),
                  .name = "owned zero BSS and heap",
                  .executable = false},
                 zeros);
    } else {
      memory.map(
          {.address = code, .size = image.size(), .name = "original CODE", .writable = false},
          image);
    }
    constexpr std::uint32_t ram = 0x4000;
    constexpr std::size_t ram_bytes = 0x5000;
    const std::vector<std::byte> initial(ram_bytes, std::byte{0xa5});
    memory.map({.address = ram, .size = initial.size(), .name = "owned caller RAM"}, initial);
    memory.map_device(
        {.address = base, .size = 0x1000, .name = "GPIO reference", .executable = false}, gpio);
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
  initial.cpsr = status;
  for (unsigned int reg = 0; reg < 13; ++reg) { initial.registers[reg] = marker + reg; }
  for (unsigned int reg = 0; reg < args.size(); ++reg) { initial.registers[reg] = args[reg]; }
  initial.registers[13] = stack;
  initial.registers[14] = sentinel;
  initial.registers[15] = entry;
  lead_a1::Arm cpu(fixture.memory, spec, initial, profile());
  constexpr unsigned int limit = 100000;
  unsigned int count{};
  while (cpu.state().registers[15] != sentinel && count < limit) {
    require(!cpu.step().exception);
    ++count;
  }
  const auto after = cpu.state();
  require(after.registers[15] == sentinel && count < limit && after.registers[13] == stack);
  for (unsigned int reg = 4; reg < 12; ++reg) { require(after.registers[reg] == marker + reg); }
  return after;
}

void bank_objects(Fixture &fixture, const std::filesystem::path &spec, Placement placement) {
  constexpr auto maximum = std::numeric_limits<std::uint32_t>::max();
  constexpr std::array<std::uint32_t, 12> banks{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0x80000000, maximum};
  for (const auto bank : banks) {
    const auto result = call(fixture, spec, placement.gpio, {object, bank, 7, 8});
    const auto wide = std::uint64_t{base} + (static_cast<std::uint64_t>(bank >> 1) * pair_stride);
    const auto pair = wide & maximum;
    const auto lane = 2 * (bank & 1);
    const std::array<std::uint64_t, 5> expected{
        pair + 16, pair + 24 + lane, pair + 28 + lane, pair + 32 + lane, pair + 16 + lane};
    for (unsigned int word = 0; word < expected.size(); ++word) {
      require(fixture.memory.read({.address = object + (word * 4), .size = 4}) ==
              (expected[word] & maximum));
    }
    for (unsigned int offset = object_bytes; offset < 32; ++offset) {
      require(fixture.memory.peek(object + offset) == guard_byte);
    }
    require(result.cpsr == status && result.registers[0] == object);
    require(fixture.gpio.events.empty());
  }
}

void directions(Fixture &fixture, const std::filesystem::path &spec, Placement placement) {
  constexpr auto maximum = std::numeric_limits<std::uint32_t>::max();
  constexpr std::array<std::uint32_t, 7> indices{0, 15, 16, 31, 32, 256, maximum};
  for (const unsigned int bank : {2U, 3U, 8U}) {
    static_cast<void>(call(fixture, spec, placement.gpio, {object, bank, 0, 0}));
    const auto offset = 16 + ((bank / 2) * pair_stride) + ((bank & 1) * 2);
    for (const auto pin : indices) {
      for (const unsigned int direction : {0U, 2U}) {
        fixture.gpio.device.write({.address = offset, .size = 2}, pattern);
        const auto before = fixture.gpio.device.state();
        const auto result =
            call(fixture, spec, placement.gpio + direction_entry, {object, pin, direction, 0});
        const auto shift = pin & 0xff;
        const auto bit = shift < 16 ? 1U << shift : 0;
        const auto expected = direction == 0 ? pattern & ~bit : pattern | bit;
        const std::vector<Event> events{
            {.offset = offset, .width = 2, .write = false, .value = pattern},
            {.offset = offset, .width = 2, .write = true, .value = expected}};
        require(fixture.gpio.events == events);
        auto after = before;
        after.banks[bank].direction = static_cast<std::uint16_t>(expected);
        require(fixture.gpio.device.state() == after);
        require(result.cpsr == (direction == 0 ? direction_zero_flags : direction_nonzero_flags));
        require(result.registers[0] == 1);
        fixture.gpio.events.clear();
      }
    }
  }
}

void prepare_board(Fixture &fixture, const std::filesystem::path &spec, Placement placement) {
  constexpr std::uint32_t member = 0x9c;
  for (unsigned int index = 0; index < 3; ++index) {
    const auto pointer = object + (index * 32);
    static_cast<void>(call(fixture, spec, placement.gpio, {pointer, 3 + index, 0, 0}));
    fixture.memory.write({.address = board + member + (index * 4), .size = 4}, pointer);
  }
}

struct Setter {
  std::uint32_t entry;
  unsigned int bank;
  unsigned int pin;
  bool inverted;
};

void board_access(Fixture &fixture, const std::filesystem::path &spec, Placement placement) {
  constexpr std::array<Setter, 4> setters{
      {{.entry = 332, .bank = 4, .pin = 14, .inverted = true},
       {.entry = 392, .bank = 4, .pin = 13, .inverted = true},
       {.entry = 432, .bank = 3, .pin = 8, .inverted = true},
       {.entry = 468, .bank = 4, .pin = 12, .inverted = false}}};
  for (const auto &setter : setters) {
    const auto bank_offset = (setter.bank / 2) * pair_stride;
    const auto lane = 2 * (setter.bank & 1);
    fixture.gpio.device.write({.address = 16 + bank_offset + lane, .size = 2}, 0);
    for (const unsigned int value : {0U, 2U}) {
      fixture.gpio.device.write({.address = object_bytes + bank_offset + lane, .size = 2}, pattern);
      const auto before = fixture.gpio.device.state();
      static_cast<void>(call(fixture, spec, placement.board + setter.entry, {board, value, 0, 0}));
      const bool high = setter.inverted ? value == 0 : value != 0;
      const auto bit = 1U << setter.pin;
      const auto offset = (high ? 24U : 28U) + bank_offset + lane;
      const std::vector<Event> expected{
          {.offset = offset, .width = 2, .write = true, .value = bit}};
      require(fixture.gpio.events == expected);
      auto after = before;
      after.banks[setter.bank].output =
          static_cast<std::uint16_t>(high ? pattern | bit : pattern & ~bit);
      require(fixture.gpio.device.state() == after);
      require(((fixture.gpio.device.drives()[setter.bank].high & bit) != 0) == high);
      fixture.gpio.events.clear();
    }
  }
  constexpr unsigned int reader_entry = 368;
  constexpr std::array<std::uint16_t, 4> samples{0, 0x40, 0x80, 0xffff};
  for (const auto value : samples) {
    fixture.gpio.device.sample(4, {.value = value, .known = all_pins});
    const auto result = call(fixture, spec, placement.board + reader_entry, {board, 0, 0, 0});
    require(result.registers[0] == ((value >> 6) & 1));
    constexpr std::uint32_t input_offset = 0x70;
    const std::vector<Event> expected{
        {.offset = input_offset, .width = 2, .write = false, .value = value}};
    require(fixture.gpio.events == expected);
    fixture.gpio.events.clear();
  }
}

void unresolved_external(Fixture &fixture, const std::filesystem::path &spec, Placement placement) {
  constexpr unsigned int entry = 504;
  const auto before = fixture.gpio.device.state();
  try {
    static_cast<void>(call(fixture, spec, placement.board + entry, {board, 0, 0, 0}));
  } catch (const std::runtime_error &error) {
    const std::string text = error.what();
    require(text.find("62004000") != std::string::npos &&
            text.find("width=1") != std::string::npos);
    require(fixture.gpio.device.state() == before && fixture.gpio.events.empty());
    return;
  }
  throw std::runtime_error("Original unknown external byte access unexpectedly succeeded");
}

struct DirectionStep {
  std::uint32_t offset;
  std::uint16_t before;
  std::uint16_t after;
};

std::vector<Event> initializer_events(unsigned int part) {
  constexpr std::array<Event, 7> writes{
      {{.offset = 0x46, .width = 2, .write = true, .value = 0x100},
       {.offset = 0x68, .width = 2, .write = true, .value = 0x1000},
       {.offset = 0x6c, .width = 2, .write = true, .value = 0x2000},
       {.offset = 0x6c, .width = 2, .write = true, .value = 0x100},
       {.offset = 0x6c, .width = 2, .write = true, .value = 0x4000},
       {.offset = 0x68, .width = 2, .write = true, .value = 8},
       {.offset = 0x68, .width = 2, .write = true, .value = 16}}};
  constexpr std::array<DirectionStep, 9> directions{
      {{.offset = 0x3a, .before = 0xffff, .after = 0xfeff},
       {.offset = 0x60, .before = 0xffff, .after = 0xefff},
       {.offset = 0x60, .before = 0xefff, .after = 0xcfff},
       {.offset = 0x60, .before = 0xcfff, .after = 0xceff},
       {.offset = 0x60, .before = 0xceff, .after = 0xceff},
       {.offset = 0x60, .before = 0xceff, .after = 0x8eff},
       {.offset = 0x60, .before = 0x8eff, .after = 0x8ef7},
       {.offset = 0x60, .before = 0x8ef7, .after = 0x8ee7},
       {.offset = 0x62, .before = 0xffff, .after = 0xffff}}};
  std::vector<Event> result(writes.begin(), writes.end());
  constexpr std::uint32_t variant_set = 0x68;
  if (part == 1) { result[4].offset = variant_set; }
  for (const auto &step : directions) {
    result.push_back({.offset = step.offset, .width = 2, .write = false, .value = step.before});
    result.push_back({.offset = step.offset, .width = 2, .write = true, .value = step.after});
  }
  return result;
}

void board_initializer(std::span<const std::byte> image,
                       const std::filesystem::path &spec,
                       unsigned int part) {
  const auto heap = heap_inputs[part];
  Fixture fixture(image, heap);
  constexpr unsigned int initialize_entry = 40;
  constexpr unsigned int member = 0x9c;
  constexpr unsigned int chunk_bytes = 24;
  constexpr unsigned int in_use_size = 25;
  const auto result =
      call(fixture, spec, placements[part].board + initialize_entry, {board, 0, 0, 0});
  require(result.registers[0] == 1 && result.cpsr == direction_nonzero_flags);
  require(fixture.gpio.events == initializer_events(part));
  for (unsigned int index = 0; index < 3; ++index) {
    const auto pointer = heap.start + 8 + (index * chunk_bytes);
    require(fixture.memory.read({.address = board + member + (index * 4), .size = 4}) == pointer);
    require(fixture.memory.read({.address = pointer - 4, .size = 4}) == in_use_size);
    const auto bank = index + 3;
    const auto pair = base + ((bank / 2) * pair_stride);
    const auto lane = (bank & 1) * 2;
    const std::array<std::uint32_t, 5> expected{
        pair + 16, pair + 24 + lane, pair + 28 + lane, pair + 32 + lane, pair + 16 + lane};
    for (unsigned int word = 0; word < expected.size(); ++word) {
      require(fixture.memory.read({.address = pointer + (word * 4), .size = 4}) == expected[word]);
    }
  }
  constexpr std::array<std::uint32_t, 2> breaks{0xc0775000, 0xc00aa000};
  require(fixture.memory.read({.address = heap.break_word, .size = 4}) == breaks[part]);
  constexpr std::uint16_t output_mask = 0x7118;
  constexpr std::array<std::uint16_t, 2> high{0x1018, 0x5018};
  require(fixture.gpio.device.drives()[4] ==
          (lead_a1::GpioDrive{.enabled = output_mask, .high = high[part]}));
  const auto state = fixture.gpio.device.state();
  constexpr std::uint16_t direction3 = 0xfeff;
  constexpr std::uint16_t direction4 = 0x8ee7;
  require(state.banks[3].direction == direction3 && state.banks[4].direction == direction4);
  require(state.banks[5].direction == all_pins && state.banks[4].input.known == 0);
  for (unsigned int bank = 0; bank < lead_a1::gpio_banks; ++bank) {
    require(state.banks[bank].pending == 0);
  }
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    for (unsigned int part = 0; part < placements.size(); ++part) {
      const auto image =
          read(std::filesystem::path(argv[2]) / (std::to_string(part) + "-CODE.bin"));
      Fixture fixture(image);
      bank_objects(fixture, argv[1], placements[part]);
      directions(fixture, argv[1], placements[part]);
      prepare_board(fixture, argv[1], placements[part]);
      board_access(fixture, argv[1], placements[part]);
      unresolved_external(fixture, argv[1], placements[part]);
      board_initializer(image, argv[1], part);
      std::cout << "Part " << part
                << ": original GPIO objects, direction shifts, complete board initialization/heap"
                   " and read/write/delay paths"
                   " preserve exact halfword accesses; unknown external byte still diagnoses\n";
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
