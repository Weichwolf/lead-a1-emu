// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_boot.hpp"
#include "lead_a1/sharc_clock.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_spi.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr unsigned int kernel_bytes = 1536;
constexpr unsigned int header_bytes = 12;
constexpr std::uint32_t reset = 0x8c000;
constexpr std::uint32_t frontier = 0x8c08e;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint32_t saved_vector = 0x8c030;
constexpr std::uint32_t self_modify = 0x8c004;
constexpr std::uint32_t sysctl = 0x30024;
constexpr std::uint32_t spi_index = 0x1080;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Original SHARC stream expectation at line " +
                             std::to_string(where.line()));
  }
}

std::vector<std::byte> read_image(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) { throw std::runtime_error("Cannot open original image: " + path.string()); }
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input), {}};
  std::vector<std::byte> result;
  result.reserve(bytes.size());
  for (const auto byte : bytes) { result.push_back(static_cast<std::byte>(byte)); }
  return result;
}

std::uint32_t word(std::span<const std::byte> data, std::size_t offset, unsigned int bytes = 4) {
  require(bytes > 0 && bytes <= 4 && offset <= data.size() && bytes <= data.size() - offset);
  std::uint32_t value{};
  for (unsigned int index = 0; index < bytes; ++index) {
    value |= std::to_integer<std::uint32_t>(data[offset + index]) << (index * 8);
  }
  return value;
}

struct Placement {
  unsigned int offset;
  unsigned int bytes;
  unsigned int records;
  unsigned int cells;
};

using CellExpectations = std::map<std::uint32_t, std::uint16_t>;

void check_cells(lead_a1::SharcMemory &memory, const CellExpectations &expected) {
  for (const auto &[address, value] : expected) {
    require(memory.read({.address = address, .width = lead_a1::SharcWidth::short16}) == value);
  }
}

constexpr std::array<std::array<Placement, 2>, 2> placements{
    {{{{.offset = 0x2c6d78, .bytes = 114112, .records = 608, .cells = 51869},
       {.offset = 0x2ab0f8, .bytes = 113788, .records = 600, .cells = 51755}}},
     {{{.offset = 0x71bc8, .bytes = 11476, .records = 26, .cells = 4042},
       {.offset = 0x701d8, .bytes = 6636, .records = 21, .cells = 1652}}}}};

class ObservedClock final : public lead_a1::MemoryDevice {
public:
  lead_a1::SharcClock device{{.configuration = 0, .input_stable = true, .reset_settled = true}};
  bool accessed{};

  std::uint64_t read(lead_a1::MemoryAccess access) override {
    accessed = true;
    return device.read(access);
  }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    accessed = true;
    device.write(access, value);
  }

  void begin_instruction() override { device.begin_instruction(); }

  void commit_instruction() noexcept override { device.commit_instruction(); }

  void abort_instruction() noexcept override { device.abort_instruction(); }
};

void bootstrap(lead_a1::SharcMemory &memory,
               lead_a1::SharcSpiBoot &boot,
               std::span<const std::byte> stream) {
  require(stream.size() >= kernel_bytes);
  boot.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  for (unsigned int index = 0; index < kernel_bytes; ++index) {
    const auto byte = std::to_integer<unsigned int>(stream[index]);
    for (unsigned int bit = 0; bit < 8; ++bit) {
      boot.observe(memory, {.selected = true, .clock = false, .mosi = {}});
      boot.observe(memory, {.selected = true, .clock = true, .mosi = ((byte >> bit) & 1) != 0});
    }
  }
  require(boot.entry() == reset && boot.state().remaining == 0);
  constexpr unsigned int instruction_bytes = 6;
  for (unsigned int offset = 0; offset < kernel_bytes; offset += instruction_bytes) {
    const auto expected =
        std::uint64_t{word(stream, offset)} | (std::uint64_t{word(stream, offset + 4, 2)} << 32);
    require(memory.read({.address = reset + (offset / instruction_bytes),
                         .width = lead_a1::SharcWidth::wide48},
                        lead_a1::AccessKind::fetch) == expected);
  }
}

void sequenced_kernel_prefix(lead_a1::SharcMemory &memory, ObservedClock &clock) {
  const auto initial_clock = clock.device.state();
  require(!clock.accessed);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(reset));
  lead_a1::SharcFetchState transport;
  transport.flow = lead_a1::SharcFetchFlow{.next = reset};
  lead_a1::SharcFetch fetch(memory, transport);
  constexpr unsigned int instructions = 8;
  constexpr unsigned int clock_limit = 12;
  unsigned int retired{};
  unsigned int clocks{};
  while (retired < instructions && clocks < clock_limit) {
    const auto event = fetch.advance_sequenced(cpu);
    require(event.address_accepted && !event.stalled);
    ++clocks;
    if (fetch.state().execute) {
      const auto step = cpu.step(fetch);
      require(step.pc == reset + retired);
      ++retired;
    }
  }
  constexpr std::uint32_t clock_setup = 0x8c091;
  constexpr std::uint32_t return_frame = 0x108c008;
  require(retired == instructions && clocks == clock_limit);
  require(cpu.state().pc == clock_setup && cpu.state().depth == 1);
  require(cpu.state().pc_stack[0] == return_frame && !cpu.state().branch);
  require(cpu.state().system[11].value == 3 && cpu.state().effective[11].value == 3);
  require(cpu.state().system[10].value == 0 && cpu.state().pending[10] == cpu.state().system[10]);
  const auto decoded = fetch.state().decode;
  require(decoded && decoded->address == clock_setup);
  require(!cpu.state().interrupt && !cpu.state().idle_pc && cpu.state().cache.valid.value == 0);
  auto expected = cpu.state();
  expected.effective[10] = expected.system[10];
  expected.pending[10].reset();
  const auto quiet = fetch.advance_sequenced(cpu);
  require(quiet.address_accepted && !quiet.stalled && !fetch.state().execute);
  require(cpu.state() == expected && !clock.accessed && clock.device.state() == initial_clock);
  require(fetch.advance_sequenced(cpu).address_accepted);
  const auto read = cpu.step(fetch);
  require(read.pc == clock_setup && read.next_pc == clock_setup + 1);
  constexpr unsigned int opcode_class_shift = 40;
  constexpr std::uint64_t ureg_mask = 127;
  constexpr std::uint64_t ustat3 = 0x7e;
  require((read.opcode >> opcode_class_shift) == 16);
  require(((read.opcode >> 32) & ureg_mask) == ustat3 && (read.opcode & full32) == 0x2000);
  expected.pc = clock_setup + 1;
  expected.system[14] = {.value = 8, .known = full32}; // USTAT3; independent PMCTL reset reference.
  expected.effective[14] = expected.system[14];
  require(cpu.state() == expected && clock.accessed && clock.device.state() == initial_clock);
  require(!memory.active());
}

struct ClearRange {
  std::uint32_t first;
  unsigned int words;
  std::uint32_t body;
};

constexpr std::array<ClearRange, 3> remaining_clears{
    {{.first = 0xb2000, .words = 49151, .body = 0x8c0bb},
     {.first = 0xc0000, .words = 32767, .body = 0x8c0be},
     {.first = 0xe0000, .words = 32767, .body = 0x8c0c1}}};

void observe_clear(const lead_a1::SharcFetchWord &executing, std::array<unsigned int, 3> &writes) {
  for (unsigned int index = 0; index < remaining_clears.size(); ++index) {
    const auto range = remaining_clears[index];
    if (executing.address != range.body) { continue; }
    const auto address = executing.dag;
    require(address && address->address == range.first + writes[index] && address->index == 0 &&
            address->post && address->width == lead_a1::SharcWidth::normal32);
    ++writes[index];
  }
}

void verify_clear_ranges(const lead_a1::SharcMemory &memory,
                         const std::array<unsigned int, 3> &writes) {
  for (unsigned int index = 0; index < remaining_clears.size(); ++index) {
    const auto range = remaining_clears[index];
    require(writes[index] == range.words);
    for (unsigned int word_index = 0; word_index < range.words; ++word_index) {
      require(memory.read({.address = range.first + word_index}) == 0);
    }
    bool unknown{};
    try {
      static_cast<void>(memory.read({.address = range.first + range.words}));
    } catch (const std::exception &error) {
      require(std::string(error.what()).find("Unknown SHARC memory bits") != std::string::npos);
      unknown = true;
    }
    require(unknown); // The original leaves one normal word untouched in these banks.
  }
}

void sequenced_remaining_clears(lead_a1::SharcMemory &memory,
                                lead_a1::Sharc &cpu,
                                lead_a1::SharcFetch &fetch,
                                ObservedClock &clock) {
  constexpr std::uint32_t return_pc = 0x8c008;
  constexpr unsigned int stores = 114685;
  constexpr unsigned int instruction_limit = stores + 8;
  constexpr unsigned int clock_limit = instruction_limit + 6;
  std::array<unsigned int, 3> writes{};
  unsigned int retired{};
  unsigned int clocks{};
  unsigned int loop_stalls{};
  unsigned int dag_stalls{};
  while (cpu.state().pc != return_pc && clocks < clock_limit) {
    clock.accessed = false;
    const auto event = fetch.advance_sequenced(cpu);
    ++clocks;
    loop_stalls += event.loop_stall ? 1U : 0U;
    dag_stalls += event.dag_stall ? 1U : 0U;
    require(event.stalled == (event.loop_stall || event.dag_stall));
    const auto executing = fetch.state().execute;
    if (executing) {
      observe_clear(*executing, writes);
      require(cpu.step(fetch).minimum_core_cycles == 1);
      ++retired;
    }
    require(!clock.accessed);
    clock.device.advance({.core_cycles = 1});
  }
  require(cpu.state().pc == return_pc && retired == instruction_limit && clocks == clock_limit);
  require(loop_stalls == 3 && dag_stalls == 3 && cpu.state().depth == 0);
  require(!cpu.state().count_loop && cpu.state().loops_empty == true);
  constexpr std::uint32_t after = 0xe7fff;
  constexpr std::uint32_t parent = 0x108c008;
  require(cpu.state().dag[0][0][0].value == after && cpu.state().pc_stack[0] == parent);
  verify_clear_ranges(memory, writes);
  require(!memory.active());
}

void sequenced_first_clear(lead_a1::SharcMemory &memory,
                           lead_a1::Sharc &cpu,
                           lead_a1::SharcFetch &fetch,
                           ObservedClock &clock,
                           std::span<const std::byte> stream) {
  constexpr std::uint32_t first = 0x92180;
  constexpr std::uint32_t body = 0x8c0b8;
  constexpr std::uint32_t next = 0x8c0b9;
  constexpr unsigned int stores = 0xbe80;
  constexpr unsigned int instruction_limit = stores + 6;
  constexpr unsigned int clock_limit = instruction_limit + 1;
  unsigned int retired{};
  unsigned int writes{};
  unsigned int clocks{};
  unsigned int stalls{};
  while (cpu.state().pc != next && clocks < clock_limit) {
    clock.accessed = false;
    const auto event = fetch.advance_sequenced(cpu);
    ++clocks;
    if (event.loop_stall) { ++stalls; }
    require(event.stalled == event.loop_stall);
    const auto executing = fetch.state().execute;
    if (executing) {
      if (executing->address == body) {
        const auto address = executing->dag;
        require(address && address->address == first + writes && address->index == 0 &&
                address->post && address->width == lead_a1::SharcWidth::normal32);
        ++writes;
      }
      const auto step = cpu.step(fetch);
      require(step.minimum_core_cycles == 1);
      ++retired;
    }
    require(!clock.accessed);
    clock.device.advance({.core_cycles = 1});
  }
  require(cpu.state().pc == next && retired == instruction_limit && clocks == clock_limit);
  require(writes == stores && stalls == 1 && !cpu.state().count_loop);
  constexpr std::uint32_t after = 0x9e000;
  require(cpu.state().dag[0][0][0].value == after && cpu.state().depth == 1);
  constexpr std::uint32_t parent = 0x108c008;
  require(cpu.state().pc_stack[0] == parent && cpu.state().loops_empty == true);
  for (unsigned int index = 0; index < stores; ++index) {
    require(memory.read({.address = first + index}) == 0);
  }
  sequenced_remaining_clears(memory, cpu, fetch, clock);
  constexpr std::uint32_t kernel = 0x92000;
  constexpr unsigned int words = kernel_bytes / 4;
  for (unsigned int index = 0; index < words; ++index) {
    require(memory.read({.address = kernel + index}) == word(stream, std::size_t{index} * 4));
  }
}

struct SerialInput {
  std::span<const std::byte> stream;
  unsigned int interval;
  unsigned int elapsed{};
  std::size_t bit = std::size_t{kernel_bytes} * 8;
  bool started{};
  bool high = true;

  void advance(lead_a1::SharcSpi &spi, lead_a1::SharcMemory &memory) {
    // The external source starts once after the guest arms its first receive.
    // Thereafter its fixed edge cadence never depends on polls/record boundaries.
    if (!started) { started = spi.state().serial.remaining != 0; }
    if (!started || bit == stream.size() * 8) { return; }
    ++elapsed;
    if (elapsed != interval) { return; }
    elapsed = 0;
    high = !high;
    if (high) {
      const auto byte = std::to_integer<unsigned int>(stream[bit / 8]);
      const bool value = ((byte >> (bit % 8)) & 1) != 0;
      spi.observe(memory, {.selected = true, .clock = true, .mosi = value});
      ++bit;
    } else {
      spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
    }
  }
};

struct HeaderDevices {
  ObservedClock &clock;
  lead_a1::SharcSpi &spi;
  unsigned int interval;
};

void advance_devices(lead_a1::SharcMemory &memory,
                     lead_a1::Sharc &cpu,
                     SerialInput &serial,
                     HeaderDevices devices) {
  require(!devices.clock.accessed);
  devices.clock.device.advance({.core_cycles = 1});
  serial.advance(devices.spi, memory);
  static_cast<void>(devices.spi.service_dma(memory));
  if (devices.spi.state().interrupt_pending) {
    cpu.latch_interrupt(0x1000);
    devices.spi.acknowledge_interrupt();
  }
  require(!devices.spi.state().overflow);
}

struct StreamRecord {
  std::size_t header;
  std::uint32_t tag;
  std::uint32_t count;
  std::uint32_t destination;
};

std::size_t check_record_payload(lead_a1::SharcMemory &memory,
                                 std::span<const std::byte> stream,
                                 StreamRecord record) {
  const auto payload = record.header + header_bytes;
  if (record.tag == 1) { return payload; }
  require(record.tag == 3 || record.tag == 4);
  const auto bytes = record.tag == 3 ? 2U : 4U;
  const auto width = record.tag == 3 ? lead_a1::SharcWidth::short16 : lead_a1::SharcWidth::normal32;
  for (std::uint32_t index = 0; index < record.count; ++index) {
    require(memory.read({.address = record.destination + index, .width = width}) ==
            word(stream, payload + (std::size_t{index} * bytes), bytes));
  }
  return payload + (((std::size_t{record.count} * bytes) + 3) & ~std::size_t{3});
}

void sequenced_next_records(lead_a1::SharcMemory &memory,
                            lead_a1::Sharc &cpu,
                            lead_a1::SharcFetch &fetch,
                            SerialInput &serial,
                            HeaderDevices devices) {
  const auto stream = serial.stream;
  std::size_t cursor =
      kernel_bytes + header_bytes + (std::size_t{word(stream, kernel_bytes + 4)} * 2);
  constexpr unsigned int limit = 8000000;
  constexpr std::uint32_t parsed = 0x8c019;
  constexpr std::uint32_t returned = 0x8c015;
  constexpr std::uint32_t buffer = 0x92003;
  unsigned int records{};
  while (cursor < stream.size()) {
    const auto tag = word(stream, cursor);
    const auto count = word(stream, cursor + 4);
    const auto destination = word(stream, cursor + 8);
    require(tag == 0 || tag == 1 || tag == 3 || tag == 4);
    bool header_seen{};
    unsigned int clocks{};
    while ((!header_seen || cpu.state().pc != returned) && clocks < limit) {
      devices.clock.accessed = false;
      static_cast<void>(fetch.advance_sequenced(cpu));
      ++clocks;
      if (fetch.state().execute) { require(cpu.step(fetch).minimum_core_cycles == 1); }
      advance_devices(memory, cpu, serial, devices);
      if (cpu.state().pc == parsed) {
        header_seen = true;
        for (unsigned int index = 0; index < 3; ++index) {
          require(memory.read({.address = buffer + index}) ==
                  word(stream, cursor + (std::size_t{index} * 4)));
        }
        if (tag == 0) {
          require(!cpu.state().count_loop && cpu.state().depth == 0 && !memory.active());
          std::cout << "  Sequenced remaining ordinary records: " << records
                    << ", terminal header parsed, next PC0x8c019\n";
          return;
        }
      }
    }
    require(header_seen && cpu.state().pc == returned && cpu.state().depth == 0);
    require(!cpu.state().count_loop && !memory.active());
    cursor = check_record_payload(
        memory, stream, {.header = cursor, .tag = tag, .count = count, .destination = destination});
    ++records;
  }
  throw std::runtime_error("Original terminal header not reached");
}

void sequenced_first_record(lead_a1::SharcMemory &memory,
                            lead_a1::Sharc &cpu,
                            lead_a1::SharcFetch &fetch,
                            std::span<const std::byte> stream,
                            SerialInput &serial,
                            HeaderDevices devices) {
  require(word(stream, kernel_bytes) == 3);
  const auto count = word(stream, kernel_bytes + 4);
  const auto destination = word(stream, kernel_bytes + 8);
  constexpr std::uint32_t next_header = 0x8c015;
  constexpr std::uint32_t setup = 0x8c04d;
  constexpr std::uint32_t reader_call = 0x8c04e;
  constexpr unsigned int limit = 80000;
  unsigned int clocks{};
  unsigned int setups{};
  unsigned int calls{};
  while (cpu.state().pc != next_header && clocks < limit) {
    devices.clock.accessed = false;
    static_cast<void>(fetch.advance_sequenced(cpu));
    ++clocks;
    const auto executing = fetch.state().execute;
    if (executing) {
      const auto pc = executing->address;
      require(cpu.step(fetch).minimum_core_cycles == 1);
      if (pc == setup) {
        const auto loop = cpu.state().count_loop;
        require(loop && loop->total == count / 2);
        ++setups;
      }
      if (pc == reader_call) { ++calls; }
    }
    require(!devices.clock.accessed);
    devices.clock.device.advance({.core_cycles = 1});
    serial.advance(devices.spi, memory);
    static_cast<void>(devices.spi.service_dma(memory));
    if (devices.spi.state().interrupt_pending) {
      cpu.latch_interrupt(0x1000);
      devices.spi.acknowledge_interrupt();
    }
    require(!devices.spi.state().overflow);
  }
  require(cpu.state().pc == next_header && !cpu.state().count_loop && cpu.state().depth == 0);
  const auto flow = fetch.state().flow;
  require(flow && !flow->long_loop && setups == 1 && calls == count / 2);
  for (std::uint32_t index = 0; index < count; ++index) {
    require(memory.read({.address = destination + index, .width = lead_a1::SharcWidth::short16}) ==
            word(stream, kernel_bytes + header_bytes + (std::size_t{index} * 2), 2));
  }
  require(!memory.active());
  std::cout << "  Sequenced first payload: " << count << " short words/" << calls
            << " actual reader calls/" << clocks << " available-bus clocks, next PC0x8c015\n";
  sequenced_next_records(memory, cpu, fetch, serial, devices);
}

void sequenced_first_header(lead_a1::SharcMemory &memory,
                            lead_a1::Sharc &cpu,
                            lead_a1::SharcFetch &fetch,
                            std::span<const std::byte> stream,
                            HeaderDevices devices) {
  SerialInput serial{.stream = stream, .interval = devices.interval};
  constexpr unsigned int limit = 4096;
  constexpr std::uint32_t parsed = 0x8c019;
  unsigned int clocks{};
  unsigned int instructions{};
  while (cpu.state().pc != parsed && clocks < limit) {
    devices.clock.accessed = false;
    static_cast<void>(fetch.advance_sequenced(cpu));
    ++clocks;
    if (fetch.state().execute) {
      require(cpu.step(fetch).minimum_core_cycles == 1);
      ++instructions;
    }
    require(!devices.clock.accessed);
    devices.clock.device.advance({.core_cycles = 1});
    serial.advance(devices.spi, memory);
    static_cast<void>(devices.spi.service_dma(memory));
    if (devices.spi.state().interrupt_pending) {
      constexpr std::uint32_t p1i = 0x1000;
      cpu.latch_interrupt(p1i);
      devices.spi.acknowledge_interrupt();
    }
    require(!devices.spi.state().overflow);
  }
  require(cpu.state().pc == parsed && cpu.state().depth == 0 && instructions != 0);
  constexpr std::array<unsigned int, 3> registers{8, 2, 3};
  constexpr std::uint32_t header = 0x92003;
  constexpr std::uint64_t full40 = 0xffffffffff;
  for (unsigned int index = 0; index < registers.size(); ++index) {
    const auto value = word(stream, kernel_bytes + (index * 4));
    require(memory.read({.address = header + index}) == value);
    require(cpu.state().data[0][registers[index]] ==
            (lead_a1::SharcKnown40{.value = std::uint64_t{value} << 8, .known = full40}));
  }
  require(devices.spi.state().serial.next_word == header + registers.size() &&
          devices.spi.state().serial.remaining == 0);
  constexpr std::uint32_t parent = 0x108c016;
  require(cpu.state().pc_stack[0] == parent && !cpu.state().interrupt && !cpu.state().idle_pc);
  require((cpu.state().effective[10].value & 0x1000) != 0 && cpu.state().effective[11].value == 3);
  constexpr std::size_t header_end = std::size_t{kernel_bytes + header_bytes} * 8;
  constexpr unsigned int word_bits = 32;
  require(serial.bit >= header_end && serial.bit < header_end + word_bits);
  require(!memory.active());
  std::cout << "  Sequenced first header: " << instructions << " instructions/" << clocks
            << " available-bus clocks, actual serial/DMA/header loads at " << devices.interval
            << " clocks per edge, next PC0x8c019\n";
  sequenced_first_record(memory, cpu, fetch, stream, serial, devices);
}

void sequenced_clock_setup(std::span<const std::byte> stream,
                           unsigned int interval,
                           const CellExpectations &expected) {
  // This fixture owns fresh RAM and PMCTL: its writes cannot seed the full stream.
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  auto boot = lead_a1::SharcSpiBoot::slave_reference_214xx();
  bootstrap(memory, boot, stream);
  ObservedClock clock;
  memory.map_io({.first_word = 0x2000, .words = 1}, clock);
  lead_a1::SharcControl control(true);
  lead_a1::SharcSpi spi(boot.state());
  memory.map_io({.first_word = sysctl, .words = 1}, control);
  memory.map_io({.first_word = spi_index, .words = 5}, spi);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(reset), &control);
  lead_a1::SharcFetchState transport;
  transport.flow = lead_a1::SharcFetchFlow{.next = reset};
  lead_a1::SharcFetch fetch(memory, transport);
  constexpr std::uint32_t flags_entry = 0x8c0b2;
  constexpr unsigned int instruction_limit = 8291;
  constexpr unsigned int clock_limit = 8302;
  constexpr std::array<unsigned int, 6> counts{16, 4096, 16, 4096, 16, 16};
  unsigned int clocks{};
  unsigned int instructions{};
  unsigned int stalls{};
  unsigned int loops{};
  while (cpu.state().pc != flags_entry && clocks < clock_limit) {
    clock.accessed = false;
    const auto event = fetch.advance_sequenced(cpu);
    ++clocks;
    if (event.loop_stall) { ++stalls; }
    require(event.stalled == event.loop_stall);
    if (fetch.state().execute) {
      const auto step = cpu.step(fetch);
      ++instructions;
      require(instructions <= instruction_limit && step.minimum_core_cycles == 1);
      constexpr unsigned int class_shift = 40;
      if ((step.opcode >> class_shift) == 12) {
        const auto active = cpu.state().count_loop;
        require(active && active->total == counts.at(loops));
        ++loops;
      }
      const auto active = cpu.state().count_loop;
      if (active) { require(step.pc == active->body - 1 || step.opcode == 0); }
    }
    if (!clock.accessed) {
      constexpr std::uint32_t bypass = 0x8000;
      clock.device.advance(
          {.core_cycles = 1,
           .clkin_cycles = (clock.device.state().control & bypass) != 0 ? 1U : 0U});
    }
  }
  require(cpu.state().pc == flags_entry && instructions == instruction_limit &&
          clocks == clock_limit);
  require(stalls == counts.size() && loops == counts.size());
  constexpr std::uint32_t return_frame = 0x108c008;
  require(cpu.state().depth == 1 && cpu.state().pc_stack[0] == return_frame);
  const auto flow = fetch.state().flow;
  require(!cpu.state().count_loop && cpu.state().loops_empty == true && flow && !flow->loop);
  constexpr std::uint32_t expected_pmctl = 0x40012;
  require(clock.device.state().control == expected_pmctl && !clock.device.state().pending_dividers);
  const auto rates = clock.device.rates();
  constexpr unsigned int core_ratio = 18;
  constexpr unsigned int external_numerator = 36;
  require(rates.core == (lead_a1::SharcClockRatio{.numerator = core_ratio}));
  require(rates.peripheral == (lead_a1::SharcClockRatio{.numerator = 9}));
  require(rates.external ==
          (lead_a1::SharcClockRatio{.numerator = external_numerator, .denominator = 5}));
  require(!memory.active());
  sequenced_first_clear(memory, cpu, fetch, clock, stream);
  sequenced_first_header(
      memory, cpu, fetch, stream, {.clock = clock, .spi = spi, .interval = interval});
  check_cells(memory, expected);
  std::cout << "  Sequenced final payload cells: " << expected.size() << " matched\n";
}

struct Inventory {
  CellExpectations cells;
  unsigned int records{};
  std::size_t terminal{};
};

struct CellSource {
  std::size_t payload;
  std::uint32_t destination;
  std::uint64_t count;
};

void add_cells(Inventory &inventory, std::span<const std::byte> stream, CellSource source) {
  for (std::uint64_t index = 0; index < source.count; ++index) {
    require(index <= full32 && std::uint64_t{source.destination} + index <= full32);
    inventory.cells[source.destination + static_cast<std::uint32_t>(index)] =
        static_cast<std::uint16_t>(word(stream, source.payload + (index * 2), 2));
  }
}

Inventory inventory(std::span<const std::byte> stream, Placement placement) {
  Inventory result;
  std::size_t cursor = kernel_bytes;
  while (cursor < stream.size()) {
    const auto tag = word(stream, cursor);
    const auto count = word(stream, cursor + 4);
    const auto destination = word(stream, cursor + 8);
    ++result.records;
    if (tag == 0) {
      require(cursor + header_bytes + kernel_bytes == stream.size());
      result.terminal = cursor;
      break;
    }
    require(tag == 1 || tag == 3 || tag == 4);
    const auto cells = tag == 3 ? std::uint64_t{count} : std::uint64_t{count} * 2;
    const auto payload = tag == 1 ? 0 : cells * 2;
    const auto padded = (payload + 3) & ~std::uint64_t{3};
    require(cursor + header_bytes + padded <= stream.size());
    if (tag != 1) {
      auto first = destination;
      if (tag == 4) {
        constexpr std::uint32_t normal = 0x80000;
        constexpr std::uint32_t short_word = 0x100000;
        constexpr std::uint32_t bank = 0x20000;
        require(destination >= normal && destination < short_word);
        const auto offset = destination - normal;
        first = short_word + ((offset / bank) * bank * 2) + ((offset % bank) * 2);
      }
      add_cells(
          result, stream, {.payload = cursor + header_bytes, .destination = first, .count = cells});
    }
    cursor += header_bytes + static_cast<std::size_t>(padded);
  }
  require(result.terminal != 0 && result.records == placement.records &&
          result.cells.size() == placement.cells);
  return result;
}

void check_frontier(const lead_a1::Sharc &cpu,
                    const lead_a1::SharcControl &control,
                    const lead_a1::SharcSpi &spi,
                    SerialInput serial,
                    const Inventory &expected) {
  const auto &state = cpu.state();
  require(!state.interrupt && !state.idle_pc);
  require(state.pc == frontier && state.depth == 1 && state.loops_empty == false &&
          !state.count_loop && !state.branch);
  if (!state.arithmetic_loop) { throw std::runtime_error("Missing original arithmetic DO state"); }
  const auto &active = *state.arithmetic_loop;
  constexpr std::uint32_t loop_start = 0x8c003;
  require(active.body == loop_start && active.end == self_modify && active.termination == 0 &&
          active.saved_depth == 0 && active.kind == lead_a1::SharcArithmeticKind::multiple &&
          active.phase == lead_a1::SharcArithmeticPhase::deferred);
  require(state.pc_stack[0] == loop_start);
  constexpr std::uint32_t primary_mask = 0x1003;
  require(state.system[11].value == primary_mask && state.effective[11] == state.system[11]);
  require(state.system[10].value == 0 && state.effective[10] == state.system[10]);
  require(state.cache.valid == (lead_a1::SharcKnown32{.known = full32}) &&
          !state.cache.pending_flush);
  constexpr std::uint32_t vector_destination = 0x92000;
  constexpr unsigned int vector_words = 384;
  require(spi.state().serial.next_word == vector_destination &&
          spi.state().serial.remaining == vector_words && spi.state().modifier == 1);
  require(state.data[0][1].value == std::uint64_t{vector_destination} * 256 &&
          state.data[0][2].value == std::uint64_t{vector_words} * 256);
  const auto saved = (std::uint64_t{word(serial.stream, expected.terminal + 8)} << 32) |
                     (word(serial.stream, expected.terminal + 4) & 0xffff0000);
  constexpr std::uint64_t full64 = 0xffffffffffffffff;
  require(state.px.value == saved && state.px.known == full64);
  constexpr std::uint64_t template_value = 0xb16b000000;
  constexpr std::uint64_t old_control = 0x400;
  require(state.data[0][11].value == template_value && state.data[0][9].value == old_control);
  require(state.data[0][9] == state.data[0][12]);
  require(state.dag[0][0][8].value == saved_vector && state.dag[0][0][12].value == self_modify);
  constexpr std::uint32_t widened = 0x204;
  require(control.state().effective == widened && control.state().control == widened &&
          !control.state().delayed);
  require(!spi.state().overflow);
  require(serial.bit >= (expected.terminal + header_bytes) * 8 &&
          serial.bit < serial.stream.size() * 8);
}

void run_stream(std::span<const std::byte> stream, Placement placement, unsigned int interval) {
  const auto expected = inventory(stream, placement); // Read-only oracle, never writes guest RAM.
  sequenced_clock_setup(stream, interval, expected.cells);
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  auto boot = lead_a1::SharcSpiBoot::slave_reference_214xx();
  bootstrap(memory, boot, stream);
  ObservedClock clock;
  memory.map_io({.first_word = 0x2000, .words = 1}, clock);
  sequenced_kernel_prefix(memory,
                          clock); // Separate read-only prefix; full stream starts fresh below.
  lead_a1::SharcControl control(true);
  lead_a1::SharcSpi spi(boot.state());
  memory.map_io({.first_word = sysctl, .words = 1}, control);
  memory.map_io({.first_word = spi_index, .words = 5}, spi);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(reset), &control);
  SerialInput serial{.stream = stream, .interval = interval};
  constexpr unsigned int limit = 12000000;
  unsigned int instructions{};
  bool stopped{};
  while (instructions < limit) {
    const auto before = cpu.state();
    const auto before_spi = spi.state();
    const auto before_control = control.state();
    clock.accessed = false;
    lead_a1::SharcStep step;
    try {
      step = cpu.step();
    } catch (const std::exception &error) {
      const std::string message(error.what());
      if (message.find("native48 pc=0x8c08e") == std::string::npos ||
          message.find("deferred arithmetic-loop fetch/branch not qualified") ==
              std::string::npos) {
        throw std::runtime_error("Unexpected original SHARC execution failure: " + message);
      }
      require(cpu.state() == before && spi.state() == before_spi &&
              control.state() == before_control && !memory.active());
      stopped = true;
      break;
    }
    ++instructions;
    if (!clock.accessed) {
      constexpr std::uint32_t bypass = 0x8000;
      clock.device.advance({.core_cycles = step.minimum_core_cycles,
                            .clkin_cycles = (clock.device.state().control & bypass) != 0
                                                ? step.minimum_core_cycles
                                                : 0});
    }
    serial.advance(spi, memory);
    static_cast<void>(spi.service_dma(memory));
    if (spi.state().interrupt_pending) {
      constexpr std::uint32_t primary_spi = 0x1000;
      cpu.latch_interrupt(primary_spi);
      spi.acknowledge_interrupt();
    }
    require(!spi.state().overflow);
  }
  require(stopped);
  check_frontier(cpu, control, spi, serial, expected);
  for (const auto &[address, value] : expected.cells) {
    require(memory.read({.address = address, .width = lead_a1::SharcWidth::short16}) == value);
  }
  std::cout << expected.records << " records, " << expected.cells.size() << " final payload cells, "
            << instructions << " DSP instructions, " << interval
            << " instructions/serial edge; vector DMA armed, deferred branch frontier\n"
            << "  Sequenced kernel prefix:9 instructions/14 granted fetch clocks; "
               "actual PMCTL read, next PC0x8c092\n"
            << "  Separate sequenced clock setup:8291 instructions/8302 granted fetch clocks; "
               "six actual wait loops, next PC0x8c0b2\n"
            << "  Same sequenced fixture:171758 instructions/171776 available-bus clocks; "
               "163453 actual zero stores, all four RAM clears and native return PC0x8c008\n"
            << std::flush;
}

}

int main(int argc, char **argv) {
  try {
    if (argc != 2) {
      throw std::runtime_error("Usage: original-sharc-test decoded-image-directory");
    }
    const std::filesystem::path directory(argv[1]);
    for (unsigned int part = 0; part < placements.size(); ++part) {
      const auto image = read_image(directory / (std::to_string(part) + "-CODE.bin"));
      for (unsigned int transport = 0; transport < placements[part].size(); ++transport) {
        const auto placement = placements[part][transport];
        require(placement.offset <= image.size() &&
                placement.bytes <= image.size() - placement.offset);
        const auto stream = std::span(image).subspan(placement.offset, placement.bytes);
        for (const auto interval : {2U, 3U}) {
          std::cout << "Part " << part << " transport " << transport << ": " << std::flush;
          run_stream(stream, placement, interval);
        }
      }
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
