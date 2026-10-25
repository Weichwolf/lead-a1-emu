// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "original_picr_expectations.hpp"
#include "original_sport_expectations.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_iop_bridge.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_boot.hpp"
#include "sharc/peripherals/sharc_clock.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/peripherals/sharc_dai_data.hpp"
#include "sharc/peripherals/sharc_dai_enable.hpp"
#include "sharc/peripherals/sharc_dai_pin.hpp"
#include "sharc/peripherals/sharc_dai_timing.hpp"
#include "sharc/peripherals/sharc_dpi_routing.hpp"
#include "sharc/peripherals/sharc_pdap.hpp"
#include "sharc/peripherals/sharc_picr.hpp"
#include "sharc/peripherals/sharc_power.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/peripherals/sharc_spi_port.hpp"
#include "sharc/peripherals/sharc_sport.hpp"
#include "sharc/peripherals/sharc_sport_dma.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/visa/sharc_visa.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <optional>
#include <source_location>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
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
  std::uint32_t visa_entry{};
  std::uint32_t second_call{};
};

using CellExpectations = std::map<std::uint32_t, std::uint16_t>;

void check_cells(lead_a1::SharcMemory &memory, const CellExpectations &expected) {
  for (const auto &[address, value] : expected) {
    require(memory.read({.address = address, .width = lead_a1::SharcWidth::short16}) == value);
  }
}

constexpr std::array<std::array<Placement, 2>, 2> placements{{{{{.offset = 0x2c6d78,
                                                                 .bytes = 114112,
                                                                 .records = 608,
                                                                 .cells = 51869,
                                                                 .visa_entry = 0x1c47d1,
                                                                 .second_call = 0x1c5b8d},
                                                                {.offset = 0x2ab0f8,
                                                                 .bytes = 113788,
                                                                 .records = 600,
                                                                 .cells = 51755,
                                                                 .visa_entry = 0x1c47d1,
                                                                 .second_call = 0x1c5b15}}},
                                                              {{{.offset = 0x71bc8,
                                                                 .bytes = 11476,
                                                                 .records = 26,
                                                                 .cells = 4042,
                                                                 .visa_entry = 0x1c058c,
                                                                 .second_call = 0x1c0754},
                                                                {.offset = 0x701d8,
                                                                 .bytes = 6636,
                                                                 .records = 21,
                                                                 .cells = 1652,
                                                                 .visa_entry = 0x1c058c,
                                                                 .second_call = 0x1c0620}}}}};

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

enum class OriginalProfile : std::uint8_t { immediate, deferred_f2, visa_reference, clocked_iop };

void map_original_power(lead_a1::SharcMemory &memory,
                        lead_a1::SharcPower &power,
                        OriginalProfile profile) {
  constexpr std::uint32_t pmctl1 = 0x2001;
  if (profile == OriginalProfile::visa_reference) {
    memory.map_io({.first_word = pmctl1, .words = 1}, power);
  }
}

struct HeaderDevices {
  ObservedClock &clock;
  lead_a1::SharcSpi &spi;
  lead_a1::SharcControl &control;
  unsigned int interval;
  OriginalProfile profile{OriginalProfile::immediate};
  Placement placement{};
  lead_a1::SharcPower *power{};
  lead_a1::SharcDaiEnable *dai{};
  lead_a1::SharcDaiPin *pin_routes{};
  lead_a1::SharcDaiData *data_routes{};
  lead_a1::SharcDaiTiming *clock_routes{};
  lead_a1::SharcDaiTiming *frame_routes{};
  lead_a1::SharcPdap *pdap{};
  lead_a1::SharcPicr *priority{};
  lead_a1::SharcIopBridge *bridge{};
  lead_a1::SharcDpiRouting *dpi_pin{};
  lead_a1::SharcDpiRouting *dpi_enable{};
  std::array<lead_a1::SharcSport *, 8> sports{};
  std::uint64_t iop_epoch{};
};

void original_spi_interrupt(lead_a1::Sharc &cpu, HeaderDevices devices) {
  if (!devices.spi.state().interrupt_pending) { return; }
  if (devices.profile == OriginalProfile::clocked_iop) {
    if (devices.priority == nullptr) { throw std::logic_error("Missing original PICR owner"); }
    constexpr unsigned int spi_high = 1;
    constexpr unsigned int p1 = 2;
    const auto destinations = devices.priority->destinations(spi_high);
    if (destinations != p1) {
      std::ostringstream message;
      message << "Original PICR SPI event pc=0x" << std::hex << cpu.state().pc
              << " source=0x1 priority-mask=0x" << destinations
              << ": core routing beyond P1I is not qualified";
      throw std::runtime_error(message.str());
    }
    devices.priority->source(spi_high, true);
    require(devices.priority->outputs()[1] == true);
  }
  constexpr std::uint32_t p1_latch = 0x1000;
  cpu.latch_interrupt(p1_latch);
  devices.spi.acknowledge_interrupt();
  if (devices.profile == OriginalProfile::clocked_iop) { devices.priority->source(1, false); }
}

void check_frontier(const lead_a1::Sharc &cpu,
                    const lead_a1::SharcControl &control,
                    const lead_a1::SharcSpi &spi,
                    SerialInput serial,
                    std::size_t terminal,
                    std::optional<lead_a1::SharcIopState> pending = {});

void advance_bound_peripherals(HeaderDevices devices) {
  if (devices.profile != OriginalProfile::clocked_iop) { return; }
  require(devices.bridge != nullptr && devices.power != nullptr && devices.dai != nullptr &&
          devices.clock_routes != nullptr && devices.frame_routes != nullptr &&
          devices.pdap != nullptr && devices.dpi_pin != nullptr && devices.dpi_enable != nullptr &&
          devices.pin_routes != nullptr && devices.data_routes != nullptr);
  devices.power->advance_half_pclk(1);
  devices.dai->advance_half_pclk(1);
  devices.pin_routes->advance_half_pclk(1);
  devices.data_routes->advance_half_pclk(1);
  devices.clock_routes->advance_half_pclk(1);
  devices.frame_routes->advance_half_pclk(1);
  devices.pdap->advance_cclk(1);
  devices.dpi_pin->advance_half_pclk(1);
  devices.dpi_enable->advance_half_pclk(1);
  for (auto *port : devices.sports) {
    if (port == nullptr) { throw std::logic_error("Missing original SPORT owner"); }
    port->advance_cclk(1);
    port->advance_half_pclk(1);
  }
  devices.bridge->tick(); // Age the elapsed interval before delivering its new writes.
}

void advance_devices(lead_a1::SharcMemory &memory,
                     lead_a1::Sharc &cpu,
                     SerialInput &serial,
                     HeaderDevices devices) {
  require(!devices.clock.accessed);
  devices.clock.device.advance({.core_cycles = 1});
  serial.advance(devices.spi, memory);
  static_cast<void>(devices.spi.service_dma(memory));
  original_spi_interrupt(cpu, devices);
  require(!devices.spi.state().overflow);
  advance_bound_peripherals(devices);
}

struct StreamRecord {
  std::size_t header;
  std::uint32_t tag;
  std::uint32_t count;
  std::uint32_t destination;
};

void sequenced_vector_receive(lead_a1::SharcMemory &memory,
                              lead_a1::Sharc &cpu,
                              lead_a1::SharcFetch &fetch,
                              SerialInput &serial,
                              HeaderDevices devices) {
  constexpr unsigned int limit = 131072;
  constexpr unsigned int vector_words = 384;
  constexpr std::uint32_t destination = 0x92000;
  constexpr std::uint32_t p1i = 0x1000;
  auto expected = cpu.state();
  const auto held = fetch.state();
  require(expected.idle_pc && !expected.pending[10]);
  unsigned int ticks{};
  bool started = devices.spi.state().serial.remaining != 0;
  while ((!started || devices.spi.state().serial.remaining != 0) && ticks < limit) {
    advance_devices(memory, cpu, serial, devices);
    ++ticks;
    if (!started && devices.spi.state().serial.remaining != 0) {
      require(devices.spi.state().serial.remaining == vector_words &&
              devices.spi.state().serial.next_word == destination);
      started = true;
    }
    if (started && devices.spi.state().serial.remaining == 0) {
      expected.system[10].value |= p1i;
      expected.effective[10].value |= p1i;
    }
    require(cpu.state() == expected && fetch.state() == held);
  }
  require(started && ticks < limit && devices.spi.state().serial.remaining == 0 &&
          devices.spi.state().serial.next_word == destination + vector_words &&
          serial.bit == serial.stream.size() * 8);
  const auto payload = serial.stream.size() - kernel_bytes;
  for (unsigned int index = 0; index < vector_words; ++index) {
    require(memory.read({.address = destination + index}) ==
            word(serial.stream, payload + (std::size_t{index} * 4)));
  }
  std::cout << "  Sequenced final vectors: all384 actual DMA words matched, CPU held for " << ticks
            << " owner ticks\n";
}

void sequenced_loop_interrupt(lead_a1::SharcMemory &memory,
                              lead_a1::Sharc &cpu,
                              lead_a1::SharcFetch &fetch,
                              SerialInput &serial,
                              HeaderDevices devices) {
  constexpr std::uint32_t loop_start = 0x8c003;
  const auto slot = cpu.state().idle_delay_word;
  if (!slot) { throw std::runtime_error("Missing original retained mask slot"); }
  require(!cpu.service_interrupt(fetch, {.vector_base = reset}) && cpu.wake_idle());
  static_cast<void>(fetch.advance_sequenced(cpu));
  require(cpu.step(fetch).opcode == *slot);
  advance_devices(memory, cpu, serial, devices);
  require(cpu.state().pc == loop_start && !cpu.state().branch && cpu.state().depth == 1);
  const auto entry = cpu.service_interrupt(fetch, {.vector_base = reset});
  if (!entry) { throw std::runtime_error("Missing original loop P1I acceptance"); }
  require(entry->return_pc == loop_start && entry->vector == saved_vector);
  constexpr std::uint32_t special = 0x2000000;
  require(cpu.state().depth == 2 && cpu.state().pc_stack[0] == loop_start &&
          cpu.state().pc_stack[1] == (special | loop_start));
  const auto suspended = cpu.state().arithmetic_loop;
  require(suspended && suspended->phase == lead_a1::SharcArithmeticPhase::fetched_suspended);
  constexpr unsigned int entry_clocks = 4;
  require(entry->minimum_core_cycles == entry_clocks);
  for (unsigned int clock = 0; clock < entry_clocks; ++clock) {
    advance_devices(memory, cpu, serial, devices);
  }
  constexpr unsigned int limit = 16;
  unsigned int instructions{};
  for (unsigned int clock = 0; clock < limit && cpu.state().interrupt; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (fetch.state().execute) {
      const auto step = cpu.step(fetch);
      require(step.pc == saved_vector && step.next_pc == loop_start &&
              step.opcode ==
                  memory.read({.address = saved_vector, .width = lead_a1::SharcWidth::wide48}));
      ++instructions;
    }
    advance_devices(memory, cpu, serial, devices);
  }
  const auto resumed = cpu.state().arithmetic_loop;
  require(instructions == 1 && cpu.state().pc == loop_start && !cpu.state().interrupt &&
          cpu.state().depth == 1 && cpu.state().loops_empty == false && resumed &&
          resumed->phase == lead_a1::SharcArithmeticPhase::fetched &&
          cpu.state().system[11].value == 3 && cpu.state().effective[11].value == 3 &&
          cpu.state().system[10].value == 0 && cpu.state().effective[10].value == 0);
  std::cout << "  Sequenced P1I/RTI: original vector executed, native loop restored at PC0x8c003\n";
}

void retire_deferred_foreground(lead_a1::Sharc &cpu,
                                lead_a1::SharcFetch &fetch,
                                bool stalled,
                                std::uint64_t original_end,
                                std::uint64_t patch,
                                unsigned int &foreground) {
  if (!fetch.state().execute) { return; }
  constexpr std::uint32_t loop_start = 0x8c003;
  const std::array<std::uint64_t, 4> opcodes{0, original_end, 0, patch};
  const auto step = cpu.step(fetch);
  require(foreground < opcodes.size());
  require(step.pc == loop_start + (foreground % 2) && step.minimum_core_cycles == 1 &&
          stalled == (foreground % 2 == 1) && step.opcode == opcodes[foreground]);
  ++foreground;
}

void complete_foreground_response(lead_a1::SharcMemory &memory,
                                  lead_a1::Sharc &cpu,
                                  lead_a1::SharcFetch &fetch,
                                  std::uint64_t patch,
                                  lead_a1::SharcControl &control,
                                  unsigned int foreground) {
  if (foreground != 2 || !fetch.state().pending_fetch2) { return; }
  require(memory.read({.address = self_modify, .width = lead_a1::SharcWidth::wide48}) == patch);
  const auto before = cpu.state();
  const auto controller = control.snapshot();
  fetch.complete_fetch2(cpu); // Explicit candidate response order; reads RAM, not patch bytes.
  const auto word = fetch.state().fetch2;
  require(word && word->address == self_modify && word->opcode == patch && cpu.state() == before &&
          control.snapshot() == controller);
}

void check_original_visa_frontend(lead_a1::SharcMemory &memory,
                                  lead_a1::Sharc &cpu,
                                  const lead_a1::SharcFetch &fetch,
                                  HeaderDevices devices) {
  const auto native = cpu.state();
  const auto transport = fetch.state();
  const auto controller = devices.control.snapshot();
  const auto entry = devices.placement.visa_entry;
  constexpr std::uint32_t jump_pc = self_modify + 1;
  constexpr std::uint64_t pc_mask = 0xffffff;
  constexpr unsigned int class_shift = 40;
  constexpr unsigned int condition_shift = 33;
  constexpr unsigned int call_shift = 39;
  constexpr unsigned int unconditional = 31;
  const auto jump = memory.read({.address = jump_pc, .width = lead_a1::SharcWidth::wide48});
  require((jump >> class_shift) == 6 && (jump & pc_mask) == entry);
  lead_a1::SharcVisaBuffer buffer(memory, {.decode = entry, .fetch = entry});
  constexpr std::uint32_t first_call = 0x1c0528;
  const std::array<std::uint32_t, 3> targets{first_call, devices.placement.second_call, entry + 6};
  for (unsigned int index = 0; index < targets.size(); ++index) {
    buffer.capture();
    require(!buffer.advance(false));
    const auto word = buffer.instruction();
    require(word && word->address == entry + index * 3 && word->words == 3);
    if (!word) { throw std::logic_error("Missing original VISA prefix"); }
    require((word->opcode >> class_shift) == 6 &&
            ((word->opcode >> condition_shift) & unconditional) == unconditional &&
            ((word->opcode >> call_shift) & 1) == (index < 2 ? 1U : 0U) &&
            (word->opcode & pc_mask) == targets[index]);
    require(buffer.advance(true) == word);
  }
  require(buffer.state().decode == entry + 9 && !buffer.instruction());
  require(cpu.state() == native && fetch.state() == transport &&
          devices.control.snapshot() == controller);
  std::cout << "  Separate VISA IAB read:3 original 48-bit entries at0x" << std::hex << entry
            << std::dec << "; no VISA instruction executed\n";
}

void settle_original_visa_fetch(lead_a1::SharcMemory &memory,
                                lead_a1::Sharc &cpu,
                                lead_a1::SharcFetch &fetch,
                                SerialInput &serial,
                                HeaderDevices devices) {
  // Deliver the real pending native F2/Decode cache fill before copying a
  // functional CPU reference. Its instruction-order API cannot deliver it.
  for (unsigned int clock = 0; clock < 2; ++clock) {
    devices.clock.accessed = false;
    require(fetch.advance_sequenced(cpu).address_accepted && !fetch.state().execute);
    advance_devices(memory, cpu, serial, devices);
  }
  const auto &transport = fetch.state();
  require(cpu.state().pc == self_modify + 1 && !transport.lookup && !transport.fill &&
          !transport.pending_fetch2 && transport.decode &&
          transport.decode->address == self_modify + 1);
}

void check_original_visa_boundary(lead_a1::SharcMemory &memory,
                                  lead_a1::Sharc &cpu,
                                  lead_a1::SharcFetch &fetch,
                                  SerialInput &serial,
                                  HeaderDevices devices) {
  check_original_visa_frontend(memory, cpu, fetch, devices);
  constexpr unsigned int clocks = 6; // Two preceding real F2/Decode clocks settled the cache.
  constexpr std::uint32_t first_call = 0x1c0528;
  unsigned int retired{};
  for (unsigned int clock = 0; clock < clocks; ++clock) {
    devices.clock.accessed = false;
    require(fetch.advance_sequenced(cpu).address_accepted);
    if (fetch.state().execute) {
      const auto step = cpu.step(fetch);
      require(step.minimum_core_cycles == 1);
      if (retired == 0) {
        require(step.pc == self_modify + 1 && step.address_units == 1 &&
                step.next_pc == devices.placement.visa_entry);
      } else {
        require(retired == 1 && step.pc == devices.placement.visa_entry &&
                step.address_units == 3 && step.next_pc == first_call);
      }
      ++retired;
    }
    advance_devices(memory, cpu, serial, devices);
  }
  constexpr std::uint32_t call_frame = 0x1000000;
  require(retired == 2 && cpu.state().pc == first_call && cpu.state().depth == 1 &&
          cpu.state().pc_stack[0] == (call_frame | (devices.placement.visa_entry + 3)) &&
          cpu.state().loops_empty == true && !cpu.state().branch && !cpu.state().count_loop &&
          !cpu.state().arithmetic_loop && !fetch.state().execute && !memory.active());
  std::cout << "  Clocked native-to-VISA:8 grants, original JUMP and first VISA CALL executed; "
               "next PC0x1c0528, actual return frame retained\n";
}

void fill_visa_reference(lead_a1::SharcVisaBuffer &buffer) {
  for (unsigned int fill = 0; fill < 2 && !buffer.instruction(); ++fill) {
    if (buffer.can_capture()) { buffer.capture(); }
    static_cast<void>(buffer.advance(false));
  }
}

lead_a1::SharcStep visa_reference_step(lead_a1::SharcMemory &memory,
                                       lead_a1::Sharc &cpu,
                                       std::optional<lead_a1::SharcVisaBuffer> &buffer) {
  constexpr std::uint32_t first_short = 0x100000;
  constexpr std::uint32_t end_short = 0x200000;
  if (cpu.state().pc < first_short || cpu.state().pc >= end_short) { return cpu.step(); }
  if (!buffer) {
    buffer.emplace(memory,
                   lead_a1::SharcVisaState{.decode = cpu.state().pc, .fetch = cpu.state().pc});
  }
  fill_visa_reference(*buffer);
  return cpu.step(*buffer);
}

struct VisaReferenceTrace {
  unsigned int instructions{};
  unsigned int native{};
  unsigned int compact{};
  unsigned int clock_accesses{};
  std::array<unsigned int, 6> waits{};
};

void record_visa_step(lead_a1::SharcStep step,
                      VisaReferenceTrace &trace,
                      const ObservedClock &clock) {
  ++trace.instructions;
  if (step.pc == self_modify + 1) {
    ++trace.native;
    require(step.address_units == 1);
  } else if (step.address_units == 1) {
    ++trace.compact;
    constexpr std::uint32_t return_pc = 0x1c058b;
    require(step.pc == return_pc);
  } else {
    require(step.address_units == 3);
  }
  constexpr std::array<std::uint32_t, 6> waits{
      0x1c0537, 0x1c0546, 0x1c0555, 0x1c0567, 0x1c0576, 0x1c0588};
  for (unsigned int index = 0; index < waits.size(); ++index) {
    if (step.pc == waits[index]) {
      require(step.opcode == 0);
      ++trace.waits[index];
    }
  }
  if (clock.accessed) { ++trace.clock_accesses; }
}

void record_visa_reference(lead_a1::SharcStep step,
                           VisaReferenceTrace &trace,
                           ObservedClock &clock) {
  record_visa_step(step, trace, clock);
  constexpr std::uint32_t pll_bypass = 0x8000;
  clock.device.advance({.core_cycles = step.minimum_core_cycles,
                        .clkin_cycles = (clock.device.state().control & pll_bypass) != 0
                                            ? step.minimum_core_cycles
                                            : 0U});
}

void advance_clocked_visa_devices(lead_a1::SharcMemory &memory,
                                  lead_a1::Sharc &cpu,
                                  SerialInput &serial,
                                  HeaderDevices devices) {
  constexpr std::uint32_t pll_bypass = 0x8000;
  devices.clock.device.advance(
      {.core_cycles = 1,
       .clkin_cycles = (devices.clock.device.state().control & pll_bypass) != 0 ? 1U : 0U});
  serial.advance(devices.spi, memory);
  static_cast<void>(devices.spi.service_dma(memory));
  original_spi_interrupt(cpu, devices);
  require(!devices.spi.state().overflow);
  advance_bound_peripherals(devices);
}

void check_original_clocked_visa_common(lead_a1::SharcMemory &memory,
                                        lead_a1::Sharc &cpu,
                                        lead_a1::SharcFetch &fetch,
                                        SerialInput &serial,
                                        HeaderDevices devices) {
  VisaReferenceTrace trace;
  unsigned int clocks{};
  unsigned int loop_stalls{};
  constexpr unsigned int limit = 20000;
  const auto return_pc = devices.placement.visa_entry + 3;
  while (cpu.state().pc != return_pc && clocks < limit) {
    devices.clock.accessed = false;
    const auto bus = fetch.address_pm_request(cpu) ? lead_a1::SharcFetchBus::pm_data
                                                   : lead_a1::SharcFetchBus::available;
    const auto event = fetch.advance_sequenced(cpu, bus);
    ++clocks;
    if (event.loop_stall) { ++loop_stalls; }
    require(!event.compute_stall && !event.control_stall && !event.dag_stall && !event.fetch_stall);
    if (fetch.state().execute) {
      const auto step = cpu.step(fetch);
      require(step.minimum_core_cycles == 1);
      record_visa_step(step, trace, devices.clock);
    }
    advance_clocked_visa_devices(memory, cpu, serial, devices);
  }
  constexpr unsigned int expected_instructions = 8284;
  constexpr unsigned int expected_clocks = 8293; // Three refill clocks and six real loop holds.
  constexpr std::array<unsigned int, 6> waits{16, 4096, 16, 4096, 16, 16};
  require(trace.instructions == expected_instructions && trace.native == 0 && trace.compact == 1 &&
          trace.clock_accesses == 11 && trace.waits == waits && clocks == expected_clocks &&
          loop_stalls == waits.size() && cpu.state().pc == return_pc);
  constexpr std::uint32_t call_frame = 0x1000000;
  require(cpu.state().depth == 0 && cpu.state().pc_stack[0] == (call_frame | return_pc) &&
          cpu.state().loops_empty == true && !cpu.state().count_loop && !cpu.state().branch &&
          !cpu.state().arithmetic_loop && !cpu.state().interrupt && !memory.active());
  const auto clock = devices.clock.device.state();
  constexpr std::uint32_t control = 0x40012;
  constexpr std::uint32_t dividers = 0x40000;
  require(clock.control == control && clock.effective_dividers == dividers &&
          !clock.pending_dividers && clock.pll_remaining == 0 && !clock.bypass_required);
  std::cout << "  Clocked VISA common call:8284 further instructions/8293 grants, six actual wait "
               "loops/11 PMCTL accesses; original RTS returned\n";
}

void original_routing_clock(HeaderDevices devices) {
  require(devices.power != nullptr && devices.dai != nullptr && devices.bridge != nullptr);
  require(devices.clock_routes != nullptr && devices.frame_routes != nullptr);
  require(devices.pdap != nullptr);
  devices.clock.device.advance({.core_cycles = 1});
  devices.power->advance_half_pclk(1);
  devices.dai->advance_half_pclk(1);
  devices.clock_routes->advance_half_pclk(1);
  devices.frame_routes->advance_half_pclk(1);
  devices.pdap->advance_cclk(1);
  require(devices.dpi_pin != nullptr && devices.dpi_enable != nullptr);
  devices.dpi_pin->advance_half_pclk(1);
  devices.dpi_enable->advance_half_pclk(1);
  devices.bridge->tick();
}

lead_a1::SharcStep original_routing_step(lead_a1::Sharc &cpu,
                                         lead_a1::SharcVisaBuffer &buffer,
                                         HeaderDevices devices) {
  require(devices.bridge != nullptr);
  fill_visa_reference(buffer);
  const auto input = buffer.instruction();
  if (!input) { throw std::runtime_error("Missing original routing instruction"); }
  const auto type = input->opcode >> 40;
  const auto address = static_cast<std::uint32_t>(input->opcode);
  constexpr std::uint32_t first_sru = 0x2400;
  constexpr std::uint32_t end_sru = 0x2500;
  constexpr std::uint32_t first_dpi = 0x1c00;
  constexpr std::uint32_t end_dpi = 0x1d00;
  constexpr unsigned int direct_read = 0x10;
  constexpr unsigned int direct_write = 0x11;
  const bool routed =
      (address >= first_sru && address < end_sru) || (address >= first_dpi && address < end_dpi);
  const bool peripheral =
      input->words == 3 && (type == direct_read || type == direct_write) && routed;
  if (peripheral) {
    // Explicit documented worst-bound inputs, not an inferred physical phase.
    devices.bridge->issue(
        {.pc = cpu.state().pc,
         .address = address,
         .operation = type == direct_read ? lead_a1::AccessKind::read : lead_a1::AccessKind::write,
         .delivery_cycles = type == direct_read ? 8U : 5U});
    while (!devices.bridge->ready()) { original_routing_clock(devices); }
  }
  const auto step = cpu.step(buffer);
  if (!peripheral || type == direct_write) {
    for (unsigned int cycle = 0; cycle < step.minimum_core_cycles; ++cycle) {
      original_routing_clock(devices);
    }
  }
  return step;
}

void original_routing_group(lead_a1::Sharc &cpu,
                            lead_a1::SharcVisaBuffer &buffer,
                            HeaderDevices devices,
                            unsigned int group,
                            std::uint32_t expected) {
  require(devices.bridge != nullptr);
  constexpr std::uint64_t full40 = 0xffffffffff;
  constexpr std::uint32_t pben2 = 0x247a;
  constexpr std::uint32_t pben3 = 0x247b;
  const bool signed_mask = group == 3 || group == 4;
  const std::array<unsigned int, 6> widths{3, signed_mask ? 2U : 3U, 1, 2, 1, 3};
  for (const auto width : widths) {
    const auto step = original_routing_step(cpu, buffer, devices);
    require(step.address_units == width && step.minimum_core_cycles == 1);
  }
  require(cpu.state().data[0][0] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{expected} << 8, .known = full40}));
  const auto bus = devices.bridge->snapshot();
  const auto posted = bus.writes[0];
  require(bus.write_count == 1 && posted && posted->value == expected &&
          posted->intent.address == (group < 3 ? pben2 : pben3));
}

void original_timing_groups(lead_a1::Sharc &cpu,
                            lead_a1::SharcVisaBuffer &buffer,
                            HeaderDevices devices) {
  require(devices.bridge != nullptr);
  constexpr std::array<unsigned int, 6> widths{3, 3, 1, 3, 1, 3};
  constexpr std::array<std::uint32_t, 2> targets{0x3de57bde, 0x3de5fbde};
  constexpr std::array<std::uint32_t, 2> addresses{0x2432, 0x2452};
  constexpr std::uint64_t full40 = 0xffffffffff;
  for (unsigned int group = 0; group < targets.size(); ++group) {
    for (const auto width : widths) {
      const auto step = original_routing_step(cpu, buffer, devices);
      require(step.address_units == width && step.minimum_core_cycles == 1);
    }
    require(cpu.state().data[0][0] ==
            (lead_a1::SharcKnown40{.value = std::uint64_t{targets[group]} << 8, .known = full40}));
    const auto bus = devices.bridge->snapshot();
    const auto posted = bus.writes[0];
    require(bus.write_count == 1 && posted && posted->value == targets[group] &&
            posted->intent.address == addresses[group]);
  }
}

void check_original_timing_routes(HeaderDevices devices) {
  require(devices.clock_routes != nullptr && devices.frame_routes != nullptr);
  constexpr std::array<std::uint32_t, 6> clocks{
      0x252630c2, 0x3def7bde, 0x3de57bde, 0x3def7bde, 0x3def7bde, 0x3def7bde};
  constexpr std::array<std::uint32_t, 6> frames{
      0x2736b4e3, 0x3def7bde, 0x3de5fbde, 0x01ef7bde, 0x000003de, 0};
  const auto clock = devices.clock_routes->snapshot();
  const auto frame = devices.frame_routes->snapshot();
  require(clock.routes == clocks && frame.routes == frames);
  for (const auto &pending : clock.pending) { require(!pending); }
  for (const auto &pending : frame.pending) { require(!pending); }
  require(!devices.clock_routes->output({.word = 2, .field = 3}).has_value());
  require(!devices.frame_routes->output({.word = 2, .field = 3}).has_value());
}

void original_pdap_setup(lead_a1::Sharc &cpu,
                         lead_a1::SharcVisaBuffer &buffer,
                         HeaderDevices devices) {
  require(devices.bridge != nullptr);
  constexpr unsigned int instructions = 4;
  constexpr std::uint32_t expected = 0x800ff000;
  constexpr std::uint32_t pp_ctl = 0x24b1;
  for (unsigned int index = 0; index < instructions; ++index) {
    const auto step = original_routing_step(cpu, buffer, devices);
    require(step.address_units == 3 && step.minimum_core_cycles == 1);
  }
  require(cpu.state().system[0] == (lead_a1::SharcKnown32{.value = expected, .known = full32}));
  const auto bus = devices.bridge->snapshot();
  const auto posted = bus.writes[0];
  require(bus.write_count == 1 && posted && posted->value == expected &&
          posted->intent.address == pp_ctl);
}

void check_original_pdap(HeaderDevices devices) {
  require(devices.pdap != nullptr);
  lead_a1::SharcPdapState expected;
  constexpr std::uint32_t control = 0x800ff000;
  expected.control[1] = control;
  require(devices.pdap->snapshot() == expected);
  require(!devices.pdap->fifo_read_ready());
  require(devices.pdap->interrupts() == lead_a1::SharcPdapInterrupts{});
}

void original_visa_routing_boundary(lead_a1::SharcMemory &memory,
                                    lead_a1::Sharc &cpu,
                                    lead_a1::SharcVisaBuffer &buffer,
                                    HeaderDevices devices) {
  require(devices.power != nullptr && devices.dai != nullptr && devices.bridge != nullptr);
  // Independently derived raw words after each original read/AND/OR/write.
  // Type17b FFC0/F03F are signed16 immediates, not zero-extended masks.
  constexpr std::array<std::uint32_t, 8> targets{
      0x1a5405d6, 0x1a0005d6, 0x000005d6, 0x1d71f780, 0x1d71f000, 0x1d700000, 0x1d000000, 0};
  for (unsigned int group = 0; group < targets.size(); ++group) {
    original_routing_group(cpu, buffer, devices, group, targets[group]);
  }
  original_timing_groups(cpu, buffer, devices);
  original_pdap_setup(cpu, buffer, devices);
  constexpr unsigned int prefix_words = 181;
  require(cpu.state().pc == devices.placement.second_call + prefix_words);
  fill_visa_reference(buffer);
  const auto before = cpu.state();
  const auto input = buffer.state();
  const auto power = devices.power->snapshot();
  const auto control = devices.control.snapshot();
  const auto spi = devices.spi.state();
  try {
    static_cast<void>(original_routing_step(cpu, buffer, devices));
  } catch (const std::exception &error) {
    const std::string message(error.what());
    if (message.find("word-address=0x1c10 operation=read") == std::string::npos ||
        message.find("Unmapped SHARC IOP register") == std::string::npos) {
      throw std::runtime_error("Unexpected original IOP frontier: " + message);
    }
    require(cpu.state() == before && buffer.state() == input &&
            devices.power->snapshot() == power && devices.control.snapshot() == control &&
            devices.spi.state() == spi && !memory.active());
    constexpr std::array<std::uint32_t, 4> routes{0x0e2482ca, 0x1348d30f, 0x000005d6, 0};
    const auto routing = devices.dai->snapshot();
    require(routing.routes == routes);
    for (const auto &pending : routing.pending) { require(!pending); }
    const auto bus = devices.bridge->snapshot();
    constexpr unsigned int elapsed_clocks = 193;
    require(bus.clocks == elapsed_clocks && bus.write_count == 0 && bus.read &&
            !bus.read->response && bus.request && bus.request->pc == before.pc);
    check_original_timing_routes(devices);
    check_original_pdap(devices);
    std::cout << "  Separate IOP routing prefix:64 actual instructions, ten original "
                 "routing groups and PDAP setup; one DAI/DPI bridge; explicit5/8 timing input; "
                 "SRU2_PIN0 remains unmapped\n";
    return;
  }
  throw std::runtime_error("Missing original SRU2_PIN0 read boundary");
}

void check_original_power_outputs(const lead_a1::Sharc &cpu, HeaderDevices devices) {
  require(devices.power != nullptr);
  const auto state = devices.power->snapshot();
  constexpr std::uint32_t expected_control = 0x8180f;
  constexpr std::uint32_t expected_shutdown = 0x8080f;
  require(state == (lead_a1::SharcPowerState{.control = expected_control,
                                             .shutdown = expected_shutdown,
                                             .accelerator = lead_a1::SharcAccelerator::fir}));
  require(devices.power->enabled(lead_a1::SharcClockGate::spi) &&
          devices.power->enabled(lead_a1::SharcClockGate::dai) &&
          !devices.power->enabled(lead_a1::SharcClockGate::uart));
  constexpr std::array<unsigned int, 8> registers{0, 1, 2, 7, 8, 9, 10, 15};
  constexpr std::array<std::uint32_t, 8> values{0, 1, 2, full32, 0, 1, 2, full32};
  for (unsigned int index = 0; index < registers.size(); ++index) {
    require(cpu.state().dag[0][1][registers[index]] ==
            (lead_a1::SharcKnown32{.value = values[index], .known = full32}));
  }
  constexpr std::uint32_t call_frame = 0x1000000;
  require(cpu.state().depth == 1 &&
          cpu.state().pc_stack[0] == (call_frame | (devices.placement.visa_entry + 6)) &&
          cpu.state().loops_empty == true && !cpu.state().count_loop &&
          !cpu.state().arithmetic_loop && !cpu.state().branch && !cpu.state().interrupt);
}

std::uint32_t original_power_pc(Placement placement, unsigned int index) {
  constexpr unsigned int full_prefix = 6;
  if (index == 0) { return placement.visa_entry + 3; }
  if (index < full_prefix) { return placement.second_call + ((index - 1) * 3); }
  return placement.second_call + 15 + ((index - full_prefix) * 2);
}

void original_visa_power_reference(lead_a1::SharcMemory &memory,
                                   lead_a1::Sharc &cpu,
                                   lead_a1::SharcVisaBuffer &buffer,
                                   HeaderDevices devices) {
  require(devices.power != nullptr);
  constexpr unsigned int instructions = 18;
  constexpr unsigned int full_prefix = 6;
  constexpr unsigned int power_write = 5;
  unsigned int elapsed_core{};
  for (unsigned int index = 0; index < instructions; ++index) {
    fill_visa_reference(buffer);
    const auto step = cpu.step(buffer);
    const auto expected_pc = original_power_pc(devices.placement, index);
    require(step.pc == expected_pc && step.address_units == (index < full_prefix ? 3U : 2U) &&
            step.minimum_core_cycles == 1);
    devices.clock.device.advance({.core_cycles = step.minimum_core_cycles});
    if (index == power_write) { require(devices.power->state().remaining_pclk == 2); }
    if (index > power_write) {
      // Count complete elapsed PCLK periods only after the delivered store.
      // No unknown pre-write CCLK/PCLK phase or core/IOP bridge is invented.
      elapsed_core += step.minimum_core_cycles;
      devices.power->advance_pclk(elapsed_core / 2);
      elapsed_core %= 2;
    }
  }
  constexpr std::uint32_t initializer_prefix = 39;
  require(cpu.state().pc == devices.placement.second_call + initializer_prefix);
  check_original_power_outputs(cpu, devices);
  original_visa_routing_boundary(memory, cpu, buffer, devices);
  std::cout << "  Separate VISA power prefix:18 actual instructions, six full/twelve compact, "
               "delayed PMCTL1 outputs and eight modifiers\n";
}

// Earlier native PMCTL/core-IOP timing is still a separately bounded input.
void original_application_clock(lead_a1::SharcMemory &memory,
                                lead_a1::Sharc &cpu,
                                SerialInput &serial,
                                HeaderDevices devices) {
  require(devices.profile == OriginalProfile::clocked_iop);
  advance_clocked_visa_devices(memory, cpu, serial, devices);
}

void original_application_prepare(lead_a1::SharcMemory &memory,
                                  lead_a1::Sharc &cpu,
                                  lead_a1::SharcFetch &fetch,
                                  SerialInput &serial,
                                  HeaderDevices devices) {
  constexpr unsigned int limit = 16;
  for (unsigned int clock = 0; clock < limit; ++clock) {
    const auto bus = fetch.address_pm_request(cpu) ? lead_a1::SharcFetchBus::pm_data
                                                   : lead_a1::SharcFetchBus::available;
    const auto event = fetch.advance_sequenced(cpu, bus);
    require(!event.compute_stall && !event.control_stall && !event.dag_stall && !event.loop_stall);
    if (fetch.state().execute) { return; }
    original_application_clock(memory, cpu, serial, devices);
  }
  throw std::runtime_error("Missing original application Execute");
}

enum class OriginalClock : std::uint8_t { native, visa };
enum class RetireClock : std::uint8_t { pending, elapsed };

bool wait_original_iop(lead_a1::SharcMemory &memory,
                       lead_a1::Sharc &cpu,
                       lead_a1::SharcFetch &fetch,
                       SerialInput &serial,
                       HeaderDevices devices,
                       OriginalClock clock_source) {
  require(devices.bridge != nullptr);
  const auto access = fetch.peripheral_access(cpu);
  const bool read = access && access->operation == lead_a1::AccessKind::read;
  if (access) {
    if (!devices.bridge->bound_to(memory, access->address)) {
      std::ostringstream message;
      message << "Original IOP owner pc=0x" << std::hex << access->pc << " word-address=0x"
              << access->address << " operation=" << (read ? "read" : "write")
              << ": no qualified bridge mapping";
      throw std::runtime_error(message.str());
    }
    devices.bridge->issue({.pc = access->pc,
                           .address = access->address,
                           .operation = access->operation,
                           .delivery_cycles = read ? 8U : 5U});
    const auto held = fetch.state();
    const auto pc = cpu.state().pc;
    while (!devices.bridge->ready()) {
      if (!read) { fetch.wait_iop(cpu, *devices.bridge); }
      if (clock_source == OriginalClock::native) {
        advance_devices(memory, cpu, serial, devices);
      } else {
        original_application_clock(memory, cpu, serial, devices);
      }
      if (read && !devices.bridge->ready()) { fetch.wait_iop(cpu, *devices.bridge); }
      require(fetch.state() == held && cpu.state().pc == pc);
    }
  }
  return read;
}

RetireClock retire_loader_instruction(lead_a1::SharcMemory &memory,
                                      lead_a1::Sharc &cpu,
                                      lead_a1::SharcFetch &fetch,
                                      SerialInput &serial,
                                      HeaderDevices devices) {
  const bool read = devices.profile == OriginalProfile::clocked_iop &&
                    wait_original_iop(memory, cpu, fetch, serial, devices, OriginalClock::native);
  require(cpu.step(fetch).minimum_core_cycles == 1);
  return read ? RetireClock::elapsed : RetireClock::pending;
}

void finish_loader_clock(lead_a1::SharcMemory &memory,
                         lead_a1::Sharc &cpu,
                         SerialInput &serial,
                         HeaderDevices devices,
                         RetireClock clock) {
  if (clock == RetireClock::pending) { advance_devices(memory, cpu, serial, devices); }
}

lead_a1::SharcStep original_application_retire(lead_a1::SharcMemory &memory,
                                               lead_a1::Sharc &cpu,
                                               lead_a1::SharcFetch &fetch,
                                               SerialInput &serial,
                                               HeaderDevices devices) {
  const bool read = wait_original_iop(memory, cpu, fetch, serial, devices, OriginalClock::visa);
  const auto step = cpu.step(fetch);
  require(step.minimum_core_cycles == 1);
  if (!read) { original_application_clock(memory, cpu, serial, devices); }
  return step;
}

struct ExpectedPosted {
  std::uint32_t pc;
  std::uint32_t address;
  std::uint32_t value;
};

void check_posted_application(HeaderDevices devices, ExpectedPosted expected) {
  const auto bus = devices.bridge->snapshot();
  unsigned int matches{};
  for (const auto &write : bus.writes) {
    if (write && write->intent.pc == expected.pc) {
      require(write->intent.address == expected.address &&
              write->intent.operation == lead_a1::AccessKind::write &&
              write->intent.delivery_cycles == 5 && write->value == expected.value &&
              bus.clocks == write->posted_at + 1);
      ++matches;
    }
  }
  require(matches == 1);
}

struct OriginalRmw {
  unsigned int offset;
  std::array<unsigned int, 6> widths;
  std::uint32_t address;
  std::uint32_t initial;
  std::uint32_t mask;
  std::uint32_t selected;
  std::uint32_t cleared;
  std::uint32_t result;
  bool verify_spi_checkpoint{};
  bool verify_pin_checkpoint{};
};

void check_post_spi_outputs(HeaderDevices devices);
void check_post_pin_outputs(HeaderDevices devices);

void original_post_spi_group(lead_a1::SharcMemory &memory,
                             lead_a1::Sharc &cpu,
                             lead_a1::SharcFetch &fetch,
                             SerialInput &serial,
                             HeaderDevices devices,
                             const OriginalRmw &group) {
  const std::array r0{
      group.initial, group.initial, group.cleared, group.cleared, group.result, group.result};
  const std::array r1{0U, group.mask, group.mask, group.selected, group.selected, group.selected};
  auto pc = devices.placement.second_call + group.offset;
  constexpr std::uint64_t full40 = 0xffffffffff;
  for (unsigned int index = 0; index < r0.size(); ++index) {
    original_application_prepare(memory, cpu, fetch, serial, devices);
    const auto access = fetch.peripheral_access(cpu);
    const auto previous_r1 = cpu.state().data[0][1];
    require(access.has_value() == (index == 0 || index == 5));
    if (access) {
      require(access->pc == pc && access->address == group.address &&
              access->operation ==
                  (index == 0 ? lead_a1::AccessKind::read : lead_a1::AccessKind::write));
    }
    const auto step = original_application_retire(memory, cpu, fetch, serial, devices);
    require(step.pc == pc && step.address_units == group.widths[index]);
    if (index == 0 && group.verify_spi_checkpoint) { check_post_spi_outputs(devices); }
    if (index == 0 && group.verify_pin_checkpoint) { check_post_pin_outputs(devices); }
    require(cpu.state().data[0][0] ==
            (lead_a1::SharcKnown40{.value = std::uint64_t{r0[index]} << 8, .known = full40}));
    require(cpu.state().data[0][1] ==
            (index == 0
                 ? previous_r1
                 : lead_a1::SharcKnown40{.value = std::uint64_t{r1[index]} << 8, .known = full40}));
    if (index == 5) {
      check_posted_application(devices,
                               {.pc = pc, .address = group.address, .value = group.result});
    }
    pc += group.widths[index];
  }
}

bool uses_clock_initializer(Placement placement) {
  if (placement.second_call == placements[0][0].second_call ||
      placement.second_call == placements[1][0].second_call) {
    return true;
  }
  require(placement.second_call == placements[0][1].second_call ||
          placement.second_call == placements[1][1].second_call);
  return false;
}

void check_post_spi_outputs(HeaderDevices devices) {
  const bool clocks = uses_clock_initializer(devices.placement);
  constexpr std::uint32_t clock_result = 0x252ef7bd;
  constexpr std::uint32_t clock_reset = 0x252630c2;
  constexpr std::uint32_t enable_clock_result = 0x0e2412ca;
  constexpr std::uint32_t enable_other_result = 0x0e2482c1;
  const auto enables = devices.dai->snapshot();
  const auto timing = devices.clock_routes->snapshot();
  require(!enables.pending[0] &&
          enables.routes[0] == (clocks ? enable_clock_result : enable_other_result));
  require(!timing.pending[0] && timing.routes[0] == (clocks ? clock_result : clock_reset));
}

bool original_sport_part1(Placement placement) {
  return placement.second_call == placements[1][0].second_call ||
         placement.second_call == placements[1][1].second_call;
}

void original_sport_step(lead_a1::SharcMemory &memory,
                         lead_a1::Sharc &cpu,
                         lead_a1::SharcFetch &fetch,
                         SerialInput &serial,
                         HeaderDevices devices,
                         original_sport::Step expected) {
  original_application_prepare(memory, cpu, fetch, serial, devices);
  const auto pc = devices.placement.second_call + expected.offset;
  const auto value = original_sport_part1(devices.placement)
                         ? expected.part1_value.value_or(expected.value)
                         : expected.value;
  const auto access = fetch.peripheral_access(cpu);
  using Operation = original_sport::Operation;
  require(access.has_value() == (expected.operation != Operation::value));
  if (access) {
    require(access->pc == pc && access->address == expected.address &&
            access->operation == (expected.operation == Operation::read
                                      ? lead_a1::AccessKind::read
                                      : lead_a1::AccessKind::write));
  }
  const auto step = original_application_retire(memory, cpu, fetch, serial, devices);
  require(step.pc == pc && step.address_units == expected.words);
  constexpr unsigned int ustat1 = 0x70;
  constexpr std::uint64_t full40 = 0xffffffffff;
  if (expected.reg == ustat1) {
    require(cpu.state().system[0] == (lead_a1::SharcKnown32{.value = value, .known = full32}));
  } else {
    require(expected.reg < cpu.state().data[0].size());
    require(cpu.state().data[0][expected.reg] ==
            (lead_a1::SharcKnown40{.value = std::uint64_t{value} << 8, .known = full40}));
  }
  if (expected.operation == Operation::write) {
    check_posted_application(devices, {.pc = pc, .address = expected.address, .value = value});
  }
}

void original_sport_secondary(lead_a1::SharcMemory &memory,
                              lead_a1::Sharc &cpu,
                              lead_a1::SharcFetch &fetch,
                              SerialInput &serial,
                              HeaderDevices devices) {
  constexpr std::array<std::uint32_t, 4> addresses{0xc1a, 0xc1b, 0x41a, 0x41b};
  constexpr unsigned int ustat1 = 0x70;
  const bool clocks = uses_clock_initializer(devices.placement);
  const auto first_offset = clocks ? 601U : 651U;
  auto offset = first_offset;
  const auto count = clocks ? 2U : 4U;
  for (unsigned int group = 0; group < count; ++group) {
    using Operation = original_sport::Operation;
    const std::array operations{Operation::read, Operation::value, Operation::write};
    for (unsigned int index = 0; index < operations.size(); ++index) {
      original_sport_step(memory,
                          cpu,
                          fetch,
                          serial,
                          devices,
                          {.offset = offset,
                           .words = 3,
                           .reg = ustat1,
                           .value = index == 0 ? 0U : 4U,
                           .operation = operations[index],
                           .address = addresses[group]});
      offset += 3;
    }
  }
}

std::span<const original_sport::Dma> original_sport_parameters(HeaderDevices devices) {
  for (unsigned int part = 0; part < placements.size(); ++part) {
    for (unsigned int transport = 0; transport < placements[part].size(); ++transport) {
      if (devices.placement.second_call == placements[part][transport].second_call) {
        return original_sport::dma[part][transport];
      }
    }
  }
  throw std::logic_error("Unknown original SPORT placement");
}

void original_sport_dma_setup(lead_a1::SharcMemory &memory,
                              lead_a1::Sharc &cpu,
                              lead_a1::SharcFetch &fetch,
                              SerialInput &serial,
                              HeaderDevices devices) {
  constexpr std::array<unsigned int, 8> widths{3, 3, 2, 3, 2, 3, 3, 3};
  constexpr std::array<std::uint32_t, 4> index_registers{0xc40, 0xc48, 0x440, 0x448};
  const auto first_offset = uses_clock_initializer(devices.placement) ? 619U : 687U;
  auto offset = first_offset;
  for (const auto &dma : original_sport_parameters(devices)) {
    const std::array values{dma.index, 1U, dma.count, dma.chain};
    const auto first = index_registers[dma.sport] + (dma.channel * 4);
    for (unsigned int index = 0; index < widths.size(); ++index) {
      const bool write = index % 2 != 0;
      original_sport_step(
          memory,
          cpu,
          fetch,
          serial,
          devices,
          {.offset = offset,
           .words = widths[index],
           .reg = 0,
           .value = values[index / 2],
           .operation = write ? original_sport::Operation::write : original_sport::Operation::value,
           .address = first + (index / 2)});
      offset += widths[index];
    }
  }
}

void check_original_sport_parameters(lead_a1::SharcMemory &memory, HeaderDevices devices) {
  for (const auto &expected : original_sport_parameters(devices)) {
    const auto port = devices.sports[expected.sport]->snapshot();
    const auto &dma = port.dma[expected.channel];
    require(dma.index == expected.index && dma.modifier == 1 && dma.count == expected.count &&
            dma.chain == expected.chain && !dma.active && !dma.load);
    const std::array pointed{expected.next_index, 1U, expected.count, expected.next_chain};
    const std::array partner{expected.index, 1U, expected.count, expected.chain};
    for (unsigned int index = 0; index < pointed.size(); ++index) {
      require(memory.read({.address = expected.chain - index}) == pointed[index] &&
              memory.read({.address = expected.next_chain - index}) == partner[index]);
    }
  }
}

void check_original_sport_controls(HeaderDevices devices) {
  const bool clocks = uses_clock_initializer(devices.placement);
  constexpr std::array<std::uint32_t, 4> first_controls{0x023c21f0, 0x003c21f0, 0x020209f0, 0};
  constexpr std::array<std::uint32_t, 4> other_controls{
      0x023c21f0, 0x003c21f0, 0x020c21f0, 0x000c21f0};
  constexpr std::uint32_t dual_tdm = 0x00800161;
  constexpr std::uint32_t single_tdm = 0x161;
  constexpr std::uint32_t selected = 0xfff;
  for (unsigned int index = 0; index < devices.sports.size(); ++index) {
    const auto snapshot = devices.sports[index]->snapshot();
    const auto &state = snapshot.serial;
    const std::array<std::array<std::uint32_t, 3>, 3> unsampled{};
    require(state.sampled == unsampled && !state.word_active && !state.frame_active &&
            !state.lines.clock && !state.lines.frame && !state.lines.data[0] &&
            !state.lines.data[1]);
    for (const auto &dma : snapshot.dma) {
      require(dma.mode == lead_a1::SharcSportDmaMode{} && !dma.active && !dma.load &&
              !dma.transmit_buffer && !dma.transmit_shift && !dma.receive_buffer &&
              !dma.interrupt_pending);
    }
    if (index >= first_controls.size()) {
      require(state.control == std::array<std::uint32_t, 3>{});
      continue;
    }
    constexpr std::uint32_t part1_sport2 = 0x02020970;
    const auto clock_control = index == 2 && original_sport_part1(devices.placement)
                                   ? part1_sport2
                                   : first_controls[index];
    require(state.control[0] == (clocks ? clock_control : other_controls[index]));
    require(state.control[1] == (index < 2 || !clocks ? 4U : 0U));
    const auto later_tdm = clocks ? 0U : single_tdm;
    require(state.control[2] == (index < 2 ? dual_tdm : later_tdm));
    require(state.select[0] == (index < 2 || !clocks ? selected : 0U));
  }
}

void original_picr_ordinary(lead_a1::SharcMemory &memory,
                            lead_a1::Sharc &cpu,
                            lead_a1::SharcFetch &fetch,
                            SerialInput &serial,
                            HeaderDevices devices,
                            unsigned int offset,
                            unsigned int words) {
  original_application_prepare(memory, cpu, fetch, serial, devices);
  require(!fetch.peripheral_access(cpu));
  const auto step = original_application_retire(memory, cpu, fetch, serial, devices);
  require(step.pc == devices.placement.second_call + offset && step.address_units == words);
}

lead_a1::SharcKnown32 original_picr_mode(lead_a1::SharcMemory &memory,
                                         lead_a1::Sharc &cpu,
                                         lead_a1::SharcFetch &fetch,
                                         SerialInput &serial,
                                         HeaderDevices devices) {
  auto saved = cpu.state().system[2];
  constexpr std::uint32_t rounding32 = 0x10000;
  constexpr std::uint32_t circular = 0x01000000;
  saved.value = (saved.value & ~rounding32) | circular;
  saved.known |= rounding32 | circular;
  constexpr std::array offsets{931U, 934U, 937U, 939U, 942U, 943U};
  constexpr std::array widths{3U, 3U, 2U, 3U, 1U, 1U};
  constexpr std::uint32_t temporary = 0x010014fc;
  constexpr std::uint32_t writable = 0x01e1fcff;
  for (unsigned int index = 0; index < offsets.size(); ++index) {
    original_picr_ordinary(memory, cpu, fetch, serial, devices, offsets[index], widths[index]);
    const auto mode = cpu.state().system[2];
    if (index == 0) { require((mode.value & rounding32) == 0 && (mode.known & rounding32) != 0); }
    if (index == 1 || index == 2) { require(mode == saved); }
    if (index >= 2) { require(cpu.state().system[0] == saved); }
    if (index >= 3) {
      require((mode.value & writable) == temporary && (mode.known & writable) == writable &&
              (mode.value & ~writable) == (saved.value & ~writable));
    }
  }
  return saved;
}

void check_original_picr_setup(const lead_a1::SharcState &state,
                               original_picr::Setup expected,
                               std::uint32_t stack,
                               lead_a1::SharcKnown32 saved) {
  using Target = original_picr::Target;
  constexpr std::uint64_t full40 = 0xffffffffff;
  constexpr std::uint32_t secondary_length = 0x100;
  switch (expected.target) {
    case Target::base0:
      require(state.dag[1][3][0] == (lead_a1::SharcKnown32{.value = stack, .known = full32}) &&
              state.dag[1][0][0] == state.dag[1][3][0]);
      break;
    case Target::length0:
      require(state.dag[1][2][0] ==
              (lead_a1::SharcKnown32{.value = secondary_length, .known = full32}));
      break;
    case Target::data0:
    case Target::data2:
    case Target::data3: {
      const auto upper_reg = expected.target == Target::data2 ? 2U : 3U;
      const auto reg = expected.target == Target::data0 ? 0U : upper_reg;
      require(
          state.data[1][reg] ==
          (lead_a1::SharcKnown40{.value = std::uint64_t{expected.value} << 8, .known = full40}));
      break;
    }
    case Target::modifier8:
      require(state.dag[1][1][8] == (lead_a1::SharcKnown32{.known = full32}));
      break;
    case Target::modifier1:
      require(state.dag[1][1][1] == (lead_a1::SharcKnown32{.value = 1, .known = full32}));
      break;
    case Target::mode:
    case Target::nop: require(state.system[2] == saved); break;
    default: throw std::logic_error("Unknown original secondary setup target");
  }
}

void original_picr_initialization(lead_a1::SharcMemory &memory,
                                  lead_a1::Sharc &cpu,
                                  lead_a1::SharcFetch &fetch,
                                  SerialInput &serial,
                                  HeaderDevices devices) {
  constexpr unsigned int ustat1 = 0x70;
  constexpr std::uint32_t picr0 = 0x2200;
  constexpr std::array offsets{919U, 922U, 925U, 928U};
  constexpr std::array values{0x0a418820U, 0x0a418020U, 0x0a41b020U, 0x0a41b020U};
  using Operation = original_sport::Operation;
  constexpr std::array operations{
      Operation::read, Operation::value, Operation::value, Operation::write};
  for (unsigned int index = 0; index < offsets.size(); ++index) {
    original_sport_step(memory,
                        cpu,
                        fetch,
                        serial,
                        devices,
                        {.offset = offsets[index],
                         .words = 3,
                         .reg = ustat1,
                         .value = values[index],
                         .operation = operations[index],
                         .address = picr0});
  }
  const auto saved = original_picr_mode(memory, cpu, fetch, serial, devices);
  const auto stack = original_sport_part1(devices.placement) ? 0x923f9U : 0x93ee1U;
  for (const auto &step : original_picr::setup) {
    original_picr_ordinary(memory, cpu, fetch, serial, devices, step.offset, step.words);
    check_original_picr_setup(cpu.state(), step, stack, saved);
  }
  require(cpu.state().effective[2] == saved);
  constexpr std::uint32_t picr_result = 0x0a41b020;
  require(devices.priority->snapshot().control[0] == picr_result &&
          devices.priority->destinations(12) == ((1U << 2) | (1U << 12)) &&
          !devices.priority->outputs()[2] && devices.priority->outputs()[1] == false);
  constexpr std::array<original_sport::Step, 11> idp{
      {{.offset = 966,
        .words = 3,
        .reg = ustat1,
        .value = 0,
        .operation = Operation::read,
        .address = 0x24b0},
       {.offset = 969, .words = 3, .reg = ustat1, .value = 0},
       {.offset = 972,
        .words = 3,
        .reg = ustat1,
        .value = 0,
        .operation = Operation::write,
        .address = 0x24b0},
       {.offset = 975,
        .words = 3,
        .reg = ustat1,
        .value = 0xffff,
        .operation = Operation::read,
        .address = 0x24b2},
       {.offset = 978, .words = 3, .reg = ustat1, .value = 0x1ffff},
       {.offset = 981, .words = 3, .reg = ustat1, .value = 0x1ff01},
       {.offset = 984, .words = 3, .reg = ustat1, .value = 0x10101},
       {.offset = 987,
        .words = 3,
        .reg = ustat1,
        .value = 0x10101,
        .operation = Operation::write,
        .address = 0x24b2},
       {.offset = 990, .words = 3, .reg = 0, .value = 0x93ee1, .part1_value = 0x923f9},
       {.offset = 993,
        .words = 3,
        .reg = 0,
        .value = 0x93ee1,
        .operation = Operation::write,
        .address = 0x2408,
        .part1_value = 0x923f9},
       {.offset = 996, .words = 3, .reg = 0, .value = 0x93f61, .part1_value = 0x92479}}};
  for (const auto &step : idp) { original_sport_step(memory, cpu, fetch, serial, devices, step); }
}

void original_idp_index_b_store(lead_a1::SharcMemory &memory,
                                lead_a1::Sharc &cpu,
                                lead_a1::SharcFetch &fetch,
                                SerialInput &serial,
                                HeaderDevices devices) {
  require(!wait_original_iop(memory, cpu, fetch, serial, devices, OriginalClock::visa));
  const auto step = cpu.step(fetch);
  constexpr unsigned int last_store = 999;
  require(step.pc == devices.placement.second_call + last_store && step.address_units == 3);
  const auto &queue = devices.bridge->snapshot();
  require(queue.write_count == 3 && !queue.request && queue.writes[2]);
  const auto &last = queue.writes[2];
  if (!last) { throw std::runtime_error("Missing posted IDP index B write"); }
  constexpr std::uint32_t index_b = 0x2418;
  const auto value = original_sport_part1(devices.placement) ? 0x92479U : 0x93f61U;
  require(last->intent.pc == step.pc && last->intent.address == index_b && last->value == value &&
          last->posted_at == queue.clocks);
}

void original_sport_failure(lead_a1::SharcMemory &memory,
                            lead_a1::Sharc &cpu,
                            lead_a1::SharcFetch &fetch,
                            SerialInput &serial,
                            HeaderDevices devices) {
  const bool clocks = uses_clock_initializer(devices.placement);
  if (clocks) {
    constexpr unsigned int pulse_write = 818;
    constexpr unsigned int pulse_width = 0x10080;
    constexpr std::uint32_t pcg_pw = 0x24c4;
    original_sport_step(memory,
                        cpu,
                        fetch,
                        serial,
                        devices,
                        {.offset = pulse_write,
                         .words = 3,
                         .reg = 0,
                         .value = pulse_width,
                         .operation = original_sport::Operation::write,
                         .address = pcg_pw});
  } else {
    original_picr_initialization(memory, cpu, fetch, serial, devices);
  }
  original_application_prepare(memory, cpu, fetch, serial, devices);
  if (!clocks) { original_idp_index_b_store(memory, cpu, fetch, serial, devices); }
  const auto core = cpu.state();
  const auto pipeline = fetch.state();
  const auto failing_pc = devices.placement.second_call + (clocks ? 818U : 987U);
  const auto address = clocks ? 0x24c4U : 0x24b2U;
  std::ostringstream prefix;
  prefix << "SHARC IOP bridge pc=0x" << std::hex << failing_pc << " word-address=0x" << address
         << " operation=write cclk=";
  bool stopped{};
  try {
    if (clocks) {
      static_cast<void>(original_application_retire(memory, cpu, fetch, serial, devices));
    } else {
      original_application_clock(memory, cpu, serial, devices);
    }
  } catch (const std::runtime_error &error) {
    const std::string message(error.what());
    if (!message.starts_with(prefix.str()) ||
        message.find(clocks ? "Unmapped SHARC IOP register"
                            : "PDAP DMA/ping-pong engine not implemented") == std::string::npos) {
      throw;
    }
    stopped = true;
  }
  const auto bus = devices.bridge->snapshot();
  require(stopped && cpu.state() == core && fetch.state() == pipeline && !memory.active() &&
          !devices.bridge->ready() && !devices.bridge->waiting());
  const auto &queued = bus.writes[0];
  if (!queued) { throw std::runtime_error("Missing original failed posted write"); }
  require(bus.write_count == (clocks ? 1U : 3U) && queued->intent.pc == failing_pc &&
          queued->intent.address == address && bus.clocks == queued->posted_at + 5);
  if (clocks) {
    const auto &request = bus.request;
    constexpr unsigned int read_offset = 821;
    constexpr unsigned int read_address = 0x24c5;
    require(request && request->pc == devices.placement.second_call + read_offset &&
            request->address == read_address && request->operation == lead_a1::AccessKind::read &&
            request->delivery_cycles == 8 && bus.read && !bus.read->response);
  } else {
    constexpr std::uint32_t ping_pong = 0x10101;
    constexpr unsigned int next_offset = 1002;
    require(!bus.request && !bus.read && queued->value == ping_pong &&
            cpu.state().pc == devices.placement.second_call + next_offset);
  }
  constexpr std::uint32_t configured = 0x4d14;
  constexpr std::uint32_t vector_end = 0x92180;
  require(devices.spi.state().port_control == configured && devices.spi.state().control == 0 &&
          devices.spi.state().serial.remaining == 0 &&
          devices.spi.state().serial.next_word == vector_end);
  check_original_sport_parameters(memory, devices);
  check_original_sport_controls(devices);
}

void original_sport_boundary(lead_a1::SharcMemory &memory,
                             lead_a1::Sharc &cpu,
                             lead_a1::SharcFetch &fetch,
                             SerialInput &serial,
                             HeaderDevices devices) {
  original_sport_secondary(memory, cpu, fetch, serial, devices);
  original_sport_dma_setup(memory, cpu, fetch, serial, devices);
  const auto controls = uses_clock_initializer(devices.placement)
                            ? std::span<const original_sport::Step>(original_sport::controls_0)
                            : std::span<const original_sport::Step>(original_sport::controls_1);
  for (const auto &step : controls) {
    original_sport_step(memory, cpu, fetch, serial, devices, step);
  }
  original_sport_failure(memory, cpu, fetch, serial, devices);
}

constexpr std::array<OriginalRmw, 8> pin_clock_groups{{
    {.offset = 279,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2460,
     .initial = 0x4c80a94,
     .mask = 0xffe03fff,
     .selected = 0xe4000,
     .cleared = 0x4c00a94,
     .result = 0x4ce4a94,
     .verify_spi_checkpoint = true},
    {.offset = 293,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2450,
     .initial = 0x2736b4e3,
     .mask = 0xfffffc1f,
     .selected = 0x3a0,
     .cleared = 0x2736b403,
     .result = 0x2736b7a3},
    {.offset = 305,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2450,
     .initial = 0x2736b7a3,
     .mask = 0xfff07fff,
     .selected = 0xe8000,
     .cleared = 0x273037a3,
     .result = 0x273eb7a3},
    {.offset = 319,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2450,
     .initial = 0x273eb7a3,
     .mask = 0xffffffe0,
     .selected = 0x1d,
     .cleared = 0x273eb7a0,
     .result = 0x273eb7bd},
    {.offset = 331,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2450,
     .initial = 0x273eb7bd,
     .mask = 0xffff83ff,
     .selected = 0x7400,
     .cleared = 0x273e83bd,
     .result = 0x273ef7bd},
    {.offset = 343,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2478,
     .initial = 0xe2412ca,
     .mask = 0xff03ffff,
     .selected = 0x40000,
     .cleared = 0xe0012ca,
     .result = 0xe0412ca},
    {.offset = 357,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2460,
     .initial = 0x4ce4a94,
     .mask = 0xf01fffff,
     .selected = 0x7600000,
     .cleared = 0xe4a94,
     .result = 0x76e4a94},
    {.offset = 371,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0xe0412ca,
     .mask = 0xffffffc0,
     .selected = 0x0,
     .cleared = 0xe0412c0,
     .result = 0xe0412c0},
}};

constexpr std::array<OriginalRmw, 6> pin_other_groups{{
    {.offset = 228,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2460,
     .initial = 0x4c80a94,
     .mask = 0xffffff80,
     .selected = 0x14,
     .cleared = 0x4c80a80,
     .result = 0x4c80a94,
     .verify_spi_checkpoint = true},
    {.offset = 240,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0xe2482c1,
     .mask = 0xfffff03f,
     .selected = 0x40,
     .cleared = 0xe248001,
     .result = 0xe248041},
    {.offset = 252,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2460,
     .initial = 0x4c80a94,
     .mask = 0xffffc07f,
     .selected = 0xa80,
     .cleared = 0x4c80014,
     .result = 0x4c80a94},
    {.offset = 264,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2478,
     .initial = 0xe248041,
     .mask = 0xc0ffffff,
     .selected = 0x1000000,
     .cleared = 0x248041,
     .result = 0x1248041},
    {.offset = 278,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2461,
     .initial = 0x4e84b96,
     .mask = 0xffffff80,
     .selected = 0x18,
     .cleared = 0x4e84b80,
     .result = 0x4e84b98},
    {.offset = 290,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2479,
     .initial = 0x1348d30f,
     .mask = 0xffffffc0,
     .selected = 0x0,
     .cleared = 0x1348d300,
     .result = 0x1348d300},
}};

void check_post_pin_outputs(HeaderDevices devices) {
  const bool clocks = uses_clock_initializer(devices.placement);
  const auto pins = devices.pin_routes->snapshot();
  const auto enables = devices.dai->snapshot();
  const auto frames = devices.frame_routes->snapshot();
  constexpr std::uint32_t pin_clock_result = 0x076e4a94;
  constexpr std::uint32_t pin_other_result = 0x04c80a94;
  constexpr std::uint32_t pin1_other_result = 0x04e84b98;
  constexpr std::uint32_t pin1_reset = 0x04e84b96;
  constexpr std::uint32_t enable_clock_result = 0x0e0412c0;
  constexpr std::uint32_t enable_other_result = 0x01248041;
  constexpr std::uint32_t enable1_other_result = 0x1348d300;
  constexpr std::uint32_t enable1_reset = 0x1348d30f;
  constexpr std::uint32_t frame_clock_result = 0x273ef7bd;
  constexpr std::uint32_t frame_reset = 0x2736b4e3;
  require(!pins.pending[0] && !pins.pending[1] &&
          pins.routes[0] == (clocks ? pin_clock_result : pin_other_result) &&
          pins.routes[1] == (clocks ? pin1_reset : pin1_other_result));
  require(!enables.pending[0] && !enables.pending[1] &&
          enables.routes[0] == (clocks ? enable_clock_result : enable_other_result) &&
          enables.routes[1] == (clocks ? enable1_reset : enable1_other_result));
  require(!frames.pending[0] && frames.routes[0] == (clocks ? frame_clock_result : frame_reset));
}

constexpr std::array<OriginalRmw, 17> data_clock_groups{{
    {.offset = 383,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2440,
     .initial = 0x8144040,
     .mask = 0xfffc0fff,
     .selected = 0x0,
     .cleared = 0x8140040,
     .result = 0x8140040,
     .verify_pin_checkpoint = true},
    {.offset = 396,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0xe0412c0,
     .mask = 0xfffff03f,
     .selected = 0x0,
     .cleared = 0xe041000,
     .result = 0xe041000},
    {.offset = 408,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2440,
     .initial = 0x8140040,
     .mask = 0xff03ffff,
     .selected = 0x40000,
     .cleared = 0x8000040,
     .result = 0x8040040},
    {.offset = 422,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2479,
     .initial = 0x1348d30f,
     .mask = 0xff03ffff,
     .selected = 0x40000,
     .cleared = 0x1300d30f,
     .result = 0x1304d30f},
    {.offset = 436,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2462,
     .initial = 0x3668c98,
     .mask = 0xffffff80,
     .selected = 0x14,
     .cleared = 0x3668c80,
     .result = 0x3668c94},
    {.offset = 448,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2479,
     .initial = 0x1304d30f,
     .mask = 0xc0ffffff,
     .selected = 0x1000000,
     .cleared = 0x4d30f,
     .result = 0x104d30f},
    {.offset = 462,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2462,
     .initial = 0x3668c94,
     .mask = 0xffffc07f,
     .selected = 0xa80,
     .cleared = 0x3668014,
     .result = 0x3668a94},
    {.offset = 474,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2478,
     .initial = 0xe041000,
     .mask = 0xc0ffffff,
     .selected = 0x1000000,
     .cleared = 0x41000,
     .result = 0x1041000},
    {.offset = 488,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2461,
     .initial = 0x4e84b96,
     .mask = 0xffffff80,
     .selected = 0x18,
     .cleared = 0x4e84b80,
     .result = 0x4e84b98},
    {.offset = 500,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2479,
     .initial = 0x104d30f,
     .mask = 0xffffffc0,
     .selected = 0x1,
     .cleared = 0x104d300,
     .result = 0x104d301},
    {.offset = 512,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2461,
     .initial = 0x4e84b98,
     .mask = 0xffffc07f,
     .selected = 0xc80,
     .cleared = 0x4e84018,
     .result = 0x4e84c98},
    {.offset = 524,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2430,
     .initial = 0x252ef7bd,
     .mask = 0xffff83ff,
     .selected = 0x7000,
     .cleared = 0x252e83bd,
     .result = 0x252ef3bd},
    {.offset = 536,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2479,
     .initial = 0x104d301,
     .mask = 0xfffff03f,
     .selected = 0x40,
     .cleared = 0x104d001,
     .result = 0x104d041},
    {.offset = 548,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2461,
     .initial = 0x4e84c98,
     .mask = 0xffe03fff,
     .selected = 0xe0000,
     .cleared = 0x4e00c98,
     .result = 0x4ee0c98},
    {.offset = 562,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2450,
     .initial = 0x273ef7bd,
     .mask = 0xffff83ff,
     .selected = 0x7000,
     .cleared = 0x273e83bd,
     .result = 0x273ef3bd},
    {.offset = 574,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2479,
     .initial = 0x104d041,
     .mask = 0xfffc0fff,
     .selected = 0x1000,
     .cleared = 0x1040041,
     .result = 0x1041041},
    {.offset = 587,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2461,
     .initial = 0x4ee0c98,
     .mask = 0xf01fffff,
     .selected = 0x7400000,
     .cleared = 0xe0c98,
     .result = 0x74e0c98},
}};

constexpr std::array<OriginalRmw, 27> data_other_groups{{
    {.offset = 302,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2441,
     .initial = 0xf38b289,
     .mask = 0xfffff03f,
     .selected = 0x140,
     .cleared = 0xf38b009,
     .result = 0xf38b149,
     .verify_pin_checkpoint = true},
    {.offset = 314,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2479,
     .initial = 0x1348d300,
     .mask = 0xff03ffff,
     .selected = 0x0,
     .cleared = 0x1300d300,
     .result = 0x1300d300},
    {.offset = 327,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2440,
     .initial = 0x8144040,
     .mask = 0xfffc0fff,
     .selected = 0x8000,
     .cleared = 0x8140040,
     .result = 0x8148040},
    {.offset = 341,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2479,
     .initial = 0x1300d300,
     .mask = 0xc0ffffff,
     .selected = 0x0,
     .cleared = 0xd300,
     .result = 0xd300},
    {.offset = 354,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2440,
     .initial = 0x8148040,
     .mask = 0xff03ffff,
     .selected = 0x240000,
     .cleared = 0x8008040,
     .result = 0x8248040},
    {.offset = 368,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0x1248041,
     .mask = 0xfffc0fff,
     .selected = 0x0,
     .cleared = 0x1240041,
     .result = 0x1240041},
    {.offset = 381,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2430,
     .initial = 0x252630c2,
     .mask = 0xffffffe0,
     .selected = 0x2,
     .cleared = 0x252630c0,
     .result = 0x252630c2},
    {.offset = 393,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0x1240041,
     .mask = 0xfffc0fff,
     .selected = 0x0,
     .cleared = 0x1240041,
     .result = 0x1240041},
    {.offset = 406,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2430,
     .initial = 0x252630c2,
     .mask = 0xffff83ff,
     .selected = 0x800,
     .cleared = 0x252600c2,
     .result = 0x252608c2},
    {.offset = 418,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0x1240041,
     .mask = 0xfffc0fff,
     .selected = 0x0,
     .cleared = 0x1240041,
     .result = 0x1240041},
    {.offset = 431,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2430,
     .initial = 0x252608c2,
     .mask = 0xfffffc1f,
     .selected = 0x40,
     .cleared = 0x25260802,
     .result = 0x25260842},
    {.offset = 443,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0x1240041,
     .mask = 0xfffc0fff,
     .selected = 0x0,
     .cleared = 0x1240041,
     .result = 0x1240041},
    {.offset = 456,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2430,
     .initial = 0x25260842,
     .mask = 0xfff07fff,
     .selected = 0x10000,
     .cleared = 0x25200842,
     .result = 0x25210842},
    {.offset = 470,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0x1240041,
     .mask = 0xff03ffff,
     .selected = 0x0,
     .cleared = 0x1000041,
     .result = 0x1000041},
    {.offset = 483,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2450,
     .initial = 0x2736b4e3,
     .mask = 0xffffffe0,
     .selected = 0x3,
     .cleared = 0x2736b4e0,
     .result = 0x2736b4e3},
    {.offset = 495,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0x1000041,
     .mask = 0xff03ffff,
     .selected = 0x0,
     .cleared = 0x1000041,
     .result = 0x1000041},
    {.offset = 508,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2450,
     .initial = 0x2736b4e3,
     .mask = 0xffff83ff,
     .selected = 0xc00,
     .cleared = 0x273680e3,
     .result = 0x27368ce3},
    {.offset = 520,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0x1000041,
     .mask = 0xff03ffff,
     .selected = 0x0,
     .cleared = 0x1000041,
     .result = 0x1000041},
    {.offset = 533,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2450,
     .initial = 0x27368ce3,
     .mask = 0xfffffc1f,
     .selected = 0x60,
     .cleared = 0x27368c03,
     .result = 0x27368c63},
    {.offset = 545,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0x1000041,
     .mask = 0xff03ffff,
     .selected = 0x0,
     .cleared = 0x1000041,
     .result = 0x1000041},
    {.offset = 558,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2450,
     .initial = 0x27368c63,
     .mask = 0xfff07fff,
     .selected = 0x18000,
     .cleared = 0x27300c63,
     .result = 0x27318c63},
    {.offset = 572,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0x1000041,
     .mask = 0xfffc0fff,
     .selected = 0x0,
     .cleared = 0x1000041,
     .result = 0x1000041},
    {.offset = 585,
     .widths = {3, 2, 1, 2, 1, 3},
     .address = 0x2479,
     .initial = 0xd300,
     .mask = 0xfffff03f,
     .selected = 0x40,
     .cleared = 0xd000,
     .result = 0xd040},
    {.offset = 597,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2461,
     .initial = 0x4e84b98,
     .mask = 0xffe03fff,
     .selected = 0x8000,
     .cleared = 0x4e00b98,
     .result = 0x4e08b98},
    {.offset = 611,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2478,
     .initial = 0x1000041,
     .mask = 0xff03ffff,
     .selected = 0x0,
     .cleared = 0x1000041,
     .result = 0x1000041},
    {.offset = 624,
     .widths = {3, 3, 1, 2, 1, 3},
     .address = 0x2479,
     .initial = 0xd040,
     .mask = 0xfffc0fff,
     .selected = 0x1000,
     .cleared = 0x40,
     .result = 0x1040},
    {.offset = 637,
     .widths = {3, 3, 1, 3, 1, 3},
     .address = 0x2461,
     .initial = 0x4e08b98,
     .mask = 0xf01fffff,
     .selected = 0x600000,
     .cleared = 0x8b98,
     .result = 0x608b98},
}};

void check_post_data_outputs(HeaderDevices devices) {
  const bool clocks = uses_clock_initializer(devices.placement);
  const auto pins = devices.pin_routes->snapshot();
  const auto enables = devices.dai->snapshot();
  const auto frames = devices.frame_routes->snapshot();
  const auto timing = devices.clock_routes->snapshot();
  constexpr std::array<std::uint32_t, 3> clock_pins{0x076e4a94, 0x074e0c98, 0x03668a94};
  constexpr std::array<std::uint32_t, 3> other_pins{0x04c80a94, 0x00608b98, 0x03668c98};
  const auto expected = clocks ? clock_pins : other_pins;
  for (unsigned int index = 0; index < expected.size(); ++index) {
    require(!pins.pending[index] && pins.routes[index] == expected[index]);
  }
  constexpr std::uint32_t clock_enable0 = 0x01041000;
  constexpr std::uint32_t other_enable0 = 0x01000041;
  constexpr std::uint32_t clock_enable1 = 0x01041041;
  constexpr std::uint32_t other_enable1 = 0x00001040;
  constexpr std::uint32_t clock_frame = 0x273ef3bd;
  constexpr std::uint32_t other_frame = 0x27318c63;
  constexpr std::uint32_t clock_clock = 0x252ef3bd;
  constexpr std::uint32_t other_clock = 0x25210842;
  require(!enables.pending[0] && !enables.pending[1] &&
          enables.routes[0] == (clocks ? clock_enable0 : other_enable0) &&
          enables.routes[1] == (clocks ? clock_enable1 : other_enable1));
  require(!frames.pending[0] && frames.routes[0] == (clocks ? clock_frame : other_frame));
  require(!timing.pending[0] && timing.routes[0] == (clocks ? clock_clock : other_clock));
}

void original_pin_boundary(lead_a1::SharcMemory &memory,
                           lead_a1::Sharc &cpu,
                           lead_a1::SharcFetch &fetch,
                           SerialInput &serial,
                           HeaderDevices devices) {
  const bool clocks = uses_clock_initializer(devices.placement);
  const auto pins = clocks ? std::span<const OriginalRmw>(pin_clock_groups)
                           : std::span<const OriginalRmw>(pin_other_groups);
  for (const auto &group : pins) {
    original_post_spi_group(memory, cpu, fetch, serial, devices, group);
  }
  const auto data = clocks ? std::span<const OriginalRmw>(data_clock_groups)
                           : std::span<const OriginalRmw>(data_other_groups);
  for (const auto &group : data) {
    original_post_spi_group(memory, cpu, fetch, serial, devices, group);
  }
  original_sport_boundary(memory, cpu, fetch, serial, devices);
  check_post_data_outputs(devices);
  const auto routed = devices.data_routes->snapshot();
  constexpr std::uint32_t first_clock_data = 0x08040040;
  constexpr std::uint32_t first_other_data = 0x08248040;
  constexpr std::uint32_t second_clock_data = 0x0f38b289;
  constexpr std::uint32_t second_other_data = 0x0f38b149;
  require(!routed.pending[0] && !routed.pending[1] &&
          routed.routes[0] == (clocks ? first_clock_data : first_other_data) &&
          routed.routes[1] == (clocks ? second_clock_data : second_other_data));
  constexpr unsigned int earlier_clock = 128;
  constexpr unsigned int earlier_other = 104;
  const auto instructions = (clocks ? earlier_clock : earlier_other) +
                            (6 * (pins.size() + data.size())) + (clocks ? 80U : 129U);
  std::cout << "  Clocked application:" << instructions
            << " actual instructions; SPORT initialization; "
            << (clocks ? "PCG_PW1 write" : "IDP_CTL1 DMA write")
            << " remains unimplemented; no serial source clock or audio\n";
}

void original_post_spi_routing(lead_a1::SharcMemory &memory,
                               lead_a1::Sharc &cpu,
                               lead_a1::SharcFetch &fetch,
                               SerialInput &serial,
                               HeaderDevices devices) {
  // Independent static fields + HWR B-14/15 reset words. Compact signed masks
  // remain conditional on the Type17b model assumption, as in the earlier RMWs.
  constexpr std::array<OriginalRmw, 5> clock_groups{{{.offset = 216,
                                                      .widths = {3, 2, 1, 2, 1, 3},
                                                      .address = 0x2430,
                                                      .initial = 0x252630c2,
                                                      .mask = 0xfffffc1f,
                                                      .selected = 0x3a0,
                                                      .cleared = 0x25263002,
                                                      .result = 0x252633a2},
                                                     {.offset = 228,
                                                      .widths = {3, 3, 1, 3, 1, 3},
                                                      .address = 0x2430,
                                                      .initial = 0x252633a2,
                                                      .mask = 0xfff07fff,
                                                      .selected = 0xe8000,
                                                      .cleared = 0x252033a2,
                                                      .result = 0x252eb3a2},
                                                     {.offset = 242,
                                                      .widths = {3, 2, 1, 2, 1, 3},
                                                      .address = 0x2430,
                                                      .initial = 0x252eb3a2,
                                                      .mask = 0xffffffe0,
                                                      .selected = 0x1d,
                                                      .cleared = 0x252eb3a0,
                                                      .result = 0x252eb3bd},
                                                     {.offset = 254,
                                                      .widths = {3, 2, 1, 2, 1, 3},
                                                      .address = 0x2430,
                                                      .initial = 0x252eb3bd,
                                                      .mask = 0xffff83ff,
                                                      .selected = 0x7400,
                                                      .cleared = 0x252e83bd,
                                                      .result = 0x252ef7bd},
                                                     {.offset = 266,
                                                      .widths = {3, 3, 1, 2, 1, 3},
                                                      .address = 0x2478,
                                                      .initial = 0x0e2482ca,
                                                      .mask = 0xfffc0fff,
                                                      .selected = 0x1000,
                                                      .cleared = 0x0e2402ca,
                                                      .result = 0x0e2412ca}}};
  constexpr OriginalRmw enable_group{.offset = 216,
                                     .widths = {3, 2, 1, 2, 1, 3},
                                     .address = 0x2478,
                                     .initial = 0x0e2482ca,
                                     .mask = 0xffffffc0,
                                     .selected = 1,
                                     .cleared = 0x0e2482c0,
                                     .result = 0x0e2482c1};
  const bool clocks = uses_clock_initializer(devices.placement);
  if (clocks) {
    for (const auto &group : clock_groups) {
      original_post_spi_group(memory, cpu, fetch, serial, devices, group);
    }
  } else {
    original_post_spi_group(memory, cpu, fetch, serial, devices, enable_group);
  }
  original_pin_boundary(memory, cpu, fetch, serial, devices);
  constexpr unsigned int earlier_instructions = 95;
  constexpr unsigned int spi_instructions = 3;
  const auto instructions =
      earlier_instructions + spi_instructions + (6 * (clocks ? clock_groups.size() : 1U));
  std::cout << "  Earlier SPI checkpoint:" << instructions
            << " actual instructions through SPI and initial SRU groups\n";
}

void original_application_spi_boundary(lead_a1::SharcMemory &memory,
                                       lead_a1::Sharc &cpu,
                                       lead_a1::SharcFetch &fetch,
                                       SerialInput &serial,
                                       HeaderDevices devices) {
  constexpr std::array<unsigned int, 3> offsets{207, 210, 213};
  constexpr std::uint32_t spi_dma = 0x1084;
  constexpr std::uint32_t spi_control = 0x1000;
  constexpr std::uint32_t configured = 0x4d14;
  const auto before = devices.spi.state();
  for (unsigned int index = 0; index < offsets.size(); ++index) {
    original_application_prepare(memory, cpu, fetch, serial, devices);
    const auto access = fetch.peripheral_access(cpu);
    require(access.has_value() == (index != 1));
    const auto pc = devices.placement.second_call + offsets[index];
    if (access) {
      require(access->pc == pc && access->operation == lead_a1::AccessKind::write &&
              access->address == (index == 0 ? spi_dma : spi_control));
    }
    const auto step = original_application_retire(memory, cpu, fetch, serial, devices);
    require(step.pc == pc && step.address_units == 3);
    const auto value = index == 0 ? 0U : configured;
    require(cpu.state().system[0] == (lead_a1::SharcKnown32{.value = value, .known = full32}));
    if (access) {
      check_posted_application(devices, {.pc = pc, .address = access->address, .value = value});
    }
    if (index == 0) { require(devices.spi.state() == before); }
  }
  original_post_spi_routing(memory, cpu, fetch, serial, devices);
}

void original_application_dpi(lead_a1::SharcMemory &memory,
                              lead_a1::Sharc &cpu,
                              lead_a1::SharcFetch &fetch,
                              SerialInput &serial,
                              HeaderDevices devices) {
  require(devices.bridge != nullptr && devices.dpi_pin != nullptr && devices.dpi_enable != nullptr);
  // Static original fields: PGR12-12/Type17b; signed extension is a model
  // assumption still needing an independent rule (sharc-visa-execution.txt).
  // R1=-4033, R0&=R1, R1=1344/640, R0|=R1. These restore field6..11.
  constexpr std::uint32_t mask = 0xfffff03f;
  constexpr std::uint32_t pin_select = 0x540;
  constexpr std::uint32_t enable_select = 0x280;
  constexpr std::uint32_t pin_reset = 0x18017556;
  constexpr std::uint32_t enable_reset = 0x0d00c28b;
  constexpr std::uint32_t pin_cleared = 0x18017016;
  constexpr std::uint32_t enable_cleared = 0x0d00c00b;
  constexpr std::array<unsigned int, 13> offsets{
      181, 184, 186, 187, 189, 190, 193, 196, 198, 199, 201, 202, 205};
  constexpr std::array<unsigned int, 13> widths{3, 2, 1, 2, 1, 3, 3, 2, 1, 2, 1, 3, 2};
  constexpr std::array<std::uint32_t, 13> r0{pin_reset,
                                             pin_reset,
                                             pin_cleared,
                                             pin_cleared,
                                             pin_reset,
                                             pin_reset,
                                             enable_reset,
                                             enable_reset,
                                             enable_cleared,
                                             enable_cleared,
                                             enable_reset,
                                             enable_reset,
                                             enable_reset};
  constexpr std::array<std::uint32_t, 13> r1{0,
                                             mask,
                                             mask,
                                             pin_select,
                                             pin_select,
                                             pin_select,
                                             pin_select,
                                             mask,
                                             mask,
                                             enable_select,
                                             enable_select,
                                             enable_select,
                                             enable_select};
  const auto initial_pins = devices.dpi_pin->snapshot();
  const auto initial_enables = devices.dpi_enable->snapshot();
  require(initial_pins.routes[0] == pin_reset && initial_enables.routes[0] == enable_reset);
  for (unsigned int index = 0; index < offsets.size(); ++index) {
    original_application_prepare(memory, cpu, fetch, serial, devices);
    const auto access = fetch.peripheral_access(cpu);
    const bool transfer = index == 0 || index == 5 || index == 6 || index == 11;
    require(access.has_value() == transfer);
    const bool write = index == 5 || index == 11;
    constexpr std::uint32_t pin_word = 0x1c10;
    constexpr std::uint32_t enable_word = 0x1c20;
    if (access) {
      require(access->address == (index < 6 ? pin_word : enable_word) &&
              access->operation ==
                  (write ? lead_a1::AccessKind::write : lead_a1::AccessKind::read));
    }
    const auto step = original_application_retire(memory, cpu, fetch, serial, devices);
    require(step.pc == devices.placement.second_call + offsets[index] &&
            step.address_units == widths[index]);
    constexpr std::uint64_t full40 = 0xffffffffff;
    require(cpu.state().data[0][0] ==
            (lead_a1::SharcKnown40{.value = std::uint64_t{r0[index]} << 8, .known = full40}));
    if (index != 0) {
      require(cpu.state().data[0][1] ==
              (lead_a1::SharcKnown40{.value = std::uint64_t{r1[index]} << 8, .known = full40}));
    }
    if (write) {
      const auto queue = devices.bridge->snapshot();
      const auto posted = queue.writes[0];
      require(queue.write_count == 1 && posted && access && posted->value == r0[index] &&
              posted->intent.pc == step.pc && posted->intent.address == access->address);
    }
    if (index == 0) {
      constexpr unsigned int first_read_clock = 217;
      require(devices.bridge->state().clocks - devices.iop_epoch == first_read_clock);
      check_original_power_outputs(cpu, devices);
      check_original_timing_routes(devices);
      check_original_pdap(devices);
    }
  }
  constexpr unsigned int elapsed = 95 + 6 + (13 * 7) + (12 * 4);
  require(devices.bridge->state().clocks - devices.iop_epoch == elapsed &&
          devices.dpi_pin->snapshot() == initial_pins &&
          devices.dpi_enable->snapshot() == initial_enables);
  original_application_spi_boundary(memory, cpu, fetch, serial, devices);
  std::cout << "  Earlier clocked checkpoint:95 instructions/240 local CCLK through SRU2; "
               "explicit5/8 IOP timing\n";
}

void original_clocked_application(lead_a1::SharcMemory &memory,
                                  lead_a1::Sharc &cpu,
                                  lead_a1::SharcFetch &fetch,
                                  SerialInput &serial,
                                  HeaderDevices devices) {
  require(devices.bridge != nullptr);
  const auto initial_bus = devices.bridge->snapshot();
  require(initial_bus.write_count == 0 && !initial_bus.request && !initial_bus.read);
  devices.iop_epoch = initial_bus.clocks;
  constexpr unsigned int power_instructions = 18;
  constexpr unsigned int instructions = 82;
  for (unsigned int index = 0; index < instructions; ++index) {
    original_application_prepare(memory, cpu, fetch, serial, devices);
    const auto step = original_application_retire(memory, cpu, fetch, serial, devices);
    if (index < power_instructions) {
      require(step.pc == original_power_pc(devices.placement, index) &&
              step.address_units == (index < 6 ? 3U : 2U));
    }
    if (index + 1 == power_instructions) { check_original_power_outputs(cpu, devices); }
  }
  constexpr unsigned int prefix_words = 181;
  require(cpu.state().pc == devices.placement.second_call + prefix_words);
  original_application_dpi(memory, cpu, fetch, serial, devices);
}

lead_a1::SharcState original_visa_instruction_reference(lead_a1::SharcMemory &memory,
                                                        const lead_a1::Sharc &checkpoint,
                                                        const lead_a1::SharcFetch &fetch,
                                                        HeaderDevices devices) {
  const auto native = checkpoint.state();
  const auto transport = fetch.state();
  const auto controller = devices.control.snapshot();
  // Independent instruction-order fork copies the verified CPU checkpoint.
  // Its CPU checkpoint precedes the separately exercised clocked transition.
  lead_a1::Sharc cpu(memory, native, &devices.control);
  require(cpu.state() == native);
  std::optional<lead_a1::SharcVisaBuffer> buffer;
  VisaReferenceTrace trace;
  constexpr unsigned int limit = 50000;
  const auto return_pc = devices.placement.visa_entry + 3;
  while (trace.instructions < limit && cpu.state().pc != return_pc) {
    devices.clock.accessed = false;
    record_visa_reference(visa_reference_step(memory, cpu, buffer), trace, devices.clock);
  }
  constexpr unsigned int expected_instructions = 8286;
  constexpr std::array<unsigned int, 6> expected_waits{16, 4096, 16, 4096, 16, 16};
  require(trace.instructions == expected_instructions && trace.native == 1 && trace.compact == 1 &&
          trace.clock_accesses == 11 && trace.waits == expected_waits &&
          cpu.state().pc == return_pc);
  constexpr std::uint32_t call_frame = 0x1000000;
  require(cpu.state().depth == 0 && cpu.state().pc_stack[0] == (call_frame | return_pc) &&
          cpu.state().loops_empty == true && !cpu.state().count_loop &&
          !cpu.state().arithmetic_loop && !cpu.state().branch && !cpu.state().interrupt);
  const auto clock = devices.clock.device.state();
  constexpr std::uint32_t expected_pmctl = 0x40012;
  constexpr std::uint32_t expected_dividers = 0x40000;
  require(clock.control == expected_pmctl && clock.effective_dividers == expected_dividers &&
          !clock.pending_dividers && clock.pll_remaining == 0 && !clock.bypass_required);
  const auto rates = devices.clock.device.rates();
  constexpr unsigned int core_ratio = 18;
  constexpr unsigned int external_numerator = 36;
  require(rates.core == (lead_a1::SharcClockRatio{.numerator = core_ratio}) &&
          rates.peripheral == (lead_a1::SharcClockRatio{.numerator = 9}) &&
          rates.external ==
              (lead_a1::SharcClockRatio{.numerator = external_numerator, .denominator = 5}));
  require(checkpoint.state() == native && fetch.state() == transport &&
          devices.control.snapshot() == controller);
  const auto common = cpu.state();
  std::cout << "  Separate VISA instruction-order fork:8286 actual instructions, six original "
               "wait loops/11 PMCTL accesses, first CALL returned\n";
  if (!buffer) { throw std::runtime_error("Missing original VISA clock-prefix input"); }
  original_visa_power_reference(memory, cpu, *buffer, devices);
  require(checkpoint.state() == native && fetch.state() == transport &&
          devices.control.snapshot() == controller);
  return common;
}

template <typename Known, std::size_t size>
void changed_known(std::string_view name,
                   const std::array<Known, size> &actual,
                   const std::array<Known, size> &expected) {
  for (std::size_t index = 0; index < size; ++index) {
    if (actual[index] == expected[index]) { continue; }
    std::cerr << name << '[' << std::dec << index << "] actual=0x" << std::hex
              << actual[index].value << "/0x" << actual[index].known << " expected=0x"
              << expected[index].value << "/0x" << expected[index].known << '\n';
  }
}

void compare_common_state(const lead_a1::SharcState &actual, const lead_a1::SharcState &expected) {
  if (actual == expected) { return; }
  std::cerr << "Common CPU state mismatch; equal fields: pc=" << (actual.pc == expected.pc)
            << " pending=" << (actual.pending == expected.pending)
            << " flags=" << (actual.flag_inputs == expected.flag_inputs)
            << " dag=" << (actual.dag == expected.dag) << " px=" << (actual.px == expected.px)
            << " stack=" << (actual.pc_stack == expected.pc_stack)
            << " depth=" << (actual.depth == expected.depth)
            << " loop_empty=" << (actual.loops_empty == expected.loops_empty)
            << " branch=" << (actual.branch == expected.branch)
            << " count=" << (actual.count_loop == expected.count_loop)
            << " arithmetic=" << (actual.arithmetic_loop == expected.arithmetic_loop)
            << " cache=" << (actual.cache == expected.cache)
            << " interrupt=" << (actual.interrupt == expected.interrupt) << " idle="
            << (actual.idle_pc == expected.idle_pc &&
                actual.idle_delay_word == expected.idle_delay_word)
            << '\n';
  changed_known("system", actual.system, expected.system);
  changed_known("effective", actual.effective, expected.effective);
  for (std::size_t bank = 0; bank < actual.data.size(); ++bank) {
    if (actual.data[bank] == expected.data[bank]) { continue; }
    std::cerr << "Data bank " << bank << '\n';
    changed_known("R", actual.data[bank], expected.data[bank]);
  }
  require(actual == expected);
}

void sequenced_deferred_foreground(lead_a1::SharcMemory &memory,
                                   lead_a1::Sharc &cpu,
                                   lead_a1::SharcFetch &fetch,
                                   SerialInput &serial,
                                   HeaderDevices devices) {
  const auto original_end =
      memory.read({.address = self_modify, .width = lead_a1::SharcWidth::wide48});
  const auto patch = cpu.state().data[0][11].value << 8;
  const auto saved_px = cpu.state().px;
  unsigned int foreground{};
  unsigned int partial_stalls{};
  bool issued{};
  constexpr unsigned int limit = 32;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != self_modify + 1; ++clock) {
    const bool requested = fetch.address_pm_request(cpu);
    const bool defer = requested && !issued;
    if (defer) { require(fetch.state().fetch1 == self_modify && !fetch.state().lookup); }
    const auto bus =
        requested ? lead_a1::SharcFetchBus::pm_data : lead_a1::SharcFetchBus::available;
    const auto read =
        defer ? lead_a1::SharcFetchRead::deferred : lead_a1::SharcFetchRead::immediate;
    const auto event = fetch.advance_sequenced(cpu, bus, read);
    if (defer) {
      issued = true;
      require(fetch.state().pending_fetch2 == self_modify && !fetch.state().fetch2);
    }
    if (event.stalled) { ++partial_stalls; }
    retire_deferred_foreground(cpu, fetch, event.stalled, original_end, patch, foreground);
    complete_foreground_response(memory, cpu, fetch, patch, devices.control, foreground);
    advance_devices(memory, cpu, serial, devices);
  }
  require(issued && foreground == 4 && partial_stalls == 2 && cpu.state().pc == self_modify + 1);
  require(cpu.state().depth == 0 && cpu.state().loops_empty == true &&
          !cpu.state().arithmetic_loop && !cpu.state().interrupt && !cpu.state().branch &&
          !fetch.state().pending_fetch2);
  require(cpu.state().px == saved_px && cpu.state().dag[0][0][8].value == saved_vector);
  require(memory.read({.address = saved_vector, .width = lead_a1::SharcWidth::wide48}) ==
          (saved_px.value >> 16));
  const auto controller = devices.control.snapshot();
  require(controller.control == 4 && controller.effective == 4 && !controller.delayed);
  std::cout << "  Deferred F2 input:4 original foreground instructions, saved SPI vector restored, "
               "PC0x8c005; response phase is not hardware-qualified\n";
  settle_original_visa_fetch(memory, cpu, fetch, serial, devices);
  const lead_a1::Sharc checkpoint(memory, cpu.state(), &devices.control);
  const lead_a1::SharcFetch checkpoint_fetch(memory, fetch.state());
  check_original_visa_boundary(memory, cpu, fetch, serial, devices);
  check_original_clocked_visa_common(memory, cpu, fetch, serial, devices);
  if (devices.profile == OriginalProfile::clocked_iop) {
    original_clocked_application(memory, cpu, fetch, serial, devices);
  }
  if (devices.profile == OriginalProfile::visa_reference) {
    const auto clocked = cpu.state();
    const auto common =
        original_visa_instruction_reference(memory, checkpoint, checkpoint_fetch, devices);
    require(cpu.state() == clocked);
    compare_common_state(clocked, common);
  }
}

void sequenced_arithmetic_entry(lead_a1::SharcMemory &memory,
                                lead_a1::Sharc &cpu,
                                lead_a1::SharcFetch &fetch,
                                SerialInput &serial,
                                HeaderDevices devices) {
  devices.clock.accessed = false;
  static_cast<void>(fetch.advance_sequenced(cpu));
  require(cpu.step(fetch).minimum_core_cycles == 1);
  advance_devices(memory, cpu, serial, devices);
  static_cast<void>(fetch.advance_sequenced(cpu));
  const auto idle = cpu.step(fetch);
  require(idle.pc == frontier + 1 && cpu.state().idle_pc == frontier + 1 &&
          cpu.state().pc == frontier + 2 && cpu.state().idle_delay_word);
  advance_devices(memory, cpu, serial, devices);
  sequenced_vector_receive(memory, cpu, fetch, serial, devices);
  sequenced_loop_interrupt(memory, cpu, fetch, serial, devices);
  if (devices.profile != OriginalProfile::immediate) {
    sequenced_deferred_foreground(memory, cpu, fetch, serial, devices);
    return;
  }
  constexpr std::uint32_t loop_start = 0x8c003;
  constexpr std::uint64_t full40 = 0xffffffffff;
  const auto patch_register = cpu.state().data[0][11];
  require(patch_register.known == full40);
  const auto patch = patch_register.value << 8;
  const auto original_end =
      memory.read({.address = self_modify, .width = lead_a1::SharcWidth::wide48});
  const auto vector = memory.read({.address = saved_vector, .width = lead_a1::SharcWidth::wide48});
  const auto saved_px = cpu.state().px;
  constexpr unsigned int limit = 32;
  unsigned int foreground_retired{};
  for (unsigned int clock = 0; clock < limit; ++clock) {
    const auto before = cpu.state();
    const auto transport = fetch.state();
    const auto control = devices.control.snapshot();
    const bool requested = fetch.address_pm_request(cpu);
    const auto bus =
        requested ? lead_a1::SharcFetchBus::pm_data : lead_a1::SharcFetchBus::available;
    lead_a1::SharcFetchEvent event;
    try {
      event = fetch.advance_sequenced(cpu, bus);
    } catch (const std::exception &error) {
      require(std::string(error.what())
                  .find("quiet SYSCTL Decode/Address width interlock not qualified") !=
              std::string::npos);
      const auto word = transport.fetch2;
      require(foreground_retired == 2 && !requested && before.pc == loop_start &&
              transport.fetch1 == loop_start && word && word->address == self_modify &&
              word->opcode == original_end && transport.decode &&
              transport.decode->address == loop_start && transport.decode->opcode == 0 &&
              !transport.address && !transport.execute);
      constexpr std::uint32_t widened = 0x204;
      require(control.control == 4 && control.effective == widened && control.delayed == 4);
      require(memory.read({.address = self_modify, .width = lead_a1::SharcWidth::wide48}) == patch);
      require(memory.read({.address = saved_vector, .width = lead_a1::SharcWidth::wide48}) ==
                  vector &&
              before.px == saved_px && before.loops_empty == false && before.depth == 1);
      require(cpu.state() == before && fetch.state() == transport &&
              devices.control.snapshot() == control && !memory.active());
      std::cout << "  Sequenced foreground: actual NOP and dual PM/SYSCTL store retired; RAM patch "
                   "matched, next quiet SYSCTL width boundary at PC0x8c003 preserves state\n";
      return;
    }
    if (fetch.state().execute) {
      const auto step = cpu.step(fetch);
      require(foreground_retired < 2 && step.pc == loop_start + foreground_retired &&
              step.minimum_core_cycles == 1 && event.stalled == (foreground_retired == 1));
      ++foreground_retired;
    }
    advance_devices(memory, cpu, serial, devices);
  }
  throw std::runtime_error(
      "Original arithmetic foreground probe budget exhausted; inspect boundary");
}

void sequenced_finalizer(lead_a1::SharcMemory &memory,
                         lead_a1::Sharc &cpu,
                         lead_a1::SharcFetch &fetch,
                         SerialInput &serial,
                         HeaderDevices devices,
                         std::size_t terminal) {
  constexpr unsigned int limit = 4096;
  unsigned int clocks{};
  while (cpu.state().pc != frontier && clocks < limit) {
    devices.clock.accessed = false;
    const auto bus = fetch.address_pm_request(cpu) ? lead_a1::SharcFetchBus::pm_data
                                                   : lead_a1::SharcFetchBus::available;
    static_cast<void>(fetch.advance_sequenced(cpu, bus));
    ++clocks;
    auto retired_clock = RetireClock::pending;
    if (fetch.state().execute) {
      retired_clock = retire_loader_instruction(memory, cpu, fetch, serial, devices);
    }
    finish_loader_clock(memory, cpu, serial, devices, retired_clock);
  }
  require(cpu.state().pc == frontier && cpu.state().depth == 1 && cpu.state().loops_empty == false);
  require(!cpu.state().count_loop && !cpu.state().branch && !memory.active());
  const auto pending = devices.profile == OriginalProfile::clocked_iop
                           ? std::optional{devices.bridge->snapshot()}
                           : std::nullopt;
  check_frontier(cpu, devices.control, devices.spi, serial, terminal, pending);
  const auto address = fetch.state().address;
  const auto flow = fetch.state().flow;
  constexpr std::uint32_t loop_start = 0x8c003;
  require(address && address->address == frontier && address->branch && address->branch->taken &&
          address->branch->target == loop_start && flow && flow->next == loop_start);
  std::cout << "  Sequenced finalizer setup: " << clocks << " grants, next PC0x8c08e\n";
  sequenced_arithmetic_entry(memory, cpu, fetch, serial, devices);
}

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
      auto retired_clock = RetireClock::pending;
      if (fetch.state().execute) {
        retired_clock = retire_loader_instruction(memory, cpu, fetch, serial, devices);
      }
      finish_loader_clock(memory, cpu, serial, devices, retired_clock);
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
          sequenced_finalizer(memory, cpu, fetch, serial, devices, cursor);
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
    auto retired_clock = RetireClock::pending;
    if (executing) {
      const auto pc = executing->address;
      retired_clock = retire_loader_instruction(memory, cpu, fetch, serial, devices);
      if (pc == setup) {
        const auto loop = cpu.state().count_loop;
        require(loop && loop->total == count / 2);
        ++setups;
      }
      if (pc == reader_call) { ++calls; }
    }
    finish_loader_clock(memory, cpu, serial, devices, retired_clock);
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
    auto retired_clock = RetireClock::pending;
    if (fetch.state().execute) {
      retired_clock = retire_loader_instruction(memory, cpu, fetch, serial, devices);
      ++instructions;
    }
    finish_loader_clock(memory, cpu, serial, devices, retired_clock);
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

struct OriginalRouting {
  lead_a1::SharcPower power;
  lead_a1::SharcDaiEnable dai{power};
  lead_a1::SharcDaiPin pins{power};
  lead_a1::SharcDaiData data{power};
  lead_a1::SharcDaiTiming clocks{power, lead_a1::SharcDaiTimingGroup::clock};
  lead_a1::SharcDaiTiming frames{power, lead_a1::SharcDaiTimingGroup::frame};
  lead_a1::SharcPdap pdap{power};
  lead_a1::SharcPicr priority;
  lead_a1::SharcDpiRouting dpi_pin{lead_a1::SharcDpiGroup::pin};
  lead_a1::SharcDpiRouting dpi_enable{lead_a1::SharcDpiGroup::enable};
  lead_a1::SharcMemory peripheral;
  std::optional<lead_a1::SharcIopBridge> bridge;
  std::optional<lead_a1::SharcIopPort> sru;
  std::optional<lead_a1::SharcIopPort> dpi;
  std::optional<lead_a1::SharcIopPort> power_port;
  std::optional<lead_a1::SharcSpiPort> spi;
  std::optional<lead_a1::SharcIopPort> spi_core;
  std::optional<lead_a1::SharcIopPort> spi_dma;
  std::optional<lead_a1::SharcIopPort> priority_port;
  std::array<std::optional<lead_a1::SharcSport>, 8> sports;
  std::array<std::optional<lead_a1::SharcSportPair>, 4> sport_pairs;
  std::array<std::optional<lead_a1::SharcIopPort>, 4> sport_ports;

  std::array<lead_a1::SharcSport *, 8> sport_devices() {
    std::array<lead_a1::SharcSport *, 8> result{};
    for (unsigned int index = 0; index < sports.size(); ++index) {
      auto &port = sports[index];
      if (port) { result[index] = &*port; }
    }
    return result;
  }

  void map_sports(lead_a1::SharcMemory &memory, lead_a1::SharcIopBridge &bus) {
    constexpr std::array<std::uint32_t, 4> bases{0xc00, 0x400, 0x800, 0x4800};
    constexpr unsigned int words = 0x68;
    for (unsigned int pair = 0; pair < bases.size(); ++pair) {
      const auto first = pair * 2;
      auto &a = sports[first].emplace(power, memory, first);
      auto &b = sports[first + 1].emplace(power, memory, first + 1);
      auto &view = sport_pairs[pair].emplace(a, b);
      const lead_a1::SharcIoMapping window{.first_word = bases[pair], .words = words};
      peripheral.map_io(window, view);
      memory.map_io(window, sport_ports[pair].emplace(bus, window));
    }
  }

  lead_a1::SharcIopBridge *
  map(lead_a1::SharcMemory &memory, OriginalProfile profile, lead_a1::SharcSpi &serial) {
    if (profile != OriginalProfile::clocked_iop) {
      memory.map_io({.first_word = spi_index, .words = 5}, serial);
    }
    if (profile != OriginalProfile::visa_reference && profile != OriginalProfile::clocked_iop) {
      return nullptr;
    }
    constexpr std::uint32_t first_pben = 0x2478;
    constexpr std::uint32_t first_clock = 0x2430;
    constexpr std::uint32_t first_frame = 0x2450;
    constexpr std::uint32_t idp_control = 0x24b0;
    // Bind only the four implemented control words here. PCG registers occupy
    // holes inside the broader IDP register range and require their own owner.
    constexpr unsigned int idp_span = 4;
    const lead_a1::SharcIoMapping window =
        profile == OriginalProfile::clocked_iop
            ? lead_a1::SharcIoMapping{.first_word = 0x400, .words = 0x4500}
            : lead_a1::SharcIoMapping{.first_word = 0x1c00, .words = 0x900};
    constexpr lead_a1::SharcIoMapping sru_window{.first_word = 0x2400, .words = 0x100};
    constexpr lead_a1::SharcIoMapping dpi_window{.first_word = 0x1c00, .words = 0x100};
    peripheral.map_io({.first_word = first_pben, .words = 4}, dai);
    peripheral.map_io({.first_word = first_clock, .words = 6}, clocks);
    peripheral.map_io({.first_word = first_frame, .words = 5}, frames);
    peripheral.map_io({.first_word = idp_control, .words = idp_span}, pdap);
    auto &bound = bridge.emplace(memory, peripheral, window);
    memory.map_io(sru_window, sru.emplace(bound, sru_window));
    memory.map_io(dpi_window, dpi.emplace(bound, dpi_window));
    if (profile == OriginalProfile::clocked_iop) {
      constexpr lead_a1::SharcIoMapping spi_window{.first_word = 0x1000, .words = 0x85};
      constexpr lead_a1::SharcIoMapping core_window{.first_word = 0x1000, .words = 7};
      constexpr lead_a1::SharcIoMapping dma_window{.first_word = 0x1080, .words = 5};
      peripheral.map_io(spi_window, spi.emplace(serial));
      memory.map_io(core_window, spi_core.emplace(bound, core_window));
      memory.map_io(dma_window, spi_dma.emplace(bound, dma_window));
      constexpr lead_a1::SharcIoMapping power_window{.first_word = 0x2001, .words = 1};
      peripheral.map_io(power_window, power);
      constexpr std::uint32_t dai_pin0 = 0x2460;
      peripheral.map_io({.first_word = dai_pin0, .words = 5}, pins);
      constexpr std::uint32_t dai_data0 = 0x2440;
      peripheral.map_io({.first_word = dai_data0, .words = 7}, data);
      map_sports(memory, bound);
      constexpr lead_a1::SharcIoMapping priority_window{.first_word = 0x2200, .words = 4};
      peripheral.map_io(priority_window, priority);
      memory.map_io(priority_window, priority_port.emplace(bound, priority_window));
      constexpr std::uint32_t dpi_pin0 = 0x1c10;
      constexpr std::uint32_t dpi_pben0 = 0x1c20;
      peripheral.map_io({.first_word = dpi_pin0, .words = 3}, dpi_pin);
      peripheral.map_io({.first_word = dpi_pben0, .words = 3}, dpi_enable);
      memory.map_io(power_window, power_port.emplace(bound, power_window));
    }
    return &bound;
  }
};

void sequenced_clock_setup(std::span<const std::byte> stream,
                           unsigned int interval,
                           const CellExpectations &expected,
                           Placement placement,
                           OriginalProfile profile) {
  // This fixture owns fresh RAM and PMCTL: its writes cannot seed the full stream.
  OriginalRouting routing;
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  auto boot = lead_a1::SharcSpiBoot::slave_reference_214xx();
  bootstrap(memory, boot, stream);
  ObservedClock clock;
  memory.map_io({.first_word = 0x2000, .words = 1}, clock);
  map_original_power(memory, routing.power, profile);
  lead_a1::SharcControl control(true);
  lead_a1::SharcSpi spi(boot.state());
  auto *bus = routing.map(memory, profile, spi);
  memory.map_io({.first_word = sysctl, .words = 1}, control);

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
  sequenced_first_header(memory,
                         cpu,
                         fetch,
                         stream,
                         {.clock = clock,
                          .spi = spi,
                          .control = control,
                          .interval = interval,
                          .profile = profile,
                          .placement = placement,
                          .power = &routing.power,
                          .dai = &routing.dai,
                          .pin_routes = &routing.pins,
                          .data_routes = &routing.data,
                          .clock_routes = &routing.clocks,
                          .frame_routes = &routing.frames,
                          .pdap = &routing.pdap,
                          .priority = &routing.priority,
                          .bridge = bus,
                          .dpi_pin = &routing.dpi_pin,
                          .dpi_enable = &routing.dpi_enable,
                          .sports = routing.sport_devices()});
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

void check_vector_arm(const lead_a1::SharcSpi &spi,
                      const std::optional<lead_a1::SharcIopState> &pending) {
  constexpr unsigned int vector_words = 384;
  if (!pending) {
    require(spi.state().serial.remaining == vector_words);
    return;
  }
  const auto &bus = *pending;
  const auto first = bus.writes[0];
  const auto last = bus.writes[1];
  constexpr std::uint32_t modifier_pc = 0x8c08b;
  constexpr std::uint32_t count_pc = 0x8c08d;
  constexpr std::uint32_t modifier_word = 0x1081;
  constexpr std::uint32_t count_word = 0x1082;
  require(spi.state().serial.remaining == 0 && bus.write_count == 2 && !bus.request && !bus.read &&
          first && last && first->intent.pc == modifier_pc &&
          first->intent.address == modifier_word && first->value == 1 &&
          last->intent.pc == count_pc && last->intent.address == count_word &&
          last->value == vector_words && first->intent.delivery_cycles == 5 &&
          last->intent.delivery_cycles == 5 && last->posted_at == first->posted_at + 2 &&
          bus.clocks == last->posted_at + 1);
}

void check_frontier(const lead_a1::Sharc &cpu,
                    const lead_a1::SharcControl &control,
                    const lead_a1::SharcSpi &spi,
                    SerialInput serial,
                    std::size_t terminal,
                    std::optional<lead_a1::SharcIopState> pending) {
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
  require(spi.state().serial.next_word == vector_destination && spi.state().modifier == 1);
  check_vector_arm(spi, pending);
  require(state.data[0][1].value == std::uint64_t{vector_destination} * 256 &&
          state.data[0][2].value == std::uint64_t{vector_words} * 256);
  const auto saved = (std::uint64_t{word(serial.stream, terminal + 8)} << 32) |
                     (word(serial.stream, terminal + 4) & 0xffff0000);
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
  require(serial.bit >= (terminal + header_bytes) * 8 && serial.bit < serial.stream.size() * 8);
}

void run_stream(std::span<const std::byte> stream,
                Placement placement,
                unsigned int interval,
                OriginalProfile profile) {
  const auto expected = inventory(stream, placement); // Read-only oracle, never writes guest RAM.
  sequenced_clock_setup(stream, interval, expected.cells, placement, profile);
  if (profile != OriginalProfile::immediate) {
    return;
  } // The existing instruction-only aggregate remains a separate fixture.
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
  check_frontier(cpu, control, spi, serial, expected.terminal);
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

OriginalProfile original_profile(int argc, char **argv) {
  if (argc == 2) { return OriginalProfile::immediate; }
  if (argc == 3) {
    const std::string option(argv[2]);
    if (option == "--deferred-f2") { return OriginalProfile::deferred_f2; }
    if (option == "--visa-instructions") { return OriginalProfile::visa_reference; }
    if (option == "--clocked-iop") { return OriginalProfile::clocked_iop; }
  }
  throw std::runtime_error("Usage: original-sharc-test decoded-image-directory "
                           "[--deferred-f2 | --visa-instructions | --clocked-iop]");
}

}

int main(int argc, char **argv) {
  try {
    const auto profile = original_profile(argc, argv);
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
          run_stream(stream, placement, interval, profile);
        }
      }
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
