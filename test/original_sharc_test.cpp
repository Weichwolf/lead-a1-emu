// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_boot.hpp"
#include "lead_a1/sharc_clock.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_dai_enable.hpp"
#include "lead_a1/sharc_dai_timing.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_iop_bridge.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_pdap.hpp"
#include "lead_a1/sharc_power.hpp"
#include "lead_a1/sharc_spi.hpp"
#include "lead_a1/sharc_visa.hpp"

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
  lead_a1::SharcDaiTiming *clock_routes{};
  lead_a1::SharcDaiTiming *frame_routes{};
  lead_a1::SharcPdap *pdap{};
  lead_a1::SharcIopBridge *bridge{};
};

void check_frontier(const lead_a1::Sharc &cpu,
                    const lead_a1::SharcControl &control,
                    const lead_a1::SharcSpi &spi,
                    SerialInput serial,
                    std::size_t terminal);

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
  while (devices.spi.state().serial.remaining != 0 && ticks < limit) {
    advance_devices(memory, cpu, serial, devices);
    ++ticks;
    if (devices.spi.state().serial.remaining == 0) {
      expected.system[10].value |= p1i;
      expected.effective[10].value |= p1i;
    }
    require(cpu.state() == expected && fetch.state() == held);
  }
  require(ticks < limit && devices.spi.state().serial.remaining == 0 &&
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
  if (devices.spi.state().interrupt_pending) {
    constexpr std::uint32_t primary_spi = 0x1000;
    cpu.latch_interrupt(primary_spi);
    devices.spi.acknowledge_interrupt();
  }
  require(!devices.spi.state().overflow);
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

// A new explicit local CCLK/PCLK epoch after the actual common-call RTS.
// Earlier native/core-IOP and PMCTL timing remains a separately bounded input.
void original_application_clock(lead_a1::SharcMemory &memory,
                                lead_a1::Sharc &cpu,
                                SerialInput &serial,
                                HeaderDevices devices) {
  require(devices.bridge != nullptr && devices.power != nullptr && devices.dai != nullptr &&
          devices.clock_routes != nullptr && devices.frame_routes != nullptr &&
          devices.pdap != nullptr);
  advance_clocked_visa_devices(memory, cpu, serial, devices);
  devices.power->advance_half_pclk(1);
  devices.dai->advance_half_pclk(1);
  devices.clock_routes->advance_half_pclk(1);
  devices.frame_routes->advance_half_pclk(1);
  devices.pdap->advance_cclk(1);
  devices.bridge->tick(); // Deliver new writes after the elapsed device interval.
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

lead_a1::SharcStep original_application_retire(lead_a1::SharcMemory &memory,
                                               lead_a1::Sharc &cpu,
                                               lead_a1::SharcFetch &fetch,
                                               SerialInput &serial,
                                               HeaderDevices devices) {
  require(devices.bridge != nullptr);
  const auto access = fetch.peripheral_access(cpu);
  const bool read = access && access->operation == lead_a1::AccessKind::read;
  if (access) {
    require(devices.bridge->bound_to(memory, access->address));
    devices.bridge->issue({.pc = access->pc,
                           .address = access->address,
                           .operation = access->operation,
                           .delivery_cycles = read ? 8U : 5U});
    const auto held = fetch.state();
    const auto pc = cpu.state().pc;
    while (!devices.bridge->ready()) {
      if (!read) { fetch.wait_iop(cpu, *devices.bridge); }
      original_application_clock(memory, cpu, serial, devices);
      if (read && !devices.bridge->ready()) { fetch.wait_iop(cpu, *devices.bridge); }
      require(fetch.state() == held && cpu.state().pc == pc);
    }
  }
  const auto step = cpu.step(fetch);
  require(step.minimum_core_cycles == 1);
  if (!read) { original_application_clock(memory, cpu, serial, devices); }
  return step;
}

void original_application_fault(lead_a1::SharcMemory &memory,
                                lead_a1::Sharc &cpu,
                                lead_a1::SharcFetch &fetch,
                                SerialInput &serial,
                                HeaderDevices devices) {
  require(devices.bridge != nullptr && devices.dai != nullptr);
  original_application_prepare(memory, cpu, fetch, serial, devices);
  const auto core = cpu.state();
  const auto transport = fetch.state();
  const auto control = devices.control.snapshot();
  const auto spi = devices.spi.state();
  const auto access = fetch.peripheral_access(cpu);
  constexpr unsigned int prefix_words = 181;
  constexpr std::uint32_t sru2_pin0 = 0x1c10;
  require(access && access->pc == devices.placement.second_call + prefix_words &&
          access->address == sru2_pin0 && access->operation == lead_a1::AccessKind::read);
  try {
    static_cast<void>(original_application_retire(memory, cpu, fetch, serial, devices));
  } catch (const std::exception &error) {
    const std::string message(error.what());
    if (message.find("word-address=0x1c10 operation=read") == std::string::npos ||
        message.find("Unmapped SHARC IOP register") == std::string::npos) {
      throw std::runtime_error("Unexpected clocked original IOP frontier: " + message);
    }
    require(cpu.state() == core && fetch.state() == transport &&
            devices.control.snapshot() == control && devices.spi.state() == spi &&
            !memory.active());
    constexpr std::array<std::uint32_t, 4> routes{0x0e2482ca, 0x1348d30f, 0x000005d6, 0};
    const auto dai = devices.dai->snapshot();
    require(dai.routes == routes);
    for (const auto &pending : dai.pending) { require(!pending); }
    const auto bus = devices.bridge->snapshot();
    // 82 retired + one faulting access + six refill clocks + twelve seven-clock
    // read extensions + eleven four-clock drains of preceding posted stores.
    constexpr unsigned int expected_clocks = 82 + 1 + 6 + (12 * 7) + (11 * 4);
    require(bus.clocks == expected_clocks && bus.write_count == 0 && bus.read &&
            !bus.read->response && bus.request && bus.request->pc == core.pc &&
            !devices.bridge->waiting() && !devices.bridge->ready());
    check_original_power_outputs(cpu, devices);
    check_original_timing_routes(devices);
    check_original_pdap(devices);
    std::cout << "  Clocked original application:82 further instructions; posted PMCTL1, ten "
                 "routing groups/PP_CTL; actual SRU2_PIN0 read fault after "
              << bus.clocks << " explicit-phase CCLK; real Execute/IAB/PC retained\n";
    return;
  }
  throw std::runtime_error("Missing clocked original SRU2_PIN0 read fault");
}

void original_clocked_application(lead_a1::SharcMemory &memory,
                                  lead_a1::Sharc &cpu,
                                  lead_a1::SharcFetch &fetch,
                                  SerialInput &serial,
                                  HeaderDevices devices) {
  require(devices.bridge != nullptr && devices.bridge->snapshot() == lead_a1::SharcIopState{});
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
  original_application_fault(memory, cpu, fetch, serial, devices);
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
    if (fetch.state().execute) { require(cpu.step(fetch).minimum_core_cycles == 1); }
    advance_devices(memory, cpu, serial, devices);
  }
  require(cpu.state().pc == frontier && cpu.state().depth == 1 && cpu.state().loops_empty == false);
  require(!cpu.state().count_loop && !cpu.state().branch && !memory.active());
  check_frontier(cpu, devices.control, devices.spi, serial, terminal);
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

struct OriginalRouting {
  lead_a1::SharcPower power;
  lead_a1::SharcDaiEnable dai{power};
  lead_a1::SharcDaiTiming clocks{power, lead_a1::SharcDaiTimingGroup::clock};
  lead_a1::SharcDaiTiming frames{power, lead_a1::SharcDaiTimingGroup::frame};
  lead_a1::SharcPdap pdap{power};
  lead_a1::SharcMemory peripheral;
  std::optional<lead_a1::SharcIopBridge> bridge;
  std::optional<lead_a1::SharcIopPort> sru;
  std::optional<lead_a1::SharcIopPort> dpi;
  std::optional<lead_a1::SharcIopPort> power_port;

  lead_a1::SharcIopBridge *map(lead_a1::SharcMemory &memory, OriginalProfile profile) {
    if (profile != OriginalProfile::visa_reference && profile != OriginalProfile::clocked_iop) {
      return nullptr;
    }
    constexpr std::uint32_t first_pben = 0x2478;
    constexpr std::uint32_t first_clock = 0x2430;
    constexpr std::uint32_t first_frame = 0x2450;
    constexpr std::uint32_t idp_control = 0x24b0;
    constexpr unsigned int idp_span = 0x21;
    constexpr lead_a1::SharcIoMapping window{.first_word = 0x1c00, .words = 0x900};
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
      constexpr lead_a1::SharcIoMapping power_window{.first_word = 0x2001, .words = 1};
      peripheral.map_io(power_window, power);
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
  auto *bus = routing.map(memory, profile);
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
                          .clock_routes = &routing.clocks,
                          .frame_routes = &routing.frames,
                          .pdap = &routing.pdap,
                          .bridge = bus});
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
                    std::size_t terminal) {
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
