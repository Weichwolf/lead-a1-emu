// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_boot.hpp"
#include "lead_a1/sharc_clock.hpp"
#include "lead_a1/sharc_control.hpp"
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
constexpr std::uint32_t frontier = 0x8c083;
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

struct Inventory {
  std::map<std::uint32_t, std::uint16_t> cells;
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
  require(state.pc == frontier && state.depth == 0 && state.loops_empty == true &&
          !state.count_loop && !state.branch);
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
  require(!spi.state().overflow && spi.state().serial.remaining == 0);
  require(serial.bit >= (expected.terminal + header_bytes) * 8 &&
          serial.bit < serial.stream.size() * 8);
}

void run_stream(std::span<const std::byte> stream, Placement placement, unsigned int interval) {
  const auto expected = inventory(stream, placement); // Read-only oracle, never writes guest RAM.
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  auto boot = lead_a1::SharcSpiBoot::slave_reference_214xx();
  bootstrap(memory, boot, stream);
  ObservedClock clock;
  lead_a1::SharcControl control(true);
  lead_a1::SharcSpi spi(boot.state());
  memory.map_io({.first_word = 0x2000, .words = 1}, clock);
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
      if (message.find("native48 pc=0x8c083") == std::string::npos ||
          message.find("instruction class not implemented") == std::string::npos) {
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
            << " instructions/serial edge; arithmetic DO frontier\n"
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
