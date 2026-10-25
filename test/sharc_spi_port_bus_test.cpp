// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_iop_bridge.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/peripherals/sharc_spi_port.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t entry = 0xc0000;
constexpr std::uint32_t short_entry = 0x180000;
constexpr std::uint32_t spi_control = 0x1000;
constexpr std::uint32_t dma_control = 0x1084;
constexpr std::uint32_t core_mode = 0x4d14;
constexpr std::uint32_t boot_mode = 0x4d22;
constexpr std::uint32_t boot_end = 0x92180;
constexpr std::uint32_t reply = 0xb9346719;
constexpr std::uint32_t incoming = 0x92120c8b;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr lead_a1::SharcIoMapping window{.first_word = spi_control, .words = 0x85};

struct IopLatency {
  unsigned int write;
  unsigned int read;
};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SPI bus expectation at line " + std::to_string(where.line()));
  }
}

template <typename Action> void rejected(const Action &action, const std::string &expected) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(expected) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SPI bus rejection: " + expected);
}

struct FaultingPort final : lead_a1::MemoryDevice {
  lead_a1::SharcSpiPort &port;
  bool fail_control{};

  explicit FaultingPort(lead_a1::SharcSpiPort &device) : port(device) {}

  std::uint64_t read(lead_a1::MemoryAccess access) override { return port.read(access); }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    port.write(access, value);
    if (access.address == 0 && fail_control) { throw std::runtime_error("owned late SPI fault"); }
  }

  void begin_instruction() override { port.begin_instruction(); }

  void commit_instruction() noexcept override { port.commit_instruction(); }

  void abort_instruction() noexcept override { port.abort_instruction(); }
};

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(entry);
  for (auto &reg : state.data[0]) { reg = {.known = full40}; }
  state.data[0][2] = {.value = std::uint64_t{core_mode} << 8, .known = full40};
  state.data[0][3] = {.value = std::uint64_t{reply} << 8, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = entry};
  return state;
}

struct Fixture {
  lead_a1::SharcSpi spi{{.clock = true, .next_word = boot_end}};
  lead_a1::SharcSpiPort port{spi};
  FaultingPort target{port};
  lead_a1::SharcMemory peripheral;
  lead_a1::SharcMemory memory;
  lead_a1::SharcIopBridge bus{memory, peripheral, window};
  lead_a1::Sharc cpu{memory, initial()};
  lead_a1::SharcFetch fetch{memory, sequenced()};

  Fixture() {
    constexpr unsigned int cells = 256;
    memory.map({.bank = 2, .halfwords = cells, .name = "owned SPI bus program"});
    for (unsigned int index = 0; index < cells; ++index) {
      memory.write({.address = short_entry + index, .width = lead_a1::SharcWidth::short16}, 0);
    }
    memory.map_io(window, bus);
    peripheral.map_io(window, target);
    // Owned native instructions, not extracted firmware: DMA off, SPI mode,
    // source change, control read, TX buffer write, RX buffer read.
    constexpr std::uint64_t stop_dma = 0x110000001084;
    constexpr std::uint64_t configure = 0x110200001000;
    constexpr std::uint64_t clear_source = 0x0f0200000000;
    constexpr std::uint64_t read_control = 0x100100001000;
    constexpr std::uint64_t send = 0x110300001003;
    constexpr std::uint64_t receive = 0x100400001004;
    memory.write({.address = entry, .width = lead_a1::SharcWidth::wide48}, stop_dma);
    memory.write({.address = entry + 1, .width = lead_a1::SharcWidth::wide48}, configure);
    memory.write({.address = entry + 2, .width = lead_a1::SharcWidth::wide48}, clear_source);
    memory.write({.address = entry + 3, .width = lead_a1::SharcWidth::wide48}, read_control);
    memory.write({.address = entry + 4, .width = lead_a1::SharcWidth::wide48}, send);
    memory.write({.address = entry + 5, .width = lead_a1::SharcWidth::wide48}, receive);
  }

  void prepare() {
    constexpr unsigned int limit = 16;
    for (unsigned int clock = 0; clock < limit; ++clock) {
      static_cast<void>(fetch.advance_sequenced(cpu));
      if (fetch.state().execute) { return; }
      bus.tick();
    }
    throw std::runtime_error("Missing SPI Execute");
  }

  lead_a1::SharcStep transfer(unsigned int latency) {
    prepare();
    const auto access = fetch.peripheral_access(cpu);
    if (!access) { throw std::runtime_error("Missing SPI IOP transfer"); }
    bus.issue({.pc = access->pc,
               .address = access->address,
               .operation = access->operation,
               .delivery_cycles = latency});
    const bool read = access->operation == lead_a1::AccessKind::read;
    const auto held = fetch.state();
    const auto pc = cpu.state().pc;
    while (!bus.ready()) {
      if (!read) { fetch.wait_iop(cpu, bus); }
      bus.tick();
      if (read && !bus.ready()) { fetch.wait_iop(cpu, bus); }
      require(fetch.state() == held && cpu.state().pc == pc);
    }
    const auto step = cpu.step(fetch);
    if (!read) { bus.tick(); }
    return step;
  }

  void clear_source() {
    prepare();
    require(!fetch.peripheral_access(cpu) && cpu.step(fetch).pc == entry + 2);
    bus.tick();
    require(cpu.state().data[0][2].value == 0);
  }
};

void actual_control_and_data(IopLatency latency) {
  Fixture fixture;
  require(fixture.transfer(latency.write).pc == entry && fixture.spi.state().control == 7);
  const auto stop = fixture.bus.snapshot().writes[0];
  require(stop && stop->intent.address == dma_control && stop->value == 0);
  require(fixture.transfer(latency.write).pc == entry + 1);
  fixture.clear_source();
  std::optional<lead_a1::SharcIopWrite> queued;
  for (const auto &write : fixture.bus.snapshot().writes) {
    if (write && write->intent.address == spi_control) { queued = write; }
  }
  require(queued && queued->value == core_mode && fixture.spi.state().port_control == boot_mode);
  require(fixture.transfer(latency.read).pc == entry + 3);
  require(fixture.spi.state().control == 0 && fixture.spi.state().port_control == core_mode);
  require(fixture.cpu.state().data[0][1].value == (std::uint64_t{core_mode} << 8));
  require(fixture.transfer(latency.write).pc == entry + 4 && !fixture.spi.state().transmit);
  while (fixture.bus.state().write_count != 0) { fixture.bus.tick(); }
  require(fixture.spi.state().transmit == reply);
  fixture.spi.observe(fixture.memory, {.selected = true, .clock = true, .mosi = {}});
  constexpr unsigned int word_bits = 32;
  for (unsigned int bit = 0; bit < word_bits; ++bit) {
    fixture.spi.observe(fixture.memory, {.selected = true, .clock = false, .mosi = {}});
    require(fixture.spi.miso() == (((reply >> bit) & 1) != 0));
    fixture.spi.observe(fixture.memory,
                        {.selected = true, .clock = true, .mosi = ((incoming >> bit) & 1) != 0});
  }
  require(fixture.spi.interrupt_request());
  require(fixture.transfer(latency.read).pc == entry + 5);
  require(fixture.cpu.state().data[0][4].value == (std::uint64_t{incoming} << 8));
  require(!fixture.spi.interrupt_request() && !fixture.memory.active());
}

void late_control_failure() {
  Fixture fixture;
  fixture.target.fail_control = true;
  require(fixture.transfer(5).pc == entry);
  require(fixture.transfer(5).pc == entry + 1);
  fixture.clear_source();
  while (fixture.spi.state().control != 0) { fixture.bus.tick(); }
  const auto before = fixture.spi.state();
  const auto cpu = fixture.cpu.state();
  const auto fetch = fixture.fetch.state();
  const auto write = fixture.bus.snapshot().writes[0];
  if (!write) { throw std::runtime_error("Missing actual SPI control post"); }
  require(write->intent.address == spi_control && write->value == core_mode);
  const auto due = write->posted_at + 5;
  while (fixture.bus.state().clocks + 1 < due) { fixture.bus.tick(); }
  rejected([&] { fixture.bus.tick(); }, "owned late SPI fault");
  require(fixture.spi.state() == before && fixture.cpu.state() == cpu &&
          fixture.fetch.state() == fetch && fixture.bus.state().clocks == due);
  rejected([&] { fixture.bus.retry_due(); }, "pc=0xc0001 word-address=0x1000 operation=write");
  fixture.target.fail_control = false;
  fixture.bus.retry_due();
  require(fixture.bus.state().clocks == due && fixture.bus.state().write_count == 0);
  require(fixture.cpu.state() == cpu && fixture.fetch.state() == fetch);
  require(fixture.spi.state().port_control == core_mode && fixture.spi.state().control == 0);
  require(fixture.transfer(8).pc == entry + 3);
}

}

int main() {
  try {
    for (const unsigned int writes : {4U, 5U}) {
      for (const unsigned int reads : {7U, 8U}) {
        actual_control_and_data({.write = writes, .read = reads});
      }
    }
    late_control_failure();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
