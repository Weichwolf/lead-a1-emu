// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_iop_bridge.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_power.hpp"
#include "sharc/peripherals/sharc_sport_dma.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t entry = 0xc0000;
constexpr std::uint32_t data_word = 0xc40;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr lead_a1::SharcIoMapping window{.first_word = 0xc40, .words = 4};
using Kind = lead_a1::AccessKind;
constexpr std::uint32_t original_index = 0x80000;
constexpr std::uint32_t first_index = 0x80020;
constexpr std::uint32_t later_index = 0x80021;
constexpr std::uint32_t modifier_word = 0xffff;

lead_a1::SharcSportDmaState parameters() {
  return {.index = original_index, .modifier = 0, .count = 0, .chain = 0};
}

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SPORT DMA bus expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing SPORT DMA bus rejection: " + reason);
}

struct FaultingData final : lead_a1::MemoryDevice {
  lead_a1::SharcSportDma &router;
  bool fail{};

  explicit FaultingData(lead_a1::SharcSportDma &bound) : router(bound) {}

  std::uint64_t read(lead_a1::MemoryAccess access) override { return router.read(access); }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    router.write(access, value);
    if (fail) { throw std::runtime_error("owned late SPORT DMA delivery fault"); }
  }

  void begin_instruction() override { router.begin_instruction(); }

  void commit_instruction() noexcept override { router.commit_instruction(); }

  void abort_instruction() noexcept override { router.abort_instruction(); }
};

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(entry);
  for (auto &reg : state.data[0]) { reg = {.known = full40}; }
  state.data[0][0] = {.value = std::uint64_t{first_index} << 8, .known = full40};
  state.data[0][2] = {.value = std::uint64_t{modifier_word} << 8, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState transport;
  transport.flow = lead_a1::SharcFetchFlow{.next = entry};
  return transport;
}

struct Fixture {
  lead_a1::SharcPower power;
  lead_a1::SharcMemory memory;
  lead_a1::SharcSportDma data{power, memory, 0, parameters()};
  FaultingData target{data};
  lead_a1::SharcMemory peripheral;
  lead_a1::SharcIopBridge bus{memory, peripheral, window};
  lead_a1::Sharc cpu{memory, initial()};
  lead_a1::SharcFetch fetch{memory, sequenced()};

  Fixture() {
    constexpr unsigned int cells = 256;
    constexpr std::uint32_t short_entry = 0x180000;
    memory.map({.bank = 2, .halfwords = cells, .name = "owned SPORT DMA bus program"});
    for (unsigned int index = 0; index < cells; ++index) {
      memory.write({.address = short_entry + index, .width = lead_a1::SharcWidth::short16}, 0);
    }
    memory.map_io(window, bus);
    peripheral.map_io({.first_word = data_word, .words = 4}, target);
    // Store R0, change R0 before delivery, read R1, store R2, read R3.
    constexpr std::uint64_t store_data = 0x110000000c40;
    constexpr std::uint64_t clear_source = 0x0f0000080021;
    constexpr std::uint64_t read_data = 0x100100000c40;
    constexpr std::uint64_t store_final = 0x110200000c41;
    constexpr std::uint64_t read_final = 0x100300000c41;
    memory.write({.address = entry, .width = lead_a1::SharcWidth::wide48}, store_data);
    memory.write({.address = entry + 1, .width = lead_a1::SharcWidth::wide48}, clear_source);
    memory.write({.address = entry + 2, .width = lead_a1::SharcWidth::wide48}, read_data);
    memory.write({.address = entry + 3, .width = lead_a1::SharcWidth::wide48}, store_final);
    memory.write({.address = entry + 4, .width = lead_a1::SharcWidth::wide48}, read_final);
  }

  void clock() {
    data.advance_cclk(1);
    bus.tick(); // Age effects before newly delivered writes.
  }

  void prepare() {
    constexpr unsigned int limit = 16;
    for (unsigned int elapsed = 0; elapsed < limit; ++elapsed) {
      static_cast<void>(fetch.advance_sequenced(cpu));
      if (fetch.state().execute) { return; }
      clock();
    }
    throw std::runtime_error("Missing actual SPORT DMA Execute");
  }

  lead_a1::SharcStep transfer(unsigned int latency) {
    prepare();
    const auto access = fetch.peripheral_access(cpu);
    if (!access) { throw std::runtime_error("Missing real SPORT DMA transfer"); }
    bus.issue({.pc = access->pc,
               .address = access->address,
               .operation = access->operation,
               .delivery_cycles = latency});
    const auto read = access->operation == Kind::read;
    const auto held = fetch.state();
    const auto pc = cpu.state().pc;
    while (!bus.ready()) {
      if (!read) { fetch.wait_iop(cpu, bus); }
      clock();
      if (read && !bus.ready()) { fetch.wait_iop(cpu, bus); }
      require(fetch.state() == held && cpu.state().pc == pc);
    }
    const auto step = cpu.step(fetch);
    if (!read) { clock(); }
    return step;
  }

  void ordinary() {
    prepare();
    require(!fetch.peripheral_access(cpu));
    require(cpu.step(fetch).pc == entry + 1);
    clock();
  }
};

void real_fetched_writes_reads_and_parameters() {
  for (const unsigned int writes : {4U, 5U}) {
    for (const unsigned int reads : {7U, 8U}) {
      Fixture fixture;
      require(fixture.transfer(writes).pc == entry);
      require(fixture.data.snapshot().index == original_index);
      fixture.ordinary();
      require(fixture.cpu.state().data[0][0].value == (std::uint64_t{later_index} << 8) &&
              fixture.data.snapshot().index == original_index);
      require(fixture.transfer(reads).pc == entry + 2);
      require(fixture.cpu.state().data[0][1].value == (std::uint64_t{first_index} << 8) &&
              fixture.data.snapshot().index == first_index && !fixture.data.snapshot().active);
      require(fixture.transfer(writes).pc == entry + 3);
      require(fixture.data.snapshot().modifier == 0);
      require(fixture.transfer(reads).pc == entry + 4);
      require(fixture.cpu.state().data[0][3].value == (std::uint64_t{modifier_word} << 8) &&
              fixture.data.snapshot().modifier == modifier_word &&
              fixture.data.snapshot().count == 0 && !fixture.data.interrupt_request() &&
              !fixture.data.request());
    }
  }
}

void actual_delivery_failure_and_no_time_retry() {
  Fixture fixture;
  fixture.target.fail = true;
  const auto before = fixture.data.snapshot();
  require(fixture.transfer(5).next_pc == entry + 1);
  const auto core = fixture.cpu.state();
  const auto transport = fixture.fetch.state();
  const auto queue = fixture.bus.snapshot();
  const auto first = queue.writes[0];
  if (!first) { throw std::runtime_error("Missing real posted SPORT DMA write"); }
  const auto due = first->posted_at + 5;
  while (fixture.bus.state().clocks + 1 < due) { fixture.clock(); }
  rejected([&fixture] { fixture.clock(); }, "owned late SPORT DMA delivery fault");
  require(fixture.data.snapshot() == before && fixture.cpu.state() == core &&
          fixture.fetch.state() == transport && fixture.bus.state().clocks == due &&
          fixture.bus.state().write_count == 1);
  rejected([&fixture] { fixture.bus.tick(); }, "retry required before clock");
  rejected([&fixture] { fixture.bus.retry_due(); },
           "pc=0xc0000 word-address=0xc40 operation=write");
  fixture.target.fail = false;
  fixture.bus.retry_due();
  require(fixture.bus.state().clocks == due && fixture.bus.state().write_count == 0 &&
          fixture.data.snapshot().index == first_index && !fixture.data.snapshot().active);
  fixture.ordinary();
  require(fixture.transfer(8).pc == entry + 2 &&
          fixture.cpu.state().data[0][1].value == (std::uint64_t{first_index} << 8));
}

}

int main() {
  try {
    real_fetched_writes_reads_and_parameters();
    actual_delivery_failure_and_no_time_retry();
    std::cout << "Real fetched SPORT DMA parameters/bridge/late-fault expectations passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
