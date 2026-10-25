// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_iop_bridge.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_power.hpp"
#include "sharc/peripherals/sharc_sport.hpp"
#include "sharc/peripherals/sharc_sport_dma.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t entry = 0xc0000;
constexpr std::uint32_t data_word = 0xc00;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr lead_a1::SharcIoMapping window{.first_word = 0xc00, .words = 0x68};
using Kind = lead_a1::AccessKind;
constexpr std::uint32_t original_control = 0;
constexpr std::uint32_t first_control = 0x23071;
constexpr std::uint32_t later_control = 0;
constexpr std::uint32_t secondary = 4;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SPORT control bus expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing SPORT control bus rejection: " + reason);
}

struct FaultingData final : lead_a1::MemoryDevice {
  lead_a1::SharcSportPair &router;
  bool fail{};

  explicit FaultingData(lead_a1::SharcSportPair &bound) : router(bound) {}

  std::uint64_t read(lead_a1::MemoryAccess access) override { return router.read(access); }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    router.write(access, value);
    if (fail) { throw std::runtime_error("owned late SPORT control delivery fault"); }
  }

  void begin_instruction() override { router.begin_instruction(); }

  void commit_instruction() noexcept override { router.commit_instruction(); }

  void abort_instruction() noexcept override { router.abort_instruction(); }
};

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(entry);
  for (auto &reg : state.data[0]) { reg = {.known = full40}; }
  state.data[0][0] = {.value = std::uint64_t{first_control} << 8, .known = full40};
  state.data[0][2] = {.value = std::uint64_t{secondary} << 8, .known = full40};
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
  lead_a1::SharcSport first{power, memory, 0};
  lead_a1::SharcSport second{power, memory, 1};
  lead_a1::SharcSportPair data{first, second};
  FaultingData target{data};
  lead_a1::SharcMemory peripheral;
  lead_a1::SharcIopBridge bus{memory, peripheral, window};
  lead_a1::Sharc cpu{memory, initial()};
  lead_a1::SharcFetch fetch{memory, sequenced()};

  Fixture() {
    constexpr unsigned int cells = 256;
    constexpr std::uint32_t short_entry = 0x180000;
    memory.map({.bank = 2, .halfwords = cells, .name = "owned SPORT control bus program"});
    for (unsigned int index = 0; index < cells; ++index) {
      memory.write({.address = short_entry + index, .width = lead_a1::SharcWidth::short16}, 0);
    }
    memory.map_io(window, bus);
    peripheral.map_io({.first_word = data_word, .words = window.words}, target);
    // Store R0, change R0 before delivery, read R1, store R2, read R3.
    constexpr std::uint64_t store_data = 0x110000000c00;
    constexpr std::uint64_t clear_source = 0x0f0000000000;
    constexpr std::uint64_t read_data = 0x100100000c00;
    constexpr std::uint64_t store_final = 0x110200000c1a;
    constexpr std::uint64_t read_final = 0x100300000c1a;
    memory.write({.address = entry, .width = lead_a1::SharcWidth::wide48}, store_data);
    memory.write({.address = entry + 1, .width = lead_a1::SharcWidth::wide48}, clear_source);
    memory.write({.address = entry + 2, .width = lead_a1::SharcWidth::wide48}, read_data);
    memory.write({.address = entry + 3, .width = lead_a1::SharcWidth::wide48}, store_final);
    memory.write({.address = entry + 4, .width = lead_a1::SharcWidth::wide48}, read_final);
  }

  void clock() {
    first.advance_cclk(1);
    second.advance_cclk(1);
    first.advance_half_pclk(1);
    second.advance_half_pclk(1);
    bus.tick(); // Age effects before newly delivered writes.
  }

  void prepare() {
    constexpr unsigned int limit = 16;
    for (unsigned int elapsed = 0; elapsed < limit; ++elapsed) {
      static_cast<void>(fetch.advance_sequenced(cpu));
      if (fetch.state().execute) { return; }
      clock();
    }
    throw std::runtime_error("Missing actual SPORT control Execute");
  }

  lead_a1::SharcStep transfer(unsigned int latency) {
    prepare();
    const auto access = fetch.peripheral_access(cpu);
    if (!access) { throw std::runtime_error("Missing real SPORT control transfer"); }
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
      require(fixture.first.snapshot().serial.control[0] == original_control);
      fixture.ordinary();
      require(fixture.cpu.state().data[0][0].value == (std::uint64_t{later_control} << 8) &&
              fixture.first.snapshot().serial.control[0] == original_control);
      require(fixture.transfer(reads).pc == entry + 2);
      require(fixture.cpu.state().data[0][1].value == (std::uint64_t{first_control} << 8) &&
              fixture.first.snapshot().serial.control[0] == first_control &&
              !fixture.first.snapshot().dma[0].active);
      require(fixture.transfer(writes).pc == entry + 3);
      require(fixture.first.snapshot().serial.control[1] == 0);
      require(fixture.transfer(reads).pc == entry + 4);
      require(fixture.cpu.state().data[0][3].value == (std::uint64_t{secondary} << 8) &&
              fixture.first.snapshot().serial.control[1] == secondary &&
              fixture.first.snapshot().serial.sampled[1] == std::array<std::uint32_t, 3>{} &&
              !fixture.first.interrupt_request() &&
              !fixture.first.dma(lead_a1::SharcSportChannel::a).request());
    }
  }
}

void actual_delivery_failure_and_no_time_retry() {
  Fixture fixture;
  fixture.target.fail = true;
  const auto before = fixture.first.snapshot();
  require(fixture.transfer(5).next_pc == entry + 1);
  const auto core = fixture.cpu.state();
  const auto transport = fixture.fetch.state();
  const auto queue = fixture.bus.snapshot();
  const auto first = queue.writes[0];
  if (!first) { throw std::runtime_error("Missing real posted SPORT control write"); }
  const auto due = first->posted_at + 5;
  while (fixture.bus.state().clocks + 1 < due) { fixture.clock(); }
  rejected([&fixture] { fixture.clock(); }, "owned late SPORT control delivery fault");
  require(fixture.first.snapshot() == before && fixture.cpu.state() == core &&
          fixture.fetch.state() == transport && fixture.bus.state().clocks == due &&
          fixture.bus.state().write_count == 1);
  rejected([&fixture] { fixture.bus.tick(); }, "retry required before clock");
  rejected([&fixture] { fixture.bus.retry_due(); },
           "pc=0xc0000 word-address=0xc00 operation=write");
  fixture.target.fail = false;
  fixture.bus.retry_due();
  require(fixture.bus.state().clocks == due && fixture.bus.state().write_count == 0 &&
          fixture.first.snapshot().serial.control[0] == first_control &&
          !fixture.first.snapshot().dma[0].active);
  fixture.ordinary();
  require(fixture.transfer(8).pc == entry + 2 &&
          fixture.cpu.state().data[0][1].value == (std::uint64_t{first_control} << 8));
}

}

int main() {
  try {
    real_fetched_writes_reads_and_parameters();
    actual_delivery_failure_and_no_time_retry();
    std::cout << "Real fetched SPORT control controls/bridge/late-fault expectations passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
