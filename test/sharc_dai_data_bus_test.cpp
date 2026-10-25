// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_iop_bridge.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_dai_data.hpp"
#include "sharc/peripherals/sharc_power.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t entry = 0xc0000;
constexpr std::uint32_t data_word = 0x2440;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr lead_a1::SharcIoMapping window{.first_word = 0x2400, .words = 0x100};
using Kind = lead_a1::AccessKind;
constexpr unsigned int final_destination = 30;
constexpr std::uint32_t final_low = 0x00fbefbe;
constexpr std::uint32_t last_word = 0xe9ffffff;
constexpr std::uint32_t low_word = 0x3efbefbe;
constexpr std::uint32_t first_high = 0x3efbefbf;

lead_a1::SharcDaiDataState low_data() {
  lead_a1::SharcDaiDataState state;
  state.routes.fill(low_word);
  state.routes[6] = final_low;
  return state;
}

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("DAI data bus expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing DAI data bus rejection: " + reason);
}

struct FaultingData final : lead_a1::MemoryDevice {
  lead_a1::SharcDaiData &router;
  bool fail{};

  explicit FaultingData(lead_a1::SharcDaiData &bound) : router(bound) {}

  std::uint64_t read(lead_a1::MemoryAccess access) override { return router.read(access); }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    router.write(access, value);
    if (fail) { throw std::runtime_error("owned late DAI data delivery fault"); }
  }

  void begin_instruction() override { router.begin_instruction(); }

  void commit_instruction() noexcept override { router.commit_instruction(); }

  void abort_instruction() noexcept override { router.abort_instruction(); }
};

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(entry);
  for (auto &reg : state.data[0]) { reg = {.known = full40}; }
  state.data[0][0] = {.value = std::uint64_t{first_high} << 8, .known = full40};
  state.data[0][2] = {.value = std::uint64_t{last_word} << 8, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState transport;
  transport.flow = lead_a1::SharcFetchFlow{.next = entry};
  return transport;
}

struct Fixture {
  lead_a1::SharcPower power;
  lead_a1::SharcDaiData data{power, low_data()};
  FaultingData target{data};
  lead_a1::SharcMemory peripheral;
  lead_a1::SharcMemory memory;
  lead_a1::SharcIopBridge bus{memory, peripheral, window};
  lead_a1::Sharc cpu{memory, initial()};
  lead_a1::SharcFetch fetch{memory, sequenced()};

  Fixture() {
    constexpr unsigned int cells = 256;
    constexpr std::uint32_t short_entry = 0x180000;
    memory.map({.bank = 2, .halfwords = cells, .name = "owned DAI data bus program"});
    for (unsigned int index = 0; index < cells; ++index) {
      memory.write({.address = short_entry + index, .width = lead_a1::SharcWidth::short16}, 0);
    }
    memory.map_io(window, bus);
    peripheral.map_io({.first_word = data_word, .words = 7}, target);
    // Store R0, change R0 before delivery, read R1, store R2, read R3.
    constexpr std::uint64_t store_data = 0x110000002440;
    constexpr std::uint64_t clear_source = 0x0f003efbefbe;
    constexpr std::uint64_t read_data = 0x100100002440;
    constexpr std::uint64_t store_final = 0x110200002446;
    constexpr std::uint64_t read_final = 0x100300002446;
    memory.write({.address = entry, .width = lead_a1::SharcWidth::wide48}, store_data);
    memory.write({.address = entry + 1, .width = lead_a1::SharcWidth::wide48}, clear_source);
    memory.write({.address = entry + 2, .width = lead_a1::SharcWidth::wide48}, read_data);
    memory.write({.address = entry + 3, .width = lead_a1::SharcWidth::wide48}, store_final);
    memory.write({.address = entry + 4, .width = lead_a1::SharcWidth::wide48}, read_final);
  }

  void clock() {
    data.advance_half_pclk(1);
    bus.tick(); // Age effects before newly delivered writes.
  }

  void prepare() {
    constexpr unsigned int limit = 16;
    for (unsigned int elapsed = 0; elapsed < limit; ++elapsed) {
      static_cast<void>(fetch.advance_sequenced(cpu));
      if (fetch.state().execute) { return; }
      clock();
    }
    throw std::runtime_error("Missing actual DAI data Execute");
  }

  lead_a1::SharcStep transfer(unsigned int latency) {
    prepare();
    const auto access = fetch.peripheral_access(cpu);
    if (!access) { throw std::runtime_error("Missing real DAI data transfer"); }
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

void real_fetched_writes_reads_and_routes() {
  for (const unsigned int writes : {4U, 5U}) {
    for (const unsigned int reads : {7U, 8U}) {
      Fixture fixture;
      require(fixture.transfer(writes).pc == entry);
      require(!fixture.data.snapshot().pending[0] && fixture.data.outputs()[0] == false);
      fixture.ordinary();
      require(fixture.cpu.state().data[0][0].value == (std::uint64_t{low_word} << 8) &&
              fixture.data.outputs()[0] == false);
      require(fixture.transfer(reads).pc == entry + 2);
      require(fixture.cpu.state().data[0][1].value == (std::uint64_t{first_high} << 8) &&
              fixture.data.snapshot().routes[0] == first_high &&
              !fixture.data.snapshot().pending[0] && fixture.data.outputs()[0] == true);
      require(fixture.transfer(writes).pc == entry + 3);
      require(fixture.data.outputs()[final_destination] == false);
      require(fixture.transfer(reads).pc == entry + 4);
      require(fixture.cpu.state().data[0][3].value == (std::uint64_t{last_word} << 8) &&
              fixture.data.snapshot().routes[6] == last_word);
      const auto outputs = fixture.data.outputs();
      for (unsigned int destination = 0; destination < outputs.size(); ++destination) {
        require(outputs[destination] == (destination == 0 || destination >= final_destination));
      }
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
  if (!first) { throw std::runtime_error("Missing real posted DAI data write"); }
  const auto due = first->posted_at + 5;
  while (fixture.bus.state().clocks + 1 < due) { fixture.clock(); }
  rejected([&fixture] { fixture.clock(); }, "owned late DAI data delivery fault");
  require(fixture.data.snapshot() == before && fixture.cpu.state() == core &&
          fixture.fetch.state() == transport && fixture.bus.state().clocks == due &&
          fixture.bus.state().write_count == 1);
  rejected([&fixture] { fixture.bus.tick(); }, "retry required before clock");
  rejected([&fixture] { fixture.bus.retry_due(); },
           "pc=0xc0000 word-address=0x2440 operation=write");
  fixture.target.fail = false;
  fixture.bus.retry_due();
  require(fixture.bus.state().clocks == due && fixture.bus.state().write_count == 0 &&
          fixture.data.snapshot().pending[0] ==
              (lead_a1::SharcDaiDataPending{.target = first_high}));
  constexpr unsigned int half_periods = 6;
  for (unsigned int clock = 0; clock < half_periods; ++clock) { fixture.clock(); }
  require(fixture.data.outputs()[0] == true && fixture.data.snapshot().routes[0] == first_high);
  fixture.ordinary();
  require(fixture.transfer(8).pc == entry + 2 &&
          fixture.cpu.state().data[0][1].value == (std::uint64_t{first_high} << 8));
}

}

int main() {
  try {
    real_fetched_writes_reads_and_routes();
    actual_delivery_failure_and_no_time_retry();
    std::cout << "Real fetched DAI data routing/bridge/late-fault expectations passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
