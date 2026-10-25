// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_iop_bridge.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_dai_enable.hpp"
#include "sharc/peripherals/sharc_dai_pin.hpp"
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
constexpr std::uint32_t pin_word = 0x2460;
constexpr std::uint32_t enable_word = 0x2478;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr lead_a1::SharcIoMapping window{.first_word = 0x2400, .words = 0x100};
using Kind = lead_a1::AccessKind;
constexpr std::uint32_t low_word = 0x0fdfbf7e;
constexpr std::uint32_t first_high = 0x0fdfbf7f;

lead_a1::SharcDaiPinState low_pins() {
  lead_a1::SharcDaiPinState state;
  state.routes.fill(low_word);
  return state;
}

lead_a1::SharcDaiEnableState disabled_pins() {
  lead_a1::SharcDaiEnableState state;
  state.routes.fill(0);
  return state;
}

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("DAI pin bus expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing DAI pin bus rejection: " + reason);
}

struct FaultingPins final : lead_a1::MemoryDevice {
  lead_a1::SharcDaiPin &router;
  bool fail{};

  explicit FaultingPins(lead_a1::SharcDaiPin &bound) : router(bound) {}

  std::uint64_t read(lead_a1::MemoryAccess access) override { return router.read(access); }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    router.write(access, value);
    if (fail) { throw std::runtime_error("owned late DAI pin delivery fault"); }
  }

  void begin_instruction() override { router.begin_instruction(); }

  void commit_instruction() noexcept override { router.commit_instruction(); }

  void abort_instruction() noexcept override { router.abort_instruction(); }
};

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(entry);
  for (auto &reg : state.data[0]) { reg = {.known = full40}; }
  state.data[0][0] = {.value = std::uint64_t{first_high} << 8, .known = full40};
  state.data[0][2] = {.value = 1U << 8, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState transport;
  transport.flow = lead_a1::SharcFetchFlow{.next = entry};
  return transport;
}

struct Fixture {
  lead_a1::SharcPower power;
  lead_a1::SharcDaiPin pins{power, low_pins()};
  lead_a1::SharcDaiEnable enables{power, disabled_pins()};
  FaultingPins target{pins};
  lead_a1::SharcMemory peripheral;
  lead_a1::SharcMemory memory;
  lead_a1::SharcIopBridge bus{memory, peripheral, window};
  lead_a1::Sharc cpu{memory, initial()};
  lead_a1::SharcFetch fetch{memory, sequenced()};

  Fixture() {
    constexpr unsigned int cells = 256;
    constexpr std::uint32_t short_entry = 0x180000;
    memory.map({.bank = 2, .halfwords = cells, .name = "owned DAI pin bus program"});
    for (unsigned int index = 0; index < cells; ++index) {
      memory.write({.address = short_entry + index, .width = lead_a1::SharcWidth::short16}, 0);
    }
    memory.map_io(window, bus);
    peripheral.map_io({.first_word = pin_word, .words = 5}, target);
    peripheral.map_io({.first_word = enable_word, .words = 4}, enables);
    // Store R0, change R0 before delivery, read R1, store R2, read R3.
    constexpr std::uint64_t store_pin = 0x110000002460;
    constexpr std::uint64_t clear_source = 0x0f000fdfbf7e;
    constexpr std::uint64_t read_pin = 0x100100002460;
    constexpr std::uint64_t store_enable = 0x110200002478;
    constexpr std::uint64_t read_enable = 0x100300002478;
    memory.write({.address = entry, .width = lead_a1::SharcWidth::wide48}, store_pin);
    memory.write({.address = entry + 1, .width = lead_a1::SharcWidth::wide48}, clear_source);
    memory.write({.address = entry + 2, .width = lead_a1::SharcWidth::wide48}, read_pin);
    memory.write({.address = entry + 3, .width = lead_a1::SharcWidth::wide48}, store_enable);
    memory.write({.address = entry + 4, .width = lead_a1::SharcWidth::wide48}, read_enable);
  }

  void clock() {
    pins.advance_half_pclk(1);
    enables.advance_half_pclk(1);
    bus.tick(); // Age effects before newly delivered writes.
  }

  void prepare() {
    constexpr unsigned int limit = 16;
    for (unsigned int elapsed = 0; elapsed < limit; ++elapsed) {
      static_cast<void>(fetch.advance_sequenced(cpu));
      if (fetch.state().execute) { return; }
      clock();
    }
    throw std::runtime_error("Missing actual DAI pin Execute");
  }

  lead_a1::SharcStep transfer(unsigned int latency) {
    prepare();
    const auto access = fetch.peripheral_access(cpu);
    if (!access) { throw std::runtime_error("Missing real DAI pin transfer"); }
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

void real_fetched_writes_reads_and_drives() {
  constexpr std::uint32_t all_disabled = 0xfffff;
  constexpr std::uint32_t remaining_disabled = 0xffffe;
  for (const unsigned int writes : {4U, 5U}) {
    for (const unsigned int reads : {7U, 8U}) {
      Fixture fixture;
      require(fixture.transfer(writes).pc == entry);
      require(!fixture.pins.snapshot().pending[0] && fixture.pins.outputs()[0] == false);
      fixture.ordinary();
      require(fixture.cpu.state().data[0][0].value == (std::uint64_t{low_word} << 8) &&
              fixture.pins.outputs()[0] == false);
      require(fixture.transfer(reads).pc == entry + 2);
      require(fixture.cpu.state().data[0][1].value == (std::uint64_t{first_high} << 8) &&
              fixture.pins.snapshot().routes[0] == first_high &&
              !fixture.pins.snapshot().pending[0] &&
              fixture.pins.drives(fixture.enables).high_impedance == all_disabled);
      require(fixture.transfer(writes).pc == entry + 3);
      require(fixture.pins.drives(fixture.enables).high_impedance == all_disabled);
      require(fixture.transfer(reads).pc == entry + 4);
      require(fixture.cpu.state().data[0][3].value == (1U << 8) &&
              fixture.enables.snapshot().routes[0] == 1 &&
              fixture.pins.drives(fixture.enables) ==
                  (lead_a1::SharcDaiPinDrive{.high = 1, .high_impedance = remaining_disabled}));
    }
  }
}

void actual_delivery_failure_and_no_time_retry() {
  Fixture fixture;
  fixture.target.fail = true;
  const auto before = fixture.pins.snapshot();
  require(fixture.transfer(5).next_pc == entry + 1);
  const auto core = fixture.cpu.state();
  const auto transport = fixture.fetch.state();
  const auto queue = fixture.bus.snapshot();
  const auto first = queue.writes[0];
  if (!first) { throw std::runtime_error("Missing real posted DAI pin write"); }
  const auto due = first->posted_at + 5;
  while (fixture.bus.state().clocks + 1 < due) { fixture.clock(); }
  rejected([&fixture] { fixture.clock(); }, "owned late DAI pin delivery fault");
  require(fixture.pins.snapshot() == before && fixture.cpu.state() == core &&
          fixture.fetch.state() == transport && fixture.bus.state().clocks == due &&
          fixture.bus.state().write_count == 1);
  rejected([&fixture] { fixture.bus.tick(); }, "retry required before clock");
  rejected([&fixture] { fixture.bus.retry_due(); },
           "pc=0xc0000 word-address=0x2460 operation=write");
  fixture.target.fail = false;
  fixture.bus.retry_due();
  require(fixture.bus.state().clocks == due && fixture.bus.state().write_count == 0 &&
          fixture.pins.snapshot().pending[0] ==
              (lead_a1::SharcDaiPinPending{.target = first_high}));
  constexpr unsigned int half_periods = 6;
  for (unsigned int clock = 0; clock < half_periods; ++clock) { fixture.clock(); }
  require(fixture.pins.outputs()[0] == true && fixture.pins.snapshot().routes[0] == first_high);
  fixture.ordinary();
  require(fixture.transfer(8).pc == entry + 2 &&
          fixture.cpu.state().data[0][1].value == (std::uint64_t{first_high} << 8));
}

}

int main() {
  try {
    real_fetched_writes_reads_and_drives();
    actual_delivery_failure_and_no_time_retry();
    std::cout << "Real fetched DAI pin routing/bridge/drive/late-fault expectations passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
