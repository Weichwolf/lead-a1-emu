// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_dai_enable.hpp"
#include "lead_a1/sharc_iop_bridge.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_power.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

using lead_a1::AccessKind;
using lead_a1::Sharc;
using lead_a1::SharcIopBridge;
using lead_a1::SharcIopState;
using lead_a1::SharcMemory;

constexpr std::uint32_t base = 0x80000;
constexpr std::uint32_t address = 0x2478;
constexpr std::uint32_t first_value = 0x11112222;
constexpr std::uint32_t second_value = 0x33334444;
constexpr std::uint32_t third_value = 0x55556666;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr lead_a1::SharcIoMapping window{.first_word = address, .words = 4};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("IOP expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing IOP rejection: " + reason);
}

// Owned transactional register bank. Counts are committed external effects,
// not scripted responses; injected faults occur after staging a real access.
struct RegisterState {
  std::array<std::uint32_t, 4> words{};
  unsigned int reads{};
  unsigned int writes{};
  bool operator==(const RegisterState &) const = default;
};

struct Registers final : lead_a1::MemoryDevice {
  RegisterState current;
  bool fail_read{};
  bool fail_write{};

  std::uint64_t read(lead_a1::MemoryAccess access) override {
    auto &state = working();
    ++state.reads;
    const auto value = state.words.at(access.address / 4);
    if (fail_read) { throw std::runtime_error("owned target read fault"); }
    return value;
  }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    auto &state = working();
    ++state.writes;
    state.words.at(access.address / 4) = static_cast<std::uint32_t>(value);
    if (fail_write) { throw std::runtime_error("owned target write fault"); }
  }

  void begin_instruction() override {
    if (staged_) { throw std::logic_error("owned foreign target transaction"); }
    staged_ = current;
  }

  void commit_instruction() noexcept override {
    if (staged_) { current = *staged_; }
    staged_.reset();
  }

  void abort_instruction() noexcept override { staged_.reset(); }

  RegisterState &working() { return staged_ ? *staged_ : current; }

  std::optional<RegisterState> staged_;
};

lead_a1::SharcState initial() {
  auto state = Sharc::reset_reference_214xx(base);
  state.data[0][0] = {.value = std::uint64_t{first_value} << 8, .known = full40};
  state.data[0][1] = {.value = std::uint64_t{second_value} << 8, .known = full40};
  state.data[0][2] = {.value = std::uint64_t{third_value} << 8, .known = full40};
  return state;
}

std::uint32_t register_value(const Sharc &cpu, unsigned int reg) {
  return static_cast<std::uint32_t>(cpu.state().data[0].at(reg).value >> 8);
}

// Native Type16 direct moves. Fields are literal ISA expectations, separate
// from the emulator decoder and from all manufacturer program inputs.
std::uint64_t move(AccessKind operation, unsigned int reg, std::uint32_t word) {
  const std::uint64_t prefix = operation == AccessKind::write ? 0x110000000000 : 0x100000000000;
  return prefix | (static_cast<std::uint64_t>(reg) << 32) | word;
}

struct Fixture {
  Registers target;
  SharcMemory peripheral;
  std::optional<SharcIopBridge> bridge;
  SharcMemory memory;
  Sharc cpu;

  explicit Fixture(SharcIopState saved = {}, lead_a1::SharcState core = initial())
      : cpu(memory, core) {
    peripheral.map_io(window, target);
    bridge.emplace(memory, peripheral, window, saved);
    memory.map({.bank = 0, .first_halfword = 0, .halfwords = 128, .name = "owned IOP code"});
    memory.map_io(window, bus());
  }

  SharcIopBridge &bus() {
    if (!bridge) { throw std::logic_error("Missing test bridge"); }
    return *bridge;
  }

  void code(unsigned int offset, std::uint64_t instruction) {
    memory.write({.address = base + offset, .width = lead_a1::SharcWidth::wide48}, instruction);
  }

  void issue(AccessKind operation, unsigned int cycles, std::uint32_t word = address) {
    bus().issue(
        {.pc = cpu.state().pc, .address = word, .operation = operation, .delivery_cycles = cycles});
  }

  void ticks(unsigned int count) {
    for (unsigned int index = 0; index < count; ++index) { bus().tick(); }
  }

  void step() { static_cast<void>(cpu.step()); }
};

void posted_write_and_no_ticket() {
  for (const unsigned int latency : {4U, 5U}) {
    Fixture fixture;
    fixture.code(0, move(AccessKind::write, 0, address));
    const auto cpu = fixture.cpu.state();
    rejected([&] { fixture.step(); }, "ready request");
    require(fixture.cpu.state() == cpu && fixture.bus().snapshot() == SharcIopState{});
    fixture.issue(AccessKind::write, latency);
    require(fixture.bus().ready());
    fixture.step();
    require(fixture.cpu.state().pc == base + 1 && fixture.target.current.writes == 0);
    const auto posted = fixture.bus().snapshot();
    require(!fixture.bus().ready() && fixture.bus().snapshot() == posted);
    fixture.ticks(latency - 1);
    require(fixture.target.current.writes == 0);
    fixture.ticks(1);
    require(fixture.target.current.writes == 1 && fixture.target.current.words[0] == first_value);
    require(fixture.bus().state().write_count == 0);
  }
}

void pipelined_write_spacing() {
  Fixture fixture;
  for (unsigned int index = 0; index < 3; ++index) {
    fixture.code(index, move(AccessKind::write, index, address + index));
  }
  fixture.issue(AccessKind::write, 5);
  fixture.step();
  fixture.ticks(1);
  fixture.issue(AccessKind::write, 4, address + 1);
  const auto state = fixture.cpu.state();
  const auto pending = fixture.bus().snapshot();
  require(!fixture.bus().ready());
  rejected([&] { fixture.step(); }, "ready request");
  require(fixture.cpu.state() == state && fixture.bus().snapshot() == pending);
  fixture.ticks(1);
  fixture.step(); // Second posted at CCLK2, delivery6.
  fixture.ticks(1);
  fixture.issue(AccessKind::write, 5, address + 2);
  require(!fixture.bus().ready());
  fixture.ticks(1);
  fixture.step(); // Third posted at CCLK4, delivery9.
  require(fixture.bus().state().write_count == 3 && fixture.target.current.writes == 0);
  fixture.ticks(1);
  require(fixture.target.current.writes == 1 && fixture.target.current.words[0] == first_value);
  fixture.ticks(1);
  require(fixture.target.current.writes == 2 && fixture.target.current.words[1] == second_value);
  fixture.ticks(2);
  require(fixture.target.current.writes == 2);
  fixture.ticks(1);
  require(fixture.target.current.writes == 3 && fixture.target.current.words[2] == third_value);
}

void core_runs_ahead_of_target() {
  Fixture fixture;
  fixture.code(0, move(AccessKind::write, 0, address));
  constexpr std::uint64_t immediate_r0 = 0x0f009abcdef0;
  constexpr std::uint32_t changed = 0x9abcdef0;
  fixture.code(1, immediate_r0);
  fixture.issue(AccessKind::write, 4);
  fixture.step();
  fixture.ticks(1);
  fixture.step();
  require(register_value(fixture.cpu, 0) == changed && fixture.target.current.writes == 0);
  fixture.ticks(3);
  require(fixture.target.current.words[0] == first_value);
}

void read_capture_and_retry() {
  for (const unsigned int latency : {7U, 8U}) {
    Fixture fixture;
    fixture.target.current.words[0] = first_value;
    fixture.code(0, move(AccessKind::read, 1, address));
    fixture.issue(AccessKind::read, latency);
    fixture.ticks(latency - 1);
    const auto cpu = fixture.cpu.state();
    const auto waiting = fixture.bus().snapshot();
    rejected([&] { fixture.step(); }, "ready request");
    require(fixture.cpu.state() == cpu && fixture.bus().snapshot() == waiting);
    require(fixture.target.current.reads == 0);
    fixture.ticks(1);
    require(fixture.bus().ready() && fixture.target.current.reads == 1);
    fixture.target.current.words[0] = second_value;
    fixture.ticks(2);
    fixture.step();
    require(register_value(fixture.cpu, 1) == first_value);
    require(fixture.target.current.reads == 1 && !fixture.bus().state().request);
  }
}

void cpu_read_rollback_keeps_external_capture() {
  auto state = initial();
  constexpr unsigned int mode1 = 2;
  constexpr std::uint32_t low_bank_selection = 0x400;
  state.effective[mode1].known &= ~low_bank_selection;
  Fixture fixture({}, state);
  fixture.target.current.words[0] = first_value;
  fixture.code(0, move(AccessKind::read, 1, address));
  fixture.issue(AccessKind::read, 7);
  fixture.ticks(7);
  const auto captured = fixture.bus().snapshot();
  rejected([&] { fixture.step(); }, "Unknown SHARC register bank selection");
  require(fixture.cpu.state() == state && fixture.bus().snapshot() == captured);
  require(fixture.target.current.reads == 1);
  state.effective[mode1].known |= low_bank_selection;
  Sharc repaired(fixture.memory, state);
  fixture.target.current.words[0] = third_value;
  require(repaired.step().next_pc == base + 1);
  require(register_value(repaired, 1) == first_value && fixture.target.current.reads == 1);
}

void atomic_write_read_bounds() {
  for (const unsigned int write_cycles : {4U, 5U}) {
    for (const unsigned int read_cycles : {7U, 8U}) {
      Fixture fixture;
      fixture.code(0, move(AccessKind::write, 0, address));
      fixture.code(1, move(AccessKind::read, 1, address));
      fixture.issue(AccessKind::write, write_cycles);
      fixture.step();
      fixture.ticks(1); // The first posted write consumes one core cycle.
      fixture.issue(AccessKind::read, read_cycles);
      fixture.ticks(write_cycles - 1);
      require(fixture.target.current.writes == 1 && fixture.target.current.reads == 0);
      fixture.ticks(read_cycles - 1);
      require(!fixture.bus().ready() && fixture.target.current.reads == 0);
      fixture.ticks(1);
      require(fixture.bus().state().clocks == write_cycles + read_cycles);
      require(fixture.bus().ready());
      fixture.step();
      require(register_value(fixture.cpu, 1) == first_value);
      require(fixture.target.current.reads == 1);
      fixture.code(2, move(AccessKind::read, 2, address));
      const auto completed = fixture.bus().snapshot();
      rejected([&] { fixture.issue(AccessKind::read, 7); }, "eight core clocks");
      require(fixture.bus().snapshot() == completed);
      fixture.issue(AccessKind::read, 8);
      fixture.ticks(8);
      fixture.step();
      require(register_value(fixture.cpu, 2) == first_value && fixture.target.current.reads == 2);
    }
  }
}

void delivery_failure_and_exact_retry() {
  for (const auto operation : {AccessKind::read, AccessKind::write}) {
    Fixture fixture;
    const unsigned int cycles = operation == AccessKind::read ? 7 : 4;
    fixture.code(0, move(operation, 0, address));
    fixture.target.current.words[0] = second_value;
    fixture.target.fail_read = true;
    fixture.target.fail_write = true;
    const auto external = fixture.target.current;
    fixture.issue(operation, cycles);
    if (operation == AccessKind::write) { fixture.step(); }
    fixture.ticks(cycles - 1);
    rejected([&] { fixture.bus().tick(); }, "pc=0x80000 word-address=0x2478 operation=");
    require(fixture.target.current == external && !fixture.peripheral.active());
    require(fixture.bus().state().clocks == cycles);
    const auto due = fixture.bus().snapshot();
    rejected([&] { fixture.bus().tick(); }, "retry required");
    rejected([&] { fixture.issue(operation, cycles); }, "still pending");
    require(fixture.bus().snapshot() == due);
    fixture.target.fail_read = false;
    fixture.target.fail_write = false;
    fixture.bus().retry_due();
    require(fixture.bus().state().clocks == cycles);
    if (operation == AccessKind::read) {
      fixture.step();
      require(register_value(fixture.cpu, 0) == second_value);
      require(fixture.target.current.reads == 1 && fixture.target.current.writes == 0);
    } else {
      require(fixture.cpu.state().pc == base + 1 && fixture.target.current.writes == 1);
      require(fixture.target.current.words[0] == first_value);
    }
    rejected([&] { fixture.bus().retry_due(); }, "Missing due");
  }
}

void snapshots_preserve_external_effects() {
  Fixture original;
  original.code(0, move(AccessKind::write, 0, address));
  original.code(1, move(AccessKind::read, 1, address));
  original.issue(AccessKind::write, 5);
  original.step();
  original.ticks(1);
  original.issue(AccessKind::read, 8);
  original.ticks(2);
  Fixture restored(original.bus().snapshot(), original.cpu.state());
  restored.code(1, move(AccessKind::read, 1, address));
  restored.target.current = original.target.current;
  original.ticks(10);
  restored.ticks(10);
  require(original.bus().snapshot() == restored.bus().snapshot());
  require(original.target.current == restored.target.current);
  Fixture response(restored.bus().snapshot(), restored.cpu.state());
  response.target.current = restored.target.current;
  response.code(1, move(AccessKind::read, 1, address));
  response.target.current.words[0] = third_value;
  response.step();
  require(register_value(response.cpu, 1) == first_value);
  require(response.target.current.reads == 1);
}

void actual_dai_write_then_read() {
  constexpr std::uint32_t reset_pben2 = 0x1a5545d6;
  constexpr std::uint32_t pin13_low = 0x1a5405d6;
  lead_a1::SharcPower power;
  lead_a1::SharcDaiEnable dai(power);
  SharcMemory peripheral;
  std::optional<SharcIopBridge> bridge;
  SharcMemory memory;
  Sharc cpu(memory, initial());
  peripheral.map_io(window, dai);
  bridge.emplace(memory, peripheral, window);
  if (!bridge) { throw std::logic_error("Missing DAI test bridge"); }
  auto &bus = *bridge;
  memory.map({.bank = 0, .first_halfword = 0, .halfwords = 128, .name = "owned DAI code"});
  memory.map_io(window, bus);
  constexpr std::array<std::uint64_t, 7> program{0x10000000247a,
                                                 0x0f01fffc0fff,
                                                 0x013e00040001,
                                                 0x0f0100000000,
                                                 0x013e00041001,
                                                 0x11000000247a,
                                                 0x10020000247a};
  for (unsigned int index = 0; index < program.size(); ++index) {
    memory.write({.address = base + index, .width = lead_a1::SharcWidth::wide48}, program[index]);
  }
  const auto tick = [&] {
    power.advance_half_pclk(1);
    dai.advance_half_pclk(1);
    bus.tick(); // The new delivery must not age during the preceding interval.
  };
  bus.issue(
      {.pc = base, .address = address + 2, .operation = AccessKind::read, .delivery_cycles = 7});
  for (unsigned int index = 0; index < 7; ++index) { tick(); }
  require(cpu.step().next_pc == base + 1 && register_value(cpu, 0) == reset_pben2);
  for (unsigned int index = 0; index < 4; ++index) {
    static_cast<void>(cpu.step());
    tick();
  }
  bus.issue({.pc = base + 5,
             .address = address + 2,
             .operation = AccessKind::write,
             .delivery_cycles = 5});
  require(cpu.step().next_pc == base + 6 && register_value(cpu, 0) == pin13_low);
  tick();
  bus.issue({.pc = base + 6,
             .address = address + 2,
             .operation = AccessKind::read,
             .delivery_cycles = 7});
  for (unsigned int index = 0; index < 4; ++index) { tick(); }
  const auto delivered = dai.snapshot();
  const auto pending = delivered.pending[2];
  require(pending && !pending->half_elapsed);
  for (unsigned int index = 0; index < 5; ++index) { tick(); }
  require(!bus.ready());
  rejected([&] { static_cast<void>(dai.read({.address = 8, .size = 4})); }, "unsettled");
  tick();
  require(dai.outputs()[12] == false && !dai.outputs()[11].has_value());
  require(!bus.ready());
  tick();
  require(bus.ready() && cpu.step().next_pc == base + 7);
  require(register_value(cpu, 2) == pin13_low);
}

void invalid_requests_and_bindings() {
  Fixture fixture;
  const auto before = fixture.bus().snapshot();
  auto request = lead_a1::SharcIopIntent{
      .pc = base, .address = address, .operation = AccessKind::write, .delivery_cycles = 4};
  for (const unsigned int latency : {0U, 3U, 6U, 7U, 8U}) {
    request.delivery_cycles = latency;
    rejected([&] { fixture.bus().issue(request); }, "operation/latency");
  }
  request.delivery_cycles = 4;
  request.pc = 0x1000000;
  rejected([&] { fixture.bus().issue(request); }, "PC/address");
  request.pc = base;
  request.address = address - 1;
  rejected([&] { fixture.bus().issue(request); }, "PC/address");
  request.address = address;
  request.operation = AccessKind::fetch;
  rejected([&] { fixture.bus().issue(request); }, "operation/latency");
  require(fixture.bus().snapshot() == before);
  rejected([&] { const SharcIopBridge other(fixture.memory, fixture.memory, window); }, "binding");
  rejected([&] { const SharcIopBridge other(fixture.memory, fixture.peripheral, {}); }, "window");
  constexpr lead_a1::SharcIoMapping core_window{.first_word = 0x30000, .words = 1};
  rejected([&] { const SharcIopBridge other(fixture.memory, fixture.peripheral, core_window); },
           "window");
}

void invalid_snapshots() {
  Fixture fixture;
  auto state = fixture.bus().snapshot();
  const auto invalid = [&] {
    const SharcIopBridge other(fixture.memory, fixture.peripheral, window, state);
  };
  state.write_count = 5;
  rejected(invalid, "queue/history");
  state = {};
  state.last_read = 1;
  rejected(invalid, "queue/history");
  state = {};
  state.clocks = 3;
  state.last_write = 0;
  rejected(invalid, "completed write history");
  state = {};
  state.read = lead_a1::SharcIopRead{};
  rejected(invalid, "Orphan");
  state = {};
  state.write_count = 1;
  rejected(invalid, "continuity");
  fixture.code(0, move(AccessKind::write, 0, address));
  fixture.issue(AccessKind::write, 4);
  fixture.step();
  state = fixture.bus().snapshot();
  state.last_write = std::nullopt;
  rejected(invalid, "last posted write");
  state = fixture.bus().snapshot();
  state.clocks = 5;
  rejected(invalid, "posted write timing");
  state = {};
  state.clocks = std::numeric_limits<std::uint64_t>::max();
  Fixture overflow(state);
  rejected([&] { overflow.issue(AccessKind::read, 7); }, "timeline overflow");
  rejected([&] { overflow.bus().tick(); }, "core-clock overflow");
}

void transaction_and_callback_guards() {
  Fixture fixture;
  fixture.code(0, move(AccessKind::write, 0, address));
  fixture.issue(AccessKind::write, 4);
  const auto before = fixture.bus().snapshot();
  fixture.memory.set_pc(base);
  fixture.memory.begin_instruction();
  fixture.memory.write({.address = address}, first_value);
  rejected([&] { fixture.bus().begin_instruction(); }, "Nested");
  rejected([&] { fixture.bus().tick(); }, "active transaction");
  rejected([&] { static_cast<void>(fixture.bus().snapshot()); }, "active transaction");
  rejected([&] { fixture.issue(AccessKind::read, 7); }, "active transaction");
  fixture.memory.abort_instruction();
  require(fixture.bus().snapshot() == before && fixture.target.current.writes == 0);
  fixture.memory.set_pc(base + 1);
  fixture.memory.begin_instruction();
  rejected([&] { fixture.memory.write({.address = address}, first_value); }, "ready request");
  fixture.memory.abort_instruction();
  fixture.memory.set_pc(base);
  fixture.memory.begin_instruction();
  rejected([&] { static_cast<void>(fixture.memory.read({.address = address})); }, "ready request");
  rejected([&] { fixture.bus().write({.size = 2}, 0); }, "bound32-bit");
  rejected([&] { fixture.bus().write({.address = 1, .size = 4}, 0); }, "bound32-bit");
  constexpr std::uint64_t oversized = 0x100000000;
  rejected([&] { fixture.bus().write({.size = 4}, oversized); }, "Oversized");
  fixture.memory.abort_instruction();
  require(fixture.bus().snapshot() == before);
  fixture.peripheral.begin_instruction();
  rejected([&] { fixture.bus().tick(); }, "active transaction");
  rejected([&] { fixture.step(); }, "Nested");
  require(fixture.peripheral.active() && fixture.bus().state() == before);
  fixture.peripheral.abort_instruction();
  fixture.step();
  fixture.ticks(4);
  require(fixture.target.current.words[0] == first_value && fixture.target.current.writes == 1);
}

void malformed_read_snapshots() {
  Fixture fixture;
  fixture.issue(AccessKind::read, 7);
  const auto started = fixture.bus().snapshot();
  auto state = started;
  const auto invalid = [&] {
    const SharcIopBridge other(fixture.memory, fixture.peripheral, window, state);
  };
  state.request.reset();
  rejected(invalid, "Orphan");
  state = started;
  state.read.reset();
  rejected(invalid, "pairing");
  state = started;
  state.read = lead_a1::SharcIopRead{.started_at = 1};
  rejected(invalid, "response timing");
  state = started;
  state.read = lead_a1::SharcIopRead{};
  rejected(invalid, "waiting read");
  state = started;
  state.read = lead_a1::SharcIopRead{.started_at = 0, .response = 0};
  rejected(invalid, "response timing");
  state = started;
  state.clocks = 8;
  rejected(invalid, "response timing");
  state = started;
  state.clocks = 3;
  state.last_read = 2;
  rejected(invalid, "response timing");
  fixture.ticks(7);
  const auto response = fixture.bus().snapshot();
  state = response;
  state.last_read = 6;
  rejected(invalid, "completion history");
  state = response;
  state.read = lead_a1::SharcIopRead{.started_at = 7};
  rejected(invalid, "back-to-back read timing");
  Fixture zero(response);
  zero.code(0, move(AccessKind::read, 0, address));
  zero.step();
  require(register_value(zero.cpu, 0) == 0 && zero.target.current.reads == 0);
}

}

int main() try {
  posted_write_and_no_ticket();
  pipelined_write_spacing();
  core_runs_ahead_of_target();
  read_capture_and_retry();
  cpu_read_rollback_keeps_external_capture();
  atomic_write_read_bounds();
  delivery_failure_and_exact_retry();
  snapshots_preserve_external_effects();
  actual_dai_write_then_read();
  invalid_requests_and_bindings();
  invalid_snapshots();
  transaction_and_callback_guards();
  malformed_read_snapshots();
  std::cout << "IOP posted writes, real read capture, bounded timing and rollback pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
