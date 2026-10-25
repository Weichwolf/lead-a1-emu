// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

using lead_a1::SharcAccelerator;
using lead_a1::SharcClockGate;
using lead_a1::SharcPower;

constexpr std::uint32_t word_address = 0x2001;
constexpr std::uint32_t reserved_bits = 0x80001000;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t base = 0x80000;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("PMCTL1 expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing PMCTL1 rejection: " + reason);
}

void write(SharcPower &power, std::uint32_t value) {
  power.write({.address = 0, .size = 4}, value);
}

std::uint32_t read(SharcPower &power) {
  return static_cast<std::uint32_t>(power.read({.address = 0, .size = 4}));
}

struct Gate {
  SharcClockGate output;
  std::uint32_t mask;
};

constexpr std::array gates{Gate{.output = SharcClockGate::uart, .mask = 0x1},
                           Gate{.output = SharcClockGate::twi, .mask = 0x2},
                           Gate{.output = SharcClockGate::pwm, .mask = 0x4},
                           Gate{.output = SharcClockGate::mtm, .mask = 0x8},
                           Gate{.output = SharcClockGate::dai, .mask = 0x10},
                           Gate{.output = SharcClockGate::external, .mask = 0x20},
                           Gate{.output = SharcClockGate::sport01, .mask = 0x40},
                           Gate{.output = SharcClockGate::sport23, .mask = 0x80},
                           Gate{.output = SharcClockGate::sport45, .mask = 0x100},
                           Gate{.output = SharcClockGate::sport67, .mask = 0x200},
                           Gate{.output = SharcClockGate::spi, .mask = 0x400},
                           Gate{.output = SharcClockGate::timers, .mask = 0x800},
                           Gate{.output = SharcClockGate::rtc, .mask = 0x2000},
                           Gate{.output = SharcClockGate::accelerator, .mask = 0x10000},
                           Gate{.output = SharcClockGate::mlb, .mask = 0x80000}};

void reset_latency_and_outputs() {
  for (const auto selected : gates) {
    SharcPower power;
    const auto reset = power.snapshot();
    require(!reset.control && !reset.accelerator && reset.remaining_pclk == 0);
    rejected([&] { static_cast<void>(read(power)); }, "reset/reserved readback");
    require(power.snapshot() == reset);
    write(power, reserved_bits | selected.mask);
    const auto pending = power.snapshot();
    require(pending.control == (reserved_bits | selected.mask) && pending.remaining_pclk == 2);
    for (const auto gate : gates) { require(power.enabled(gate.output)); }
    power.advance_pclk(0);
    require(power.snapshot() == pending);
    rejected([&] { static_cast<void>(read(power)); }, "unsettled readback");
    rejected([&] { write(power, 0); }, "overlapping write");
    require(power.snapshot() == pending);
    power.advance_pclk(1);
    for (const auto gate : gates) { require(power.enabled(gate.output)); }
    require(power.state().remaining_pclk == 1);
    power.advance_pclk(1);
    for (const auto gate : gates) {
      require(power.enabled(gate.output) == (gate.mask != selected.mask));
    }
    require(read(power) == (reserved_bits | selected.mask) &&
            power.state().accelerator == SharcAccelerator::fir);
    const auto settled = power.snapshot();
    static_cast<void>(read(power));
    power.advance_pclk(std::numeric_limits<std::uint32_t>::max());
    require(power.snapshot() == settled);
    write(power, reserved_bits);
    require(!power.enabled(selected.output));
    power.advance_pclk(2);
    for (const auto gate : gates) { require(power.enabled(gate.output)); }
    require(read(power) == reserved_bits); // Reserved bit12 is not a 2148x link gate.
  }
}

void half_period_progress_and_restore() {
  SharcPower power;
  write(power, 1);
  power.advance_half_pclk(0);
  require(!power.state().half_elapsed && power.state().remaining_pclk == 2);
  power.advance_half_pclk(1);
  const auto half = power.snapshot();
  require(half.half_elapsed && half.remaining_pclk == 2);
  SharcPower restored(half);
  restored.advance_pclk(1);
  require(restored.state().half_elapsed && restored.state().remaining_pclk == 1);
  require(restored.enabled(SharcClockGate::uart));
  restored.advance_half_pclk(1);
  require(!restored.enabled(SharcClockGate::uart));
  require(!restored.state().half_elapsed && restored.state().remaining_pclk == 0);
  power.advance_half_pclk(2);
  require(power.enabled(SharcClockGate::uart) && power.state().half_elapsed);
  power.advance_half_pclk(1);
  require(power.snapshot() == restored.snapshot());
  auto invalid = power.snapshot();
  invalid.half_elapsed = true;
  rejected([&] { const SharcPower other(invalid); }, "state");
  write(power, 0);
  power.advance_half_pclk(std::numeric_limits<std::uint32_t>::max());
  require(power.enabled(SharcClockGate::uart) && !power.state().half_elapsed);
  power.begin_instruction();
  rejected([&] { power.advance_half_pclk(1); }, "active instruction");
  power.abort_instruction();
}

void selection_and_restore() {
  struct Example {
    std::uint32_t value;
    SharcAccelerator expected;
  };

  constexpr std::array examples{Example{.value = 0, .expected = SharcAccelerator::fir},
                                Example{.value = 0x20000, .expected = SharcAccelerator::iir},
                                Example{.value = 0x40000, .expected = SharcAccelerator::fft}};
  for (const auto example : examples) {
    SharcPower power;
    write(power, example.value);
    power.advance_pclk(1);
    SharcPower restored(power.snapshot());
    require(restored.snapshot() == power.snapshot() && !restored.state().accelerator);
    restored.advance_pclk(1);
    require(restored.state().accelerator == example.expected && read(restored) == example.value &&
            power.state().remaining_pclk == 1);
    power.advance_pclk(1);
    require(power.snapshot() == restored.snapshot());
  }
}

void transaction_ownership() {
  SharcPower power;
  const auto reset = power.snapshot();
  power.begin_instruction();
  write(power, 1);
  require(power.state().control == 1 && power.enabled(SharcClockGate::uart));
  rejected([&] { power.begin_instruction(); }, "Nested");
  rejected([&] { power.advance_pclk(2); }, "active instruction");
  rejected([&] { static_cast<void>(power.snapshot()); }, "active instruction");
  power.abort_instruction();
  require(power.snapshot() == reset);
  power.begin_instruction();
  write(power, 1);
  power.commit_instruction();
  power.advance_pclk(2);
  require(!power.enabled(SharcClockGate::uart) && read(power) == 1);
  power.begin_instruction();
  write(power, 0);
  power.abort_instruction();
  require(!power.enabled(SharcClockGate::uart) && read(power) == 1);
}

void malformed_accesses_and_states() {
  SharcPower power;
  const auto before = power.snapshot();
  rejected([&] { power.write({.address = 4, .size = 4}, 1); }, "mapped offset0");
  rejected([&] { power.write({.size = 2}, 1); }, "32-bit access");
  rejected([&] { static_cast<void>(power.read({.size = 8})); }, "32-bit access");
  constexpr std::uint64_t oversized = 0x100000000;
  constexpr std::uint32_t reserved_selection = 0x60000;
  rejected([&] { power.write({.size = 4}, oversized); }, "Oversized");
  rejected([&] { write(power, reserved_selection); }, "reserved accelerator");
  constexpr std::uint32_t absent_link = 0x1000;
  require(power.snapshot() == before);
  rejected([] { const SharcPower invalid({.shutdown = absent_link}); }, "clock/accelerator");
  rejected([] { const SharcPower invalid({.remaining_pclk = 1}); }, "reset knowledge");
  rejected([] { const SharcPower invalid({.control = 1}); }, "settled outputs");
  rejected([] { const SharcPower invalid({.control = 0, .remaining_pclk = 3}); }, "state");
  rejected([reserved_selection] { const SharcPower invalid({.control = reserved_selection}); },
           "reserved accelerator");
  rejected([] { const SharcPower invalid({.control = 0, .accelerator = SharcAccelerator::iir}); },
           "settled outputs");
}

struct FaultingPower final : lead_a1::MemoryDevice {
  SharcPower &power;
  bool fail{true};

  explicit FaultingPower(SharcPower &bound) : power(bound) {}

  std::uint64_t read(lead_a1::MemoryAccess access) override { return power.read(access); }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    power.write(access, value);
    if (fail) { throw std::runtime_error("owned late IOP delivery fault"); }
  }

  void begin_instruction() override { power.begin_instruction(); }

  void commit_instruction() noexcept override { power.commit_instruction(); }

  void abort_instruction() noexcept override { power.abort_instruction(); }
};

void actual_cpu_transaction_rollback() {
  SharcPower power;
  FaultingPower delivery(power);
  lead_a1::SharcMemory memory;
  constexpr unsigned int cells = 256;
  memory.map({.bank = 0, .halfwords = cells, .name = "owned native power program"});
  memory.map_io({.first_word = word_address, .words = 1}, delivery);
  constexpr std::uint64_t direct_write = 0x110600002001;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, direct_write);
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.data[0][6] = {.value = std::uint64_t{reserved_bits | 1U} << 8, .known = full40};
  lead_a1::Sharc cpu(memory, state);
  const auto reset = power.snapshot();
  rejected([&] { static_cast<void>(cpu.step()); }, "owned late IOP delivery fault");
  require(cpu.state() == state && power.snapshot() == reset && !memory.active());
  delivery.fail = false; // Retry the unchanged instruction/CPU after bus repair.
  require(cpu.step().next_pc == base + 1 && power.state().remaining_pclk == 2);
  require(power.enabled(SharcClockGate::uart));
  power.advance_pclk(2);
  require(!power.enabled(SharcClockGate::uart) && read(power) == (reserved_bits | 1U));
}

}

int main() try {
  reset_latency_and_outputs();
  half_period_progress_and_restore();
  selection_and_restore();
  transaction_ownership();
  malformed_accesses_and_states();
  actual_cpu_transaction_rollback();
  std::cout << "PMCTL1 delivered writes, delayed clock outputs, selections and rollback pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
