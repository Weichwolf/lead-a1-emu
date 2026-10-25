// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_iop_bridge.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_pdap.hpp"
#include "sharc/peripherals/sharc_power.hpp"

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

using lead_a1::SharcPdap;
using lead_a1::SharcPdapPort;

constexpr std::uint32_t pins = 0xfffff;
constexpr std::uint32_t enable = 0x80000000;
constexpr std::uint32_t reset = 0x40000000;
constexpr std::uint32_t falling = 0x20000000;
constexpr std::uint32_t ami = 0x04000000;
constexpr std::uint32_t global_enable = 0x80;
constexpr std::uint32_t fifo = 0x80;
constexpr std::uint32_t status = 0x20;
constexpr std::uint32_t byte_mask = 0xff000;
constexpr std::uint32_t packed_bytes = 0x78563412;
constexpr std::uint32_t overrun_status = 0x80000100;
constexpr std::uint32_t full_status = 0x80000000;
constexpr std::uint32_t fifo_clear = 0x80000000;
constexpr std::array<std::uint32_t, 4> bytes{0x12, 0x34, 0x56, 0x78};
constexpr std::uint32_t base = 0x80000;
constexpr std::uint32_t pdap_word = 0x24b0;
constexpr std::uint32_t fifo_word = 0x24d0;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("PDAP expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing PDAP rejection: " + reason);
}

void write(SharcPdap &pdap, unsigned int index, std::uint32_t value) {
  pdap.write({.address = index * 4, .size = 4}, value);
}

std::uint32_t read(SharcPdap &pdap, std::uint32_t offset) {
  return static_cast<std::uint32_t>(pdap.read({.address = offset, .size = 4}));
}

void configure(SharcPdap &pdap, std::uint32_t configuration) {
  write(pdap, 1, configuration);
  write(pdap, 2, 1); // Only PDAP/channel0; no ignored active SIP channels.
  write(pdap, 0, global_enable);
  pdap.advance_sclk(1); // Explicit completed input period, not an inferred Hz.
}

void sample(SharcPdap &pdap, std::uint32_t input, SharcPdapPort port = SharcPdapPort::dai) {
  const bool active = (pdap.state().control[1] & falling) == 0;
  pdap.observe(port, {.clock = !active, .hold = false, .data = input, .known = pins});
  pdap.observe(port, {.clock = active, .hold = false, .data = input, .known = pins});
}

void byte_word(SharcPdap &pdap) {
  for (const auto byte : bytes) { sample(pdap, byte << 12); }
}

void reset_and_enable_requirements() {
  lead_a1::SharcPower power;
  SharcPdap pdap(power);
  constexpr std::array<std::uint32_t, 4> controls{0, 0, 0xffff, 0};
  require(pdap.snapshot().control == controls && read(pdap, status) == 0);
  require(!pdap.fifo_read_ready() && !pdap.fifo_write_ready());
  pdap.observe(SharcPdapPort::dai, {});
  require(pdap.state().count == 0);
  write(pdap, 1, enable | byte_mask);
  byte_word(pdap); // PP enable alone does not enable reception.
  require(pdap.state().count == 0 && pdap.state().samples == 0);
  const auto before = pdap.snapshot();
  rejected([&] { write(pdap, 0, global_enable); }, "SIP channels");
  require(pdap.snapshot() == before);
  write(pdap, 2, 1);
  write(pdap, 0, global_enable);
  require(pdap.state().startup_sclk == 1);
  pdap.observe(SharcPdapPort::dai, {.clock = false});
  const auto waiting = pdap.snapshot();
  rejected([&] { pdap.observe(SharcPdapPort::dai, {.clock = true, .hold = false}); }, "startup");
  require(pdap.snapshot() == waiting);
  pdap.advance_sclk(0);
  require(pdap.snapshot() == waiting);
  pdap.advance_sclk(1);
  byte_word(pdap);
  require(pdap.state().count == 1 && read(pdap, fifo) == packed_bytes);
}

void independent_packing_expectations() {
  struct Example {
    std::uint32_t mode;
    unsigned int count;
    std::array<std::uint32_t, 4> inputs;
    std::uint32_t expected;
  };

  constexpr std::array examples{
      Example{.mode = 0,
              .count = 4,
              .inputs = {0x12000, 0x34000, 0x56000, 0x78000},
              .expected = packed_bytes},
      Example{.mode = 1, .count = 3, .inputs = {0x55400, 0x68a00, 0xcf000}, .expected = 0xcf0d1555},
      Example{.mode = 2, .count = 2, .inputs = {0x12340, 0xabcd0}, .expected = 0xabcd1234},
      Example{.mode = 3, .count = 1, .inputs = {0xabcde}, .expected = 0xabcde000}};
  constexpr unsigned int mode_shift = 27;
  for (const auto &example : examples) {
    for (const auto edge : {0U, falling}) {
      lead_a1::SharcPower power;
      SharcPdap pdap(power);
      configure(pdap, enable | edge | pins | (example.mode << mode_shift));
      for (unsigned int index = 0; index < example.count; ++index) {
        sample(pdap, example.inputs[index]);
        require(pdap.state().count == (index + 1 == example.count ? 1U : 0U));
      }
      require(read(pdap, fifo) == example.expected);
      require(pdap.state().count == 0 && pdap.state().samples == 0 && pdap.state().partial == 0);
    }
  }
}

void held_unknown_and_masked_inputs() {
  lead_a1::SharcPower power;
  SharcPdap pdap(power);
  configure(pdap, enable | byte_mask);
  sample(pdap, bytes[0] << 12);
  const auto partial = pdap.state().partial;
  pdap.observe(SharcPdapPort::dai, {.clock = false});
  pdap.observe(SharcPdapPort::dai, {.clock = true, .hold = true});
  require(pdap.state().samples == 1 && pdap.state().partial == partial);
  pdap.observe(SharcPdapPort::dai, {.clock = false});
  const auto before = pdap.snapshot();
  rejected([&] { pdap.observe(SharcPdapPort::dai, {.clock = true}); }, "Unknown PDAP HOLD");
  require(pdap.snapshot() == before);
  rejected([&] { pdap.observe(SharcPdapPort::dai, {.clock = true, .hold = false}); },
           "Unknown unmasked");
  require(pdap.snapshot() == before);
  pdap.observe(SharcPdapPort::dai,
               {.clock = true, .hold = false, .data = bytes[1] << 12, .known = byte_mask});
  sample(pdap, bytes[2] << 12);
  sample(pdap, bytes[3] << 12);
  require(read(pdap, fifo) == packed_bytes);
  write(pdap, 1, enable); // Masked unknown bits become known zero, not guessed pad levels.
  for (unsigned int index = 0; index < 4; ++index) {
    pdap.observe(SharcPdapPort::dai, {.clock = false});
    pdap.observe(SharcPdapPort::dai, {.clock = true, .hold = false});
  }
  require(read(pdap, fifo) == 0);
}

void input_selection_and_clock_history() {
  lead_a1::SharcPower power;
  SharcPdap pdap(power);
  configure(pdap, enable | ami | byte_mask);
  byte_word(pdap);
  require(pdap.state().count == 0);
  const auto initial = pdap.snapshot();
  rejected([&] { pdap.observe(SharcPdapPort::ami, {.clock = true, .hold = false}); }, "history");
  rejected([&] { pdap.observe(SharcPdapPort::ami, {}); }, "acquisition clock");
  require(pdap.snapshot() == initial);
  for (const auto byte : bytes) { sample(pdap, byte << 12, SharcPdapPort::ami); }
  require(read(pdap, fifo) == packed_bytes);
  require(pdap.state().inputs[0].clock == true && pdap.state().inputs[1].clock == true);
}

void fifo_order_overflow_and_interrupts() {
  lead_a1::SharcPower power;
  SharcPdap pdap(power);
  constexpr std::uint32_t no_packing = 0x18000000;
  configure(pdap, enable | no_packing | pins);
  for (unsigned int index = 0; index < 8; ++index) { sample(pdap, index); }
  require(!pdap.fifo_write_ready() && pdap.fifo_read_ready());
  require(read(pdap, status) == full_status);
  require(pdap.interrupts() == (lead_a1::SharcPdapInterrupts{.threshold = true}));
  sample(pdap, pins);
  require(read(pdap, status) == overrun_status && pdap.interrupts().overflow);
  constexpr std::uint32_t seven = 7;
  write(pdap, 0, global_enable | seven);
  require(pdap.interrupts().threshold);
  require(read(pdap, fifo) == 0);
  require(!pdap.interrupts().threshold && pdap.interrupts().overflow);
  constexpr std::uint32_t extra = 0xabc;
  sample(pdap, extra); // Wrap tail into the freed slot; the dropped sample stays absent.
  for (unsigned int index = 1; index < 8; ++index) { require(read(pdap, fifo) == (index << 12)); }
  require(read(pdap, fifo) == (extra << 12));
  require(!pdap.interrupts().threshold && pdap.interrupts().overflow);
  pdap.acknowledge_overflow();
  require(pdap.interrupts() == lead_a1::SharcPdapInterrupts{});
  sample(pdap, 1);
  write(pdap, 0, global_enable | 8U);
  require(!pdap.interrupts().threshold);
}

void commands_and_delayed_disable() {
  lead_a1::SharcPower power;
  SharcPdap pdap(power);
  configure(pdap, enable | byte_mask);
  byte_word(pdap);
  sample(pdap, bytes[0] << 12);
  write(pdap, 1, enable | byte_mask | reset);
  require(pdap.state().samples == 0 && pdap.state().count == 1);
  require(read(pdap, 4) == (enable | byte_mask));
  write(pdap, 2, 1U | fifo_clear);
  require(pdap.state().count == 0 && read(pdap, 8) == 1);
  for (unsigned int index = 0; index < 9; ++index) { byte_word(pdap); }
  require(pdap.state().overflow);
  write(pdap, 2, 1U | fifo_clear);
  require(pdap.state().count == 0 && pdap.state().overflow);
  byte_word(pdap);
  constexpr std::uint32_t clear_overflow = 0x40;
  write(pdap, 0, global_enable | clear_overflow);
  require(pdap.state().count == 0 && !pdap.state().overflow);
  require(read(pdap, 0) == global_enable);
  byte_word(pdap);
  write(pdap, 0, 0);
  const auto disabled = pdap.snapshot();
  require(disabled.flush_cclk == 2 && disabled.count == 1);
  rejected([&] { static_cast<void>(read(pdap, status)); }, "flush still pending");
  pdap.advance_cclk(0);
  require(pdap.snapshot() == disabled);
  pdap.advance_cclk(1);
  SharcPdap restored(power, pdap.snapshot());
  restored.advance_cclk(1);
  pdap.advance_cclk(std::numeric_limits<std::uint32_t>::max());
  require(restored.snapshot() == pdap.snapshot());
  require(pdap.state().count == 0 && pdap.state().flush_cclk == 0);
}

void restore_partial_and_ownership() {
  lead_a1::SharcPower power;
  SharcPdap pdap(power);
  configure(pdap, enable | byte_mask);
  sample(pdap, bytes[0] << 12);
  sample(pdap, bytes[1] << 12);
  const auto halfway = pdap.snapshot();
  SharcPdap restored(power, halfway);
  for (unsigned int index = 2; index < bytes.size(); ++index) {
    sample(pdap, bytes[index] << 12);
    sample(restored, bytes[index] << 12);
  }
  require(pdap.snapshot() == restored.snapshot());
  const auto full = pdap.snapshot();
  pdap.begin_instruction();
  require(read(pdap, fifo) == packed_bytes && pdap.state().count == 0);
  require(pdap.interrupts().threshold); // Committed state, no tentative IRQ changes.
  rejected([&] { pdap.begin_instruction(); }, "Nested");
  rejected([&] { pdap.observe(SharcPdapPort::dai, {}); }, "active instruction");
  rejected([&] { pdap.advance_cclk(1); }, "active instruction");
  rejected([&] { pdap.advance_sclk(1); }, "active instruction");
  rejected([&] { pdap.acknowledge_overflow(); }, "active instruction");
  rejected([&] { static_cast<void>(pdap.snapshot()); }, "active instruction");
  pdap.abort_instruction();
  require(pdap.snapshot() == full);
  pdap.begin_instruction();
  require(read(pdap, fifo) == packed_bytes);
  pdap.commit_instruction();
  require(!pdap.interrupts().threshold);
}

void malformed_state_and_accesses() {
  lead_a1::SharcPower power;
  SharcPdap pdap(power);
  const auto before = pdap.snapshot();
  rejected([&] { static_cast<void>(pdap.read({.size = 2})); }, "aligned32-bit");
  rejected([&] { pdap.write({.address = 1, .size = 4}, 0); }, "aligned32-bit");
  constexpr std::uint64_t oversized = 0x100000000;
  rejected([&] { pdap.write({.size = 4}, oversized); }, "Oversized");
  constexpr std::uint32_t stat1 = 0x28;
  rejected([&] { static_cast<void>(read(pdap, stat1)); }, "Unimplemented");
  rejected([&] { pdap.write({.address = status, .size = 4}, 0); }, "Read-only");
  constexpr std::uint32_t dma = 0x20;
  rejected([&] { write(pdap, 0, dma); }, "DMA/ping-pong");
  constexpr std::uint32_t ping = 0x10000;
  rejected([&] { write(pdap, 2, ping); }, "DMA/ping-pong");
  rejected([&] { pdap.observe(SharcPdapPort::dai, {.data = 1}); }, "input knowledge");
  constexpr std::uint32_t high_pin = 0x100000;
  rejected([&] { pdap.observe(SharcPdapPort::dai, {.known = high_pin}); }, "input knowledge");
  require(pdap.snapshot() == before);
  auto invalid = before;
  const auto construct = [&] { const SharcPdap other(power, invalid); };
  invalid.count = 9;
  rejected(construct, "FIFO/clock snapshot");
  invalid = before;
  invalid.head = 8;
  rejected(construct, "FIFO/clock snapshot");
  invalid = before;
  invalid.fifo[0] = 1;
  rejected(construct, "unoccupied FIFO");
  invalid = before;
  invalid.control[1] = reset;
  rejected(construct, "command bits");
  invalid = before;
  invalid.partial = 1;
  rejected(construct, "partial packing");
  invalid = before;
  invalid.startup_sclk = 1;
  rejected(construct, "FIFO/clock snapshot");
  invalid = before;
  invalid.flush_cclk = 3;
  rejected(construct, "FIFO/clock snapshot");
  invalid = before;
  invalid.count = 1; // A globally disabled receiver cannot invent a queued word.
  rejected(construct, "FIFO/clock snapshot");
  configure(pdap, enable | byte_mask);
  sample(pdap, bytes[0] << 12);
  invalid = pdap.snapshot();
  invalid.inputs[0].clock = std::nullopt;
  rejected(construct, "clock history");
}

void guarded_reconfiguration_and_hangs() {
  lead_a1::SharcPower power;
  SharcPdap pdap(power);
  configure(pdap, enable | byte_mask);
  rejected([&] { static_cast<void>(read(pdap, fifo)); }, "empty-FIFO core hang");
  constexpr std::uint32_t bhd = 0x10;
  write(pdap, 0, global_enable | bhd);
  rejected([&] { static_cast<void>(read(pdap, fifo)); }, "BHD empty-FIFO");
  sample(pdap, bytes[0] << 12);
  const auto partial = pdap.snapshot();
  rejected([&] { write(pdap, 1, enable | ami | byte_mask); }, "partial packing");
  rejected([&] { write(pdap, 0, 0); }, "partial packing");
  require(pdap.snapshot() == partial);
  write(pdap, 1, enable | byte_mask | reset);
  for (unsigned int index = 0; index < 8; ++index) {
    pdap.write({.address = fifo, .size = 4}, packed_bytes + index);
  }
  rejected([&] { pdap.write({.address = fifo, .size = 4}, 0); }, "BHD full-FIFO");
  write(pdap, 0, global_enable);
  rejected([&] { pdap.write({.address = fifo, .size = 4}, 0); }, "full-FIFO core hang");
  for (unsigned int index = 0; index < 8; ++index) {
    require(read(pdap, fifo) == packed_bytes + index);
  }
  constexpr std::uint32_t stop_dai = 0x10;
  power.write({.size = 4}, stop_dai);
  power.advance_pclk(2);
  rejected([&] { static_cast<void>(read(pdap, 0)); }, "stopped DAI clock");
  rejected([&] { pdap.advance_sclk(1); }, "stopped DAI clock");
}

struct FaultedReceiver final : lead_a1::MemoryDevice {
  SharcPdap &receiver;
  bool fail{true};

  explicit FaultedReceiver(SharcPdap &pdap) : receiver(pdap) {}

  std::uint64_t read(lead_a1::MemoryAccess access) override {
    const auto value = receiver.read(access);
    if (fail) { throw std::runtime_error("owned fault after actual PDAP FIFO pop"); }
    return value;
  }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    receiver.write(access, value);
  }

  void begin_instruction() override { receiver.begin_instruction(); }

  void commit_instruction() noexcept override { receiver.commit_instruction(); }

  void abort_instruction() noexcept override { receiver.abort_instruction(); }
};

void real_core_delivery_and_cached_pop_rollback() {
  lead_a1::SharcPower power;
  SharcPdap pdap(power);
  FaultedReceiver device(pdap);
  lead_a1::SharcMemory peripheral;
  std::optional<lead_a1::SharcIopBridge> bridge;
  lead_a1::SharcMemory memory;
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  constexpr unsigned int mode1 = 2;
  constexpr std::uint32_t bank_selection = 0x400;
  state.effective[mode1].known &= ~bank_selection; // Destination fails after real bus read.
  lead_a1::Sharc cpu(memory, state);
  constexpr unsigned int register_span = 0x21;
  peripheral.map_io({.first_word = pdap_word, .words = register_span}, device);
  constexpr lead_a1::SharcIoMapping window{.first_word = 0x2400, .words = 0x100};
  auto &bus = bridge.emplace(memory, peripheral, window);
  memory.map_io(window, bus);
  memory.map({.bank = 0, .first_halfword = 0, .halfwords = 128, .name = "owned PDAP load"});
  constexpr std::uint64_t load_r1 = 0x1001000024d0;
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, load_r1);
  configure(pdap, enable | byte_mask);
  byte_word(pdap);
  const auto full = pdap.snapshot();
  bus.issue({.pc = base,
             .address = fifo_word,
             .operation = lead_a1::AccessKind::read,
             .delivery_cycles = 8});
  for (unsigned int cycle = 0; cycle < 7; ++cycle) { bus.tick(); }
  rejected([&] { bus.tick(); }, "owned fault after actual PDAP FIFO pop");
  require(pdap.snapshot() == full && bus.state().clocks == 8 && cpu.state() == state);
  device.fail = false;
  bus.retry_due();
  require(bus.ready() && pdap.state().count == 0 && !pdap.interrupts().threshold);
  const auto response = bus.snapshot();
  rejected([&] { static_cast<void>(cpu.step()); }, "Unknown SHARC register bank selection");
  require(bus.snapshot() == response && pdap.state().count == 0 && cpu.state() == state);
  state.effective[mode1].known |= bank_selection;
  lead_a1::Sharc repaired(memory, state);
  require(repaired.step().next_pc == base + 1);
  require((repaired.state().data[0][1].value >> 8) == packed_bytes && pdap.state().count == 0);
}

}

int main() try {
  reset_and_enable_requirements();
  independent_packing_expectations();
  held_unknown_and_masked_inputs();
  input_selection_and_clock_history();
  fifo_order_overflow_and_interrupts();
  commands_and_delayed_disable();
  restore_partial_and_ownership();
  malformed_state_and_accesses();
  guarded_reconfiguration_and_hangs();
  real_core_delivery_and_cached_pop_rollback();
  std::cout << "PDAP real acquisition, packing, FIFO, interrupts, reset and rollback pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
