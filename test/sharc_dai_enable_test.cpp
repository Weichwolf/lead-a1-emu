// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_dai_enable.hpp"
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

using lead_a1::SharcDaiEnable;
using lead_a1::SharcPower;

constexpr unsigned int source_count = 48;
constexpr std::uint32_t all_high = 0x01041041;
constexpr std::uint32_t mixed = 0xc8bc2040;
constexpr std::array<std::uint32_t, 4> reset{0x0e2482ca, 0x1348d30f, 0x1a5545d6, 0x1d71f79b};
// Independent HWR B-15/A-147..150 field identities. SPORT2/4 clock/FS do
// not receive invented extra pins from an assumed uniform four-pin pattern.
constexpr std::array<unsigned int, 20> default_sources{10, 11, 8,  9,  14, 15, 12, 13, 18, 19,
                                                       22, 23, 20, 21, 26, 27, 30, 31, 28, 29};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("DAI enable expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing DAI enable rejection: " + reason);
}

void write(SharcDaiEnable &dai, unsigned int index, std::uint32_t value) {
  dai.write({.address = index * 4, .size = 4}, value);
}

std::uint32_t read(SharcDaiEnable &dai, unsigned int index) {
  return static_cast<std::uint32_t>(dai.read({.address = index * 4, .size = 4}));
}

void dynamic_low(SharcDaiEnable &dai) {
  for (unsigned int source = 2; source < source_count; ++source) { dai.source(source, false); }
}

void reset_and_live_sources() {
  SharcPower power;
  SharcDaiEnable dai(power);
  for (unsigned int index = 0; index < reset.size(); ++index) {
    require(read(dai, index) == reset[index]);
  }
  for (const auto output : dai.outputs()) { require(!output.has_value()); }
  dynamic_low(dai);
  for (unsigned int source = 2; source < source_count; ++source) {
    dai.source(source, true);
    const auto outputs = dai.outputs();
    for (unsigned int pin = 0; pin < outputs.size(); ++pin) {
      require(outputs[pin] == (default_sources[pin] == source));
    }
    dai.source(source, std::nullopt);
    const auto unknown = dai.outputs();
    for (unsigned int pin = 0; pin < unknown.size(); ++pin) {
      const auto expected =
          default_sources[pin] == source ? std::optional<bool>{} : std::optional<bool>{false};
      require(unknown[pin] == expected);
    }
    dai.source(source, false);
  }
  require(dai.snapshot().routes == reset);
}

void prefix(const SharcDaiEnable &dai, const std::array<std::optional<bool>, 5> &expected) {
  const auto outputs = dai.outputs();
  for (unsigned int pin = 0; pin < expected.size(); ++pin) {
    require(outputs[pin] == expected[pin]);
  }
}

void latency_and_ambiguous_interval() {
  SharcPower power;
  SharcDaiEnable dai(power);
  dynamic_low(dai);
  dai.source(2, true);                        // MISCA0.
  dai.source(8, true);                        // SPORT0_CLK_PBEN_O.
  dai.source(source_count - 1, std::nullopt); // SPORT7_TDV_PBEN_O is not supplied.
  // Five fields LOW,HIGH,MISCA0,SPORT7_TDV,SPORT0_CLK; reserved31/30 stay raw.
  write(dai, 0, mixed);
  const auto pending = dai.snapshot();
  prefix(dai, {false, false, true, false, false});
  rejected([&] { static_cast<void>(read(dai, 0)); }, "unsettled readback");
  rejected([&] { write(dai, 0, all_high); }, "overlapping write");
  dai.advance_pclk(0);
  require(dai.snapshot() == pending);
  dai.advance_pclk(1);
  prefix(dai, {false, false, true, false, false});
  dai.advance_pclk(1);
  prefix(dai, {false, std::nullopt, true, std::nullopt, std::nullopt});
  const auto interval = dai.snapshot();
  SharcPower restored_power;
  SharcDaiEnable restored(restored_power, interval);
  prefix(restored, {false, std::nullopt, true, std::nullopt, std::nullopt});
  restored.advance_pclk(1);
  prefix(restored, {false, true, true, std::nullopt, true});
  require(read(restored, 0) == mixed && dai.snapshot() == interval);
  dai.advance_pclk(1);
  require(dai.snapshot() == restored.snapshot());
  const auto settled = dai.snapshot();
  dai.advance_pclk(std::numeric_limits<std::uint32_t>::max());
  require(dai.snapshot() == settled);
  dai.source(source_count - 1, true);
  prefix(dai, {false, true, true, true, true});
  dai.source(2, std::nullopt);
  prefix(dai, {false, true, std::nullopt, true, true});
}

void half_period_intervals_and_restore() {
  SharcPower power;
  SharcDaiEnable dai(power);
  dynamic_low(dai);
  write(dai, 0, all_high);
  dai.advance_half_pclk(1);
  const auto half = dai.snapshot();
  const auto pending = half.pending[0];
  require(pending && pending->half_elapsed);
  SharcDaiEnable restored(power, half);
  restored.advance_pclk(1); // 1.5 periods: old output remains known low.
  prefix(restored, {false, false, false, false, false});
  restored.advance_half_pclk(1); // Two periods: earliest possible effect.
  prefix(restored, {std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt});
  restored.advance_half_pclk(1); // 2.5 periods: latest bound is still pending.
  prefix(restored, {std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt});
  restored.advance_half_pclk(1);
  prefix(restored, {true, true, true, true, true});
  require(read(restored, 0) == all_high);
  dai.advance_half_pclk(5);
  require(dai.snapshot() == restored.snapshot());
  write(dai, 0, 0);
  dai.advance_half_pclk(std::numeric_limits<std::uint32_t>::max());
  prefix(dai, {false, false, false, false, false});
  dai.begin_instruction();
  rejected([&] { dai.advance_half_pclk(1); }, "active instruction");
  dai.abort_instruction();
}

void all_source_selections() {
  SharcPower power;
  SharcDaiEnable dai(power);
  dynamic_low(dai);
  // An owned six-bit value repeated in five fields; expected levels come
  // from independent constant/external inputs, never a decoder result.
  constexpr std::uint32_t repeat_field = 0x01041041;
  for (unsigned int encoding = 0; encoding < source_count; ++encoding) {
    const auto word = static_cast<std::uint32_t>(encoding) * repeat_field;
    if (encoding >= 2) { dai.source(encoding, true); }
    for (unsigned int index = 0; index < 4; ++index) { write(dai, index, word); }
    dai.advance_pclk(3);
    for (const auto output : dai.outputs()) { require(output == (encoding != 0)); }
    for (unsigned int index = 0; index < 4; ++index) { require(read(dai, index) == word); }
    if (encoding >= 2) {
      dai.source(encoding, std::nullopt);
      for (const auto output : dai.outputs()) { require(!output.has_value()); }
      dai.source(encoding, false);
      for (const auto output : dai.outputs()) { require(output == false); }
    }
  }
}

void independent_words_and_ownership() {
  SharcPower power;
  SharcDaiEnable dai(power);
  dynamic_low(dai);
  const auto before = dai.snapshot();
  dai.begin_instruction();
  write(dai, 0, all_high);
  rejected([&] { dai.begin_instruction(); }, "Nested");
  rejected([&] { dai.advance_pclk(3); }, "active instruction");
  rejected([&] { dai.source(2, true); }, "active instruction");
  rejected([&] { static_cast<void>(dai.snapshot()); }, "active instruction");
  prefix(dai, {false, false, false, false, false}); // Tentative routes do not leak.
  dai.abort_instruction();
  require(dai.snapshot() == before);
  dai.begin_instruction();
  write(dai, 0, all_high);
  dai.commit_instruction();
  dai.advance_pclk(1);
  write(dai, 2, all_high);
  dai.advance_pclk(2);
  prefix(dai, {true, true, true, true, true});
  require(read(dai, 0) == all_high && read(dai, 1) == reset[1] && read(dai, 3) == reset[3]);
  rejected([&] { static_cast<void>(read(dai, 2)); }, "unsettled readback");
  dai.advance_pclk(1);
  for (unsigned int pin = 10; pin < 15; ++pin) { require(dai.outputs()[pin] == true); }
  require(read(dai, 2) == all_high);
}

void malformed_and_gated_inputs() {
  SharcPower power;
  SharcDaiEnable dai(power);
  const auto before = dai.snapshot();
  rejected([&] { dai.write({.address = 16, .size = 4}, 0); }, "aligned32-bit");
  rejected([&] { static_cast<void>(dai.read({.address = 2, .size = 4})); }, "aligned32-bit");
  rejected([&] { static_cast<void>(dai.read({.size = 2})); }, "aligned32-bit");
  constexpr std::uint64_t oversized = 0x100000000;
  rejected([&] { dai.write({.size = 4}, oversized); }, "Oversized");
  constexpr std::uint32_t reserved_source = 0x30000000;
  rejected([&] { write(dai, 0, reserved_source); }, "Reserved");
  rejected([&] { dai.source(0, true); }, "dynamic source");
  rejected([&] { dai.source(1, false); }, "dynamic source");
  rejected([&] { dai.source(source_count, false); }, "dynamic source");
  require(dai.snapshot() == before);
  auto invalid = before;
  invalid.routes[3] = reserved_source;
  rejected([&] { const SharcDaiEnable other(power, invalid); }, "Reserved");
  invalid = before;
  invalid.pending[0] = lead_a1::SharcDaiEnablePending{.target = reserved_source};
  rejected([&] { const SharcDaiEnable other(power, invalid); }, "Reserved");
  invalid.pending[0] = lead_a1::SharcDaiEnablePending{.elapsed_pclk = 3};
  rejected([&] { const SharcDaiEnable other(power, invalid); }, "pending interval");
  constexpr std::uint32_t dai_off = 0x10;
  power.write({.size = 4}, dai_off);
  power.advance_pclk(2);
  rejected([&] { static_cast<void>(dai.outputs()); }, "stopped DAI clock");
  rejected([&] { static_cast<void>(read(dai, 0)); }, "stopped DAI clock");
  rejected([&] { write(dai, 0, all_high); }, "stopped DAI clock");
  rejected([&] { dai.advance_pclk(3); }, "stopped DAI clock");
  require(dai.snapshot() == before);
}

struct FaultingDai final : lead_a1::MemoryDevice {
  SharcDaiEnable &dai;
  bool fail{true};

  explicit FaultingDai(SharcDaiEnable &bound) : dai(bound) {}

  std::uint64_t read(lead_a1::MemoryAccess access) override { return dai.read(access); }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    dai.write(access, value);
    if (fail) { throw std::runtime_error("owned late routing delivery fault"); }
  }

  void begin_instruction() override { dai.begin_instruction(); }

  void commit_instruction() noexcept override { dai.commit_instruction(); }

  void abort_instruction() noexcept override { dai.abort_instruction(); }
};

void actual_cpu_store_rollback() {
  SharcPower power;
  SharcDaiEnable dai(power);
  FaultingDai delivery(dai);
  lead_a1::SharcMemory memory;
  constexpr unsigned int cells = 256;
  constexpr std::uint32_t base = 0x80000;
  constexpr std::uint32_t first_register = 0x2478;
  constexpr std::uint64_t full40 = 0xffffffffff;
  constexpr std::uint64_t direct_store = 0x11060000247a;
  memory.map({.bank = 0, .halfwords = cells, .name = "owned native routing program"});
  memory.map_io({.first_word = first_register, .words = 4}, delivery);
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, direct_store);
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.data[0][6] = {.value = std::uint64_t{all_high} << 8, .known = full40};
  lead_a1::Sharc cpu(memory, state);
  const auto before = dai.snapshot();
  rejected([&] { static_cast<void>(cpu.step()); }, "owned late routing delivery fault");
  require(cpu.state() == state && dai.snapshot() == before && !memory.active());
  delivery.fail = false;
  require(cpu.step().next_pc == base + 1);
  rejected([&] { static_cast<void>(read(dai, 2)); }, "unsettled readback");
  dai.advance_pclk(2);
  for (const auto output : dai.outputs()) { require(!output.has_value()); }
  dai.advance_pclk(1);
  for (unsigned int pin = 10; pin < 15; ++pin) { require(dai.outputs()[pin] == true); }
  require(read(dai, 2) == all_high);
}

}

int main() try {
  reset_and_live_sources();
  latency_and_ambiguous_interval();
  half_period_intervals_and_restore();
  all_source_selections();
  independent_words_and_ownership();
  malformed_and_gated_inputs();
  actual_cpu_store_rollback();
  std::cout << "DAI enable selectors, live unknown signals, latency bounds and rollback pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
