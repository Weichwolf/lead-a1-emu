// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_clock.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t bypass = 0x8000;
constexpr std::uint32_t diven = 0x200;
constexpr std::uint32_t divide_input = 0x100;
constexpr std::uint32_t pll_wait = 4096;
constexpr std::uint32_t divider_wait = 14;
constexpr std::uint32_t cfg0_multiplier = 8;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC clock expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC clock rejection: " + reason);
}

void write(lead_a1::SharcClock &clock, std::uint32_t value) {
  clock.write({.address = 0, .size = 4}, value);
}

std::uint32_t read(lead_a1::SharcClock &clock) {
  return static_cast<std::uint32_t>(clock.read({.address = 0, .size = 4}));
}

void reset_and_unknowns() {
  constexpr std::array<std::uint32_t, 3> values{8, 0x10020, 0x20010};
  constexpr std::array<std::uint32_t, 3> ratios{8, 32, 16};
  for (unsigned int cfg = 0; cfg < values.size(); ++cfg) {
    lead_a1::SharcClock clock({.configuration = cfg, .input_stable = true, .reset_settled = true});
    require(read(clock) == values[cfg]);
    require((clock.rates().core == lead_a1::SharcClockRatio{.numerator = ratios[cfg]}));
    require((clock.rates().peripheral == lead_a1::SharcClockRatio{.numerator = ratios[cfg] / 2}));
    require((clock.rates().external == lead_a1::SharcClockRatio{.numerator = ratios[cfg] / 2}));
    const auto before = clock.state();
    static_cast<void>(read(clock));
    require(clock.state() == before);
  }
  lead_a1::SharcClock unknown({.configuration = {}, .input_stable = true, .reset_settled = true});
  const auto before = unknown.state();
  rejected([&] { static_cast<void>(read(unknown)); }, "Unknown SHARC CLK_CFG");
  require(unknown.state() == before && !unknown.rates().core);
  rejected([] { const lead_a1::SharcClock reserved({.configuration = 3}); }, "Invalid");
  rejected(
      [] {
        const lead_a1::SharcClock unstable(
            {.configuration = {}, .input_stable = false, .reset_settled = true});
      },
      "Invalid");
}

void divider_shadow_and_latency() {
  lead_a1::SharcClock clock({.configuration = 0, .input_stable = true, .reset_settled = true});
  constexpr std::uint32_t divide4 = 0x40;
  constexpr std::uint32_t divide8 = 0x80;
  write(clock, cfg0_multiplier | divide4);
  require(read(clock) == (cfg0_multiplier | divide4));
  require((clock.rates().core == lead_a1::SharcClockRatio{.numerator = 8}));
  write(clock, cfg0_multiplier | divide4 | diven);
  require((read(clock) & diven) == 0 && clock.state().divider_remaining == divider_wait);
  require(!clock.rates().core);
  const auto pending = clock.state();
  rejected([&] { write(clock, cfg0_multiplier | diven); }, "already pending");
  require(clock.state() == pending);
  clock.advance({.core_cycles = divider_wait - 1});
  require(!clock.rates().core && clock.state().divider_remaining == 1);
  // Shadow writes do not replace an already captured divider command.
  write(clock, cfg0_multiplier | divide8);
  clock.advance({.core_cycles = 1});
  require((clock.rates().core == lead_a1::SharcClockRatio{.numerator = 4}));
  write(clock, cfg0_multiplier | divide8 | diven);
  clock.advance({.core_cycles = divider_wait});
  require((clock.rates().core == lead_a1::SharcClockRatio{.numerator = 2}));
  constexpr std::uint32_t external_code1 = 0x40000;
  write(clock, cfg0_multiplier | external_code1 | diven);
  clock.advance({.core_cycles = divider_wait});
  require((clock.rates().core == lead_a1::SharcClockRatio{.numerator = 8}));
  require((clock.rates().external == lead_a1::SharcClockRatio{.numerator = 16, .denominator = 5}));
}

void pll_waits_and_ratios() {
  lead_a1::SharcClock clock({.configuration = 0, .input_stable = true, .reset_settled = true});
  constexpr std::uint32_t final_multiplier = 18;
  write(clock, bypass | final_multiplier);
  require((clock.rates().core == lead_a1::SharcClockRatio{.numerator = 1}));
  const auto pending = clock.state();
  rejected([&] { write(clock, final_multiplier); }, "before guaranteed CLKIN settling");
  require(clock.state() == pending);
  // Core clocks alone cannot count as stable CLKIN clocks.
  clock.advance({.core_cycles = pll_wait});
  require(clock.state().pll_remaining == pll_wait);
  clock.advance({.clkin_cycles = pll_wait - 1});
  rejected([&] { write(clock, final_multiplier); }, "before guaranteed CLKIN settling");
  clock.advance({.clkin_cycles = 1});
  write(clock, final_multiplier);
  require((clock.rates().core == lead_a1::SharcClockRatio{.numerator = final_multiplier}));
  constexpr unsigned int reentry = 6;
  clock.advance({.core_cycles = reentry - 1});
  rejected([&] { write(clock, bypass | final_multiplier); }, "six core clocks");
  clock.advance({.core_cycles = 1});
  constexpr std::uint32_t maximum_multiplier = 63;
  write(clock, bypass | maximum_multiplier | divide_input);
  clock.advance({.clkin_cycles = pll_wait});
  write(clock, maximum_multiplier | divide_input);
  require((clock.rates().core ==
           lead_a1::SharcClockRatio{.numerator = maximum_multiplier, .denominator = 2}));
  clock.advance({.core_cycles = reentry});
  write(clock, bypass); // PLLM0 encodes128, not zero or a stopped clock.
  clock.advance({.clkin_cycles = pll_wait});
  write(clock, 0);
  require((clock.rates().core == lead_a1::SharcClockRatio{.numerator = 64}));
}

void normal_vco_change_and_reset_delay() {
  lead_a1::SharcClock clock({.configuration = 0, .input_stable = true, .reset_settled = true});
  constexpr std::uint32_t multiplier16 = 16;
  // HWR23-14 method1 permits programming VCO before entering bypass.
  write(clock, multiplier16 | diven);
  clock.advance({.core_cycles = pll_wait, .clkin_cycles = pll_wait});
  require(!clock.rates().core && clock.state().bypass_required);
  write(clock, multiplier16 | bypass);
  require(!clock.state().bypass_required && clock.state().pll_remaining == pll_wait);
  clock.advance({.clkin_cycles = pll_wait});
  write(clock, multiplier16);
  require((clock.rates().core == lead_a1::SharcClockRatio{.numerator = multiplier16}));
  lead_a1::SharcClock reset({.configuration = 0, .input_stable = true, .reset_settled = false});
  require(!reset.rates().core);
  reset.advance({.clkin_cycles = pll_wait - 1});
  require(!reset.rates().core && !reset.state().reset_settled);
  reset.advance({.clkin_cycles = 1});
  require((reset.rates().core == lead_a1::SharcClockRatio{.numerator = 8}));
  reset.input_stable(false);
  reset.advance({.clkin_cycles = pll_wait});
  require(!reset.rates().core && reset.state().pll_remaining == pll_wait);
  reset.input_stable(true);
  reset.advance({.clkin_cycles = pll_wait});
  require((reset.rates().core == lead_a1::SharcClockRatio{.numerator = 8}));
}

void masks_transactions_and_cpu_failure() {
  lead_a1::SharcClock clock({.configuration = 0, .input_stable = true, .reset_settled = true});
  constexpr std::uint32_t other_straps = 0x30000;
  write(clock, cfg0_multiplier | other_straps);
  require(read(clock) == cfg0_multiplier);
  const auto before = clock.state();
  clock.begin_instruction();
  write(clock, cfg0_multiplier | diven);
  rejected([&] { clock.advance({.core_cycles = 1}); }, "active instruction");
  rejected([&] { clock.input_stable(false); }, "active instruction");
  clock.abort_instruction();
  require(clock.state() == before);
  clock.begin_instruction();
  write(clock, cfg0_multiplier | diven);
  clock.commit_instruction();
  require(clock.state().divider_remaining == divider_wait);
  constexpr std::uint32_t invalid_external = 0x140000;
  lead_a1::SharcClock clean({.configuration = 0, .input_stable = true, .reset_settled = true});
  rejected([&] { write(clean, cfg0_multiplier | bypass | diven); }, "separately");
  rejected([&] { write(clean, cfg0_multiplier | invalid_external | diven); }, "Reserved");
  rejected([&] { clean.write({.size = 2}, 0); }, "32-bit");
  rejected([&] { clean.write({.size = 4}, std::numeric_limits<std::uint64_t>::max()); },
           "Oversized");
  // A native register-read failure does not qualify unknown external strap pins.
  lead_a1::SharcClock unknown({.configuration = {}, .input_stable = true, .reset_settled = true});
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  constexpr std::uint32_t pc = 0xc0000;
  constexpr std::uint64_t read_clock = 0x107100002000;
  memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, read_clock);
  constexpr std::uint32_t pmctl = 0x2000;
  memory.map_io({.first_word = pmctl, .words = 1}, unknown);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(pc));
  const auto cpu_before = cpu.state();
  const auto device_before = unknown.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Unknown SHARC CLK_CFG");
  require(cpu.state() == cpu_before && unknown.state() == device_before && !memory.active());
}

}

int main() {
  try {
    reset_and_unknowns();
    divider_shadow_and_latency();
    pll_waits_and_ratios();
    normal_vco_change_and_reset_delay();
    masks_transactions_and_cpu_failure();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
