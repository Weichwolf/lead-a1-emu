// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_pcg.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using Input = lead_a1::SharcPcgInput;
constexpr std::uint32_t enable_clock = 0x80000000;
constexpr std::uint32_t enable_both = 0xc0000000;
constexpr unsigned int inputs = 14;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("PCG expectation line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw;
  }
  throw std::runtime_error("Missing PCG rejection: " + reason);
}

lead_a1::SharcPcgState known_inputs() {
  lead_a1::SharcPcgState initial;
  initial.inputs.fill(false); // Explicit owned source levels, not a board default.
  return initial;
}

struct Fixture {
  lead_a1::SharcPower power;
  lead_a1::SharcPcg pcg;

  explicit Fixture(lead_a1::SharcPcgState initial = known_inputs()) : pcg(power, initial) {}

  void write(unsigned int word, std::uint32_t value) {
    pcg.write({.address = word * 4, .size = 4}, value);
  }

  void cycle(Input input) {
    pcg.observe(input, true);
    pcg.observe(input, false);
  }

  void cycles(Input input, unsigned int count) {
    for (unsigned int index = 0; index < count; ++index) { cycle(input); }
  }
};

void source_selected_clock(unsigned int unit,
                           Input input,
                           unsigned int divisor,
                           std::string_view pattern) {
  Fixture f;
  constexpr std::array<unsigned int, 4> controls{0, 2, 6, 8};
  constexpr std::array<unsigned int, 4> synchronizers{5, 5, 11, 11};
  const auto word = controls[unit];
  constexpr std::uint32_t external = 0x80000000;
  f.write(word + 1, divisor | external); // PCLK override must beat EXT selection.
  if (input == Input::pclk) { f.write(synchronizers[unit], unit % 2 == 0 ? 4U : 0x40000U); }
  if (input == Input::clkin) { f.write(word + 1, divisor); }
  f.write(word, enable_clock);
  const auto before = f.pcg.snapshot();
  require(f.pcg.read({.address = word * 4, .size = 4}) == enable_clock);
  require(f.pcg.snapshot() == before && !f.pcg.signals(unit).clock);
  const auto unrelated = input == Input::pclk ? Input::clkin : Input::pclk;
  f.cycle(unrelated);
  require(f.pcg.snapshot().lanes[unit][0].startup == (divisor > 1 ? 3U : 1U));
  if (divisor > 1) {
    f.cycle(input);
    f.cycle(input);
    require(!f.pcg.signals(unit).clock);
  }
  for (const auto bit : pattern) {
    f.pcg.observe(input, true);
    require(f.pcg.signals(unit).clock == (bit == '1'));
    f.pcg.observe(input, false);
    require(f.pcg.signals(unit).clock == (divisor > 1 && bit == '1'));
  }
}

void literal_clock_matrix() {
  struct Case {
    unsigned int divisor;
    std::string_view bits;
  };

  constexpr std::array cases{Case{.divisor = 0, .bits = "11111111"},
                             Case{.divisor = 1, .bits = "11111111"},
                             Case{.divisor = 2, .bits = "10101010"},
                             Case{.divisor = 3, .bits = "100100100"},
                             Case{.divisor = 4, .bits = "11001100"},
                             Case{.divisor = 5, .bits = "1100011000"}};
  constexpr std::array external{
      Input::external_a, Input::external_b, Input::external_c, Input::external_d};
  for (unsigned int unit = 0; unit < external.size(); ++unit) {
    for (const auto &word : cases) {
      for (const auto input : {Input::clkin, Input::pclk, external[unit]}) {
        source_selected_clock(unit, input, word.divisor, word.bits);
      }
    }
  }
}

void frame_waveforms_and_phase() {
  struct Case {
    unsigned int divisor;
    unsigned int width;
    std::string_view bits;
  };

  constexpr std::array cases{
      Case{.divisor = 5, .width = 0, .bits = "1100011000"},
      Case{.divisor = 7, .width = 0, .bits = "11100001110000"},
      Case{.divisor = 7, .width = 1, .bits = "10000001000000"},
      Case{.divisor = 16, .width = 8, .bits = "11111111000000001111111100000000"}};
  for (const auto &word : cases) {
    Fixture f;
    f.write(1, 4);
    f.write(4, word.width);
    f.write(0, enable_both | word.divisor);
    f.cycle(Input::clkin);
    f.cycle(Input::clkin);
    for (const auto bit : word.bits) {
      f.pcg.observe(Input::clkin, true);
      require(f.pcg.signals(0).frame == (bit == '1'));
      f.pcg.observe(Input::clkin, false);
    }
  }
  Fixture f;
  constexpr unsigned int delayed_frame = 16;
  constexpr unsigned int phase2_clock4 = 0x00200004;
  f.write(1, phase2_clock4);
  f.write(4, 8);
  f.write(0, enable_both | delayed_frame);
  f.cycle(Input::clkin);
  f.cycle(Input::clkin);
  constexpr unsigned int first_complete = 14;
  for (unsigned int count = 0; count < first_complete; ++count) {
    f.cycle(Input::clkin);
    require(!f.pcg.signals(0).frame); // Unqualified truncated first pulse stays unknown.
  }
  for (const auto bit : std::string_view("11111111000000001111111100000000")) {
    f.pcg.observe(Input::clkin, true);
    require(f.pcg.signals(0).frame == (bit == '1'));
    f.pcg.observe(Input::clkin, false);
  }
}

void bypass_frame_and_missing_clock() {
  for (const unsigned int divisor : {0U, 1U}) {
    for (const unsigned int inversion : {0U, 2U}) {
      Fixture f;
      f.write(1, divisor);
      f.write(4, inversion);
      f.write(0, enable_both | divisor);
      require(!f.pcg.signals(0).clock && !f.pcg.signals(0).frame);
      f.pcg.observe(Input::clkin, true);
      require(f.pcg.signals(0).clock == true && f.pcg.signals(0).frame == (inversion == 0));
      f.pcg.observe(Input::clkin, false);
      require(f.pcg.signals(0).clock == false && f.pcg.signals(0).frame == (inversion != 0));
    }
  }
  const lead_a1::SharcPcgState unknown; // No source assumptions.
  Fixture f(unknown);
  f.write(1, 4);
  f.write(0, enable_clock);
  for (unsigned int period = 0; period < inputs; ++period) { f.cycle(Input::pclk); }
  require(!f.pcg.signals(0).clock && f.pcg.snapshot().lanes[0][0].lost_phase);
  f.pcg.observe(Input::clkin, false);
  f.cycle(Input::clkin);
  require(!f.pcg.signals(0).clock); // Cannot reconstruct lost divided-clock phase.
}

void split_sources_and_frame_banks() {
  constexpr std::array<unsigned int, 4> controls{0, 2, 6, 8};
  constexpr std::array<unsigned int, 4> synchronizers{5, 5, 11, 11};
  constexpr std::array external{
      Input::external_a, Input::external_b, Input::external_c, Input::external_d};
  for (unsigned int unit = 0; unit < controls.size(); ++unit) {
    Fixture f;
    const auto word = controls[unit];
    constexpr unsigned int external_frame_clock4 = 0x40000004;
    f.write(word + 1, external_frame_clock4);
    f.write(synchronizers[unit], unit % 2 == 0 ? 4U : 0x40000U);
    f.write(synchronizers[unit] - 1, unit % 2 == 0 ? 1U : 0x10000U);
    f.write(word, enable_both | 7U);
    f.cycles(Input::pclk, 3);
    require(f.pcg.signals(unit).clock == true && !f.pcg.signals(unit).frame);
    f.cycles(external[unit], 2);
    for (const auto bit : std::string_view("10000001000000")) {
      f.cycle(external[unit]);
      require(f.pcg.signals(unit).frame == (bit == '1') && f.pcg.signals(unit).clock == true);
    }
    f.cycles(Input::pclk, 2);
    require(f.pcg.signals(unit).clock == false && f.pcg.signals(unit).frame == false);
  }
}

void wide_dividers_and_phase_fields() {
  Fixture f;
  constexpr unsigned int maximum_divider = 0xfffff;
  f.write(1, maximum_divider);
  f.write(0, enable_clock);
  f.cycles(Input::clkin, 3);
  require(f.pcg.signals(0).clock == true);
  constexpr unsigned int remaining_high = 524286;
  f.cycles(Input::clkin, remaining_high);
  require(f.pcg.signals(0).clock == true);
  f.cycle(Input::clkin);
  require(f.pcg.signals(0).clock == false);
  constexpr unsigned int remaining_low = 524287;
  f.cycles(Input::clkin, remaining_low);
  require(f.pcg.signals(0).clock == false);
  f.cycle(Input::clkin);
  require(f.pcg.signals(0).clock == true);

  Fixture phase;
  constexpr unsigned int low_phase2_clock4 = 0x00200004;
  constexpr unsigned int high_phase1_frame2048 = 0x00100800;
  phase.write(1, low_phase2_clock4);
  phase.write(4, 1);
  phase.write(0, enable_both | high_phase1_frame2048);
  phase.cycles(Input::clkin, 2);
  constexpr unsigned int first_partial_frame = 1022;
  phase.cycles(Input::clkin, first_partial_frame);
  require(!phase.pcg.signals(0).frame);
  phase.cycle(Input::clkin);
  require(phase.pcg.signals(0).frame == true && phase.pcg.signals(0).clock == false);
  phase.cycle(Input::clkin);
  require(phase.pcg.signals(0).frame == false);
}

void lost_running_phase_and_paired_disable() {
  Fixture f;
  f.write(1, 4);
  f.write(0, enable_both | 16U);
  f.cycles(Input::clkin, 3);
  require(f.pcg.signals(0).clock == true && f.pcg.signals(0).frame == true);
  const auto paired = f.pcg.snapshot();
  rejected([&] { f.write(0, enable_clock | 16U); }, "both enable bits together");
  require(f.pcg.snapshot() == paired);
  f.pcg.observe(Input::clkin, std::nullopt);
  Fixture lost(f.pcg.snapshot());
  lost.pcg.observe(Input::clkin, false);
  constexpr unsigned int subsequent_source_cycles = 20;
  lost.cycles(Input::clkin, subsequent_source_cycles);
  require(!lost.pcg.signals(0).clock && !lost.pcg.signals(0).frame);
  lost.write(0, 16);
  lost.pcg.advance_half_periods(Input::clkin, 2);
  lost.write(0, enable_both | 16U);
  lost.cycles(Input::clkin, 3);
  require(lost.pcg.signals(0).clock == true && lost.pcg.signals(0).frame == true);
}

void disable_elapsed_source_and_restores() {
  Fixture f;
  f.write(1, 4);
  f.write(0, enable_clock);
  f.cycle(Input::clkin);
  Fixture startup(f.pcg.snapshot());
  startup.cycle(Input::clkin);
  startup.pcg.observe(Input::clkin, true);
  require(startup.pcg.signals(0).clock == true && !f.pcg.signals(0).clock);
  Fixture running(startup.pcg.snapshot());
  running.pcg.observe(Input::clkin, false);
  running.cycle(Input::clkin);
  require(running.pcg.signals(0).clock == true);
  running.cycle(Input::clkin);
  require(running.pcg.signals(0).clock == false);
  f.write(0, 0);
  const auto waiting = f.pcg.snapshot();
  rejected([&] { f.write(1, 5); }, "disable settled");
  require(f.pcg.snapshot() == waiting);
  f.pcg.advance_half_periods(Input::pclk, 2);
  require(f.pcg.snapshot() == waiting);
  f.pcg.advance_half_periods(Input::clkin, 1);
  Fixture restored(f.pcg.snapshot());
  rejected([&] { restored.write(0, enable_clock); }, "reenable");
  restored.pcg.advance_half_periods(Input::clkin, 1);
  restored.write(1, 5);
  restored.write(0, enable_clock);
  restored.cycle(Input::clkin);
  restored.cycle(Input::clkin);
  restored.cycle(Input::clkin);
  require(restored.pcg.signals(0).clock == true &&
          f.pcg.snapshot().lanes[0][0].stop_half_periods == 1);
}

void invalid_modes_are_atomic() {
  Fixture f;
  f.write(1, 4);
  f.write(4, 8);
  f.write(0, enable_both | 4);
  const auto invalid = f.pcg.snapshot();
  rejected([&] { f.cycle(Input::clkin); }, "pulse/phase exceeds");
  require(f.pcg.snapshot() == invalid);
  Fixture triggered;
  triggered.write(5, 2);
  triggered.write(0, enable_clock);
  const auto pending = triggered.pcg.snapshot();
  rejected([&] { triggered.cycle(Input::clkin); }, "trigger phase");
  require(triggered.pcg.snapshot() == pending);
  Fixture strobe;
  strobe.write(4, 1);
  strobe.write(0, 0x40000000);
  rejected([&] { strobe.cycle(Input::clkin); }, "one-shot");
}

void transactions_shapes_and_gates() {
  Fixture f;
  const auto before = f.pcg.snapshot();
  f.pcg.begin_instruction();
  f.write(1, 4);
  f.write(0, enable_clock);
  require(f.pcg.read({.address = 0, .size = 4}) == enable_clock && !f.pcg.signals(0).clock);
  rejected([&] { f.cycle(Input::clkin); }, "active instruction");
  rejected([&] { static_cast<void>(f.pcg.snapshot()); }, "active instruction");
  f.pcg.abort_instruction();
  require(f.pcg.snapshot() == before);
  rejected([&] { f.pcg.write({.address = 0, .size = 4}, std::uint64_t{1} << 32); }, "Oversized");
  rejected([&] { static_cast<void>(f.pcg.read({.address = 1, .size = 4})); }, "aligned32");
  constexpr unsigned int outside = 48;
  rejected([&] { f.pcg.write({.address = outside, .size = 4}, 0); }, "aligned32");
  auto malformed = before;
  malformed.active = 1;
  rejected([&] { const Fixture invalid(malformed); }, "active owner mask");
  malformed = before;
  malformed.lanes[0][0].startup = 4;
  rejected([&] { const Fixture invalid(malformed); }, "Malformed PCG disabled lane phase");
  constexpr unsigned int stop_dai = 0x10;
  f.power.write({.address = 0, .size = 4}, stop_dai);
  f.power.advance_pclk(2);
  rejected([&] { f.cycle(Input::clkin); }, "stopped DAI");
  require(f.pcg.snapshot() == before);
}
}

int main() try {
  literal_clock_matrix();
  frame_waveforms_and_phase();
  bypass_frame_and_missing_clock();
  split_sources_and_frame_banks();
  wide_dividers_and_phase_fields();
  lost_running_phase_and_paired_disable();
  disable_elapsed_source_and_restores();
  invalid_modes_are_atomic();
  transactions_shapes_and_gates();
  std::cout << "PCG literal source/divider/frame/phase/restore/transaction expectations pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
