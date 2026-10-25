// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/peripherals/sharc_dai_enable.hpp"
#include "sharc/peripherals/sharc_dai_pin.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <algorithm>
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

using Router = lead_a1::SharcDaiPin;
using State = lead_a1::SharcDaiPinState;
constexpr std::uint32_t low_word = 0x0fdfbf7e;
constexpr std::uint32_t high_word = 0x0fffffff;
constexpr std::uint32_t first_high = 0x0fdfbf7f;
constexpr std::uint32_t first_input_one = 0x0fdfbf01;
constexpr unsigned int pin_count = 20;
constexpr unsigned int low_source = 126;
constexpr unsigned int high_source = 127;
constexpr unsigned int pin18 = 17;
constexpr unsigned int pin19 = 18;
constexpr unsigned int pin20 = 19;
constexpr unsigned int first_reserved = 50;
constexpr unsigned int last_dynamic = 83;
constexpr unsigned int past_dynamic = 84;
constexpr std::array<std::uint32_t, 5> reset_words{
    0x04c80a94, 0x04e84b96, 0x03668c98, 0x03a714a3, 0x05694f9e};
// HWR TableA-79; numeric51/52 corroborated by the vendor21489 header because
// that manual's binary strings disagree with its hex column at those two rows.
constexpr std::array<unsigned int, 81> codes{
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16,  17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37,  38, 39, 40, 41,
    42, 43, 44, 45, 46, 47, 48, 49, 52, 53, 54, 55, 56, 57, 58, 59, 61,  62, 63, 64, 65,
    66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 80, 81, 82, 83, 126, 127};
constexpr std::array<unsigned int, 20> default_sources{20, 21, 32, 38, 22, 23, 33, 39, 24, 25,
                                                       26, 27, 35, 41, 28, 29, 30, 31, 37, 43};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("DAI pin expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing DAI pin rejection: " + reason);
}

State all_low() {
  State state;
  state.routes.fill(low_word);
  return state;
}

struct Word {
  unsigned int index{};
  std::uint32_t value{};
};

void store(Router &router, Word word) {
  router.write({.address = word.index * 4, .size = 4}, word.value);
}

std::uint32_t load(Router &router, unsigned int index) {
  return static_cast<std::uint32_t>(router.read({.address = index * 4, .size = 4}));
}

struct Selection {
  unsigned int pin;
  unsigned int source;
  bool inverted{};
};

State selected(Selection choice) {
  auto state = all_low();
  constexpr unsigned int field_bits = 7;
  constexpr unsigned int mask = 127;
  const auto shift = (choice.pin % 4) * field_bits;
  state.routes[choice.pin / 4] &= ~(mask << shift);
  state.routes[choice.pin / 4] |= choice.source << shift;
  if (choice.inverted) { state.routes[4] |= 1U << (choice.pin + 10); }
  return state;
}

void reset_and_live_inputs() {
  lead_a1::SharcPower power;
  Router router(power);
  for (unsigned int index = 0; index < reset_words.size(); ++index) {
    require(load(router, index) == reset_words[index]);
  }
  for (const auto value : router.outputs()) { require(!value.has_value()); }
  for (const auto source : codes) {
    if (source < low_source) { router.source(source, false); }
  }
  for (const auto source : codes) {
    if (source >= low_source) { continue; }
    router.source(source, true);
    const auto outputs = router.outputs();
    for (unsigned int pin = 0; pin < outputs.size(); ++pin) {
      require(outputs[pin] == (default_sources[pin] == source));
    }
    router.source(source, std::nullopt);
    const auto missing = router.outputs();
    for (unsigned int pin = 0; pin < missing.size(); ++pin) {
      require(missing[pin] ==
              (default_sources[pin] == source ? std::nullopt : std::optional<bool>{false}));
    }
    router.source(source, false);
  }
  require(router.snapshot().routes == reset_words);
}

void one_selection(Selection choice) {
  lead_a1::SharcPower power;
  Router router(power, selected(choice));
  for (const auto signal : {std::optional<bool>{false}, {true}, {}}) {
    if (choice.source < low_source) { router.source(choice.source, signal); }
    auto value =
        choice.source >= low_source ? std::optional<bool>{choice.source == high_source} : signal;
    if (choice.inverted && value) { value = !*value; }
    lead_a1::SharcDaiPinOutputs expected;
    expected.fill(false);
    expected[choice.pin] = value;
    require(router.outputs() == expected);
  }
}

void every_source_destination_inversion() {
  unsigned int cases{};
  for (unsigned int pin = 0; pin < pin_count; ++pin) {
    for (const auto source : codes) {
      one_selection({.pin = pin, .source = source});
      ++cases;
      if (pin >= pin19) {
        one_selection({.pin = pin, .source = source, .inverted = true});
        ++cases;
      }
    }
  }
  constexpr unsigned int expected_cases = 1782;
  require(cases == expected_cases);
}

void effect_phases_and_equal_inputs() {
  constexpr std::array<std::optional<bool>, 7> expected{false, false, false, false, {}, {}, true};
  lead_a1::SharcPower power;
  Router router(power, all_low());
  store(router, {.value = first_high});
  for (unsigned int phase = 0; phase < expected.size(); ++phase) {
    require(router.outputs()[0] == expected[phase]);
    if (phase + 1 < expected.size()) {
      rejected([&] { static_cast<void>(load(router, 0)); }, "unsettled readback");
      router.advance_half_pclk(1);
    }
  }
  require(load(router, 0) == first_high && !router.snapshot().pending[0]);
  Router equal(power, selected({.pin = 0, .source = 0}));
  equal.source(0, false);
  equal.source(1, false);
  store(equal, {.value = first_input_one});
  equal.advance_pclk(2);
  require(equal.outputs()[0] == false);
  equal.source(1, true);
  require(!equal.outputs()[0]);
  equal.source(0, true);
  require(equal.outputs()[0] == true);
  equal.source(1, std::nullopt);
  require(!equal.outputs()[0]);
  equal.advance_pclk(1);
  require(!equal.outputs()[0]);
  equal.source(1, true);
  require(equal.outputs()[0] == true);
}

void inversion_timing_and_reserved_bits() {
  lead_a1::SharcPower power;
  Router reserved(power, all_low());
  constexpr std::uint32_t first_word = 0xf0000000 | high_word;
  store(reserved, {.value = first_word});
  reserved.advance_pclk(3);
  require(load(reserved, 0) == first_word);
  for (unsigned int pin = 0; pin < pin_count; ++pin) {
    require(reserved.outputs()[pin] == (pin < 4));
  }
  Router router(power, all_low());
  constexpr std::uint32_t inverted = 0xf0000000 | high_word;
  store(router, {.index = 4, .value = inverted});
  router.advance_pclk(2);
  require(!router.outputs()[16] && !router.outputs()[pin18] && router.outputs()[pin19] == false &&
          router.outputs()[pin20] == false);
  router.advance_pclk(1);
  require(load(router, 4) == inverted);
  require(router.outputs()[16] == true && router.outputs()[pin18] == true &&
          router.outputs()[pin19] == false && router.outputs()[pin20] == false);
  store(router, {.index = 4, .value = high_word});
  router.advance_half_pclk(3);
  require(router.outputs()[pin19] == false && router.outputs()[pin20] == false);
  router.advance_half_pclk(1);
  require(!router.outputs()[pin19] && !router.outputs()[pin20]);
  router.advance_half_pclk(2);
  require(router.outputs()[pin19] == true && router.outputs()[pin20] == true);
}

void independent_intervals_and_restores() {
  lead_a1::SharcPower power;
  for (unsigned int phase = 0; phase < 6; ++phase) {
    Router original(power, all_low());
    store(original, {.value = high_word});
    original.advance_half_pclk(phase);
    Router restored(power, original.snapshot());
    store(original, {.index = 4, .value = high_word});
    store(restored, {.index = 4, .value = high_word});
    original.advance_half_pclk(6 - phase);
    restored.advance_half_pclk(6 - phase);
    require(original.snapshot() == restored.snapshot() && original.outputs() == restored.outputs());
    require(load(original, 0) == high_word);
    original.advance_pclk(std::numeric_limits<std::uint32_t>::max());
    restored.advance_half_pclk(std::numeric_limits<std::uint32_t>::max());
    require(original.snapshot() == restored.snapshot() && load(original, 4) == high_word);
  }
}

void drive_composition() {
  lead_a1::SharcPower power;
  auto data = all_low();
  constexpr std::uint32_t mixed_data = 0x0fc03f7f;
  data.routes[0] = mixed_data; // HIGH,LOW,unknown PB01,LOW.
  Router pins(power, data);
  lead_a1::SharcDaiEnableState enables_state;
  enables_state.routes.fill(0);
  constexpr std::uint32_t mixed_enable = 0x00081041;
  enables_state.routes[0] = mixed_enable; // enabled pins1..3, unknown enable4; pin5 disabled.
  lead_a1::SharcDaiEnable enables(power, enables_state);
  constexpr std::uint32_t disabled = 0xffff0;
  require(pins.drives(enables) ==
          (lead_a1::SharcDaiPinDrive{
              .high = 1, .low = 2, .high_impedance = disabled, .unresolved = 12}));
  pins.source(0, true);
  require(pins.drives(enables).unresolved == 8);
  enables.source(2, false);
  constexpr std::uint32_t newly_disabled = 0xffff8;
  require(pins.drives(enables) ==
          (lead_a1::SharcDaiPinDrive{.high = 5, .low = 2, .high_impedance = newly_disabled}));
}

void transaction_and_malformed_state() {
  lead_a1::SharcPower power;
  Router router(power, all_low());
  const auto before = router.snapshot();
  router.begin_instruction();
  store(router, {.value = high_word});
  require(router.state().pending[0].has_value() && router.outputs()[0] == false);
  rejected([&] { static_cast<void>(router.snapshot()); }, "active instruction");
  rejected([&] { router.begin_instruction(); }, "Nested");
  rejected([&] { router.source(0, true); }, "active instruction");
  rejected([&] { router.advance_half_pclk(0); }, "active instruction");
  router.abort_instruction();
  require(router.snapshot() == before);
  router.begin_instruction();
  store(router, {.value = high_word});
  router.commit_instruction();
  const auto posted = router.snapshot();
  rejected([&] { store(router, {.value = low_word}); }, "overlapping");
  require(router.snapshot() == posted);
  router.advance_pclk(3);
  require(load(router, 0) == high_word);
  auto invalid = before;
  invalid.pending[0] = lead_a1::SharcDaiPinPending{.target = high_word, .half_periods = 6};
  rejected([&] { const Router broken(power, invalid); }, "pending interval");
  invalid = before;
  invalid.sources[first_reserved] = false;
  rejected([&] { const Router broken(power, invalid); }, "source input");
}

void reserved_sources_and_accesses() {
  lead_a1::SharcPower power;
  Router router(power, all_low());
  const auto before = router.snapshot();
  constexpr unsigned int encodings = 128;
  for (unsigned int code = 0; code < encodings; ++code) {
    const bool allowed = std::ranges::find(codes, code) != codes.end();
    if (allowed && code < low_source) { continue; }
    rejected([&] { router.source(code, false); }, "source encoding");
    if (allowed) { continue; }
    for (unsigned int pin = 0; pin < pin_count; ++pin) {
      const auto invalid = selected({.pin = pin, .source = code});
      rejected([&] { const Router broken(power, invalid); }, "source selection");
      const auto index = pin / 4;
      rejected([&] { store(router, {.index = index, .value = invalid.routes[index]}); },
               "source selection");
    }
  }
  require(router.snapshot() == before);
  router.source(last_dynamic, true);
  rejected([&] { router.source(past_dynamic, true); }, "source encoding");
  rejected([&] { router.source(std::numeric_limits<unsigned int>::max(), true); },
           "source encoding");
  rejected([&] { router.write({.size = 4}, std::uint64_t{1} << 32); }, "Oversized");
  constexpr std::array<lead_a1::MemoryAccess, 5> accesses{lead_a1::MemoryAccess{.size = 1},
                                                          {.size = 2},
                                                          {.size = 8},
                                                          {.address = 1, .size = 4},
                                                          {.address = 20, .size = 4}};
  for (const auto access : accesses) {
    rejected([&] { static_cast<void>(router.read(access)); }, "aligned32-bit");
    rejected([&] { router.write(access, low_word); }, "aligned32-bit");
  }
}

void stopped_clock_preserves_state() {
  lead_a1::SharcPower power;
  Router router(power, all_low());
  store(router, {.value = high_word});
  const auto before = router.snapshot();
  power.write({.size = 4}, 0x10);
  power.advance_pclk(2);
  rejected([&] { static_cast<void>(router.outputs()); }, "stopped DAI clock");
  rejected([&] { static_cast<void>(load(router, 0)); }, "stopped DAI clock");
  rejected([&] { store(router, {.index = 1, .value = high_word}); }, "stopped DAI clock");
  rejected([&] { router.advance_pclk(3); }, "stopped DAI clock");
  require(router.snapshot() == before);
  power.write({.size = 4}, 0);
  power.advance_pclk(2);
  router.advance_pclk(3);
  require(load(router, 0) == high_word);
}

}

int main() try {
  reset_and_live_inputs();
  every_source_destination_inversion();
  effect_phases_and_equal_inputs();
  inversion_timing_and_reserved_bits();
  independent_intervals_and_restores();
  drive_composition();
  transaction_and_malformed_state();
  reserved_sources_and_accesses();
  stopped_clock_preserves_state();
  std::cout
      << "DAI Group D source/destination/inversion/effect/drive/transaction expectations pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
