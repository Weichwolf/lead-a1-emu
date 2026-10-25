// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/peripherals/sharc_dai_data.hpp"
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

using Router = lead_a1::SharcDaiData;
using State = lead_a1::SharcDaiDataState;
constexpr unsigned int final_destination = 30;
constexpr std::uint32_t final_low = 0x00fbefbe;
constexpr std::uint32_t final_high = 0x00ffffff;
constexpr std::uint32_t low_word = 0x3efbefbe;
constexpr std::uint32_t high_word = 0x3fffffff;
constexpr std::uint32_t first_high = 0x3efbefbf;
constexpr std::uint32_t first_input_one = 0x3efbef81;
constexpr unsigned int destination_count = 34;
constexpr unsigned int low_source = 62;
constexpr unsigned int high_source = 63;
constexpr unsigned int first_reserved = 41;
constexpr unsigned int last_dynamic = 48;
constexpr unsigned int past_dynamic = 49;
constexpr std::array<std::uint32_t, 7> reset_words{
    0x08144040, 0x0f38b289, 0x00000450, 0, 0, 0, 0x00fbefbe};
// HWR Table A-77 and the pinned vendor21489 header. Codes41..43 are absent
// from Group B; clock/frame signals do not become serial-data inputs.
constexpr std::array<unsigned int, 48> codes{
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 44, 45, 46, 47, 48, 62, 63};
constexpr std::array<unsigned int, 34> default_sources{0, 1, 4, 5, 8, 9, 10, 11, 14, 15, 16, 17,
                                                       0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0,
                                                       0, 0, 0, 0, 0, 0, 62, 62, 62, 62};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("DAI data expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing DAI data rejection: " + reason);
}

State all_low() {
  State state;
  state.routes.fill(low_word);
  state.routes[6] = final_low;
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
  unsigned int destination;
  unsigned int source;
};

State selected(Selection choice) {
  auto state = all_low();
  constexpr unsigned int field_bits = 6;
  constexpr unsigned int mask = 63;
  const auto shift = (choice.destination % 5) * field_bits;
  state.routes[choice.destination / 5] &= ~(mask << shift);
  state.routes[choice.destination / 5] |= choice.source << shift;
  return state;
}

void reset_and_live_inputs() {
  lead_a1::SharcPower power;
  Router router(power);
  for (unsigned int index = 0; index < reset_words.size(); ++index) {
    require(load(router, index) == reset_words[index]);
  }
  const auto initial_outputs = router.outputs();
  for (unsigned int destination = 0; destination < destination_count; ++destination) {
    require(initial_outputs[destination] ==
            (destination >= final_destination ? std::optional<bool>{false} : std::nullopt));
  }
  for (const auto source : codes) {
    if (source < low_source) { router.source(source, false); }
  }
  for (const auto source : codes) {
    if (source >= low_source) { continue; }
    router.source(source, true);
    const auto outputs = router.outputs();
    for (unsigned int destination = 0; destination < outputs.size(); ++destination) {
      require(outputs[destination] == (default_sources[destination] == source));
    }
    router.source(source, std::nullopt);
    const auto missing = router.outputs();
    for (unsigned int destination = 0; destination < missing.size(); ++destination) {
      require(missing[destination] ==
              (default_sources[destination] == source ? std::nullopt : std::optional<bool>{false}));
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
    lead_a1::SharcDaiDataOutputs expected;
    expected.fill(false);
    expected[choice.destination] = value;
    require(router.outputs() == expected);
  }
}

void every_source_destination() {
  unsigned int cases{};
  for (unsigned int destination = 0; destination < destination_count; ++destination) {
    for (const auto source : codes) {
      one_selection({.destination = destination, .source = source});
      ++cases;
    }
  }
  constexpr unsigned int expected_cases = 1632;
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
  Router equal(power, selected({.destination = 0, .source = 0}));
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

void final_word_fields_and_reserved_bits() {
  lead_a1::SharcPower power;
  Router router(power, all_low());
  // Upper8 bits of DAT6 are reserved. Even a numeric reserved selector there
  // must not be interpreted as a nonexistent fifth destination.
  constexpr std::uint32_t last_word = 0xe9ffffff;
  store(router, {.index = 6, .value = last_word});
  router.advance_pclk(2);
  auto expected = router.outputs();
  for (unsigned int destination = 0; destination < final_destination; ++destination) {
    require(expected[destination] == false);
  }
  for (unsigned int destination = final_destination; destination < destination_count;
       ++destination) {
    require(!expected[destination]);
  }
  router.advance_pclk(1);
  require(load(router, 6) == last_word);
  expected = router.outputs();
  for (unsigned int destination = 0; destination < destination_count; ++destination) {
    require(expected[destination] == (destination >= final_destination));
  }
  constexpr std::uint32_t first_word = 0xffffffff;
  store(router, {.value = first_word});
  router.advance_pclk(3);
  require(load(router, 0) == first_word);
  for (unsigned int destination = 0; destination < destination_count; ++destination) {
    require(router.outputs()[destination] == (destination < 5 || destination >= final_destination));
  }
}

void independent_intervals_and_restores() {
  lead_a1::SharcPower power;
  for (unsigned int phase = 0; phase < 6; ++phase) {
    Router original(power, all_low());
    store(original, {.value = high_word});
    original.advance_half_pclk(phase);
    Router restored(power, original.snapshot());
    store(original, {.index = 6, .value = final_high});
    store(restored, {.index = 6, .value = final_high});
    original.advance_half_pclk(6 - phase);
    restored.advance_half_pclk(6 - phase);
    require(original.snapshot() == restored.snapshot() && original.outputs() == restored.outputs());
    require(load(original, 0) == high_word);
    original.advance_pclk(std::numeric_limits<std::uint32_t>::max());
    restored.advance_half_pclk(std::numeric_limits<std::uint32_t>::max());
    require(original.snapshot() == restored.snapshot() && load(original, 6) == final_high);
  }
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
  invalid.pending[0] = lead_a1::SharcDaiDataPending{.target = high_word, .half_periods = 6};
  rejected([&] { const Router broken(power, invalid); }, "pending interval");
  invalid = before;
  invalid.sources[first_reserved] = false;
  rejected([&] { const Router broken(power, invalid); }, "source input");
}

void reserved_sources_and_accesses() {
  lead_a1::SharcPower power;
  Router router(power, all_low());
  const auto before = router.snapshot();
  constexpr unsigned int encodings = 64;
  for (unsigned int code = 0; code < encodings; ++code) {
    const bool allowed = std::ranges::find(codes, code) != codes.end();
    if (allowed && code < low_source) { continue; }
    rejected([&] { router.source(code, false); }, "source encoding");
    if (allowed) { continue; }
    if (code <= last_dynamic) {
      auto invalid_input = before;
      invalid_input.sources[code] = false;
      rejected([&] { const Router broken(power, invalid_input); }, "source input");
    }
    for (unsigned int destination = 0; destination < destination_count; ++destination) {
      const auto invalid = selected({.destination = destination, .source = code});
      rejected([&] { const Router broken(power, invalid); }, "source selection");
      const auto index = destination / 5;
      auto pending = before;
      pending.pending[index] = lead_a1::SharcDaiDataPending{.target = invalid.routes[index]};
      rejected([&] { const Router broken(power, pending); }, "source selection");
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
                                                          {.address = 28, .size = 4}};
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
  every_source_destination();
  effect_phases_and_equal_inputs();
  final_word_fields_and_reserved_bits();
  independent_intervals_and_restores();
  transaction_and_malformed_state();
  reserved_sources_and_accesses();
  stopped_clock_preserves_state();
  std::cout << "DAI Group B source/destination/effect/transaction expectations pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
