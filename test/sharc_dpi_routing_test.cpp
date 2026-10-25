// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_dpi_routing.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>

namespace {

using Group = lead_a1::SharcDpiGroup;
using Router = lead_a1::SharcDpiRouting;
using State = lead_a1::SharcDpiState;

// Independent HWR TablesA-113/114 and B-15, including the holes inside each group.
constexpr std::array pin_codes{0U,  1U,  2U,  3U,  4U,  5U,  6U,  7U,  8U,  9U,  10U, 11U, 12U,
                               13U, 14U, 15U, 16U, 17U, 19U, 21U, 22U, 23U, 24U, 25U, 26U, 27U,
                               28U, 29U, 30U, 31U, 32U, 33U, 34U, 35U, 36U, 37U, 38U, 39U, 40U,
                               41U, 42U, 43U, 44U, 45U, 46U, 47U, 48U, 49U, 50U};
constexpr std::array enable_codes{0U,  1U,  2U,  3U,  4U,  5U,  6U,  8U,  10U, 11U, 12U,
                                  13U, 14U, 15U, 16U, 17U, 18U, 19U, 20U, 21U, 22U, 23U,
                                  24U, 25U, 26U, 27U, 28U, 29U, 30U, 31U, 32U, 33U, 34U,
                                  35U, 36U, 37U, 38U, 39U, 40U, 41U, 42U, 43U};
constexpr std::array<std::uint32_t, 3> pin_reset{0x18017556, 0x004db699, 0x00450000};
constexpr std::array<std::uint32_t, 3> enable_reset{0x0d00c28b, 0x002103ce, 0x00185964};
constexpr std::array<unsigned int, 14> pin_defaults{
    22, 21, 23, 0, 24, 25, 26, 27, 19, 0, 0, 0, 16, 17};
constexpr std::array<unsigned int, 14> enable_defaults{
    11, 10, 12, 0, 13, 14, 15, 16, 8, 0, 36, 37, 5, 6};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("DPI routing expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing DPI rejection: " + reason);
}

std::span<const unsigned int> codes(Group group) {
  if (group == Group::pin) { return pin_codes; }
  return enable_codes;
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
};

State selected(Selection choice) {
  State state;
  constexpr unsigned int fields = 5;
  constexpr unsigned int bits = 6;
  state.routes.at(choice.pin / fields) = choice.source << ((choice.pin % fields) * bits);
  return state;
}

void reset_and_live_defaults(Group group) {
  Router router(group);
  const auto &raw = group == Group::pin ? pin_reset : enable_reset;
  const auto &sources = group == Group::pin ? pin_defaults : enable_defaults;
  for (unsigned int index = 0; index < raw.size(); ++index) {
    require(load(router, index) == raw[index]);
  }
  const auto unknown = router.outputs();
  for (unsigned int pin = 0; pin < sources.size(); ++pin) {
    require(unknown[pin] == (sources[pin] == 0 ? std::optional<bool>{false} : std::nullopt));
  }
  for (const auto source : codes(group)) {
    if (source >= 2) { router.source(source, false); }
  }
  for (const auto source : codes(group)) {
    if (source < 2) { continue; }
    router.source(source, true);
    const auto outputs = router.outputs();
    for (unsigned int pin = 0; pin < sources.size(); ++pin) {
      require(outputs[pin] == (sources[pin] == source));
    }
    router.source(source, std::nullopt);
    const auto missing = router.outputs();
    for (unsigned int pin = 0; pin < sources.size(); ++pin) {
      require(missing[pin] == (sources[pin] == source ? std::nullopt : std::optional<bool>{false}));
    }
    router.source(source, false);
  }
  require(router.snapshot().routes == raw);
}

void every_source_and_destination() {
  constexpr unsigned int pins = 14;
  for (const auto group : {Group::pin, Group::enable}) {
    for (unsigned int pin = 0; pin < pins; ++pin) {
      for (const auto source : codes(group)) {
        Router router(group, selected({.pin = pin, .source = source}));
        for (const auto signal : {std::optional<bool>{false}, {true}, {}}) {
          if (source >= 2) { router.source(source, signal); }
          lead_a1::SharcDpiOutputs expected;
          expected.fill(false);
          expected[pin] = source < 2 ? std::optional<bool>{source == 1} : signal;
          require(router.outputs() == expected);
        }
      }
    }
  }
}

void effect_interval_and_live_equal_routes() {
  const std::array<std::optional<bool>, 7> phases{false, false, false, false, {}, {}, true};
  for (const auto group : {Group::pin, Group::enable}) {
    Router router(group, State{});
    store(router, {.value = 1});
    for (unsigned int phase = 0; phase < phases.size(); ++phase) {
      require(router.outputs()[0] == phases[phase]);
      if (phase + 1 < phases.size()) {
        rejected([&] { static_cast<void>(load(router, 0)); }, "unsettled readback");
        router.advance_half_pclk(1);
      }
    }
    require(load(router, 0) == 1 && !router.snapshot().pending[0]);
    Router equal(group, selected({.pin = 0, .source = 2}));
    equal.source(2, false);
    equal.source(3, false);
    store(equal, {.value = 3});
    equal.advance_pclk(2);
    require(equal.outputs()[0] == false);
    equal.source(3, true);
    require(!equal.outputs()[0].has_value());
    equal.source(2, true);
    require(equal.outputs()[0] == true);
    equal.source(3, std::nullopt);
    require(!equal.outputs()[0].has_value());
    equal.advance_pclk(1);
    require(!equal.outputs()[0].has_value() && load(equal, 0) == 3);
  }
}

void fractional_restore_and_reserved_readback() {
  constexpr std::uint32_t reserved = 0xff000000;
  for (const auto group : {Group::pin, Group::enable}) {
    Router router(group, State{});
    store(router, {.value = 1});
    router.advance_half_pclk(1);
    store(router, {.index = 1, .value = 1});
    router.advance_pclk(2);
    store(router, {.index = 2, .value = reserved | 1});
    const auto saved = router.snapshot();
    require(saved.pending[0] == (lead_a1::SharcDpiPending{.target = 1, .half_periods = 5}) &&
            saved.pending[1] == (lead_a1::SharcDpiPending{.target = 1, .half_periods = 4}) &&
            saved.pending[2] == (lead_a1::SharcDpiPending{.target = reserved | 1}));
    Router copy(group, saved);
    router.advance_half_pclk(1);
    copy.advance_half_pclk(1);
    require(router.snapshot() == copy.snapshot() && load(router, 0) == 1 &&
            !router.outputs()[5].has_value() && router.outputs()[10] == false);
    router.advance_pclk(1);
    copy.advance_half_pclk(2);
    require(router.snapshot() == copy.snapshot() && load(router, 1) == 1);
    router.advance_half_pclk(3);
    copy.advance_half_pclk(3);
    require(router.snapshot() == copy.snapshot() && load(router, 2) == (reserved | 1) &&
            router.outputs()[10] == true);
    store(router, {.index = 2});
    router.advance_pclk(std::numeric_limits<std::uint32_t>::max());
    require(load(router, 2) == 0 && router.outputs()[10] == false);
  }
}

void invalid_fields_inputs_and_transactions() {
  constexpr unsigned int encodings = 64;
  constexpr unsigned int pins = 14;
  for (const auto group : {Group::pin, Group::enable}) {
    Router router(group, State{});
    const auto before = router.snapshot();
    for (unsigned int encoding = 0; encoding < encodings; ++encoding) {
      if (std::ranges::find(codes(group), encoding) != codes(group).end()) { continue; }
      for (unsigned int pin = 0; pin < pins; ++pin) {
        const auto bad = selected({.pin = pin, .source = encoding});
        rejected([&] { const Router invalid(group, bad); }, "Reserved");
        const auto index = pin / 5;
        rejected([&] { store(router, {.index = index, .value = bad.routes[index]}); }, "Reserved");
      }
      rejected([&] { router.source(encoding, false); }, "dynamic source");
    }
    rejected([&] { router.source(0, false); }, "dynamic source");
    rejected([&] { router.source(1, true); }, "dynamic source");
    auto bad = before;
    bad.sources[0] = false;
    bad.sources.back() = false; // PIN code50 is valid; enable code50 is not.
    if (group == Group::enable) {
      rejected([&] { const Router invalid(group, bad); }, "source input");
    }
    bad = before;
    bad.sources[group == Group::pin ? 16U : 5U] = false;
    rejected([&] { const Router invalid(group, bad); }, "source input");
    bad = before;
    const unsigned int invalid_source = group == Group::pin ? 18U : 7U;
    bad.pending[0] = lead_a1::SharcDpiPending{.target = invalid_source};
    rejected([&] { const Router invalid(group, bad); }, "Reserved");
    bad.pending[0] = lead_a1::SharcDpiPending{.half_periods = 6};
    rejected([&] { const Router invalid(group, bad); }, "effect interval");
    rejected([&] { router.write({.address = 1, .size = 4}, 0); }, "aligned32");
    rejected([&] { router.write({.address = 12, .size = 4}, 0); }, "aligned32");
    rejected([&] { static_cast<void>(router.read({.size = 2})); }, "aligned32");
    constexpr std::uint64_t oversized = 0x100000000;
    rejected([&] { router.write({.size = 4}, oversized); }, "Oversized");
    require(router.snapshot() == before);
    router.begin_instruction();
    store(router, {.value = 1});
    require(router.outputs()[0] == false && router.state().pending[0].has_value());
    rejected([&] { router.begin_instruction(); }, "Nested");
    rejected([&] { static_cast<void>(router.snapshot()); }, "active instruction");
    rejected([&] { router.source(2, true); }, "active instruction");
    rejected([&] { router.advance_pclk(0); }, "active instruction");
    rejected([&] { router.advance_half_pclk(1); }, "active instruction");
    router.abort_instruction();
    require(router.snapshot() == before);
    router.begin_instruction();
    store(router, {.value = 1});
    router.commit_instruction();
    const auto pending = router.snapshot();
    rejected([&] { store(router, {.value = 0}); }, "overlapping write");
    require(router.snapshot() == pending);
    router.advance_pclk(3);
    require(load(router, 0) == 1 && router.outputs()[0] == true);
  }
  rejected([] { const Router invalid(std::bit_cast<Group>(std::uint8_t{2})); }, "routing group");
}

void pin_drive_composition() {
  constexpr unsigned int fourth_pin_shift = 18;
  State data;
  data.routes[0] = 1;         // Pin1 HIGH, pin2 LOW, remaining LOW unless supplied below.
  data.routes[0] |= 2U << 12; // Pin3 selects explicit external DPI_PB01_O.
  data.routes[0] |= 2U << fourth_pin_shift; // Pin4 selects the same independent input.
  State control;
  control.routes[0] =
      1 | (1U << 6) | (2U << fourth_pin_shift); // Pin3 disabled; pin4 MISCB0 unknown.
  Router pins(Group::pin, data);
  Router enables(Group::enable, control);
  constexpr std::uint16_t high_z = 0x3ff4;
  require(pins.drives(enables) ==
          (lead_a1::SharcDpiDrive{.high = 1, .low = 2, .high_impedance = high_z, .unresolved = 8}));
  enables.source(2, false);
  constexpr std::uint16_t remaining_disabled = 0x3ffc;
  require(pins.drives(enables).high_impedance == remaining_disabled);
  enables.source(2, true);
  require(pins.drives(enables).unresolved == 8);
  pins.source(2, true);
  require(pins.drives(enables).high == 9 && pins.drives(enables).unresolved == 0);
  pins.source(2, false);
  require(pins.drives(enables).low == 10);
  rejected([&] { static_cast<void>(enables.drives(pins)); }, "pin then enable");
  rejected([&] { static_cast<void>(pins.drives(pins)); }, "pin then enable");
}

}

int main() {
  try {
    for (const auto group : {Group::pin, Group::enable}) { reset_and_live_defaults(group); }
    every_source_and_destination();
    effect_interval_and_live_equal_routes();
    fractional_restore_and_reserved_readback();
    invalid_fields_inputs_and_transactions();
    pin_drive_composition();
    std::cout << "DPI source/destination/interval/drive/restore/fault expectations passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
