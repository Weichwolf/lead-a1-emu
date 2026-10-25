// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_picr.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("PICR expectation line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw;
  }
  throw std::runtime_error("Missing PICR rejection: " + reason);
}

struct Position {
  unsigned int offset;
  unsigned int shift;
};

constexpr std::array<Position, 19> positions{{{.offset = 0, .shift = 0},
                                              {.offset = 0, .shift = 5},
                                              {.offset = 0, .shift = 10},
                                              {.offset = 0, .shift = 15},
                                              {.offset = 0, .shift = 20},
                                              {.offset = 0, .shift = 25},
                                              {.offset = 4, .shift = 0},
                                              {.offset = 4, .shift = 5},
                                              {.offset = 4, .shift = 10},
                                              {.offset = 4, .shift = 15},
                                              {.offset = 4, .shift = 20},
                                              {.offset = 4, .shift = 25},
                                              {.offset = 8, .shift = 0},
                                              {.offset = 8, .shift = 5},
                                              {.offset = 8, .shift = 10},
                                              {.offset = 8, .shift = 15},
                                              {.offset = 8, .shift = 20},
                                              {.offset = 8, .shift = 25},
                                              {.offset = 12, .shift = 0}}};
constexpr std::array<bool, 32> dynamic{true, true,  true, true, true,  true, true,  true,
                                       true, true,  true, true, true,  true, true,  true,
                                       true, false, true, true, false, true, false, true,
                                       true, true,  true, true, true,  true, false, false};
constexpr std::uint32_t all_priorities = 0x7ffff;
constexpr unsigned int software = 31;
constexpr unsigned int disabled_priority = 17;
constexpr unsigned int last_priority = 18;

void reset_and_live_inputs() {
  lead_a1::SharcPicr router;
  constexpr std::array<std::uint32_t, 4> resets{0x0a418820, 0x16a4a0e6, 0x2307b9ac, 0x12};
  for (unsigned int index = 0; index < resets.size(); ++index) {
    require(router.read({.address = index * 4, .size = 4}) == resets[index]);
  }
  const auto initial = router.outputs();
  for (unsigned int priority = 0; priority < initial.size(); ++priority) {
    require(initial[priority] ==
            (priority == disabled_priority ? std::optional<bool>{false} : std::nullopt));
  }
  router.source(1, true);
  require(router.outputs()[1] == true && router.destinations(1) == 2);
  router.source(1, false);
  require(router.outputs()[1] == false);
  router.source(1, std::nullopt);
  require(!router.outputs()[1]);
}

void source_destination_case(unsigned int priority, unsigned int code, std::optional<bool> input) {
  constexpr lead_a1::SharcPicrState software_only{
      .control = {0x3fffffff, 0x3fffffff, 0x3fffffff, 31}};
  const auto position = positions[priority];
  lead_a1::SharcPicr router(software_only);
  if (dynamic[code]) { router.source(code, input); }
  const auto old = static_cast<std::uint32_t>(router.read({.address = position.offset, .size = 4}));
  constexpr unsigned int selector_mask = 31;
  const auto value = (old & ~(selector_mask << position.shift)) | (code << position.shift);
  router.write({.address = position.offset, .size = 4}, value);
  const auto expected = dynamic[code] ? input : std::optional<bool>{code == software};
  const auto levels = router.outputs();
  for (unsigned int other = 0; other < levels.size(); ++other) {
    require(levels[other] == (other == priority ? expected : std::optional<bool>{true}));
  }
  require(router.destinations(code) == (code == software ? all_priorities : 1U << priority));
  require(router.read({.address = position.offset, .size = 4}) == value);
}

void every_source_at_every_destination() {
  for (unsigned int priority = 0; priority < positions.size(); ++priority) {
    for (unsigned int code = 0; code < dynamic.size(); ++code) {
      for (const auto input :
           {std::optional<bool>{false}, std::optional<bool>{true}, std::optional<bool>{}}) {
        source_destination_case(priority, code, input);
      }
    }
  }
}

void transactions_reserved_bits_and_independent_restore() {
  lead_a1::SharcPicr original;
  constexpr unsigned int dai_low = 12;
  original.source(dai_low, true);
  const auto before = original.snapshot();
  original.begin_instruction();
  constexpr std::uint32_t changed = 0xca41b020;
  original.write({.address = 0, .size = 4}, changed);
  require(original.read({.address = 0, .size = 4}) == changed && original.outputs()[2] != true);
  rejected([&] { original.source(1, true); }, "active instruction");
  rejected([&] { static_cast<void>(original.snapshot()); }, "active instruction");
  original.abort_instruction();
  require(original.snapshot() == before);
  original.begin_instruction();
  original.write({.address = 0, .size = 4}, changed);
  original.commit_instruction();
  require(original.outputs()[2] == true &&
          original.destinations(dai_low) == ((1U << 2) | (1U << 12)));
  lead_a1::SharcPicr restored(original.snapshot());
  original.source(dai_low, std::nullopt);
  require(restored.read({.address = 0, .size = 4}) == changed && restored.outputs()[2] == true &&
          !original.outputs()[2]);
  constexpr std::uint32_t raw_tail = 0xfffffff1;
  restored.write({.address = 12, .size = 4}, raw_tail);
  require(restored.read({.address = 12, .size = 4}) == raw_tail &&
          restored.outputs()[last_priority] == false);
}

void malformed_and_constant_sources() {
  lead_a1::SharcPicr router;
  const auto before = router.snapshot();
  for (unsigned int code = 0; code < dynamic.size(); ++code) {
    if (dynamic[code]) { continue; }
    rejected([&] { router.source(code, false); }, "dynamic source");
    auto state = before;
    state.sources[code] = false;
    rejected([&] { const lead_a1::SharcPicr invalid(state); }, "constant source");
  }
  constexpr unsigned int outside = 32;
  rejected([&] { router.source(outside, true); }, "Invalid PICR");
  rejected([&] { static_cast<void>(router.destinations(outside)); }, "Invalid PICR");
  rejected([&] { router.write({.address = 0, .size = 4}, std::uint64_t{1} << 32); }, "Oversized");
  for (const unsigned int address : {1U, 2U, 3U, 16U}) {
    rejected([&] { static_cast<void>(router.read({.address = address, .size = 4})); }, "aligned32");
    rejected([&] { router.write({.address = address, .size = 4}, 0); }, "aligned32");
  }
  for (const unsigned int size : {0U, 1U, 2U, 8U}) {
    rejected([&] { static_cast<void>(router.read({.address = 0, .size = size})); }, "aligned32");
  }
  router.begin_instruction();
  rejected([&] { router.begin_instruction(); }, "Nested");
  router.abort_instruction();
  require(router.snapshot() == before);
}

}

int main() try {
  reset_and_live_inputs();
  every_source_at_every_destination();
  transactions_reserved_bits_and_independent_restore();
  malformed_and_constant_sources();
  std::cout << "PICR independent32-source/19-priority/knowledge/transaction expectations pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
