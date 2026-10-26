// SPDX-License-Identifier: GPL-3.0-only
#include "board/scheduler.hpp"

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

using Event = lead_a1::BoardEvent;
constexpr auto max_time = std::numeric_limits<std::uint64_t>::max();

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Board scheduler expectation at line " + std::to_string(where.line()));
  }
}

template <typename Value> Value present(const std::optional<Value> &value) {
  if (!value) { throw std::runtime_error("Missing scheduled board dispatch"); }
  return *value;
}

template <typename Action> void rejected(const Action &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected " + reason + ": " + error.what());
  }
  throw std::runtime_error("Missing scheduler rejection: " + reason);
}

void literal_order_and_source_cadences() {
  lead_a1::BoardScheduler scheduler({.sources = {{.next = 0, .priority = 10},
                                                 {.next = 0, .priority = 0},
                                                 {.next = std::nullopt, .priority = 0}}});
  constexpr std::array<Event, 10> events{{{.timestamp = 0, .source = 1, .sequence = 0},
                                          {.timestamp = 0, .source = 0, .sequence = 0},
                                          {.timestamp = 3, .source = 0, .sequence = 1},
                                          {.timestamp = 5, .source = 1, .sequence = 1},
                                          {.timestamp = 6, .source = 0, .sequence = 2},
                                          {.timestamp = 9, .source = 0, .sequence = 3},
                                          {.timestamp = 10, .source = 1, .sequence = 2},
                                          {.timestamp = 12, .source = 0, .sequence = 4},
                                          {.timestamp = 15, .source = 1, .sequence = 3},
                                          {.timestamp = 15, .source = 0, .sequence = 5}}};
  constexpr std::array<std::uint64_t, 2> periods{3, 5};
  for (const auto expected : events) {
    require(scheduler.next() == expected);
    const auto dispatch = scheduler.claim();
    require(dispatch && dispatch->event == expected && !dispatch->retired &&
            scheduler.state().frontier == expected.timestamp && scheduler.claim() == dispatch);
    scheduler.retire(expected);
    scheduler.complete(expected, expected.timestamp + periods[expected.source]);
  }
  constexpr std::array<std::uint64_t, 2> following{18, 20};
  require(scheduler.state().sources[0].next == following[0] &&
          scheduler.state().sources[1].next == following[1] && !scheduler.state().sources[2].next);
}

void retained_execution_and_publication() {
  lead_a1::BoardScheduler scheduler({.sources = {{.next = 7}}});
  constexpr Event event{.timestamp = 7};
  rejected([&] { scheduler.retire(event); }, "active event/sequence");
  const auto dispatch = scheduler.claim();
  require(dispatch && dispatch->event == event && !dispatch->retired);
  const auto execution = scheduler.state();
  rejected([&] { scheduler.complete(event, 8); }, "has not retired");
  require(scheduler.state() == execution && scheduler.claim() == dispatch);
  lead_a1::BoardScheduler before(execution);
  require(before.claim() == dispatch);
  scheduler.retire(event);
  const auto publication = scheduler.state();
  require(present(publication.active).retired);
  rejected([&] { scheduler.retire(event); }, "already retired");
  rejected([&] { scheduler.complete(event, 7); }, "future timestamp");
  rejected([&] { scheduler.rearm(0, 8); }, "idle parked source");
  require(scheduler.state() == publication && present(scheduler.claim()).retired);
  lead_a1::BoardScheduler restored(publication);
  scheduler.complete(event, 8);
  restored.complete(event, 8);
  require(scheduler.state() == restored.state() && !scheduler.state().active &&
          scheduler.next() == (Event{.timestamp = 8, .sequence = 1}));
}

void park_rearm_and_stale_tickets() {
  lead_a1::BoardScheduler scheduler({.sources = {{.next = 7}, {.next = 7}}});
  constexpr Event first{.timestamp = 7};
  require(present(scheduler.claim()).event == first);
  scheduler.retire(first);
  scheduler.complete(first, {});
  require(scheduler.next() == (Event{.timestamp = 7, .source = 1}));
  rejected([&] { scheduler.rearm(0, 6); }, "nonpast timestamp");
  scheduler.rearm(0, 7);
  constexpr Event resumed{.timestamp = 7, .sequence = 1};
  require(present(scheduler.claim()).event == resumed);
  const auto saved = scheduler.state();
  rejected([&] { scheduler.retire(first); }, "active event/sequence");
  require(scheduler.state() == saved);
  scheduler.retire(resumed);
  scheduler.complete(resumed, {});
  constexpr Event second{.timestamp = 7, .source = 1};
  require(present(scheduler.claim()).event == second);
  scheduler.retire(second);
  scheduler.complete(second, {});
  require(!scheduler.next() && !scheduler.claim());
}

void malformed_state_and_boundaries() {
  rejected([] { const lead_a1::BoardScheduler empty({}); }, "explicit bounded sources");
  lead_a1::BoardScheduleState bad{.sources = {{.next = 1}}, .frontier = 2};
  rejected([&] { const lead_a1::BoardScheduler invalid(bad); }, "causal frontier");
  bad = {.sources = {{.next = 7}, {.next = 7}},
         .frontier = 7,
         .active = lead_a1::BoardDispatch{.event = {.timestamp = 7, .source = 1}}};
  rejected([&] { const lead_a1::BoardScheduler invalid(bad); }, "active board dispatch");
  bad.active = lead_a1::BoardDispatch{.event = {.timestamp = 7, .sequence = 1}};
  rejected([&] { const lead_a1::BoardScheduler invalid(bad); }, "active board dispatch");
  lead_a1::BoardScheduler scheduler({.sources = {{}}});
  scheduler.rearm(0, max_time);
  constexpr Event last{.timestamp = max_time};
  require(present(scheduler.claim()).event == last);
  scheduler.retire(last);
  scheduler.complete(last, {});
  require(!scheduler.next() && scheduler.state().frontier == max_time);
  scheduler.rearm(0, max_time);
  constexpr Event same_time{.timestamp = max_time, .sequence = 1};
  require(present(scheduler.claim()).event == same_time);
  scheduler.retire(same_time);
  const auto saved = scheduler.state();
  rejected([&] { scheduler.complete(same_time, 0); }, "future timestamp");
  require(scheduler.state() == saved);
  lead_a1::BoardScheduler exhausted({.sources = {{.next = 0, .sequence = max_time}}});
  const auto event = present(exhausted.claim()).event;
  exhausted.retire(event);
  const auto before = exhausted.state();
  rejected([&] { exhausted.complete(event, {}); }, "unexhausted sequence");
  require(exhausted.state() == before);
}

}

int main() try {
  literal_order_and_source_cadences();
  retained_execution_and_publication();
  park_rearm_and_stale_tickets();
  malformed_state_and_boundaries();
  std::cout << "Board event order, retained publication, source rearm and snapshot guards pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
