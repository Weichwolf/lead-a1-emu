// SPDX-License-Identifier: GPL-3.0-only
#include "board/scheduler.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace lead_a1 {
namespace {

std::optional<BoardEvent> earliest(const BoardScheduleState &state) {
  std::optional<BoardEvent> result;
  for (unsigned int index = 0; index < state.sources.size(); ++index) {
    const auto &source = state.sources[index];
    if (!source.next) { continue; }
    if (result) {
      const auto &previous = state.sources[result->source];
      if (*source.next > result->timestamp ||
          (*source.next == result->timestamp && source.priority >= previous.priority)) {
        continue;
      }
    }
    result = BoardEvent{.timestamp = *source.next, .source = index, .sequence = source.sequence};
  }
  return result;
}

}

BoardScheduler::BoardScheduler(BoardScheduleState initial) : state_(std::move(initial)) {
  validate();
}

void BoardScheduler::validate() const {
  if (state_.sources.empty() || state_.sources.size() > std::numeric_limits<unsigned int>::max()) {
    throw std::invalid_argument("Board scheduler requires explicit bounded sources");
  }
  for (const auto &source : state_.sources) {
    if (source.next && *source.next < state_.frontier) {
      throw std::invalid_argument("Board source precedes the causal frontier");
    }
  }
  if (state_.active && (state_.active->event != earliest(state_) ||
                        state_.active->event.timestamp != state_.frontier)) {
    throw std::invalid_argument("Invalid active board dispatch snapshot");
  }
}

std::optional<BoardEvent> BoardScheduler::next() const {
  return state_.active ? std::optional{state_.active->event} : earliest(state_);
}

std::optional<BoardDispatch> BoardScheduler::claim() {
  if (state_.active) { return state_.active; }
  const auto event = earliest(state_);
  if (!event) { return {}; }
  state_.frontier = event->timestamp;
  state_.active = BoardDispatch{.event = *event};
  return state_.active;
}

BoardDispatch &BoardScheduler::checked_active(BoardEvent event) {
  if (!state_.active || state_.active->event != event) {
    throw std::logic_error("Board operation requires its exact active event/sequence");
  }
  return *state_.active;
}

void BoardScheduler::retire(BoardEvent event) {
  auto &dispatch = checked_active(event);
  if (dispatch.retired) { throw std::logic_error("Board guest action already retired"); }
  dispatch.retired = true;
}

void BoardScheduler::complete(BoardEvent event, std::optional<std::uint64_t> next) {
  const auto &dispatch = checked_active(event);
  if (!dispatch.retired) { throw std::logic_error("Board guest action has not retired"); }
  auto &source = state_.sources[event.source];
  if ((next && *next <= event.timestamp) ||
      source.sequence == std::numeric_limits<std::uint64_t>::max()) {
    throw std::invalid_argument(
        "Board completion needs a future timestamp and unexhausted sequence");
  }
  source.next = next;
  ++source.sequence;
  state_.active.reset();
}

void BoardScheduler::rearm(unsigned int source, std::uint64_t timestamp) {
  if (state_.active || source >= state_.sources.size() || state_.sources[source].next ||
      timestamp < state_.frontier) {
    throw std::invalid_argument("Board rearm requires an idle parked source and nonpast timestamp");
  }
  state_.sources[source].next = timestamp;
}

}
