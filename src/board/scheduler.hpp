// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace lead_a1 {

struct BoardSource {
  std::optional<std::uint64_t> next{std::nullopt};
  std::uint32_t priority{};
  std::uint64_t sequence{};
  bool operator==(const BoardSource &) const = default;
};

struct BoardEvent {
  std::uint64_t timestamp{};
  unsigned int source{};
  std::uint64_t sequence{};
  bool operator==(const BoardEvent &) const = default;
};

struct BoardDispatch {
  BoardEvent event;
  // A committed guest action must not run again if signal publication fails.
  bool retired{};
  bool operator==(const BoardDispatch &) const = default;
};

struct BoardScheduleState {
  std::vector<BoardSource> sources;
  std::uint64_t frontier{};
  std::optional<BoardDispatch> active{std::nullopt};
  bool operator==(const BoardScheduleState &) const = default;
};

// One owning thread. Owners supply timestamps in an explicit common unit; this
// calendar supplies no clock rates, CPU execution, peripheral or ready events.
// Sources order by timestamp, explicit priority, then stable source index.
class BoardScheduler final {
public:
  explicit BoardScheduler(BoardScheduleState initial);

  [[nodiscard]] const BoardScheduleState &state() const { return state_; }

  [[nodiscard]] std::optional<BoardEvent> next() const;
  // Repeated claim preserves the current execution/publication phase.
  [[nodiscard]] std::optional<BoardDispatch> claim();
  void retire(BoardEvent event);
  // Complete publication, then park or queue strictly later work. Failure
  // retains the retired phase for repair without executing the guest again.
  void complete(BoardEvent event, std::optional<std::uint64_t> next);
  // Rearm only a parked source outside an active dispatch, never in the past.
  void rearm(unsigned int source, std::uint64_t timestamp);

private:
  void validate() const;
  [[nodiscard]] BoardDispatch &checked_active(BoardEvent event);
  BoardScheduleState state_;
};

}
