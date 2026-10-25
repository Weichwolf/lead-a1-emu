// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_visa.hpp"
#include "sharc_fetch_internal.hpp"

#include <optional>
#include <stdexcept>

namespace lead_a1::detail {

void sharc_fetch_visa_loop_entry(const SharcFetchState &previous, const SharcFetchState &next) {
  constexpr unsigned int class_shift = 40;
  constexpr unsigned int immediate_do = 12;
  const auto word = next.address;
  if (!word || !word->visa || (word->opcode >> class_shift) != immediate_do) { return; }
  if (!previous.flow || previous.flow->loop || !previous.visa || previous.visa->count != 3 ||
      previous.visa->delay || previous.visa->decode != word->address + 3 ||
      previous.fetch1 != word->address + 6 || next.fetch1 != word->address + 9 || !next.decode ||
      !next.decode->visa || next.decode->visa->words != 3 ||
      next.decode->address != word->address + 3) {
    throw std::runtime_error("SHARC clocked VISA single-body input alignment/length not qualified");
  }
}

bool sharc_fetch_visa_loop_skip(const SharcFetchState &previous,
                                const std::optional<SharcFetchWord> &address) {
  const auto loop = previous.flow ? previous.flow->loop : std::nullopt;
  if (!loop || loop->units != 3) { return false; }
  const auto issuing = static_cast<unsigned int>(address && address->address == loop->body);
  const bool held_decode =
      loop->phase == SharcFetchLoopPhase::setup || loop->phase == SharcFetchLoopPhase::seed;
  const bool aborted_body =
      previous.visa && previous.visa->decode == loop->body && loop->issued + issuing >= loop->total;
  if (!held_decode && !aborted_body) { return false; }
  if (!previous.visa || previous.visa->count != 3 || previous.visa->delay) {
    throw std::runtime_error("SHARC VISA loop Decode override requires one actual burst");
  }
  return true;
}

void sharc_fetch_visa_loop_response(const SharcFetchState &previous, SharcFetchState &next) {
  const auto loop = previous.flow ? previous.flow->loop : std::nullopt;
  if (!loop || loop->units != 3 || !previous.fetch1 || !next.visa ||
      next.visa->fetch == previous.fetch1) {
    return;
  }
  if (previous.fetch1 != loop->body || next.visa->count != 0 || next.visa->delay) {
    throw std::runtime_error("SHARC VISA loop F2 requires its actual refetched body burst");
  }
  next.visa = SharcVisaState{.decode = *previous.fetch1, .fetch = *previous.fetch1};
}

bool sharc_fetch_visa_loop_request(const SharcFetchState &state) {
  const auto loop = state.flow ? state.flow->loop : std::nullopt;
  return loop && loop->units == 3 && state.fetch1 == loop->body && state.visa &&
         state.visa->count == 3 && !state.visa->delay && !state.visa_fetch_aborted;
}

}
