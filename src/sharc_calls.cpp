// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "sharc_internal.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

struct Stack {
  std::array<std::optional<std::uint32_t>, SharcState::pc_stack_capacity> &frames;
  std::optional<unsigned int> &depth;
};

Stack stack(SharcState &state) {
  if (state.count_loop && state.count_loop->control) {
    auto &control = *state.count_loop->control;
    return {.frames = control.frames, .depth = control.depth};
  }
  return {.frames = state.pc_stack, .depth = state.depth};
}

bool final_four_call(const SharcState &state) {
  if (!sharc_four_loop(state) || !state.count_loop) { return false; }
  const auto &loop = *state.count_loop;
  return !loop.control && !loop.frame_live && loop.remaining == 1 && state.pc == loop.body;
}

bool final_four_return(const SharcState &state, unsigned int depth, std::uint32_t address) {
  if (!sharc_four_loop(state) || !state.count_loop) { return false; }
  const auto &loop = *state.count_loop;
  return !loop.control && !loop.frame_live && loop.remaining == 1 &&
         depth == loop.saved_depth + 1 && address == loop.end;
}

}

void sharc_push_call(SharcState &state, std::uint32_t address) {
  auto current = stack(state);
  if (!current.depth) { throw std::runtime_error("Unknown SHARC PC stack depth"); }
  constexpr unsigned int before_almost_full = 28;
  if (*current.depth >= before_almost_full) {
    throw std::runtime_error("SHARC stack-full interrupt boundary not implemented");
  }
  const auto metadata = sharc_call_frame | (final_four_call(state) ? sharc_special_frame : 0);
  current.frames[*current.depth] = metadata | address;
  ++*current.depth;
}

std::uint32_t sharc_pop_call(SharcState &state) {
  auto current = stack(state);
  if (!current.depth) { throw std::runtime_error("Unknown SHARC PC stack depth"); }
  if (*current.depth == 0) { throw std::runtime_error("SHARC empty-stack RTS not qualified"); }
  const auto frame = current.frames[*current.depth - 1];
  if (!frame) { throw std::runtime_error("Unknown SHARC return frame"); }
  const auto address = *frame & sharc_pc_mask;
  if ((*frame & sharc_call_frame) == 0 ||
      ((*frame & sharc_special_frame) != 0 && !final_four_return(state, *current.depth, address))) {
    throw std::runtime_error("SHARC loop/interrupt return frame not implemented");
  }
  --*current.depth;
  return address;
}

}
