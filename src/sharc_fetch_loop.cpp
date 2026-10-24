// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int class_shift = 40;
constexpr unsigned int immediate_loop = 12;
constexpr unsigned int counter_shift = 24;
constexpr std::uint64_t counter_mask = 0xffff;
constexpr std::uint64_t full_counter = std::uint64_t{1} << 32;

bool body_supported(std::uint64_t opcode) {
  return opcode == 0 || (opcode >> class_shift) == 1 || (opcode >> class_shift) == 2;
}

void begin(const SharcState &cpu, const SharcFetchFlow &previous, SharcFetchState &next) {
  if (!next.address || !next.flow) { throw std::logic_error("Missing SHARC loop setup phase"); }
  const auto word = *next.address;
  const auto total = (word.opcode >> counter_shift) & counter_mask;
  if ((word.opcode & sharc_pc_mask) != 1 || total == 1) {
    throw std::runtime_error("SHARC sequenced loop length/count-one timing not qualified");
  }
  if (cpu.branch || cpu.count_loop || previous.discard_decode != 0 ||
      next.flow->discard_decode != 0 || !next.decode || next.decode->address != word.address + 1 ||
      !body_supported(next.decode->opcode)) {
    throw std::runtime_error("SHARC sequenced loop setup/body context not qualified");
  }
  sharc_fetch_address(word.address + 2);
  const auto iterations = total == 0 ? full_counter : total;
  next.flow->loop = SharcFetchLoop{.setup = word.address,
                                   .body = word.address + 1,
                                   .total = iterations,
                                   .fetch_remaining = iterations,
                                   .held = next.decode};
  next.flow->next = word.address + 1;
}

void issue(SharcFetchState &next, SharcFetchLoop &loop) {
  if (next.address && next.address->address == loop.body) {
    if (loop.issued >= loop.total || !body_supported(next.address->opcode)) {
      throw std::runtime_error("SHARC sequenced loop body issue not qualified");
    }
    next.address->loop_body = true;
    ++loop.issued;
  }
  // Aborted Decode copies are absent words, never synthetic executable NOPs.
  if (next.decode && next.decode->address == loop.body && loop.issued == loop.total) {
    next.decode.reset();
  }
}

void counter(SharcFetchFlow &flow, SharcFetchLoop &loop, std::optional<std::uint32_t> fetch1) {
  if (loop.terminal) { return; }
  if (fetch1 != loop.body) {
    throw std::runtime_error("SHARC single-loop F1 counter address mismatch");
  }
  constexpr unsigned int no_tail_stalls = 4;
  const auto threshold = loop.total >= no_tail_stalls ? no_tail_stalls : 1U;
  if (loop.fetch_remaining == threshold) {
    loop.terminal = true;
    loop.pop = SharcFetchLoopPop::due;
    flow.next = loop.body + 1;
  } else {
    --loop.fetch_remaining;
    flow.next = loop.body;
  }
}

}

bool sharc_fetch_loop_stall(const SharcFetchState &transport) {
  return transport.flow && transport.flow->loop &&
         transport.flow->loop->phase == SharcFetchLoopPhase::hold;
}

void sharc_fetch_loop_flow(const SharcState &cpu,
                           const SharcFetchState &previous,
                           SharcFetchState &next) {
  if (!previous.flow || !next.flow) { throw std::logic_error("Missing SHARC loop flow"); }
  if (!previous.flow->loop) {
    if (next.address && (next.address->opcode >> class_shift) == immediate_loop) {
      begin(cpu, *previous.flow, next);
    }
    return;
  }
  auto flow = *next.flow;
  if (!flow.loop) { throw std::logic_error("Missing SHARC loop continuation"); }
  auto &loop = *flow.loop;
  if (loop.phase == SharcFetchLoopPhase::setup || loop.phase == SharcFetchLoopPhase::seed) {
    if (!loop.held) { throw std::logic_error("Missing SHARC retained loop Decode word"); }
    next.decode = loop.held;
    if (loop.phase == SharcFetchLoopPhase::setup) {
      loop.phase = SharcFetchLoopPhase::hold;
      flow.next = loop.body;
    } else {
      loop.phase = SharcFetchLoopPhase::stream;
      loop.held.reset();
    }
  }
  issue(next, loop);
  next.flow = flow;
}

void sharc_fetch_loop_clock(SharcState &cpu,
                            const SharcFetchState &previous,
                            SharcFetchState &next,
                            bool startup_stall) {
  if (!previous.flow || !previous.flow->loop) { return; }
  if (!next.flow) { throw std::logic_error("Missing SHARC loop clock flow"); }
  const auto old_loop = *previous.flow->loop;
  auto flow = *next.flow;
  if (!flow.loop) { throw std::logic_error("Missing SHARC clocked loop"); }
  auto &loop = *flow.loop;
  if (old_loop.pop == SharcFetchLoopPop::due) {
    if (next.execute) {
      if (!next.execute->loop_body) {
        throw std::runtime_error("SHARC loop pre-pop requires body Execute or bubble");
      }
      next.execute->loop_pop = true;
      loop.pop = SharcFetchLoopPop::execute;
    } else {
      sharc_clocked_loop_pop(cpu);
      loop.pop = SharcFetchLoopPop::done;
    }
  }
  if (startup_stall) { loop.phase = SharcFetchLoopPhase::seed; }
  if (old_loop.phase != SharcFetchLoopPhase::setup) { counter(flow, loop, next.fetch1); }
  next.flow = flow;
  sharc_fetch_loop_finish(next);
}

}
