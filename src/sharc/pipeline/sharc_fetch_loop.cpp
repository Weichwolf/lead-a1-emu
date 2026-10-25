// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_dag_internal.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/pipeline/sharc_fetch_internal.hpp"
#include "sharc/pipeline/sharc_fetch_long.hpp"
#include "sharc/visa/sharc_visa.hpp"

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
  return opcode == 0 || (opcode >> class_shift) == 1 || (opcode >> class_shift) == 2 ||
         sharc_dag_loop_body(opcode);
}

void begin(const SharcState &cpu, const SharcFetchState &previous, SharcFetchState &next) {
  if (!next.address || !next.flow || !previous.flow) {
    throw std::logic_error("Missing SHARC loop setup phase");
  }
  const auto word = *next.address;
  // A DAG hold can leave Decode absent as the unrelated DO advances to Address.
  // Keep the actual F2 word; its operands wait until it really enters Decode.
  const auto held = next.decode ? next.decode : previous.fetch2;
  const auto total = (word.opcode >> counter_shift) & counter_mask;
  const auto units = word.visa ? word.visa->words : 1U;
  if ((word.opcode & sharc_pc_mask) != units || total == 1) {
    throw std::runtime_error("SHARC sequenced loop length/count-one timing not qualified");
  }
  if (cpu.branch || cpu.count_loop || previous.flow->discard_decode != 0 ||
      next.flow->discard_decode != 0 || !held || held->address != word.address + units ||
      !body_supported(held->opcode) || (word.visa && (!held->visa || held->visa->words != 3))) {
    throw std::runtime_error("SHARC sequenced loop setup/body context not qualified");
  }
  sharc_fetch_program_address(word.address + (units * 2));
  const auto iterations = total == 0 ? full_counter : total;
  next.flow->loop = SharcFetchLoop{.setup = word.address,
                                   .body = word.address + units,
                                   .total = iterations,
                                   .fetch_remaining = iterations,
                                   .held = held,
                                   .units = units};
  next.flow->next = word.address + units;
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
    flow.next = loop.body + loop.units;
  } else {
    --loop.fetch_remaining;
    flow.next = loop.body;
  }
}

}

bool sharc_fetch_loop_stall(const SharcFetchState &transport) {
  if (!transport.flow) { return false; }
  return (transport.flow->loop && transport.flow->loop->phase == SharcFetchLoopPhase::hold) ||
         (transport.flow->long_loop &&
          transport.flow->long_loop->four == SharcFetchFourPhase::hold) ||
         sharc_fetch_long_return_stall(transport);
}

void sharc_fetch_loop_flow(const SharcState &cpu,
                           const SharcFetchState &previous,
                           SharcFetchState &next) {
  if (!previous.flow || !next.flow) { throw std::logic_error("Missing SHARC loop flow"); }
  if (sharc_fetch_long_flow(cpu, previous, next)) { return; }
  if (!previous.flow->loop) {
    if (next.address && (next.address->opcode >> class_shift) == immediate_loop) {
      begin(cpu, previous, next);
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
  if (previous.flow && previous.flow->long_loop) {
    if (startup_stall && previous.flow->long_loop->four == SharcFetchFourPhase::hold) {
      sharc_fetch_long_clock(cpu, next);
    }
    const auto native = cpu.count_loop;
    if (native && native->four_pop_pending) {
      if (next.execute) {
        throw std::runtime_error("SHARC four-loop pop requires its real bubble");
      }
      sharc_four_loop_clock_pop(cpu);
    }
    return;
  }
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
  if (old_loop.phase != SharcFetchLoopPhase::setup && loop.issued != 0) {
    counter(flow, loop, next.fetch1);
  }
  next.flow = flow;
  sharc_fetch_loop_finish(next);
}

}
