// SPDX-License-Identifier: GPL-3.0-only
#include "sharc_fetch_long.hpp"

#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int class_shift = 40;
constexpr unsigned int immediate_do = 12;
constexpr unsigned int register_do = 13;
constexpr std::uint64_t call = 0x8000000000;
constexpr std::uint64_t delayed = 0x4000000;

bool inside(const SharcFetchLongLoop &loop, std::uint32_t address) {
  return address > loop.setup && address <= loop.end;
}

void begin(const SharcState &cpu, const SharcFetchState &previous, SharcFetchState &next) {
  if (!next.address || !next.flow || !previous.flow) {
    throw std::logic_error("Missing SHARC long-loop setup");
  }
  const auto word = *next.address;
  const auto end = std::uint64_t{word.address} + (word.opcode & sharc_pc_mask);
  constexpr std::uint32_t native_end = 0x100000;
  if (end >= native_end || cpu.count_loop || cpu.arithmetic_loop || cpu.branch ||
      cpu.loops_empty != true || previous.flow->loop || previous.flow->discard_decode != 0 ||
      next.flow->discard_decode != 0) {
    throw std::runtime_error("SHARC long-loop setup context not qualified");
  }
  next.flow->long_loop = SharcFetchLongLoop{.setup = word.address,
                                            .end = static_cast<std::uint32_t>(end),
                                            .opcode = word.opcode,
                                            .counter = {}};
  if (end == std::uint64_t{word.address} + 4) {
    next.flow->long_loop->four = SharcFetchFourPhase::setup;
  }
}

void body_control(const SharcFetchLongLoop &loop, const SharcFetchState &next) {
  if (!next.address) { return; }
  const auto word = *next.address;
  const auto type = word.opcode >> class_shift;
  if (type == immediate_do || type == register_do) {
    throw std::runtime_error("SHARC nested fetched counter loops not qualified");
  }
  if (!inside(loop, word.address) || !word.branch) { return; }
  constexpr unsigned int absolute = 6;
  constexpr unsigned int relative = 7;
  if (loop.four != SharcFetchFourPhase::none) {
    if ((type != absolute && type != relative) ||
        (word.opcode & (call | delayed)) != (call | delayed) || word.address != loop.setup + 1 ||
        (word.branch->target && inside(loop, *word.branch->target))) {
      throw std::runtime_error("SHARC fetched four-loop body branch not qualified");
    }
    return;
  }
  if ((type != absolute && type != relative) || (word.opcode & call) == 0 ||
      (word.opcode & delayed) != 0 || word.address > loop.end - 4 ||
      (word.branch->target && inside(loop, *word.branch->target))) {
    throw std::runtime_error("SHARC fetched long-loop body branch not qualified");
  }
}

void end_fetch(const SharcState &cpu, SharcFetchState &next, bool preserve_next) {
  if (!next.flow || !next.flow->long_loop) { throw std::logic_error("Missing SHARC end fetch"); }
  auto &loop = *next.flow->long_loop;
  const auto native = cpu.count_loop;
  const auto depth = native && native->control ? native->control->depth : cpu.depth;
  if (!loop.counter || !native || !depth ||
      *depth != native->saved_depth + (native->frame_live ? 1U : 0U)) {
    throw std::runtime_error("SHARC long-loop end fetch during a callee not qualified");
  }
  auto &count = *loop.counter;
  if (loop.refetch_end) {
    loop.refetch_end = false;
  } else {
    if (count.terminal) { throw std::runtime_error("SHARC repeated long-loop end fetch"); }
    if (count.remaining == 1) {
      count.terminal = true;
    } else {
      --count.remaining;
    }
    loop.refetch_end = preserve_next;
  }
  if (!preserve_next) { next.flow->next = count.terminal ? loop.end + 1 : loop.setup + 1; }
}

}

bool sharc_fetch_long_flow(const SharcState &cpu,
                           const SharcFetchState &previous,
                           SharcFetchState &next) {
  if (!previous.flow || !next.flow) { throw std::logic_error("Missing SHARC long-loop flow"); }
  if (!previous.flow->long_loop) {
    const auto word = next.address;
    const auto type = word ? word->opcode >> class_shift : 0;
    if (!word || (type != immediate_do && type != register_do) ||
        (word->opcode & sharc_pc_mask) < 4) {
      return false;
    }
    begin(cpu, previous, next);
    return true;
  }
  auto &loop = next.flow->long_loop;
  if (!loop) { throw std::logic_error("Lost SHARC fetched long loop"); }
  body_control(*loop, next);
  if (loop->four == SharcFetchFourPhase::setup) {
    if (!next.decode || next.decode->address != loop->setup + 2) {
      throw std::runtime_error("SHARC missing four-loop startup Decode");
    }
    loop->held = next.decode;
    next.decode.reset();
    loop->four = SharcFetchFourPhase::hold;
    // The held raw word already consumed its position in the branch mask.
    return true;
  }
  if (next.fetch1 != loop->end) { return true; }
  const auto word = next.address;
  end_fetch(cpu, next, word && word->branch && word->branch->taken);
  return true;
}

void sharc_fetch_long_clock(const SharcState &cpu, SharcFetchState &next) {
  if (!next.flow || !next.flow->long_loop ||
      next.flow->long_loop->four != SharcFetchFourPhase::hold) {
    throw std::logic_error("Missing SHARC four-loop release");
  }
  auto &loop = *next.flow->long_loop;
  if (!loop.held || next.decode) { throw std::logic_error("Invalid SHARC four-loop held Decode"); }
  next.decode = loop.held;
  loop.held.reset();
  loop.four = SharcFetchFourPhase::stream;
  const auto word = next.execute;
  end_fetch(cpu, next, word && word->branch && word->branch->taken);
}

bool sharc_fetch_long_return_stall(const SharcFetchState &transport) {
  if (!transport.flow || !transport.flow->long_loop) { return false; }
  const auto loop = *transport.flow->long_loop;
  if (loop.four != SharcFetchFourPhase::stream || !loop.refetch_end) { return false; }
  const auto word = transport.address;
  constexpr unsigned int rts = 10;
  return word && (word->opcode >> class_shift) == rts && word->branch && word->branch->taken &&
         word->branch->target == loop.end;
}

void sharc_fetch_long_retirement(const SharcState &cpu, SharcFetchState &transport) {
  if (!transport.flow || !transport.flow->long_loop || !transport.execute) {
    throw std::logic_error("Missing SHARC long-loop retirement");
  }
  auto &loop = *transport.flow->long_loop;
  const auto word = *transport.execute;
  if (!loop.counter) {
    if (word.address != loop.setup) { return; } // An older actual instruction may retire first.
    if (word.opcode != loop.opcode) {
      throw std::runtime_error("SHARC retained long DO disagrees with setup");
    }
    // PGR4-45: Address selects flow; the native DO loads its counter in Execute.
    // Validate on a CPU copy before the actual instruction transaction commits.
    auto checked = cpu;
    sharc_begin_count_loop(checked, word.opcode);
    if (!checked.count_loop) { throw std::logic_error("Missing validated native counter"); }
    const auto native = *checked.count_loop;
    loop.counter = SharcFetchLongCounter{
        .total = native.total, .remaining = native.total, .saved_depth = native.saved_depth};
    return;
  }
  if (word.address == loop.end) {
    if (!cpu.count_loop || cpu.pc != loop.end || loop.counter->retired >= loop.counter->total) {
      throw std::runtime_error("SHARC fetched long-loop end retirement mismatch");
    }
    ++loop.counter->retired;
  }
}

void sharc_fetch_long_finish(SharcFetchState &transport) noexcept {
  if (!transport.flow || !transport.flow->long_loop) { return; }
  const auto loop = *transport.flow->long_loop;
  if (!loop.counter || loop.counter->retired != loop.counter->total || !loop.counter->terminal ||
      (transport.fetch1 && inside(loop, *transport.fetch1))) {
    return;
  }
  for (const auto &word :
       {transport.fetch2, transport.decode, transport.address, transport.execute}) {
    if (word && inside(loop, word->address)) { return; }
  }
  transport.flow->long_loop.reset();
}

}
