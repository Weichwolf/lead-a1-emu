// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_fetch_long.hpp"
#include "sharc_internal.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

bool setup_word(const SharcFetchLongLoop &loop, const SharcFetchState &transport) {
  const std::array words{transport.address, transport.execute};
  return std::ranges::any_of(words, [&](const auto &word) {
    return word && word->address == loop.setup && word->opcode == loop.opcode;
  });
}

void counter(const SharcFetchLongLoop &loop) {
  if (!loop.counter) { return; }
  const auto count = *loop.counter;
  constexpr std::uint64_t full_count = std::uint64_t{1} << 32;
  constexpr unsigned int stack_limit = 28;
  if (count.total == 0 || count.total > full_count || count.remaining == 0 ||
      count.remaining > count.total || count.retired > count.total ||
      count.saved_depth >= stack_limit || (count.terminal && count.remaining != 1) ||
      (count.retired == count.total && !count.terminal)) {
    throw std::invalid_argument("Invalid SHARC fetched long-loop counter");
  }
  constexpr unsigned int class_shift = 40;
  constexpr unsigned int immediate = 12;
  constexpr unsigned int count_shift = 24;
  constexpr std::uint64_t count_mask = 0xffff;
  if ((loop.opcode >> class_shift) == immediate) {
    const auto value = (loop.opcode >> count_shift) & count_mask;
    if (count.total != (value == 0 ? full_count : value)) {
      throw std::invalid_argument("Invalid SHARC immediate long-loop count");
    }
  }
}

}

void sharc_fetch_long_validate(const SharcFetchState &transport) {
  if (!transport.flow || !transport.flow->long_loop) { return; }
  if (transport.flow->loop || transport.flow->multiplier_branch_wait) {
    throw std::invalid_argument("Invalid overlapping SHARC fetched loop contexts");
  }
  const auto loop = *transport.flow->long_loop;
  sharc_fetch_address(loop.setup);
  sharc_fetch_address(loop.end);
  sharc_fetch_address(loop.end + 1);
  constexpr unsigned int class_shift = 40;
  constexpr unsigned int immediate = 12;
  constexpr unsigned int ureg = 13;
  constexpr std::uint64_t address_mask = 0xffffff;
  constexpr std::uint64_t reserved = 0x80ff000000;
  const auto type = loop.opcode >> class_shift;
  if ((type != immediate && type != ureg) || loop.end <= loop.setup || loop.end - loop.setup < 5 ||
      (loop.opcode & address_mask) != loop.end - loop.setup ||
      (type == ureg && (loop.opcode & reserved) != 0) ||
      (!loop.counter && !setup_word(loop, transport))) {
    throw std::invalid_argument("Invalid SHARC fetched long-loop setup");
  }
  counter(loop);
}

void sharc_fetch_long_guard(const SharcState &cpu, const SharcFetchState &transport) {
  if (!transport.flow || !transport.flow->long_loop) {
    throw std::logic_error("Missing SHARC fetched long-loop context");
  }
  if (cpu.branch || cpu.interrupt || cpu.arithmetic_loop || transport.lookup || transport.fill) {
    throw std::runtime_error("SHARC fetched long loop with branch/ISR/cache context not qualified");
  }
  for (const auto &pending : cpu.pending) {
    if (pending) { throw std::runtime_error("SHARC fetched long loop with pending control"); }
  }
  const auto loop = *transport.flow->long_loop;
  if (!loop.counter) {
    if (cpu.count_loop || cpu.loops_empty != true || (cpu.pc > loop.setup && cpu.pc <= loop.end)) {
      throw std::runtime_error("SHARC fetched long loop before native DO mismatch");
    }
    return;
  }
  const auto count = *loop.counter;
  const auto native = cpu.count_loop;
  if (count.retired == count.total) {
    if (native || cpu.pc != loop.end + 1 || cpu.depth != count.saved_depth) {
      throw std::runtime_error("SHARC fetched long-loop drain mismatch");
    }
    return;
  }
  if (!native || native->body != loop.setup + 1 || native->end != loop.end ||
      native->total != count.total || native->remaining != count.total - count.retired ||
      native->saved_depth != count.saved_depth || native->control ||
      count.remaining > native->remaining || native->remaining - count.remaining > 1 ||
      (count.terminal && native->remaining != 1) || (!count.terminal && !native->frame_live)) {
    throw std::runtime_error("SHARC fetched long-loop/native counter mismatch");
  }
  sharc_validate_count_loop(cpu);
}

}
