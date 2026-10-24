// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_fetch_long.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {

SharcLoopClock sharc_fetch_loop_retirement(const SharcState &cpu, SharcFetchState &transport) {
  if (sharc_fetched_arithmetic(cpu)) { return sharc_fetch_arithmetic_retirement(cpu, transport); }
  if (transport.flow && transport.flow->long_loop) {
    sharc_fetch_long_retirement(cpu, transport);
    return transport.flow->long_loop->four == SharcFetchFourPhase::none ? SharcLoopClock::functional
                                                                        : SharcLoopClock::clocked;
  }
  if (!transport.flow || !transport.flow->loop) { return SharcLoopClock::functional; }
  auto &loop = *transport.flow->loop;
  const auto word = transport.execute;
  if (!word) { throw std::logic_error("Missing SHARC loop Execute"); }
  if (!loop.saved_depth && loop.phase == SharcFetchLoopPhase::setup && !word->loop_body) {
    return SharcLoopClock::functional;
  }
  if (word->address == loop.setup && !loop.saved_depth) {
    constexpr unsigned int class_shift = 40;
    constexpr unsigned int immediate_loop = 12;
    constexpr unsigned int counter_shift = 24;
    constexpr std::uint64_t count_mask = 0xffff;
    constexpr std::uint64_t address_mask = 0xffffff;
    constexpr std::uint64_t full_count = std::uint64_t{1} << 32;
    const auto count = (word->opcode >> counter_shift) & count_mask;
    if (!cpu.depth || loop.phase != SharcFetchLoopPhase::hold) {
      throw std::runtime_error("SHARC loop native DO retirement phase mismatch");
    }
    if ((word->opcode >> class_shift) != immediate_loop || (word->opcode & address_mask) != 1 ||
        loop.total != (count == 0 ? full_count : count)) {
      throw std::runtime_error("SHARC retained DO disagrees with loop metadata");
    }
    loop.saved_depth = cpu.depth;
    return SharcLoopClock::functional;
  }
  if (!word->loop_body || !loop.saved_depth || loop.retired >= loop.total) {
    throw std::runtime_error("SHARC loop native body retirement phase mismatch");
  }
  ++loop.retired;
  if (word->loop_pop) {
    loop.pop = SharcFetchLoopPop::done;
    return SharcLoopClock::clocked_pop;
  }
  return SharcLoopClock::clocked;
}

void sharc_fetch_loop_finish(SharcFetchState &transport) noexcept {
  sharc_fetch_long_finish(transport);
  if (!transport.flow || !transport.flow->loop) { return; }
  const auto &loop = *transport.flow->loop;
  if (loop.retired != loop.total || loop.pop != SharcFetchLoopPop::done ||
      transport.fetch1 == loop.body) {
    return;
  }
  for (const auto &word :
       {transport.fetch2, transport.decode, transport.address, transport.execute}) {
    if (word && word->address == loop.body) { return; }
  }
  transport.flow->loop.reset();
}

}
