// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "sharc_fetch_internal.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint64_t full_counter = std::uint64_t{1} << 32;
constexpr std::uint64_t immediate_max = 0xffff;
constexpr unsigned int stack_limit = 28;

void loop_word(const SharcFetchWord &word, const SharcFetchState &transport, bool execute) {
  const auto loop = transport.flow ? transport.flow->loop : std::nullopt;
  constexpr unsigned int class_shift = 40;
  const auto type = word.opcode >> class_shift;
  if (word.loop_body && word.opcode != 0 && type != 1 && type != 2) {
    throw std::invalid_argument("Invalid SHARC inherited loop body encoding");
  }
  if (word.loop_body &&
      (!loop || word.address != loop->body || word.branch || word.pm_data.value_or(false))) {
    throw std::invalid_argument("Invalid SHARC inherited loop body word");
  }
  if (word.loop_pop &&
      (!execute || !word.loop_body || !loop || loop->pop != SharcFetchLoopPop::execute)) {
    throw std::invalid_argument("Invalid SHARC inherited loop pop word");
  }
}

void metadata(const SharcFetchLoop &loop) {
  sharc_fetch_address(loop.setup);
  sharc_fetch_address(loop.body);
  sharc_fetch_address(loop.body + 1);
  if (loop.body != loop.setup + 1 || loop.total < 2 ||
      (loop.total > immediate_max && loop.total != full_counter) || loop.fetch_remaining == 0 ||
      loop.fetch_remaining > loop.total || loop.issued > loop.total || loop.retired > loop.issued ||
      loop.phase > SharcFetchLoopPhase::stream || loop.pop > SharcFetchLoopPop::done ||
      loop.terminal != (loop.pop != SharcFetchLoopPop::live)) {
    throw std::invalid_argument("Invalid SHARC inherited loop metadata");
  }
  if ((loop.phase == SharcFetchLoopPhase::stream) == loop.held.has_value() ||
      (loop.saved_depth && *loop.saved_depth >= stack_limit) ||
      (!loop.saved_depth && (loop.phase > SharcFetchLoopPhase::hold || loop.retired != 0)) ||
      (loop.saved_depth && loop.phase == SharcFetchLoopPhase::setup)) {
    throw std::invalid_argument("Invalid SHARC inherited loop phase");
  }
  if (loop.held) {
    const auto word = *loop.held;
    constexpr unsigned int class_shift = 40;
    const auto type = word.opcode >> class_shift;
    if (word.address != loop.body || word.pm_data || word.branch || word.loop_body ||
        word.loop_pop || (word.opcode != 0 && type != 1 && type != 2)) {
      throw std::invalid_argument("Invalid SHARC retained loop Decode word");
    }
  }
}

void phase(const SharcFetchLoop &loop, const SharcFetchState &transport) {
  const auto in_flight =
      static_cast<unsigned int>(transport.address && transport.address->loop_body) +
      static_cast<unsigned int>(transport.execute && transport.execute->loop_body);
  if (loop.issued - loop.retired != in_flight) {
    throw std::invalid_argument("Invalid SHARC loop issued/retired latches");
  }
  if (loop.phase == SharcFetchLoopPhase::setup &&
      (loop.issued != 0 || loop.retired != 0 || loop.saved_depth)) {
    throw std::invalid_argument("Invalid SHARC inherited loop setup phase");
  }
  if (loop.phase == SharcFetchLoopPhase::hold && (loop.issued != 1 || loop.retired != 0)) {
    throw std::invalid_argument("Invalid SHARC inherited loop hold phase");
  }
  if (loop.phase <= SharcFetchLoopPhase::hold &&
      (loop.fetch_remaining != loop.total || loop.terminal)) {
    throw std::invalid_argument("Invalid SHARC inherited early loop counter");
  }
  if (loop.phase == SharcFetchLoopPhase::seed && loop.issued != 1) {
    throw std::invalid_argument("Invalid SHARC inherited loop seed phase");
  }
  if (loop.phase == SharcFetchLoopPhase::stream && loop.issued < 2) {
    throw std::invalid_argument("Invalid SHARC inherited loop stream phase");
  }
  constexpr unsigned int no_tail_stalls = 4;
  const auto threshold = loop.total >= no_tail_stalls ? no_tail_stalls : 1U;
  if (loop.fetch_remaining < threshold || (loop.terminal && loop.fetch_remaining != threshold)) {
    throw std::invalid_argument("Invalid SHARC inherited loop termination count");
  }
}

void context(const SharcState &cpu, const SharcFetchLoop &loop) {
  if (!loop.saved_depth) {
    if (cpu.count_loop || cpu.loops_empty != true || loop.retired != 0) {
      throw std::runtime_error("SHARC sequenced loop before native DO mismatch");
    }
    return;
  }
  const bool live = loop.pop != SharcFetchLoopPop::done;
  if (cpu.depth != *loop.saved_depth + (live ? 1U : 0U) || cpu.loops_empty != !live) {
    throw std::runtime_error("SHARC sequenced loop stack mismatch");
  }
  if (loop.retired == loop.total) {
    if (cpu.count_loop || live || cpu.pc != loop.body + 1) {
      throw std::runtime_error("SHARC sequenced loop drain mismatch");
    }
    return;
  }
  const auto core = cpu.count_loop;
  if (!core || core->body != loop.body || core->end != loop.body || core->total != loop.total ||
      core->remaining != loop.total - loop.retired || core->saved_depth != loop.saved_depth ||
      core->frame_live != live || cpu.pc != loop.body || core->control ||
      (live && cpu.pc_stack[*loop.saved_depth] != loop.body)) {
    throw std::runtime_error("SHARC sequenced loop native state mismatch");
  }
}

}

void sharc_fetch_loop_validate(const SharcFetchState &transport) {
  for (const auto &word : {transport.fetch2, transport.decode}) {
    if (word && (word->loop_body || word->loop_pop)) {
      throw std::invalid_argument("Invalid SHARC loop word before Address");
    }
  }
  if (transport.address) { loop_word(*transport.address, transport, false); }
  if (transport.execute) { loop_word(*transport.execute, transport, true); }
  if (transport.flow && transport.flow->loop) {
    metadata(*transport.flow->loop);
    phase(*transport.flow->loop, transport);
    if (transport.flow->loop->pop == SharcFetchLoopPop::execute &&
        (!transport.execute || !transport.execute->loop_pop)) {
      throw std::invalid_argument("Missing SHARC pending loop pop Execute");
    }
  }
}

void sharc_fetch_loop_guard(const SharcState &cpu, const SharcFetchState &transport) {
  const auto loop = transport.flow ? transport.flow->loop : std::nullopt;
  if (!loop) {
    if (cpu.count_loop || cpu.loops_empty != true) {
      throw std::runtime_error("SHARC sequenced loop/ISR/IDLE context not qualified");
    }
    return;
  }
  if (!transport.flow || cpu.branch || transport.flow->discard_decode != 0 || transport.lookup ||
      transport.fill) {
    throw std::runtime_error("SHARC loop with branch/cache phases not qualified");
  }
  for (const auto &pending : cpu.pending) {
    if (pending) { throw std::runtime_error("SHARC loop with pending control not qualified"); }
  }
  context(cpu, *loop);
}

}
