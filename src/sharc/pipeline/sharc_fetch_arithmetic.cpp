// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/pipeline/sharc_fetch_internal.hpp"
#include "sharc/pipeline/sharc_fetch_mode.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

bool draining(const SharcState &cpu) {
  return cpu.arithmetic_loop &&
         cpu.arithmetic_loop->phase == SharcArithmeticPhase::fetched_draining;
}

bool exit_marker(const std::optional<SharcFetchWord> &word) {
  return word && word->arithmetic_exit;
}

bool marked_pm_end(const std::optional<SharcFetchWord> &word) {
  return word && word->arithmetic_exit && word->pm_data == true &&
         sharc_cache_pm_encoding(word->opcode);
}

void entry_branch(const SharcArithmeticLoop &loop, const SharcFetchWord &word) {
  if (!sharc_fetch_branch_encoding(word.opcode)) { return; }
  constexpr unsigned int class_shift = 40;
  constexpr unsigned int absolute_jump = 6;
  constexpr unsigned int relative_jump = 7;
  constexpr std::uint64_t call = 0x8000000000;
  const auto type = word.opcode >> class_shift;
  const bool inside = word.address >= loop.body && word.address <= loop.end;
  if (inside || (type != absolute_jump && type != relative_jump) || (word.opcode & call) != 0 ||
      !word.branch || !word.branch->taken || word.branch->target != loop.body) {
    throw std::runtime_error("SHARC arithmetic fetch body/other entry branch not qualified");
  }
}

void branch_word(const SharcState &cpu, const SharcFetchWord &word) {
  if (!cpu.arithmetic_loop) { throw std::logic_error("Missing arithmetic branch context"); }
  const auto loop = *cpu.arithmetic_loop;
  if (loop.phase == SharcArithmeticPhase::fetched_suspended) {
    constexpr unsigned int class_shift = 40;
    constexpr unsigned int rti = 11;
    if (sharc_fetch_branch_encoding(word.opcode) && (word.opcode >> class_shift) != rti) {
      throw std::runtime_error("SHARC suspended-loop ISR branch other than RTI not qualified");
    }
    return;
  }
  entry_branch(loop, word);
}

void control_context(const SharcState &cpu, const SharcFetchState &transport) {
  if (!cpu.arithmetic_loop) { throw std::logic_error("Missing arithmetic control context"); }
  if (cpu.branch && (draining(cpu) || cpu.branch->target != cpu.arithmetic_loop->body ||
                     (cpu.pc >= cpu.arithmetic_loop->body && cpu.pc <= cpu.arithmetic_loop->end))) {
    throw std::runtime_error("SHARC arithmetic pending branch is not its external entry");
  }
  for (const auto &word : {transport.address, transport.execute}) {
    if (word) { branch_word(cpu, *word); }
  }
  for (const auto &word :
       {transport.fetch2, transport.decode, transport.address, transport.execute}) {
    if (word && sharc_float_multiply_encoding(word->opcode)) {
      throw std::runtime_error("SHARC multiplier arithmetic-loop condition latency not qualified");
    }
  }
  if (sharc_fetched_arithmetic(cpu) && cpu.pending[sharc_mode1]) {
    throw std::runtime_error("SHARC arithmetic-loop pending MODE1 timing not qualified");
  }
}

void deferred_window(const SharcState &cpu, const SharcFetchState &transport) {
  if (!cpu.arithmetic_loop) { throw std::logic_error("Missing deferred arithmetic context"); }
  constexpr std::uint64_t modulus = std::uint64_t{1} << 24;
  constexpr unsigned int window = 4;
  const auto end = cpu.arithmetic_loop->end;
  for (const auto address : {std::optional{cpu.pc}, transport.fetch1}) {
    if (address && ((modulus + end - *address) & sharc_pc_mask) <= window) {
      throw std::runtime_error("SHARC fetched deferred arithmetic-loop end window not qualified");
    }
  }
}

void drain_columns(const SharcArithmeticLoop &loop, const SharcFetchState &transport) {
  if (!transport.flow || !transport.fetch2) {
    throw std::invalid_argument("Missing SHARC arithmetic drain pipeline");
  }
  const auto mask = transport.flow->discard_decode;
  const auto f2 = transport.fetch2->address;
  bool valid{};
  if (transport.decode && transport.decode->arithmetic_exit) {
    valid = mask == 3 && transport.fetch1 == loop.body + 1 && f2 == loop.body;
  } else if (transport.address && transport.address->arithmetic_exit) {
    valid =
        mask == 1 && !transport.decode && transport.fetch1 == loop.end + 1 && f2 == loop.body + 1;
  } else if (marked_pm_end(transport.execute) && transport.lookup && !transport.lookup->hit &&
             !transport.lookup->wait && mask == 1) {
    // Table4-31's partial stall advances the marked Address word to Execute;
    // it does not consume either held speculative Decode input.
    valid = !transport.decode && !transport.address && transport.fetch1 == loop.end + 1 &&
            f2 == loop.body + 1;
  } else {
    valid = mask == 0 && !transport.decode && !transport.address &&
            transport.fetch1 == loop.end + 2 && f2 == loop.end + 1;
  }
  if (!valid) { throw std::invalid_argument("Invalid SHARC arithmetic drain pipeline columns"); }
}

void exit_word(const SharcState &cpu, const SharcFetchState &transport) {
  if (!cpu.arithmetic_loop) { throw std::logic_error("Missing arithmetic end context"); }
  unsigned int markers{};
  for (const auto &word : {transport.decode, transport.address, transport.execute}) {
    if (word && word->arithmetic_exit) {
      if (word->address != cpu.arithmetic_loop->end) {
        throw std::invalid_argument("SHARC arithmetic exit marker has the wrong end address");
      }
      ++markers;
    }
  }
  if (markers != (draining(cpu) ? 1U : 0U)) {
    throw std::invalid_argument("SHARC arithmetic drain requires its actual retained end word");
  }
  if (draining(cpu)) { drain_columns(*cpu.arithmetic_loop, transport); }
}

void mark_exit(SharcFetchState &transport, std::uint32_t end) {
  // In the qualified unstalled stream, Decode retains the final end word.
  // F1/F2 are flushed, rather than becoming guest NOPs (PGR4-54/58).
  if (transport.decode && transport.decode->address == end) {
    transport.decode->arithmetic_exit = true;
    return;
  }
  throw std::runtime_error("SHARC arithmetic termination before a retained end not qualified");
}

void terminate(SharcState &cpu, SharcFetchState &transport) {
  if (!cpu.arithmetic_loop || !transport.flow) {
    throw std::logic_error("Missing arithmetic termination context");
  }
  auto &loop = *cpu.arithmetic_loop;
  if (transport.lookup && transport.lookup->wait) {
    throw std::runtime_error(
        "SHARC arithmetic termination with pending cache transfer not qualified");
  }
  if (transport.flow->discard_decode != 0 || cpu.branch) {
    throw std::runtime_error("SHARC arithmetic termination with entry branch not qualified");
  }
  mark_exit(transport, loop.end);
  cpu.depth = loop.saved_depth;
  cpu.loops_empty = true;
  loop.phase = SharcArithmeticPhase::fetched_draining;
  sharc_stack_status(cpu);
  transport.flow->next = loop.end + 1;
  transport.flow->discard_decode = 3; // The next two Decode inputs, currently F2/F1.
}

}

void sharc_fetch_arithmetic_validate(const SharcFetchState &transport) {
  if (transport.fetch2 && transport.fetch2->arithmetic_exit) {
    throw std::invalid_argument("SHARC arithmetic exit marker before Decode");
  }
  unsigned int markers{};
  for (const auto &word : {transport.decode, transport.address, transport.execute}) {
    if (word && word->arithmetic_exit) { ++markers; }
  }
  if (markers > 1 ||
      (markers != 0 && (!transport.flow || transport.flow->loop || transport.flow->long_loop))) {
    throw std::invalid_argument("Invalid SHARC arithmetic exit transport context");
  }
}

void sharc_fetch_arithmetic_guard(const SharcState &cpu, const SharcFetchState &transport) {
  if (!cpu.arithmetic_loop) {
    if (exit_marker(transport.decode) || exit_marker(transport.address) ||
        exit_marker(transport.execute)) {
      throw std::invalid_argument("SHARC arithmetic end marker without its native frame");
    }
    return;
  }
  if (!transport.flow) { throw std::logic_error("Missing SHARC fetched arithmetic-loop context"); }
  const bool suspended = cpu.arithmetic_loop->phase == SharcArithmeticPhase::fetched_suspended;
  const bool draining_cache = draining(cpu) && (!transport.lookup || !transport.lookup->wait ||
                                                marked_pm_end(transport.address));
  const bool cached = cpu.arithmetic_loop->phase == SharcArithmeticPhase::fetched || draining_cache;
  if (cpu.count_loop || (cpu.interrupt && !suspended) || transport.flow->loop ||
      transport.flow->long_loop || ((transport.lookup || transport.fill) && !cached)) {
    throw std::runtime_error("SHARC fetched mixed arithmetic-loop context not qualified");
  }
  sharc_validate_arithmetic_loop(cpu);
  control_context(cpu, transport);
  exit_word(cpu, transport);
  if (cpu.arithmetic_loop->phase == SharcArithmeticPhase::deferred) {
    deferred_window(cpu, transport);
  } else if (!sharc_fetched_arithmetic(cpu) &&
             cpu.arithmetic_loop->phase != SharcArithmeticPhase::active) {
    throw std::runtime_error("SHARC fetched active/mixed arithmetic-loop context not qualified");
  }
}

void sharc_fetch_arithmetic_clock(SharcState &cpu,
                                  SharcFetchState &transport,
                                  const SharcFetchEvent &event) {
  if (!cpu.arithmetic_loop) { return; }
  if (!transport.flow) { throw std::logic_error("Missing arithmetic fetch cursor"); }
  if (event.control_stall && sharc_fetch_interrupt_mode(cpu)) {
    sharc_fetch_interrupt_mode_guard(cpu, transport);
    return; // No future-loop entry, termination test or frame change on this hold.
  }
  if (event.stalled && !event.address_accepted && !event.control_stall && !event.loop_stall &&
      !event.dag_stall && !event.compute_stall && transport.lookup && !transport.lookup->wait &&
      !cpu.branch) {
    const auto phase = cpu.arithmetic_loop->phase;
    if ((phase == SharcArithmeticPhase::fetched && transport.flow->discard_decode == 0) ||
        (phase == SharcArithmeticPhase::fetched_draining && transport.flow->discard_decode == 1 &&
         marked_pm_end(transport.execute))) {
      return; // Held F1 and the flush mask do not advance on the cache partial stall.
    }
  }
  if (!event.address_accepted || event.stalled) {
    throw std::runtime_error("SHARC arithmetic-loop fetch stall/cache timing not qualified");
  }
  auto &loop = *cpu.arithmetic_loop;
  if (loop.phase == SharcArithmeticPhase::deferred) {
    if (transport.fetch1 != loop.body) { return; }
    if (loop.end <= loop.body) {
      throw std::runtime_error("SHARC wrapped/single arithmetic fetch entry not qualified");
    }
    loop.phase = SharcArithmeticPhase::fetched;
  } else if (loop.phase == SharcArithmeticPhase::active) {
    loop.phase = SharcArithmeticPhase::fetched;
  }
  if (loop.phase != SharcArithmeticPhase::fetched) { return; }
  if (transport.fetch1 == loop.end) { transport.flow->next = loop.body; }
  constexpr unsigned int forever = 31;
  if (transport.fetch1 == loop.body + 1 && loop.termination != forever &&
      sharc_condition(cpu, loop.termination)) {
    terminate(cpu, transport);
  }
}

SharcLoopClock sharc_fetch_arithmetic_retirement(const SharcState &cpu,
                                                 const SharcFetchState &transport) {
  if (!transport.flow || !transport.execute || !cpu.arithmetic_loop ||
      !sharc_fetched_arithmetic(cpu)) {
    throw std::logic_error("Missing SHARC fetched arithmetic retirement");
  }
  const auto word = *transport.execute;
  branch_word(cpu, word);
  return word.arithmetic_exit ? SharcLoopClock::arithmetic_exit : SharcLoopClock::arithmetic;
}

}
