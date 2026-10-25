// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_visa.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint64_t class_mask = 0xff0000000000;
constexpr std::uint64_t condition_mask = 0x3e00000000;
constexpr std::uint64_t delayed = 0x4000000;
constexpr std::uint64_t call = 0x8000000000;

void deferred_arithmetic_setup(const SharcState &cpu, const SharcFetchWord &word) {
  constexpr std::uint64_t fields = 0x0e3e00ffffff;
  constexpr unsigned int counter_condition = 15;
  const auto condition = (word.opcode >> 33) & 31;
  const auto end = static_cast<std::uint32_t>(
      (std::uint64_t{word.address} + (word.opcode & sharc_pc_mask)) & sharc_pc_mask);
  const bool short_forward = end > word.address && end - word.address < 5;
  if ((word.opcode & ~fields) != 0 || condition == counter_condition || short_forward ||
      cpu.branch || cpu.count_loop || cpu.arithmetic_loop || cpu.loops_empty != true) {
    throw std::runtime_error("SHARC fetched arithmetic DO setup not qualified");
  }
  sharc_fetch_address(end);
  sharc_fetch_address(word.address + 1);
}

void other_control(const SharcState &cpu, const SharcFetchWord &word) {
  constexpr unsigned int class_shift = 40;
  const auto type = word.opcode >> class_shift;
  constexpr unsigned int first_other_control = 8;
  constexpr unsigned int last_other_control = 14;
  constexpr unsigned int immediate_loop = 12;
  constexpr unsigned int register_loop = 13;
  constexpr unsigned int arithmetic_loop = 14;
  constexpr std::uint64_t idle = 0x008000000000;
  if (word.visa && type == immediate_loop && word.visa->words == 3 &&
      (word.opcode & sharc_pc_mask) == 3) {
    return; // The loop transport separately checks the actual buffered body.
  }
  if (word.visa && ((type >= first_other_control && type <= last_other_control && type != 10) ||
                    word.opcode == idle)) {
    throw std::runtime_error("SHARC clocked VISA loop/indirect/RTI timing not qualified");
  }
  if (type == arithmetic_loop) {
    deferred_arithmetic_setup(cpu, word);
    return;
  }
  if (type == immediate_loop || (type == register_loop && (word.opcode & sharc_pc_mask) >= 4)) {
    return;
  }
  if (word.opcode == idle) { return; }
  if ((type >= first_other_control && type <= last_other_control) ||
      (type == 0 && word.opcode != 0)) {
    throw std::runtime_error("SHARC sequenced RTI/loop/indirect/IDLE control not qualified");
  }
}

std::optional<SharcFetchBranch> branch(const SharcState &cpu, const SharcFetchWord &word) {
  constexpr unsigned int class_shift = 40;
  constexpr unsigned int relative = 7;
  constexpr unsigned int rts = 10;
  constexpr unsigned int rti = 11;
  const auto type = word.opcode >> class_shift;
  if (sharc_fetch_branch_encoding(word.opcode)) {
    if (word.visa && type == rti) {
      throw std::runtime_error("SHARC clocked VISA RTI timing not qualified");
    }
    if (word.visa && (word.opcode & delayed) != 0) {
      throw std::runtime_error("SHARC clocked VISA delayed branch timing not qualified");
    }
    constexpr auto direct_fields = class_mask | condition_mask | delayed | call | sharc_pc_mask;
    constexpr auto return_fields = class_mask | condition_mask | delayed;
    if ((word.opcode & ~(type == rts || type == rti ? return_fields : direct_fields)) != 0) {
      throw std::runtime_error("SHARC sequenced parallel/LA/CI/reserved branch not qualified");
    }
    const auto condition = static_cast<unsigned int>((word.opcode >> 33) & 31);
    static_cast<void>(sharc_fetch_condition_mask(condition));
    SharcFetchBranch decision{.taken = sharc_condition(cpu, condition)};
    if (!decision.taken) { return decision; }
    auto address = static_cast<std::uint32_t>(word.opcode) & sharc_pc_mask;
    if (type == relative) { address = (word.address + address) & sharc_pc_mask; }
    if (type == rts) {
      auto stack = cpu;
      address = sharc_pop_call(stack);
    }
    if (type == rti) {
      auto stack = cpu;
      const auto returned = sharc_return_interrupt(stack, word.opcode, true);
      if (!returned) { throw std::logic_error("Missing SHARC interrupt return target"); }
      address = *returned;
    }
    sharc_fetch_program_address(address);
    decision.target = address;
    return decision;
  }
  other_control(cpu, word);
  return {};
}

}

void sharc_fetch_flow_guard(const SharcState &cpu, const SharcFetchState &transport) {
  if (cpu.idle_pc) {
    throw std::runtime_error("SHARC sequenced loop/ISR/IDLE context not qualified");
  }
  sharc_fetch_loop_guard(cpu, transport);
  sharc_interrupt_guard(cpu);
}

bool sharc_fetch_flow(const SharcState &cpu,
                      const SharcFetchState &previous,
                      SharcFetchState &next,
                      bool address_accepted) try {
  const auto flow = previous.flow;
  if (!flow) { throw std::logic_error("Missing SHARC sequencer state"); }
  auto updated = *flow;
  constexpr std::uint32_t first_short = 0x100000;
  const auto stride = flow->next >= first_short ? 3U : 1U;
  if (address_accepted) { updated.next = (flow->next + stride) & sharc_pc_mask; }
  if (next.address) {
    const auto decision = branch(cpu, *next.address);
    next.address->branch = decision;
    if (decision && decision->taken) {
      if (flow->discard_decode != 0) {
        throw std::runtime_error("SHARC overlapping sequenced branches not qualified");
      }
      if (previous.lookup || previous.fill || next.lookup || next.fill) {
        throw std::runtime_error("SHARC sequenced branch with pending cache fill not qualified");
      }
      if (!decision->target) { throw std::logic_error("Missing SHARC taken branch target"); }
      updated.next = *decision->target;
      // Table4-5 aborts n+1/n+2/n+3; Table4-7 retains the two real slots.
      constexpr unsigned int immediate_discard = 7;
      constexpr unsigned int delayed_discard = 4;
      updated.discard_decode =
          (next.address->opcode & delayed) != 0 ? delayed_discard : immediate_discard;
    }
  }
  const bool suppressed = (updated.discard_decode & 1U) != 0;
  if (suppressed) { next.decode.reset(); }
  updated.discard_decode >>= 1;
  next.flow = updated;
  return suppressed;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC sequencer Address=";
  if (next.address) {
    message << "0x" << std::hex << next.address->address << " opcode=0x" << next.address->opcode;
  } else {
    message << "bubble";
  }
  message << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
