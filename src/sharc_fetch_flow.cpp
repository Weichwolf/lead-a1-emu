// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
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

std::optional<std::uint32_t> target(const SharcFetchWord &word) {
  constexpr unsigned int class_shift = 40;
  constexpr unsigned int absolute = 6;
  constexpr unsigned int relative = 7;
  const auto type = word.opcode >> class_shift;
  if (type == absolute || type == relative) {
    constexpr auto allowed = class_mask | condition_mask | delayed | call | sharc_pc_mask;
    if ((word.opcode & ~allowed) != 0 || (word.opcode & condition_mask) != condition_mask) {
      throw std::runtime_error("SHARC sequenced conditional/LA/CI/reserved branch not qualified");
    }
    auto address = static_cast<std::uint32_t>(word.opcode) & sharc_pc_mask;
    if (type == relative) { address = (word.address + address) & sharc_pc_mask; }
    sharc_fetch_address(address);
    return address;
  }
  constexpr unsigned int first_other_control = 8;
  constexpr unsigned int last_other_control = 14;
  if ((type >= first_other_control && type <= last_other_control) ||
      (type == 0 && word.opcode != 0)) {
    throw std::runtime_error("SHARC sequenced return/loop/indirect/IDLE control not qualified");
  }
  return {};
}

}

void sharc_fetch_flow_guard(const SharcState &cpu) {
  if (cpu.count_loop || cpu.arithmetic_loop || cpu.loops_empty != true || cpu.interrupt ||
      cpu.idle_pc) {
    throw std::runtime_error("SHARC sequenced loop/ISR/IDLE context not qualified");
  }
  sharc_interrupt_guard(cpu);
}

void sharc_fetch_flow(const SharcFetchState &previous, SharcFetchState &next) try {
  const auto flow = previous.flow;
  if (!flow) { throw std::logic_error("Missing SHARC sequencer state"); }
  auto updated = *flow;
  updated.next = (flow->next + 1) & sharc_pc_mask;
  if (next.address) {
    const auto destination = target(*next.address);
    if (destination) {
      if (flow->discard_decode != 0) {
        throw std::runtime_error("SHARC overlapping sequenced branches not qualified");
      }
      if (previous.lookup || previous.fill || next.lookup || next.fill) {
        throw std::runtime_error("SHARC sequenced branch with pending cache fill not qualified");
      }
      updated.next = *destination;
      // Table4-5 aborts n+1/n+2/n+3; Table4-7 retains the two real slots.
      constexpr unsigned int immediate_discard = 7;
      constexpr unsigned int delayed_discard = 4;
      updated.discard_decode =
          (next.address->opcode & delayed) != 0 ? delayed_discard : immediate_discard;
    }
  }
  if ((updated.discard_decode & 1U) != 0) { next.decode.reset(); }
  updated.discard_decode >>= 1;
  next.flow = updated;
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
