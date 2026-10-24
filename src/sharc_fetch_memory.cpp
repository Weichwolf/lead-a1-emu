// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "sharc_fetch_internal.hpp"

#include <exception>
#include <ios>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {

void SharcFetch::complete_fetch2(const Sharc &cpu) try {
  if (&cpu.memory_ != &memory_ || memory_.active()) {
    throw std::logic_error("SHARC F2 completion requires its idle bound memory");
  }
  if (!state_.pending_fetch2 || state_.fetch2 || state_.fill) {
    throw std::runtime_error("SHARC F2 completion requires a pending uncached RAM response");
  }
  if (cpu.control_ != nullptr) { static_cast<void>(cpu.control_->snapshot()); }
  auto next = state_;
  const auto address = *state_.pending_fetch2;
  const auto opcode =
      memory_.read({.address = address, .width = SharcWidth::wide48}, AccessKind::fetch);
  next.fetch2 = SharcFetchWord{.address = address, .opcode = opcode};
  next.pending_fetch2.reset();
  detail::sharc_fetch_validate(next);
  state_ = next;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC F2 memory completion pc=0x" << std::hex << cpu.state_.pc << " word-address=";
  if (state_.pending_fetch2) {
    message << "0x" << *state_.pending_fetch2;
  } else {
    message << "absent";
  }
  message << " operation=fetch bits=48: " << error.what();
  throw std::runtime_error(message.str());
}

}
