// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/visa/sharc_visa.hpp"
#include "sharc/visa/sharc_visa_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace lead_a1 {
namespace {

bool redirects(const SharcState &state, std::uint64_t opcode) {
  if (state.branch && state.branch->slots == 1) { return true; }
  constexpr unsigned int class_shift = 40;
  const auto type = opcode >> class_shift;
  if (type != 6 && type != 7 && type != 10 && type != 11) { return false; }
  constexpr std::uint64_t delayed = 0x4000000;
  if ((opcode & delayed) != 0) { return false; }
  constexpr unsigned int condition_shift = 33;
  constexpr unsigned int condition_mask = 31;
  return detail::sharc_condition(
      state, static_cast<unsigned int>((opcode >> condition_shift) & condition_mask));
}

}

SharcStep Sharc::step(SharcVisaBuffer &buffer) try {
  if (&buffer.memory_ != &memory_ || memory_.active()) {
    throw std::logic_error("SHARC VISA retirement requires its idle bound memory");
  }
  const auto instruction = buffer.instruction();
  if (!instruction || instruction->address != state_.pc) {
    throw std::runtime_error("SHARC VISA retirement requires a matching complete IAB instruction");
  }
  const auto opcode = detail::sharc_visa_decode(*instruction);
  detail::sharc_visa_execution_guard(state_, *instruction);
  SharcVisaBuffer staged(memory_, buffer.state_);
  static_cast<void>(staged.advance(true));
  const bool redirect = redirects(state_, opcode);
  const auto result =
      step_opcode(opcode, false, std::nullopt, SharcLoopClock::functional, nullptr, &*instruction);
  auto next = staged.state_;
  if (redirect || result.next_pc != next.decode) {
    const auto address = detail::sharc_visa_address(result.next_pc) ? result.next_pc : next.decode;
    next = SharcVisaState{.decode = address, .fetch = address};
  }
  // All rejecting work precedes actual CPU/memory/device commit. The remaining
  // buffer publication cannot throw; a failed instruction retains its inputs.
  static_assert(std::is_nothrow_copy_assignable_v<SharcVisaState>);
  buffer.state_ = next;
  return result;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC VISA captured execution pc=0x" << std::hex << state_.pc << " short-address=0x"
          << buffer.state().decode << " operation=execute: " << error.what();
  throw std::runtime_error(message.str());
}

}
