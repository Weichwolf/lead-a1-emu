// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/pipeline/sharc_fetch_internal.hpp"

#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace lead_a1 {

void SharcFetch::complete_fetch2(const Sharc &cpu) {
  complete_fetch2(cpu, nullptr);
}

void SharcFetch::complete_fetch2(const Sharc &cpu, SharcMemoryBus &bus) {
  complete_fetch2(cpu, &bus);
}

void SharcFetch::complete_fetch2(const Sharc &cpu, SharcMemoryBus *bus) try {
  if (&cpu.memory_ != &memory_ || memory_.active()) {
    throw std::logic_error("SHARC F2 completion requires its idle bound memory");
  }
  if (!state_.pending_fetch2 || state_.fetch2 || state_.fill) {
    throw std::runtime_error("SHARC F2 completion requires a pending uncached RAM response");
  }
  if (cpu.control_ != nullptr) { static_cast<void>(cpu.control_->snapshot()); }
  auto next = state_;
  const auto address = *state_.pending_fetch2;
  const SharcMemoryRequest request{.access = {.address = address, .width = SharcWidth::wide48},
                                   .kind = AccessKind::fetch};
  std::optional<SharcMemoryBus> staged;
  if (bus != nullptr) {
    if (&bus->memory() != &memory_) { throw std::logic_error("SHARC F2 grant has foreign RAM"); }
    if (bus->grant(SharcMemoryMaster::pm) != request) {
      throw std::runtime_error("SHARC F2 response requires its exact PM/native48 fetch grant");
    }
    // This request can read only native internal RAM, never side-effecting IOP.
    // Stage grant consumption until every actual fetched-word invariant passes.
    staged.emplace(memory_, bus->state());
  }
  const auto opcode =
      staged ? staged->execute(SharcMemoryMaster::pm) : memory_.read(request.access, request.kind);
  next.fetch2 = SharcFetchWord{.address = address, .opcode = opcode};
  next.pending_fetch2.reset();
  detail::sharc_fetch_validate(next);
  static_assert(std::is_nothrow_copy_assignable_v<SharcMemoryBusState>);
  static_assert(std::is_nothrow_copy_assignable_v<SharcFetchState>);
  if (bus != nullptr && staged) { bus->state_ = staged->state(); }
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
