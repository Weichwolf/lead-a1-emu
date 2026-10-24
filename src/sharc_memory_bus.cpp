// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_memory_bus.hpp"

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_memory_view.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <ios>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace lead_a1 {
namespace {

unsigned int index(SharcMemoryMaster master) {
  const auto value = std::to_underlying(master);
  constexpr unsigned int buses = 4;
  if (value >= buses) { throw std::invalid_argument("Invalid SHARC memory bus master"); }
  return value;
}

unsigned int block(SharcMemoryRequest request) {
  const auto view = sharc_memory_view(request.access);
  constexpr std::uint32_t cells = 0x40000;
  if (view.bank >= 4 || view.cell >= cells || view.cells > cells - view.cell) {
    throw std::invalid_argument("Invalid SHARC memory request physical block span");
  }
  return view.bank;
}

void validate_request(unsigned int bus, SharcMemoryRequest request) {
  static_cast<void>(block(request));
  if (request.kind != AccessKind::read && request.kind != AccessKind::write &&
      request.kind != AccessKind::fetch) {
    throw std::invalid_argument("Invalid SHARC memory request operation");
  }
  constexpr unsigned int pm = 3;
  if (request.kind == AccessKind::fetch &&
      (bus != pm || request.access.width != SharcWidth::wide48)) {
    throw std::invalid_argument("SHARC instruction request requires PM/native48");
  }
  if (bus < 2 && request.access.width != SharcWidth::normal32) {
    throw std::invalid_argument("SHARC IOP memory bus reference requires normal32");
  }
  const auto bits = static_cast<unsigned int>(request.access.width);
  const auto mask = request.access.width == SharcWidth::long64
                        ? std::numeric_limits<std::uint64_t>::max()
                        : (std::uint64_t{1} << bits) - 1;
  if ((request.kind != AccessKind::write && request.value != 0) || (request.value & ~mask) != 0) {
    throw std::invalid_argument("Invalid SHARC memory request value");
  }
}

bool occupied(const SharcMemoryBusState &state) {
  return std::ranges::any_of(state.granted, [](const auto &grant) { return grant.has_value(); });
}

bool eligible(unsigned int bus, SharcPeripheralPhase phase, const SharcMemoryBusState &state) {
  return bus >= 2 || phase == SharcPeripheralPhase::active || state.carried[bus];
}

void validate(const SharcMemoryBusState &state) {
  if (state.next_phase != SharcPeripheralPhase::active &&
      state.next_phase != SharcPeripheralPhase::gap) {
    throw std::invalid_argument("Invalid SHARC peripheral clock phase");
  }
  if (state.carried[0] || (state.carried[1] && (!state.pending[1] || state.clocks == 0 ||
                                                state.next_phase != SharcPeripheralPhase::gap))) {
    throw std::invalid_argument("Invalid SHARC carried IOD1 request");
  }
  unsigned int used{};
  const auto last = state.next_phase == SharcPeripheralPhase::active ? SharcPeripheralPhase::gap
                                                                     : SharcPeripheralPhase::active;
  for (unsigned int bus = 0; bus < state.pending.size(); ++bus) {
    const auto pending_request = state.pending[bus];
    if (pending_request) { validate_request(bus, *pending_request); }
    const auto grant = state.granted[bus];
    if (!grant) { continue; }
    validate_request(bus, *grant);
    const auto mask = 1U << block(*grant);
    // An IO1 grant on a gap clock is the previously carried request, consumed
    // by this clock. Its eligibility bit is no longer present in the result.
    const bool carried_grant = bus == 1 && last == SharcPeripheralPhase::gap && state.clocks >= 2;
    const bool allowed = carried_grant || eligible(bus, last, state);
    if (state.pending[bus] || state.clocks == 0 || !allowed || (used & mask) != 0) {
      throw std::invalid_argument("Invalid SHARC inherited memory grant");
    }
    used |= mask;
    for (unsigned int earlier = 0; earlier < bus; ++earlier) {
      const auto pending = state.pending[earlier];
      if (pending && eligible(earlier, last, state) && block(*pending) == block(*grant)) {
        throw std::invalid_argument("Invalid SHARC memory grant priority");
      }
    }
  }
}

void external(const SharcMemory &memory, const SharcMemoryBusState &state) {
  if (memory.active() || occupied(state)) {
    throw std::logic_error("SHARC memory bus requires idle memory and completed grants");
  }
}

}

SharcMemoryBus::SharcMemoryBus(SharcMemory &memory, SharcPeripheralPhase initial_phase)
    : SharcMemoryBus(memory, SharcMemoryBusState{.next_phase = initial_phase}) {}

SharcMemoryBus::SharcMemoryBus(SharcMemory &memory, SharcMemoryBusState initial)
    : memory_(memory), state_(initial) {
  validate(state_);
}

void SharcMemoryBus::request(SharcMemoryMaster master, SharcMemoryRequest request) try {
  external(memory_, state_);
  const auto bus = index(master);
  validate_request(bus, request);
  if (state_.pending[bus]) { throw std::runtime_error("SHARC memory requester already pending"); }
  state_.pending[bus] = request;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC RAM request pc=0x" << std::hex << memory_.pc() << " word-address=0x"
          << request.access.address << std::dec
          << " bits=" << static_cast<unsigned int>(request.access.width)
          << " bus=" << static_cast<unsigned int>(master)
          << " operation=" << static_cast<unsigned int>(request.kind) << ": " << error.what();
  throw std::runtime_error(message.str());
}

void SharcMemoryBus::advance() {
  external(memory_, state_);
  if (state_.clocks == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("SHARC memory bus clock counter overflow");
  }
  auto next = state_;
  unsigned int used{};
  for (unsigned int bus = 0; bus < next.pending.size(); ++bus) {
    const auto request = next.pending[bus];
    if (!request || !eligible(bus, state_.next_phase, state_)) { continue; }
    const auto mask = 1U << block(*request);
    if ((used & mask) != 0) {
      if (bus < 2) { next.carried[bus] = true; }
      continue;
    }
    used |= mask;
    next.granted[bus] = request;
    next.pending[bus].reset();
    if (bus < 2) { next.carried[bus] = false; }
  }
  ++next.clocks;
  next.next_phase = state_.next_phase == SharcPeripheralPhase::active
                        ? SharcPeripheralPhase::gap
                        : SharcPeripheralPhase::active;
  validate(next);
  state_ = next;
}

std::optional<SharcMemoryRequest> SharcMemoryBus::grant(SharcMemoryMaster master) const {
  return state_.granted[index(master)];
}

std::uint64_t SharcMemoryBus::execute(SharcMemoryMaster master) try {
  const auto bus = index(master);
  if (memory_.active()) { throw std::logic_error("SHARC memory grant during another transaction"); }
  const auto request = state_.granted[bus];
  if (!request) { throw std::runtime_error("SHARC memory requester has no grant"); }
  auto value = request->value;
  if (request->kind == AccessKind::write) {
    memory_.write(request->access, value);
  } else {
    value = memory_.read(request->access, request->kind);
  }
  state_.granted[bus].reset();
  return value;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC RAM grant pc=0x" << std::hex << memory_.pc() << std::dec
          << " bus=" << static_cast<unsigned int>(master) << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
