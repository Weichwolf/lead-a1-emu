// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {
namespace {

SharcFetchEvent normal_clock(SharcMemory &memory,
                             SharcCacheState &cache,
                             const SharcFetchState &previous,
                             SharcFetchState &next,
                             SharcKnown32 mode,
                             std::optional<std::uint32_t> address,
                             SharcFetchBus bus) {
  SharcFetchEvent event{.address_accepted = true};
  next.execute = previous.address;
  next.address = previous.decode;
  if (bus == SharcFetchBus::pm_data &&
      (!next.address || !detail::sharc_cache_pm_encoding(next.address->opcode))) {
    throw std::runtime_error("SHARC PM request requires an Address-stage PM instruction");
  }
  if (next.address) { next.address->pm_data = bus == SharcFetchBus::pm_data; }
  next.decode = previous.fetch2;
  next.fetch2.reset();
  next.fill.reset();
  if (previous.fetch1) {
    next.fetch2 = detail::sharc_fetch_read(memory, cache, previous, event.forwarded);
    if (previous.lookup && previous.lookup->entry && !previous.lookup->hit) {
      next.fill = SharcFetchFill{.mode = previous.lookup->mode, .entry = *previous.lookup->entry};
    }
  }
  // F2 cache hits may use the input load bus above; Decode then stores that input.
  detail::sharc_fetch_fill(cache, previous);
  next.fetch1 = address;
  next.lookup.reset();
  if (address && bus == SharcFetchBus::pm_data) {
    next.lookup = detail::sharc_fetch_lookup(cache, mode, *address);
    event.cache_hit = next.lookup->hit;
  }
  return event;
}

void bubble_guard(const SharcState &cpu, const SharcControl *control) {
  for (const auto &pending : cpu.pending) {
    if (pending) {
      throw std::runtime_error("SHARC fetch bubble control-effect clock not qualified");
    }
  }
  if (cpu.cache.pending_flush || (control != nullptr && control->state().delayed)) {
    throw std::runtime_error("SHARC fetch bubble cache/SYSCTL effect clock not qualified");
  }
}

}

SharcFetchEvent
SharcFetch::advance(Sharc &cpu, std::optional<std::uint32_t> next_address, SharcFetchBus bus) {
  return advance_clock(cpu, next_address, bus, false);
}

SharcFetchEvent SharcFetch::advance_sequenced(Sharc &cpu, SharcFetchBus bus) {
  const auto flow = state_.flow;
  if (!flow) { throw std::logic_error("SHARC sequenced clock requires its F1 cursor"); }
  return advance_clock(cpu, flow->next, bus, true);
}

SharcFetchEvent SharcFetch::advance_clock(Sharc &cpu,
                                          std::optional<std::uint32_t> next_address,
                                          SharcFetchBus bus,
                                          bool sequenced) try {
  if (&cpu.memory_ != &memory_ || memory_.active()) {
    throw std::logic_error("SHARC cache fetch requires its idle bound memory");
  }
  if (state_.execute) { throw std::runtime_error("SHARC fetch Execute word has not retired"); }
  if (sequenced != state_.flow.has_value()) {
    throw std::runtime_error("SHARC sequenced fetch requires its sequenced clock");
  }
  if (sequenced) { detail::sharc_fetch_flow_guard(cpu.state_); }
  if (next_address) { detail::sharc_fetch_address(*next_address); }
  if (bus == SharcFetchBus::block_conflict) {
    throw std::runtime_error("SHARC block-conflict arbitration not qualified; not cached");
  }
  if (bus != SharcFetchBus::available && bus != SharcFetchBus::pm_data) {
    throw std::invalid_argument("Invalid SHARC fetch bus event");
  }
  const auto mode = cpu.state_.effective[4];
  detail::sharc_validate_cache(cpu.state_.cache);
  detail::sharc_fetch_mode(state_, mode);
  auto next = state_;
  auto cache = cpu.state_.cache;
  SharcFetchEvent event;
  if (state_.lookup && state_.lookup->wait) {
    if (bus != SharcFetchBus::available) {
      throw std::runtime_error("SHARC pending fetch stall requires an empty Address phase");
    }
    next.execute = state_.address;
    next.address.reset();
    detail::sharc_fetch_reserve(cache, next);
    event.stalled = true;
  } else {
    event = normal_clock(memory_, cache, state_, next, mode, next_address, bus);
    if (sequenced) { detail::sharc_fetch_flow(state_, next); }
  }
  if (!next.execute) { bubble_guard(cpu.state_, cpu.control_); }
  detail::sharc_fetch_validate(next);
  detail::sharc_validate_cache(cache);
  state_ = next;
  cpu.state_.cache = cache;
  return event;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC cache clock pc=0x" << std::hex << cpu.state_.pc << " F1=";
  if (state_.fetch1) {
    message << "0x" << *state_.fetch1;
  } else {
    message << "bubble";
  }
  message << " next=";
  if (next_address) {
    message << "0x" << *next_address;
  } else {
    message << "bubble";
  }
  message << " bus=" << static_cast<unsigned int>(bus) << " MODE2=0x"
          << cpu.state_.effective[4].value << "/0x" << cpu.state_.effective[4].known << ": "
          << error.what();
  throw std::runtime_error(message.str());
}

}
