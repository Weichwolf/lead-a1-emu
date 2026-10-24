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
#include <type_traits>

namespace lead_a1 {
namespace {

void validate_clock(const SharcFetchState &state,
                    std::optional<std::uint32_t> next_address,
                    SharcFetchBus bus,
                    bool sequenced) {
  if (state.execute) { throw std::runtime_error("SHARC fetch Execute word has not retired"); }
  if (sequenced != state.flow.has_value()) {
    throw std::runtime_error("SHARC sequenced fetch requires its sequenced clock");
  }
  if (next_address) { detail::sharc_fetch_address(*next_address); }
  if (bus == SharcFetchBus::block_conflict) {
    throw std::runtime_error("SHARC block-conflict arbitration not qualified; not cached");
  }
  if (bus != SharcFetchBus::available && bus != SharcFetchBus::pm_data) {
    throw std::invalid_argument("Invalid SHARC fetch bus event");
  }
}

SharcFetchEvent stalled_clock(const SharcFetchState &previous,
                              SharcFetchState &next,
                              SharcCacheState &cache,
                              SharcFetchBus bus,
                              bool control) {
  if (control && previous.lookup && previous.lookup->wait) {
    throw std::runtime_error("SHARC simultaneous cache/control stalls not qualified");
  }
  if (control && (bus != SharcFetchBus::available || previous.lookup || previous.fill)) {
    throw std::runtime_error("SHARC control stall with PM/cache phases not qualified");
  }
  if (bus != SharcFetchBus::available) {
    throw std::runtime_error("SHARC pending fetch stall requires an empty Address phase");
  }
  next.execute = previous.address;
  next.address.reset();
  if (!control) { detail::sharc_fetch_reserve(cache, next); }
  return {.stalled = true, .control_stall = control};
}

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

SharcFetchEvent loop_startup_clock(const SharcFetchState &previous,
                                   SharcFetchState &next,
                                   SharcFetchBus bus,
                                   bool interlock,
                                   bool cache_stall) {
  if (interlock || cache_stall || bus != SharcFetchBus::available) {
    throw std::runtime_error("SHARC simultaneous loop/bus/control stall not qualified");
  }
  next.execute = previous.address;
  next.address.reset();
  return {.stalled = true, .loop_stall = true};
}

SharcFetchEvent dag_stalled_clock(const SharcFetchState &previous, SharcFetchState &next) {
  // Tables4-51/52 hold F1/F2; older Decode/Address instructions still advance.
  next.execute = previous.address;
  next.address = previous.decode;
  if (next.address) { next.address->pm_data = false; }
  next.decode.reset();
  return {.stalled = true, .dag_stall = true};
}

SharcFetchEvent clock_transport(SharcMemory &memory,
                                const SharcState &cpu,
                                SharcCacheState &cache,
                                const SharcFetchState &previous,
                                SharcFetchState &next,
                                std::optional<std::uint32_t> next_address,
                                SharcFetchBus bus,
                                bool sequenced) {
  if (sequenced && detail::sharc_fetch_dag_stall(cpu, previous, bus)) {
    return dag_stalled_clock(previous, next);
  }
  const bool interlock = sequenced && detail::sharc_fetch_interlock(cpu, previous);
  const bool cache_stall = previous.lookup && previous.lookup->wait;
  if (sequenced && detail::sharc_fetch_loop_stall(previous)) {
    return loop_startup_clock(previous, next, bus, interlock, cache_stall);
  }
  if (cache_stall || interlock) { return stalled_clock(previous, next, cache, bus, interlock); }
  const auto event =
      normal_clock(memory, cache, previous, next, cpu.effective[4], next_address, bus);
  if (sequenced) { detail::sharc_fetch_flow(cpu, previous, next); }
  return event;
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
  validate_clock(state_, next_address, bus, sequenced);
  if (sequenced) {
    detail::sharc_fetch_flow_guard(cpu.state_, state_);
    detail::sharc_fetch_multiplier_guard(state_);
  }
  const auto mode = cpu.state_.effective[4];
  detail::sharc_validate_cache(cpu.state_.cache);
  detail::sharc_fetch_mode(state_, mode);
  auto next = state_;
  auto cache = cpu.state_.cache;
  const auto event =
      clock_transport(memory_, cpu.state_, cache, state_, next, next_address, bus, sequenced);
  auto core = cpu.state_;
  core.cache = cache;
  if (sequenced) { detail::sharc_fetch_loop_clock(core, state_, next, event.loop_stall); }
  if (sequenced && event.address_accepted) {
    detail::sharc_fetch_dag_prepare(core, next, cpu.control_);
  }
  if (!next.execute) { detail::sharc_fetch_quiet_effects(core, next, cpu.control_); }
  detail::sharc_fetch_validate(next);
  detail::sharc_validate_cache(core.cache);
  if (sequenced) { detail::sharc_fetch_loop_guard(core, next); }
  // The only external commit can reject ownership before any state change.
  // Fixed-value assignments below cannot throw after a successful device clock.
  static_assert(std::is_nothrow_copy_assignable_v<SharcState>);
  static_assert(std::is_nothrow_copy_assignable_v<SharcFetchState>);
  if (!next.execute && cpu.control_ != nullptr) { cpu.control_->advance_internal_clocks(1); }
  state_ = next;
  cpu.state_ = core;
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
