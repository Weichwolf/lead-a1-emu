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
                    bool sequenced,
                    SharcFetchRead read) {
  if (state.execute) { throw std::runtime_error("SHARC fetch Execute word has not retired"); }
  if (sequenced != state.flow.has_value()) {
    throw std::runtime_error("SHARC sequenced fetch requires its sequenced clock");
  }
  if (next_address) { detail::sharc_fetch_program_address(*next_address); }
  if (bus == SharcFetchBus::block_conflict) {
    throw std::runtime_error("SHARC block-conflict arbitration not qualified; not cached");
  }
  if (bus != SharcFetchBus::available && bus != SharcFetchBus::pm_data) {
    throw std::invalid_argument("Invalid SHARC fetch bus event");
  }
  if (read != SharcFetchRead::immediate && read != SharcFetchRead::deferred) {
    throw std::invalid_argument("Invalid SHARC F2 memory response input");
  }
  if (read == SharcFetchRead::deferred && bus != SharcFetchBus::pm_data) {
    throw std::runtime_error("SHARC deferred F2 reference requires an Address PM request");
  }
}

void pending_context(const SharcState &cpu, const SharcFetchState &next) {
  if (next.pending_fetch2 &&
      (cpu.branch || (next.flow && (next.flow->loop || next.flow->long_loop ||
                                    next.flow->discard_decode != 0)))) {
    throw std::runtime_error("SHARC deferred F2 with branch/counter/flush transport not qualified");
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
                             SharcFetchBus bus,
                             SharcFetchRead read) {
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
  if (read == SharcFetchRead::deferred) {
    if (!previous.fetch1 || previous.lookup) {
      throw std::runtime_error("SHARC deferred F2 requires an uncached RAM request");
    }
    next.pending_fetch2 = previous.fetch1;
  } else if (previous.fetch1) {
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
                                bool sequenced,
                                SharcFetchRead read) {
  if (previous.pending_fetch2 && (!previous.lookup || !previous.lookup->wait)) {
    throw std::runtime_error("SHARC F2 memory response must arrive before advancing Decode");
  }
  if (sequenced && detail::sharc_fetch_dag_stall(cpu, previous, bus)) {
    const auto event = dag_stalled_clock(previous, next);
    // The unrelated DO still reaches Address while its body waits before Decode.
    detail::sharc_fetch_loop_flow(cpu, previous, next);
    return event;
  }
  const auto interlock =
      sequenced ? detail::sharc_fetch_interlock(cpu, previous) : detail::SharcFetchInterlock::none;
  const bool interlocked = interlock != detail::SharcFetchInterlock::none;
  const bool cache_stall = previous.lookup && previous.lookup->wait;
  if (sequenced && detail::sharc_fetch_loop_stall(previous)) {
    return loop_startup_clock(previous, next, bus, interlocked, cache_stall);
  }
  if (cache_stall || interlocked) {
    auto event = stalled_clock(previous, next, cache, bus, interlocked);
    if (interlock == detail::SharcFetchInterlock::fixed_compute) {
      event.control_stall = false;
      event.compute_stall = true;
    }
    if (interlocked && next.flow) {
      next.flow->multiplier_branch_wait = interlock == detail::SharcFetchInterlock::multiplier;
    }
    return event;
  }
  constexpr std::uint32_t first_short = 0x100000;
  if (previous.visa || previous.visa_fetch_aborted ||
      (previous.fetch1 && *previous.fetch1 >= first_short) ||
      (next_address && *next_address >= first_short)) {
    if (!sequenced) { throw std::runtime_error("SHARC VISA fetch requires its sequencer"); }
    return detail::sharc_fetch_visa_clock(memory, cpu, previous, next, next_address, bus, read);
  }
  const auto event =
      normal_clock(memory, cache, previous, next, cpu.effective[4], next_address, bus, read);
  if (sequenced) { static_cast<void>(detail::sharc_fetch_flow(cpu, previous, next)); }
  return event;
}

}

bool SharcFetch::address_pm_request(const Sharc &cpu) const try {
  if (&cpu.memory_ != &memory_ || memory_.active() || state_.execute) {
    throw std::logic_error("SHARC PM request requires its idle bound core and retired Execute");
  }
  if (state_.flow) {
    detail::sharc_fetch_flow_guard(cpu.state_, state_);
    if (detail::sharc_fetch_dag_stall(cpu.state_, state_, SharcFetchBus::available) ||
        detail::sharc_fetch_interlock(cpu.state_, state_) != detail::SharcFetchInterlock::none ||
        detail::sharc_fetch_loop_stall(state_)) {
      return false;
    }
  }
  if (state_.lookup && state_.lookup->wait) { return false; }
  const auto word = state_.decode;
  if (!word || !detail::sharc_cache_pm_encoding(word->opcode)) { return false; }
  constexpr std::uint64_t type3_mask = 0xe00000000000;
  constexpr std::uint64_t type3 = 0x400000000000;
  constexpr std::uint64_t type4_mask = 0xf00000000000;
  constexpr std::uint64_t type4 = 0x600000000000;
  const bool conditional_form =
      (word->opcode & type3_mask) == type3 || (word->opcode & type4_mask) == type4;
  constexpr unsigned int condition_shift = 33;
  constexpr unsigned int unconditional = 31;
  if (conditional_form && ((word->opcode >> condition_shift) & unconditional) != unconditional) {
    throw std::runtime_error("SHARC conditional PM request timing not qualified");
  }
  return detail::sharc_cache_pm_request(cpu.state_, word->opcode);
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC Address PM request pc=0x" << std::hex << cpu.state_.pc << " Decode=";
  if (state_.decode) {
    message << "0x" << state_.decode->address << " opcode=0x" << state_.decode->opcode;
  } else {
    message << "bubble";
  }
  message << ": " << error.what();
  throw std::runtime_error(message.str());
}

SharcFetchEvent SharcFetch::advance(Sharc &cpu,
                                    std::optional<std::uint32_t> next_address,
                                    SharcFetchBus bus,
                                    SharcFetchRead read) {
  return advance_clock(cpu, next_address, bus, false, read);
}

SharcFetchEvent SharcFetch::advance_sequenced(Sharc &cpu, SharcFetchBus bus, SharcFetchRead read) {
  const auto flow = state_.flow;
  if (!flow) { throw std::logic_error("SHARC sequenced clock requires its F1 cursor"); }
  return advance_clock(cpu, flow->next, bus, true, read);
}

SharcFetchEvent SharcFetch::advance_clock(Sharc &cpu,
                                          std::optional<std::uint32_t> next_address,
                                          SharcFetchBus bus,
                                          bool sequenced,
                                          SharcFetchRead read) try {
  if (&cpu.memory_ != &memory_ || memory_.active()) {
    throw std::logic_error("SHARC cache fetch requires its idle bound memory");
  }
  if (detail::sharc_fetched_arithmetic(cpu.state_) && !sequenced) {
    throw std::runtime_error("SHARC fetched arithmetic state requires its sequenced transport");
  }
  if (cpu.state_.idle_pc) {
    throw std::runtime_error("SHARC IDLE fetch requires an owner wake event");
  }
  validate_clock(state_, next_address, bus, sequenced, read);
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
      clock_transport(memory_, cpu.state_, cache, state_, next, next_address, bus, sequenced, read);
  pending_context(cpu.state_, next);
  auto core = cpu.state_;
  core.cache = cache;
  if (sequenced) { detail::sharc_fetch_arithmetic_clock(core, next, event); }
  if (sequenced) { detail::sharc_fetch_loop_clock(core, state_, next, event.loop_stall); }
  if (sequenced &&
      (!event.stalled || (event.loop_stall && state_.flow && state_.flow->long_loop &&
                          state_.flow->long_loop->four == SharcFetchFourPhase::hold))) {
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
