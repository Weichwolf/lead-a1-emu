// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_visa.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_visa_internal.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

bool short_request(std::optional<std::uint32_t> address) {
  return address && sharc_visa_address(*address);
}

void clock_guard(const SharcState &cpu,
                 const SharcFetchState &state,
                 SharcFetchBus bus,
                 SharcFetchRead read) {
  if (!state.flow || (state.flow->loop && state.flow->loop->units != 3) || state.flow->long_loop ||
      state.lookup || state.fill || cpu.arithmetic_loop || state.pending_fetch2 ||
      bus != SharcFetchBus::available || read != SharcFetchRead::immediate) {
    throw std::runtime_error("SHARC VISA cache/PM/deferred/loop fetch timing not qualified");
  }
}

void redirect(SharcFetchState &state, std::uint32_t target) {
  state.visa.reset();
  if (sharc_visa_address(target)) {
    state.visa = SharcVisaState{.decode = target, .fetch = target};
  }
  state.visa_fetch_aborted = short_request(state.fetch1);
}

std::optional<SharcFetchWord> decode(SharcVisaBuffer &buffer, bool suppressed) {
  if (suppressed) { return {}; }
  const auto instruction = buffer.instruction();
  if (!instruction) { return {}; }
  const auto opcode = sharc_visa_decode(*instruction);
  static_cast<void>(buffer.advance(true));
  return SharcFetchWord{.address = instruction->address, .opcode = opcode, .visa = instruction};
}

bool dag_wait(const SharcState &cpu, const SharcFetchState &previous, SharcFetchState &next) {
  if (!next.decode || !next.decode->visa) { return false; }
  auto input = previous;
  input.fetch2 = next.decode;
  if (!sharc_fetch_dag_stall(cpu, input, SharcFetchBus::available)) { return false; }
  // PGR4-123: hold the input and F1/F2, while the two older stages advance.
  next.fetch1 = previous.fetch1;
  next.fetch2 = previous.fetch2;
  next.decode.reset();
  next.flow = previous.flow;
  next.visa = previous.visa;
  return true;
}

void capture(SharcMemory &memory,
             const SharcFetchState &previous,
             SharcFetchState &next,
             bool held,
             bool redirected) {
  if (!previous.fetch1 || previous.visa_fetch_aborted || held) { return; }
  if (!short_request(previous.fetch1)) {
    const auto opcode =
        memory.read({.address = *previous.fetch1, .width = SharcWidth::wide48}, AccessKind::fetch);
    next.fetch2 = SharcFetchWord{.address = *previous.fetch1, .opcode = opcode};
    return;
  }
  if (redirected) { return; }
  sharc_fetch_visa_loop_response(previous, next);
  if (!next.visa || next.visa->fetch != previous.fetch1) {
    throw std::runtime_error("SHARC VISA F2 request does not match its IAB epoch");
  }
  SharcVisaBuffer buffer(memory, *next.visa);
  buffer.capture();
  static_cast<void>(buffer.advance(false));
  next.visa = buffer.state();
}

}

void sharc_fetch_visa_validate(const SharcFetchState &state) {
  if (state.visa) {
    sharc_visa_validate(*state.visa);
    if (!state.flow || !sharc_visa_address(state.flow->next) ||
        (state.visa->delay && state.visa->count < 3) || (state.fetch2 && state.visa->count != 0)) {
      throw std::invalid_argument("Invalid SHARC clocked VISA buffer phase");
    }
  }
  if (state.visa_fetch_aborted && (!state.flow || !short_request(state.fetch1))) {
    throw std::invalid_argument("Invalid SHARC aborted VISA fetch request");
  }
  if (short_request(state.fetch1) && !state.visa_fetch_aborted &&
      (!state.visa || state.visa->fetch != state.fetch1) && !sharc_fetch_visa_loop_request(state)) {
    throw std::invalid_argument("Invalid SHARC inherited VISA F1/IAB cursor");
  }
  if (state.fetch2 && (state.fetch2->visa || sharc_visa_address(state.fetch2->address))) {
    throw std::invalid_argument("Invalid SHARC VISA word outside its IAB");
  }
  for (const auto &word : {state.decode, state.address, state.execute}) {
    if (!word) { continue; }
    if (sharc_visa_address(word->address) != word->visa.has_value()) {
      throw std::invalid_argument("Missing or unexpected SHARC captured VISA encoding");
    }
    if (word->visa && (!state.flow || word->address != word->visa->address ||
                       word->opcode != sharc_visa_decode(*word->visa))) {
      throw std::invalid_argument("Invalid SHARC captured VISA semantic operation");
    }
  }
}

SharcFetchEvent sharc_fetch_visa_clock(SharcMemory &memory,
                                       const SharcState &cpu,
                                       const SharcFetchState &previous,
                                       SharcFetchState &next,
                                       std::optional<std::uint32_t> address,
                                       SharcFetchBus bus,
                                       SharcFetchRead read) {
  clock_guard(cpu, previous, bus, read);
  // The delay register's occupancy at the START of this clock controls F2.
  // Decode may free it this clock; the held F1 request completes next clock.
  const bool held = previous.visa && previous.visa->delay && short_request(previous.fetch1) &&
                    !previous.visa_fetch_aborted;
  SharcFetchEvent event{.address_accepted = !held, .fetch_stall = held};
  next.execute = previous.address;
  next.address = previous.decode;
  if (next.address) { next.address->pm_data = false; }
  next.decode = previous.fetch2;
  next.fetch2.reset();
  next.fetch1 = held ? previous.fetch1 : address;
  next.visa_fetch_aborted = false;
  const bool suppressed = sharc_fetch_flow(cpu, previous, next, !held);
  const auto branch = next.address ? next.address->branch : std::nullopt;
  const bool redirected = branch && branch->taken;
  if (redirected) {
    if (!branch || !branch->target) {
      throw std::logic_error("Missing SHARC VISA redirect target");
    }
    // A taken branch cancels a speculative SW request, even with a full IAB.
    next.fetch1 = address;
    event.address_accepted = true;
    event.fetch_stall = false;
    redirect(next, *branch->target);
  } else {
    if (!next.visa && address && sharc_visa_address(*address)) {
      next.visa = SharcVisaState{.decode = *address, .fetch = *address};
    }
    if (previous.visa && !next.decode) {
      if (sharc_fetch_visa_loop_skip(previous, next.address)) {
        next.visa = SharcVisaState{.decode = previous.visa->fetch, .fetch = previous.visa->fetch};
      } else {
        SharcVisaBuffer buffer(memory, *previous.visa);
        next.decode = decode(buffer, suppressed);
        next.visa = buffer.state();
      }
      if (dag_wait(cpu, previous, next)) { return {.stalled = true, .dag_stall = true}; }
    }
  }
  sharc_fetch_visa_loop_entry(previous, next);
  sharc_fetch_loop_flow(cpu, previous, next);
  capture(memory, previous, next, held, redirected);
  return event;
}

}
