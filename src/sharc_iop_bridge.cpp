// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_iop_bridge.hpp"

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <ios>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace lead_a1 {
namespace {

constexpr unsigned int write_spacing = 2;
constexpr unsigned int minimum_write_cycles = 4;
constexpr unsigned int aligned_read_cycles = 8;
constexpr unsigned int maximum_cycles = 8;
constexpr std::uint32_t peripheral_end = 0x30000;
constexpr std::uint32_t pc_end = 0x1000000;

std::uint64_t deadline(SharcIopIntent intent, std::uint64_t start) {
  if (start > std::numeric_limits<std::uint64_t>::max() - intent.delivery_cycles) {
    throw std::overflow_error("SHARC IOP delivery deadline overflow");
  }
  return start + intent.delivery_cycles;
}

bool due(const SharcIopState &state) {
  const auto first = state.writes[0];
  if (state.write_count != 0 && first) {
    return deadline(first->intent, first->posted_at) <= state.clocks;
  }
  return state.request && state.read && state.read->started_at && !state.read->response &&
         deadline(*state.request, *state.read->started_at) <= state.clocks;
}

bool ready(const SharcIopState &state) {
  if (!state.request || due(state)) { return false; }
  if (state.request->operation == AccessKind::read) {
    return state.read && state.read->response.has_value();
  }
  return state.write_count < state.writes.size() &&
         (!state.last_write || state.clocks - *state.last_write >= write_spacing);
}

void start_read(SharcIopState &state) {
  if (state.read && !state.read->started_at && state.write_count == 0) {
    state.read->started_at = state.clocks;
  }
}

const char *operation_name(AccessKind operation) {
  switch (operation) {
    case AccessKind::read: return "read";
    case AccessKind::write: return "write";
    case AccessKind::fetch: return "fetch";
    default: return "unknown";
  }
}

std::string context(SharcIopIntent intent, std::uint64_t clock, const std::exception &error) {
  std::ostringstream message;
  message << "SHARC IOP bridge pc=0x" << std::hex << intent.pc << " word-address=0x"
          << intent.address << " operation=" << operation_name(intent.operation) << std::dec
          << " cclk=" << clock << ": " << error.what();
  return message.str();
}

void validate_read(const SharcIopState &state) {
  if (!state.read) { return; }
  if (!state.request || state.request->operation != AccessKind::read) {
    throw std::invalid_argument("Orphan SHARC IOP read state");
  }
  if (!state.read->started_at) {
    if (state.write_count == 0 || state.read->response) {
      throw std::invalid_argument("Invalid SHARC IOP waiting read");
    }
    return;
  }
  const auto start = *state.read->started_at;
  const auto end = deadline(*state.request, start);
  if (start > state.clocks || state.write_count != 0 ||
      (!state.read->response && state.last_read && *state.last_read > start) ||
      (state.read->response && state.clocks < end) ||
      (!state.read->response && state.clocks > end)) {
    throw std::invalid_argument("Invalid SHARC IOP read response timing");
  }
  if (state.read->response && state.last_read != end) {
    throw std::invalid_argument("Invalid SHARC IOP read completion history");
  }
  if (state.last_read == start && state.request->delivery_cycles != aligned_read_cycles) {
    throw std::invalid_argument("Invalid SHARC IOP back-to-back read timing");
  }
}

}

SharcIopBridge::SharcIopBridge(SharcMemory &core,
                               SharcMemory &peripheral,
                               SharcIoMapping window,
                               SharcIopState initial)
    : core_(core), peripheral_(peripheral), window_(window), current_(initial) {
  if (&core == &peripheral || window.words == 0 || window.first_word >= peripheral_end ||
      window.words > peripheral_end - window.first_word) {
    throw std::invalid_argument("Invalid SHARC peripheral bridge binding/window");
  }
  idle();
  validate_state(current_);
}

void SharcIopBridge::validate_intent(SharcIopIntent intent) const {
  if (intent.pc >= pc_end || intent.address < window_.first_word ||
      intent.address - window_.first_word >= window_.words) {
    throw std::invalid_argument("Invalid SHARC IOP request PC/address");
  }
  const bool write = intent.operation == AccessKind::write &&
                     (intent.delivery_cycles == 4 || intent.delivery_cycles == 5);
  const bool read = intent.operation == AccessKind::read &&
                    (intent.delivery_cycles == 7 || intent.delivery_cycles == aligned_read_cycles);
  if (!write && !read) { throw std::invalid_argument("Invalid SHARC IOP operation/latency"); }
}

void SharcIopBridge::validate_state(const SharcIopState &state) const {
  if (state.write_count > state.writes.size() ||
      (state.last_write && *state.last_write > state.clocks) ||
      (state.last_read && *state.last_read > state.clocks)) {
    throw std::invalid_argument("Invalid SHARC IOP queue/history");
  }
  if (state.write_count == 0 && state.last_write &&
      state.clocks - *state.last_write < minimum_write_cycles) {
    throw std::invalid_argument("Invalid SHARC IOP completed write history");
  }
  std::optional<std::uint64_t> previous;
  for (unsigned int index = 0; index < state.writes.size(); ++index) {
    const auto write = state.writes[index];
    if ((index < state.write_count) != write.has_value()) {
      throw std::invalid_argument("Invalid SHARC IOP queue continuity");
    }
    if (!write) { continue; }
    validate_intent(write->intent);
    if (write->intent.operation != AccessKind::write || write->posted_at > state.clocks ||
        deadline(write->intent, write->posted_at) < state.clocks ||
        (previous &&
         (write->posted_at < *previous || write->posted_at - *previous < write_spacing))) {
      throw std::invalid_argument("Invalid SHARC IOP posted write timing");
    }
    previous = write->posted_at;
  }
  if (previous && state.last_write != previous) {
    throw std::invalid_argument("Invalid SHARC IOP last posted write");
  }
  if (state.request) {
    validate_intent(*state.request);
    if ((state.request->operation == AccessKind::read) != state.read.has_value()) {
      throw std::invalid_argument("Invalid SHARC IOP request/read pairing");
    }
  }
  validate_read(state);
}

SharcIopState SharcIopBridge::state() const {
  return working();
}

SharcIopState SharcIopBridge::snapshot() const {
  idle();
  return current_;
}

void SharcIopBridge::idle() const {
  if (staged_ || core_.active() || peripheral_.active()) {
    throw std::logic_error("SHARC IOP bridge event during active transaction");
  }
}

void SharcIopBridge::issue(SharcIopIntent intent) try {
  idle();
  validate_intent(intent);
  if (current_.request || due(current_)) {
    throw std::runtime_error("SHARC IOP request or delivery still pending");
  }
  if (intent.operation == AccessKind::read && current_.write_count == 0 &&
      current_.last_read == current_.clocks && intent.delivery_cycles != aligned_read_cycles) {
    throw std::runtime_error("SHARC back-to-back IOP read requires eight core clocks");
  }
  auto latest = current_.clocks;
  for (const auto &write : current_.writes) {
    if (write) { latest = std::max(latest, deadline(write->intent, write->posted_at)); }
  }
  if (latest > std::numeric_limits<std::uint64_t>::max() - maximum_cycles) {
    throw std::overflow_error("SHARC IOP request timeline overflow");
  }
  current_.request = intent;
  if (intent.operation == AccessKind::read) {
    current_.read = SharcIopRead{};
    start_read(current_);
  }
} catch (const std::exception &error) {

  throw std::runtime_error(context(intent, current_.clocks, error));
}

bool SharcIopBridge::ready() const {
  return lead_a1::ready(current_);
}

bool SharcIopBridge::waiting() const {
  return current_.request && !due(current_) && !lead_a1::ready(current_);
}

bool SharcIopBridge::bound_to(const SharcMemory &core, std::uint32_t address) const {
  if (&core != &core_) { return false; }
  const auto found = std::ranges::find_if(core_.io_, [address](const auto &region) {
    return address >= region.first_word && address - region.first_word < region.words;
  });
  if (found == core_.io_.end()) { return false; }
  const auto *region = &*found;
  if (region->device == this) {
    return region->first_word == window_.first_word && region->words == window_.words;
  }
  const auto *port = dynamic_cast<const SharcIopPort *>(region->device);
  return port != nullptr && &port->bridge_ == this &&
         region->first_word == port->window_.first_word && region->words == port->window_.words;
}

void SharcIopBridge::tick() {
  idle();
  if (due(current_)) { throw std::runtime_error("SHARC IOP delivery retry required before clock"); }
  if (current_.clocks == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("SHARC IOP core-clock overflow");
  }
  ++current_.clocks;
  deliver_due();
}

void SharcIopBridge::retry_due() {
  idle();
  if (!due(current_)) { throw std::runtime_error("Missing due SHARC IOP delivery"); }
  deliver_due();
}

std::uint32_t SharcIopBridge::transfer(SharcIopIntent intent, std::uint32_t value) {
  bool begun{};
  try {
    peripheral_.set_pc(intent.pc);
    peripheral_.begin_instruction();
    begun = true;
    auto result = value;
    if (intent.operation == AccessKind::read) {
      result = static_cast<std::uint32_t>(peripheral_.read({.address = intent.address}));
    } else {
      peripheral_.write({.address = intent.address}, value);
    }
    peripheral_.commit_instruction();
    begun = false;
    return result;
  } catch (...) {
    if (begun) { peripheral_.abort_instruction(); }
    throw;
  }
}

void SharcIopBridge::deliver_due() {
  if (!due(current_)) { return; }
  auto next = current_;
  const bool write = next.write_count != 0;
  if ((write && !next.writes[0]) || (!write && !next.request)) {
    throw std::logic_error("Missing due SHARC IOP request");
  }
  SharcIopIntent intent;
  std::uint32_t value{};
  std::optional<std::uint32_t> *response{};
  if (write) {
    const auto first = next.writes[0];
    if (!first) { throw std::logic_error("Missing due SHARC IOP write"); }
    intent = first->intent;
    value = first->value;
    for (unsigned int index = 1; index < next.write_count; ++index) {
      next.writes[index - 1] = next.writes[index];
    }
    next.writes[--next.write_count].reset();
    start_read(next);
  } else {
    if (!next.read || !next.request) { throw std::logic_error("Missing SHARC IOP read capture"); }
    intent = *next.request;
    response = &next.read->response;
    next.last_read = next.clocks;
  }
  try {
    value = transfer(intent, value);
  } catch (const std::exception &error) {
    throw std::runtime_error(context(intent, current_.clocks, error));
  }
  if (response != nullptr) { *response = value; }
  static_assert(std::is_nothrow_copy_assignable_v<SharcIopState>);
  current_ = next;
}

SharcIopState &SharcIopBridge::retirement(MemoryAccess access, AccessKind operation) {
  if (!staged_ || !core_.active() || access.size != 4 || access.address % 4 != 0 ||
      access.address / 4 >= window_.words) {
    throw std::runtime_error("SHARC IOP retirement requires its bound32-bit transaction");
  }
  auto &state = *staged_;
  if (!state.request || state.request->pc != core_.pc() ||
      state.request->address != window_.first_word + (access.address / 4) ||
      state.request->operation != operation || !lead_a1::ready(state)) {
    throw std::runtime_error("SHARC IOP retirement lacks its exact ready request");
  }
  return state;
}

std::uint64_t SharcIopBridge::read(MemoryAccess access) {
  auto &state = retirement(access, AccessKind::read);
  if (!state.read || !state.read->response) {
    throw std::logic_error("Missing actual IOP response");
  }
  const auto value = *state.read->response;
  state.request.reset();
  state.read.reset();
  return value;
}

void SharcIopBridge::write(MemoryAccess access, std::uint64_t value) {
  auto &state = retirement(access, AccessKind::write);
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized posted SHARC IOP write");
  }
  if (!state.request) { throw std::logic_error("Missing SHARC IOP write request"); }
  static_cast<void>(deadline(*state.request, state.clocks));
  state.writes[state.write_count++] = SharcIopWrite{.intent = *state.request,
                                                    .value = static_cast<std::uint32_t>(value),
                                                    .posted_at = state.clocks};
  state.last_write = state.clocks;
  state.request.reset();
}

void SharcIopBridge::begin_instruction() {
  if (staged_ || peripheral_.active()) { throw std::logic_error("Nested SHARC IOP retirement"); }
  staged_ = current_;
}

void SharcIopBridge::commit_instruction() noexcept {
  if (staged_) {
    current_ = *staged_;
    staged_.reset();
  }
}

void SharcIopBridge::abort_instruction() noexcept {
  staged_.reset();
}

SharcIopState &SharcIopBridge::working() {
  return staged_ ? *staged_ : current_;
}

const SharcIopState &SharcIopBridge::working() const {
  return staged_ ? *staged_ : current_;
}

}
