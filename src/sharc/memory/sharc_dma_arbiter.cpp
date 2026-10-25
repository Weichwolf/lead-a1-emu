// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/memory/sharc_dma_arbiter.hpp"

#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/memory/sharc_memory_view.hpp"
#include "sharc/peripherals/sharc_control.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <ios>
#include <limits>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace lead_a1 {
namespace {
constexpr unsigned int groups = 14;
constexpr std::uint16_t all_heads = 0x3fff;
using Nominees = std::array<std::optional<SharcDmaCandidate>, groups>;

unsigned int group_index(SharcDmaGroup group) {
  const auto index = std::to_underlying(group);
  if (index >= groups) { throw std::invalid_argument("Invalid peripheral DMA group"); }
  return index;
}

void validate_candidate(const SharcDmaCandidate &candidate) {
  constexpr std::array<unsigned int, groups> channels{4, 4, 4, 8, 1, 0, 1, 1, 1, 2, 0, 4, 1, 1};
  const auto group = group_index(candidate.group);
  if (channels[group] == 0) { throw std::runtime_error("MLB/link DMA arbitration not qualified"); }
  if (candidate.channel >= channels[group]) {
    throw std::invalid_argument("Invalid DMA group channel ordinal");
  }
  const auto request = candidate.request;
  if (request.access.width != SharcWidth::normal32 ||
      (request.kind != AccessKind::read && request.kind != AccessKind::write) ||
      request.value > std::numeric_limits<std::uint32_t>::max() ||
      (request.kind != AccessKind::write && request.value != 0)) {
    throw std::invalid_argument("Peripheral DMA requires normal32 RAM read/write");
  }
  const auto view = sharc_memory_view(request.access);
  constexpr std::uint32_t cells = 0x40000;
  if (view.bank >= 4 || view.cell >= cells || view.cells > cells - view.cell) {
    throw std::invalid_argument("Invalid peripheral DMA memory block span");
  }
}

Nominees nominees(std::span<const SharcDmaCandidate> requests) {
  Nominees result;
  std::array<unsigned int, groups> seen{};
  for (const auto &candidate : requests) {
    validate_candidate(candidate);
    const auto index = group_index(candidate.group);
    const auto mask = 1U << candidate.channel;
    if ((seen[index] & mask) != 0) {
      throw std::invalid_argument("Duplicate DMA producer request");
    }
    seen[index] |= mask;
    auto &current = result[index];
    if (!current || candidate.channel < current->channel) { current = candidate; }
  }
  return result;
}

std::optional<unsigned int> first(const Nominees &requests, unsigned int head) {
  for (unsigned int offset = 0; offset < groups; ++offset) {
    const auto index = (head + offset) % groups;
    if (requests[index]) { return index; }
  }
  return {};
}

std::optional<unsigned int> winner(const Nominees &requests, SharcDmaArbiterState state) {
  if (state.mode == false) { return first(requests, 0); }
  std::optional<unsigned int> result;
  if (!state.mode) { result = first(requests, 0); } // Unknown mode includes fixed priority.
  for (unsigned int head = 0; head < groups; ++head) {
    if ((state.heads & (1U << head)) == 0) { continue; }
    const auto next = first(requests, head);
    if (result && next != result) {
      throw std::runtime_error("Unknown rotating DMA priority changes the winning group");
    }
    result = next;
  }
  return result;
}

void validate(const SharcDmaArbiterState &state) {
  if (state.heads == 0 || (state.heads & ~all_heads) != 0) {
    throw std::invalid_argument("Malformed DMA rotating priority possibilities");
  }
  if (!state.transfer) { return; }
  const auto &transfer = *state.transfer;
  validate_candidate(transfer.candidate);
  if (transfer.remaining_half_pclk > 2 ||
      (transfer.submitted && transfer.remaining_half_pclk != 0)) {
    throw std::invalid_argument("Malformed DMA captured grant phase");
  }
}
}

SharcDmaArbiter::SharcDmaArbiter(SharcControl &control,
                                 SharcMemory &memory,
                                 SharcDmaArbiterState initial)
    : control_(control), memory_(memory), current_(initial) {
  validate(current_);
}

void SharcDmaArbiter::idle() const {
  if (memory_.active()) { throw std::logic_error("DMA arbitration during a memory transaction"); }
}

SharcDmaArbiterState SharcDmaArbiter::snapshot() const {
  idle();
  return current_;
}

std::optional<SharcDmaCandidate>
SharcDmaArbiter::select(std::span<const SharcDmaCandidate> requests) try {
  idle();
  if (current_.transfer) { throw std::logic_error("DMA grant still owned by its producer"); }
  const auto pending = nominees(requests);
  auto next = current_;
  const auto mode = control_.dma_rotation();
  if (mode != next.mode) { next.heads = all_heads; } // Mode-history phase remains unresolved.
  next.mode = mode;
  const auto selected = winner(pending, next);
  if (!selected) {
    current_ = next;
    return {};
  }
  const auto candidate = pending[*selected];
  if (!candidate) { throw std::logic_error("Missing selected DMA producer"); }
  if (mode != false) {
    const auto head = static_cast<std::uint16_t>(1U << ((*selected + 1) % groups));
    next.heads = mode ? head : static_cast<std::uint16_t>(next.heads | head);
  }
  next.transfer = SharcDmaTransfer{.candidate = *candidate};
  current_ = next;
  return candidate;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC peripheral DMA selection pc=0x" << std::hex << memory_.pc();
  for (const auto &candidate : requests) {
    message << " [group=" << std::dec << static_cast<unsigned int>(candidate.group)
            << " channel=" << candidate.channel << " word-address=0x" << std::hex
            << candidate.request.access.address
            << " operation=" << (candidate.request.kind == AccessKind::write ? "write" : "read")
            << ']';
  }
  message << ": " << error.what();
  throw std::runtime_error(message.str());
}

void SharcDmaArbiter::advance_half_pclk(unsigned int half_periods) {
  idle();
  auto &transfer = current_.transfer;
  if (!transfer || transfer->submitted) { return; }
  auto &remaining = transfer->remaining_half_pclk;
  remaining = half_periods >= remaining ? 0U : remaining - half_periods;
}

void SharcDmaArbiter::launch(SharcMemoryBus &bus) {
  idle();
  if (&bus.memory() != &memory_ || !current_.transfer) {
    throw std::runtime_error("DMA launch requires its captured producer and bound memory bus");
  }
  auto &transfer = *current_.transfer;
  if (transfer.submitted || transfer.remaining_half_pclk != 0 || bus.state().pending[0] ||
      bus.grant(SharcMemoryMaster::io0)) {
    throw std::runtime_error("DMA launch before grant latency or with an occupied IOD0 bus");
  }
  bus.request(SharcMemoryMaster::io0, transfer.candidate.request);
  transfer.submitted = true;
}

void SharcDmaArbiter::retire(SharcMemoryBus &bus) {
  idle();
  if (&bus.memory() != &memory_ || !current_.transfer || !current_.transfer->submitted ||
      bus.state().pending[0] || bus.grant(SharcMemoryMaster::io0)) {
    throw std::runtime_error("DMA retirement requires its consumed bound IOD0 grant");
  }
  current_.transfer.reset();
}
}
