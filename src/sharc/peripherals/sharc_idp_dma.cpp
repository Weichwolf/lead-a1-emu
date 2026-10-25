// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_idp_dma.hpp"

#include "common/memory.hpp"
#include "sharc/peripherals/sharc_pdap.hpp"
#include "sharc/peripherals/sharc_pdap_internal.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace lead_a1::detail {
namespace {
constexpr std::uint32_t index_mask = 0x7ffff;
constexpr std::uint32_t count_mask = 0xffff;
constexpr std::uint32_t full_count = 65536;
constexpr unsigned int count_field = 4;
constexpr unsigned int ping_count_field = 5;
constexpr std::array<std::uint32_t, 6> masks{
    index_mask, index_mask, 0x3f, index_mask, count_mask, count_mask};

struct Parameter {
  unsigned int channel;
  unsigned int field;
};

Parameter parameter(MemoryAccess access) {
  constexpr unsigned int span = 192;
  constexpr unsigned int channels = 8;
  if (access.size != 4 || access.address % 4 != 0 || access.address >= span) {
    throw std::runtime_error("IDP DMA requires aligned32-bit native2400..242f parameter offsets");
  }
  const auto word = access.address / 4;
  return {.channel = word % channels, .field = word / channels};
}

bool channel_enabled(const SharcPdapState &state, unsigned int channel) {
  constexpr unsigned int shift = 8;
  return (state.control[2] & (1U << (shift + channel))) != 0;
}

bool ping(const SharcPdapState &state, unsigned int channel) {
  constexpr unsigned int shift = 16;
  return (state.control[2] & (1U << (shift + channel))) != 0;
}

std::uint32_t expanded_count(std::uint32_t value) {
  return value == 0 ? full_count : value;
}

bool unconfigured(const SharcIdpDmaChannel &channel) {
  if (channel.count_written || channel.ping_count_written) { return false; }
  return std::ranges::all_of(channel.parameters, [](const auto value) { return value == 0; });
}

void start(SharcPdapState &state, unsigned int index) {
  auto &channel = state.dma[index];
  if (channel.phase != SharcIdpDmaPhase::idle || !idp_dma_selected(state, index)) { return; }
  const bool paired = ping(state, index);
  if (!(paired ? channel.ping_count_written : channel.count_written)) {
    if (!unconfigured(channel)) {
      throw std::runtime_error(
          "IDP DMA partially configured channel has no explicit transfer count");
    }
    return;
  }
  if (paired) { channel.parameters[0] = channel.parameters[1]; }
  channel.remaining = expanded_count(channel.parameters[paired ? ping_count_field : count_field]);
  channel.parameters[count_field] = channel.remaining & count_mask;
  channel.phase = paired ? SharcIdpDmaPhase::ping_a : SharcIdpDmaPhase::standard;
}

void validate_channel(const SharcPdapState &state, unsigned int index) {
  const auto &channel = state.dma[index];
  for (unsigned int field = 0; field < masks.size(); ++field) {
    if ((channel.parameters[field] & ~masks[field]) != 0) {
      throw std::invalid_argument("Malformed IDP DMA parameter width");
    }
  }
  const bool active = idp_dma_active(channel);
  if (channel.phase == SharcIdpDmaPhase::idle && idp_dma_selected(state, index) &&
      !unconfigured(channel)) {
    throw std::invalid_argument("Malformed IDP DMA unstarted configured channel");
  }
  const bool paired =
      channel.phase == SharcIdpDmaPhase::ping_a || channel.phase == SharcIdpDmaPhase::ping_b;
  if (channel.remaining > full_count ||
      (active &&
       (channel.remaining == 0 || !channel_enabled(state, index) || paired != ping(state, index) ||
        !(paired ? channel.ping_count_written : channel.count_written) ||
        channel.parameters[count_field] != (channel.remaining & count_mask))) ||
      (!active && channel.remaining != 0)) {
    throw std::invalid_argument("Malformed IDP DMA active/count phase");
  }
  if (channel.phase == SharcIdpDmaPhase::complete &&
      (!channel.count_written || channel.parameters[count_field] != 0 || ping(state, index))) {
    throw std::invalid_argument("Malformed IDP DMA completed phase");
  }
  if (!channel.ping_count_written && channel.parameters[ping_count_field] != 0) {
    throw std::invalid_argument("Malformed IDP DMA unwritten reload count");
  }
}
}

bool idp_dma_active(const SharcIdpDmaChannel &channel) {
  switch (channel.phase) {
    case SharcIdpDmaPhase::standard:
    case SharcIdpDmaPhase::ping_a:
    case SharcIdpDmaPhase::ping_b: return true;
    case SharcIdpDmaPhase::idle:
    case SharcIdpDmaPhase::complete: return false;
    default: throw std::invalid_argument("Malformed IDP DMA phase identity");
  }
}

bool idp_dma_selected(const SharcPdapState &state, unsigned int channel) {
  // HWR12-23/A-181: DMA status is armed by DMA_EN/ENx. IDP_EN separately
  // gates acquisition/actual FIFO transfers; it must not erase retained DMA.
  return (state.control[0] & idp_dma) != 0 && channel_enabled(state, channel);
}

std::uint8_t idp_dma_status(const SharcPdapState &state) {
  std::uint8_t status{};
  for (unsigned int channel = 0; channel < state.dma.size(); ++channel) {
    if (idp_dma_selected(state, channel) && idp_dma_active(state.dma[channel])) {
      status |= static_cast<std::uint8_t>(1U << channel);
    }
  }
  return status;
}

void idp_dma_validate(const SharcPdapState &state) {
  for (unsigned int channel = 0; channel < state.dma.size(); ++channel) {
    validate_channel(state, channel);
  }
}

void idp_dma_configure(SharcPdapState &state, const SharcPdapState &before) {
  for (unsigned int index = 0; index < state.dma.size(); ++index) {
    auto &channel = state.dma[index];
    if (channel.phase != SharcIdpDmaPhase::idle && ping(state, index) != ping(before, index)) {
      throw std::runtime_error("IDP DMA ping mode change with retained transfer not qualified");
    }
    if (channel_enabled(before, index) && !channel_enabled(state, index)) {
      channel.phase = SharcIdpDmaPhase::idle;
      channel.remaining = 0;
    }
    start(state, index);
  }
}

std::uint64_t idp_dma_read(const SharcPdapState &state, MemoryAccess access) {
  const auto target = parameter(access);
  return state.dma[target.channel].parameters[target.field];
}

void idp_dma_write(SharcPdapState &state, MemoryAccess access, std::uint64_t value) {
  const auto target = parameter(access);
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("Oversized IDP DMA parameter write");
  }
  auto &channel = state.dma[target.channel];
  const bool active = idp_dma_active(channel);
  if ((active || idp_dma_selected(state, target.channel)) && target.field != count_field) {
    throw std::runtime_error("IDP DMA parameter change with retained transfer not qualified");
  }
  channel.parameters[target.field] = static_cast<std::uint32_t>(value) & masks[target.field];
  if (target.field == count_field) {
    channel.count_written = true;
    if (active) {
      channel.remaining = expanded_count(channel.parameters[count_field]);
      return;
    }
  }
  if (target.field == ping_count_field) { channel.ping_count_written = true; }
  channel.phase = SharcIdpDmaPhase::idle;
  start(state, target.channel);
}

void idp_dma_finish(SharcIdpDmaChannel &channel) {
  if (!idp_dma_active(channel) || channel.remaining == 0) {
    throw std::logic_error("Missing active IDP DMA transfer");
  }
  constexpr unsigned int modifier_field = 2;
  constexpr unsigned int modifier_sign = 32;
  constexpr unsigned int modifier_ring = 64;
  constexpr std::int64_t address_ring = 0x80000;
  const auto raw = channel.parameters[modifier_field];
  const auto modifier =
      static_cast<std::int64_t>(raw) - (raw >= modifier_sign ? modifier_ring : 0U);
  channel.parameters[0] =
      static_cast<std::uint32_t>(channel.parameters[0] + modifier + address_ring) & index_mask;
  --channel.remaining;
  channel.parameters[count_field] = channel.remaining & count_mask;
  if (channel.remaining != 0) { return; }
  channel.interrupt_pending = true;
  if (channel.phase == SharcIdpDmaPhase::standard) {
    channel.phase = SharcIdpDmaPhase::complete;
    return;
  }
  const bool next_b = channel.phase == SharcIdpDmaPhase::ping_a;
  channel.phase = next_b ? SharcIdpDmaPhase::ping_b : SharcIdpDmaPhase::ping_a;
  channel.parameters[0] = channel.parameters[next_b ? 3U : 1U];
  channel.remaining = expanded_count(channel.parameters[ping_count_field]);
  channel.parameters[count_field] = channel.remaining & count_mask;
}
}
