// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/board/sharc_dma_fabric.hpp"

#include "sharc/memory/sharc_dma_arbiter.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/peripherals/sharc_pdap.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/peripherals/sharc_sport.hpp"
#include "sharc/peripherals/sharc_sport_dma.hpp"

#include <algorithm>
#include <array>
#include <ios>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {
namespace {
constexpr std::array sport_groups{
    SharcDmaGroup::sport01, SharcDmaGroup::sport23, SharcDmaGroup::sport45, SharcDmaGroup::sport67};
constexpr std::array sport_channels{SharcSportChannel::a, SharcSportChannel::b};

bool bound(const SharcDmaCandidate &candidate) {
  if (candidate.group == SharcDmaGroup::spi || candidate.group == SharcDmaGroup::idp) {
    return candidate.channel == 0;
  }
  return candidate.channel < 4 &&
         std::ranges::find(sport_groups, candidate.group) != sport_groups.end();
}
}

SharcDmaFabric::SharcDmaFabric(SharcControl &control,
                               SharcMemory &memory,
                               SharcSpi &spi,
                               SharcPdap &pdap,
                               std::array<SharcSport *, 8> sports,
                               SharcDmaFabricState initial)
    : control_(control),
      memory_(memory),
      spi_(spi),
      pdap_(pdap),
      sports_(sports),
      bus_(memory, initial.memory_bus),
      arbiter_(control, memory, initial.arbiter) {
  for (unsigned int index = 0; index < sports_.size(); ++index) {
    if (sports_[index] == nullptr || sports_[index]->number() != index) {
      throw std::invalid_argument("DMA fabric requires eight distinct ordered SPORT owners");
    }
  }
  const auto transfer = initial.arbiter.transfer;
  const auto pending = initial.memory_bus.pending[0];
  const auto grant = initial.memory_bus.granted[0];
  if (transfer && !bound(transfer->candidate)) {
    throw std::invalid_argument("DMA fabric snapshot has an unbound producer");
  }
  if (!transfer || !transfer->submitted) {
    if (pending || grant) {
      throw std::invalid_argument("DMA fabric snapshot has an unowned IOD0 request");
    }
  } else if (pending != transfer->candidate.request && grant != transfer->candidate.request) {
    throw std::invalid_argument("DMA fabric snapshot has no matching submitted request");
  }
}

void SharcDmaFabric::idle() const {
  if (memory_.active()) {
    throw std::logic_error("DMA fabric during an active memory transaction");
  }
}

SharcDmaFabricState SharcDmaFabric::snapshot() const {
  idle();
  return {.memory_bus = bus_.state(), .arbiter = arbiter_.snapshot()};
}

SharcDmaFabric::Candidates SharcDmaFabric::candidates() const {
  Candidates result;
  const auto spi = spi_.dma_word_request();
  if (spi) { result.values[result.count++] = {.group = SharcDmaGroup::spi, .request = *spi}; }
  const auto pdap = pdap_.dma_request();
  if (pdap) { result.values[result.count++] = {.group = SharcDmaGroup::idp, .request = *pdap}; }
  for (unsigned int port = 0; port < sports_.size(); ++port) {
    for (unsigned int lane = 0; lane < sport_channels.size(); ++lane) {
      const auto &dma = sports_[port]->dma(sport_channels[lane]);
      if (dma.locks_iop_buses()) {
        const auto state = dma.snapshot();
        if (!state.load) { throw std::logic_error("Missing locked SPORT TCB load"); }
        std::ostringstream message;
        message << "SHARC DMA fabric pc=0x" << std::hex << memory_.pc() << " word-address=0x"
                << state.load->index_address - state.load->received << " operation=read SPORT"
                << std::dec << port << " lane=" << lane
                << ": TCB bus lock/dual-bus coupling not qualified";
        throw std::runtime_error(message.str());
      }
      const auto request = dma.request();
      if (request) {
        result.values[result.count++] = {.group = sport_groups[port / 2],
                                         .channel = (port % 2 == 0 ? 2U : 0U) + lane,
                                         .request = *request};
      }
    }
  }
  return result;
}

void SharcDmaFabric::service(const SharcDmaCandidate &candidate) {
  const auto grant = bus_.grant(SharcMemoryMaster::io0);
  if (!grant || grant != candidate.request) {
    throw std::runtime_error("DMA fabric service requires its exact selected IOD0 grant");
  }
  if (candidate.group == SharcDmaGroup::spi && candidate.channel == 0) {
    spi_.service_dma_word(bus_);
    return;
  }
  if (candidate.group == SharcDmaGroup::idp && candidate.channel == 0) {
    pdap_.service_dma(bus_);
    return;
  }
  for (unsigned int pair = 0; pair < sport_groups.size(); ++pair) {
    if (candidate.group != sport_groups[pair]) { continue; }
    const auto port = (pair * 2) + (candidate.channel < 2 ? 1U : 0U);
    sports_[port]->dma(sport_channels[candidate.channel % 2]).service(bus_);
    return;
  }
  throw std::runtime_error("DMA fabric producer group is not bound");
}

void SharcDmaFabric::tick() {
  idle();
  for (const auto &grant : bus_.state().granted) {
    if (grant) { throw std::runtime_error("DMA fabric retry/consume grants before next clock"); }
  }
  control_.advance_half_pclk(1);
  arbiter_.advance_half_pclk(1);
  auto transfer = arbiter_.snapshot().transfer;
  if (transfer && !transfer->submitted && transfer->remaining_half_pclk == 0) {
    arbiter_.launch(bus_);
  }
  bus_.advance();
  transfer = arbiter_.snapshot().transfer;
  if (bus_.grant(SharcMemoryMaster::io0)) {
    if (!transfer) { throw std::logic_error("Unowned fabric IOD0 grant"); }
    service(transfer->candidate);
    arbiter_.retire(bus_);
  }
  if (!arbiter_.snapshot().transfer) {
    if (pdap_.dma_discard_ready()) { pdap_.discard_dma_word(); }
    const auto pending = candidates();
    static_cast<void>(arbiter_.select(std::span(pending.values).first(pending.count)));
  }
}

void SharcDmaFabric::retry() {
  idle();
  const auto transfer = arbiter_.snapshot().transfer;
  if (!transfer || !transfer->submitted || !bus_.grant(SharcMemoryMaster::io0)) {
    throw std::runtime_error("DMA fabric has no retained grant to retry");
  }
  service(transfer->candidate);
  arbiter_.retire(bus_);
}
}
