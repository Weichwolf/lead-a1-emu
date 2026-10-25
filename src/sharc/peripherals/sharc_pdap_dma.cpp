// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_idp_dma.hpp"
#include "sharc/peripherals/sharc_pdap.hpp"
#include "sharc/peripherals/sharc_pdap_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {
namespace detail {
void pdap_pop(SharcPdapState &state) {
  if (state.count == 0) { throw std::logic_error("Empty IDP FIFO pop"); }
  state.fifo[state.head] = 0;
  --state.count;
  state.head =
      state.count == 0 ? 0 : (state.head + 1) % static_cast<unsigned int>(state.fifo.size());
}
}

void SharcPdap::acknowledge_dma(std::uint8_t channels) {
  idle();
  require_clock();
  for (unsigned int channel = 0; channel < current_.dma.size(); ++channel) {
    if ((channels & (1U << channel)) != 0) { current_.dma[channel].interrupt_pending = false; }
  }
}

std::optional<SharcMemoryRequest> SharcPdap::dma_request() const {
  idle();
  require_clock();
  if (current_.flush_cclk != 0 || (current_.control[0] & detail::idp_enable) == 0 ||
      !detail::idp_dma_selected(current_, 0) || current_.count == 0 ||
      !detail::idp_dma_active(current_.dma[0])) {
    return {};
  }
  require_dma_owner(current_);
  constexpr std::uint32_t normal_base = 0x80000;
  return SharcMemoryRequest{.access = {.address = normal_base | current_.dma[0].parameters[0]},
                            .kind = AccessKind::write,
                            .value = current_.fifo[current_.head]};
}

void SharcPdap::service_dma(SharcMemoryBus &bus) try {
  const auto request = dma_request();
  if (memory_ != &bus.memory() || !request || bus.grant(SharcMemoryMaster::io0) != request) {
    throw std::runtime_error("IDP DMA service requires its exact bound IOD0 memory grant");
  }
  auto next = current_;
  detail::idp_dma_finish(next.dma[0]);
  detail::pdap_pop(next);
  static_cast<void>(bus.execute(SharcMemoryMaster::io0));
  current_ = next;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "IDP DMA channel0 operation=write pc=0x" << std::hex << bus.memory().pc()
          << " raw-index=0x" << current_.dma[0].parameters[0] << std::dec
          << " remaining=" << current_.dma[0].remaining << ": " << error.what();
  throw std::runtime_error(message.str());
}

bool SharcPdap::dma_discard_ready() const {
  idle();
  require_clock();
  return current_.flush_cclk == 0 && detail::idp_dma_selected(current_, 0) && current_.count != 0 &&
         current_.dma[0].phase == SharcIdpDmaPhase::complete;
}

void SharcPdap::discard_dma_word() {
  if (!dma_discard_ready()) { throw std::runtime_error("No completed-channel IDP DMA discard"); }
  detail::pdap_pop(current_);
}

std::uint64_t SharcPdap::dma_read(MemoryAccess access) {
  require_clock();
  return detail::idp_dma_read(working(), access);
}

void SharcPdap::dma_write(MemoryAccess access, std::uint64_t value) {
  require_clock();
  if (memory_ == nullptr) {
    throw std::runtime_error("IDP DMA parameters require bound RAM owner");
  }
  auto next = working();
  detail::idp_dma_write(next, access, value);
  working() = next;
}
}
