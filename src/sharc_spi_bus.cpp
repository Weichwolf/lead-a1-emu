// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_memory_bus.hpp"
#include "lead_a1/sharc_spi.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace lead_a1 {
namespace {

void consume(SharcSpiState &state) {
  constexpr std::uint32_t normal_base = 0x80000;
  constexpr std::uint32_t index_mask = normal_base - 1;
  constexpr std::uint32_t sign = 0x8000;
  constexpr std::uint32_t extension = 0xffff0000;
  const auto modifier = static_cast<std::uint32_t>(state.modifier);
  const auto step = (modifier & sign) != 0 ? modifier | extension : modifier;
  state.serial.next_word =
      normal_base |
      static_cast<std::uint32_t>((std::uint64_t{state.serial.next_word} + step) & index_mask);
  --state.serial.remaining;
  --state.burst_remaining;
  for (unsigned int index = 1; index < state.occupied; ++index) {
    state.fifo[index - 1] = state.fifo[index];
  }
  --state.occupied;
  constexpr std::uint32_t interrupt_enable = 4;
  if (state.serial.remaining == 0 && (state.control & interrupt_enable) != 0) {
    state.interrupt_pending = true;
  }
  if (state.receive) {
    state.fifo[state.occupied] = *state.receive;
    ++state.occupied;
    state.receive.reset();
  }
}

}

std::optional<SharcMemoryRequest> SharcSpi::dma_word_request() const {
  const auto &next = state();
  constexpr std::uint32_t enabled = 1;
  if ((next.control & enabled) == 0 || next.serial.remaining == 0) { return {}; }
  if (next.burst_remaining == 0 && dma_request() == 0) { return {}; }
  if (next.occupied == 0) { throw std::logic_error("SHARC SPI active group has no FIFO word"); }
  return SharcMemoryRequest{.access = {.address = next.serial.next_word},
                            .kind = AccessKind::write,
                            .value = next.fifo[0]};
}

void SharcSpi::service_dma_word(SharcMemoryBus &bus) try {
  external_event(bus.memory());
  const auto request = dma_word_request();
  if (!request || bus.grant(SharcMemoryMaster::io0) != request) {
    throw std::runtime_error("SHARC SPI word requires its exact IOD0 memory grant");
  }
  auto next = current_;
  if (next.burst_remaining == 0) { next.burst_remaining = dma_request(); }
  consume(next);
  // RAM failure keeps the grant and the real FIFO/cursor/count/IRQ unchanged.
  static_cast<void>(bus.execute(SharcMemoryMaster::io0));
  static_assert(std::is_nothrow_copy_assignable_v<SharcSpiState>);
  current_ = next;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC arbitrated SPI word pc=0x" << std::hex << bus.memory().pc()
          << " word-address=0x" << current_.serial.next_word << std::dec
          << " remaining=" << current_.serial.remaining << " burst=" << current_.burst_remaining
          << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
