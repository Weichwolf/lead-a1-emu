// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_spi_port.hpp"

#include "common/memory.hpp"
#include "sharc/peripherals/sharc_spi.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t dma_offset = 0x200;
constexpr std::uint32_t dma_last = dma_offset + 16;
constexpr std::uint32_t receive_offset = 16;
constexpr std::uint32_t transmit_offset = 12;
constexpr std::uint32_t initiation_mask = 3;
constexpr std::uint32_t dma_mode = 2;

void validate_port(MemoryAccess access) {
  if (access.size != 4 || access.address % 4 != 0 || access.address > dma_last) {
    throw std::runtime_error("SHARC SPI port requires aligned32-bit IOP1000..1084 access");
  }
}

}

std::uint64_t SharcSpi::port_read(MemoryAccess access) {
  auto &next = working();
  if (access.address == 0) { return next.port_control; }
  if (access.address != receive_offset) {
    throw std::runtime_error("SHARC SPI port register read not implemented");
  }
  if (!next.receive_latch) {
    throw std::runtime_error("SHARC RXSPI data latch unknown after bootstrap handoff");
  }
  const auto result = *next.receive_latch;
  next.receive.reset();
  return result;
}

void SharcSpi::port_write(MemoryAccess access, std::uint64_t value) {
  if (value > std::numeric_limits<std::uint32_t>::max()) {
    throw std::runtime_error("SHARC SPI port write exceeds32 bits");
  }
  auto &next = working();
  if (access.address == transmit_offset) {
    if (next.transmit) { throw std::runtime_error("SHARC full TXSPI overwrite not qualified"); }
    next.transmit = static_cast<std::uint32_t>(value);
    return;
  }
  if (access.address != 0) {
    throw std::runtime_error("SHARC SPI port register write not implemented");
  }
  constexpr std::uint32_t supported_variable_bits = 0x3f;
  constexpr std::uint32_t slave32_mode3_enabled = 0x4d00;
  const auto mode = value & initiation_mask;
  if ((value & ~std::uint64_t{supported_variable_bits}) != slave32_mode3_enabled ||
      (mode != 0 && mode != dma_mode)) {
    throw std::runtime_error("SHARC SPI control mode/enable/format change not qualified");
  }
  if (value != next.port_control &&
      (next.serial.bits != 0 || next.burst_remaining != 0 || (next.control & 1) != 0)) {
    throw std::runtime_error("SHARC active SPI/DMA reconfiguration not qualified");
  }
  next.port_control = static_cast<std::uint32_t>(value);
}

std::optional<bool> SharcSpi::miso() const {
  const auto &next = state();
  constexpr std::uint32_t disable_miso = 0x20;
  if (!next.serial.selected || (next.port_control & disable_miso) != 0) { return {}; }
  if (!next.transmit_shift) { throw std::runtime_error("SHARC driven MISO data unknown"); }
  return ((*next.transmit_shift >> next.transmit_bit) & 1) != 0;
}

bool SharcSpi::interrupt_request() const {
  const auto &next = state();
  const bool core_receive = (next.port_control & initiation_mask) == 0 && next.receive.has_value();
  constexpr std::uint32_t error_enable = 0x100;
  constexpr std::uint32_t receive_dma = 2;
  const bool error = (next.control & receive_dma) != 0 ? next.overflow : next.transmit_underflow;
  return next.interrupt_pending || core_receive || ((next.control & error_enable) != 0 && error);
}

std::uint64_t SharcSpiPort::read(MemoryAccess access) {
  validate_port(access);
  if (access.address < dma_offset) { return spi_.port_read(access); }
  access.address -= dma_offset;
  return spi_.read(access);
}

void SharcSpiPort::write(MemoryAccess access, std::uint64_t value) {
  validate_port(access);
  if (access.address < dma_offset) {
    spi_.port_write(access, value);
    return;
  }
  access.address -= dma_offset;
  spi_.write(access, value);
}

void SharcSpiPort::begin_instruction() {
  spi_.begin_instruction();
}

void SharcSpiPort::commit_instruction() noexcept {
  spi_.commit_instruction();
}

void SharcSpiPort::abort_instruction() noexcept {
  spi_.abort_instruction();
}

}
