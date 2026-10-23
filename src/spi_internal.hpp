// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/spi.hpp"

#include <cstdint>

namespace lead_a1::detail {

inline constexpr std::uint32_t spi_enable = 0x01000000;
inline constexpr std::uint32_t spi_loopback = 0x10000;
inline constexpr std::uint32_t spi_tx_flag = 0x200;
inline constexpr std::uint32_t spi_rx_flag = 0x100;
inline constexpr std::uint32_t spi_overrun = 0x40;
inline constexpr std::uint32_t spi_error_mask = 0x1f;
inline constexpr std::uint32_t spi_flags_mask = 0x35f;
inline constexpr std::uint32_t spi_hold = 0x10000000;
inline constexpr std::uint32_t spi_word_delay = 0x04000000;
inline constexpr std::uint32_t spi_polarity = 0x20000;
inline constexpr std::uint32_t spi_phase = 0x10000;
inline constexpr std::uint32_t spi_reverse = 0x100000;
inline constexpr std::uint32_t spi_parity = 0x400000;
inline constexpr std::uint32_t spi_odd = 0x800000;
inline constexpr std::uint16_t spi_clock_pin = 0x200;
inline constexpr std::uint16_t spi_output_pin = 0x400;
inline constexpr std::uint16_t spi_serial_pins = 0xe00;

[[nodiscard]] SpiState spi_reset(SpiResetReference input);
void spi_reset_data(SpiState &state);
void spi_submit(SpiState &state, SpiWord word);
void spi_advance(SpiState &state, SpiInput input);
void spi_consume(SpiState &state);
[[nodiscard]] std::uint32_t spi_pending(const SpiState &state);

}
