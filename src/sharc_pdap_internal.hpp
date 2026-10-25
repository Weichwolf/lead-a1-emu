// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/sharc_pdap.hpp"

#include <cstdint>

namespace lead_a1::detail {

inline constexpr std::uint32_t pdap_pin_mask = 0xfffff;
inline constexpr std::uint32_t pdap_enable = 0x80000000;
inline constexpr std::uint32_t pdap_reset = 0x40000000;
inline constexpr std::uint32_t pdap_edge = 0x20000000;
inline constexpr std::uint32_t pdap_select = 0x4000000;
inline constexpr std::uint32_t pdap_configuration = 0xbc0fffff;
inline constexpr std::uint32_t idp_enable = 0x80;
inline constexpr std::uint32_t idp_dma = 0x20;
inline constexpr std::uint32_t idp_clear_overflow = 0x40;
inline constexpr std::uint32_t idp_bhd = 0x10;
inline constexpr std::uint32_t idp_fifo_clear = 0x80000000;
inline constexpr std::uint32_t idp_ping_pong = 0xff0000;

[[nodiscard]] unsigned int pdap_mode(const SharcPdapState &state);
[[nodiscard]] unsigned int pdap_samples(unsigned int mode);
[[nodiscard]] bool pdap_active(const SharcPdapState &state);
void pdap_validate(const SharcPdapState &state);
void pdap_validate_controls(const SharcPdapState &state);
void pdap_validate_input(SharcPdapInput input);
void pdap_flush(SharcPdapState &state, bool overflow);
void pdap_push(SharcPdapState &state, std::uint32_t word);
void pdap_sample(SharcPdapState &state, SharcPdapInput input);

}
