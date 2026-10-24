// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"

#include <cstdint>
#include <optional>

namespace lead_a1::detail {

inline constexpr std::uint32_t sharc_fetch_controls = 0x80010;
void sharc_fetch_validate(const SharcFetchState &state);
void sharc_fetch_address(std::uint32_t address);
void sharc_fetch_mode(const SharcFetchState &state, SharcKnown32 mode);
[[nodiscard]] SharcFetchLookup
sharc_fetch_lookup(SharcCacheState &cache, SharcKnown32 mode, std::uint32_t address);
void sharc_fetch_reserve(SharcCacheState &cache, SharcFetchState &state);
[[nodiscard]] SharcFetchWord sharc_fetch_read(const SharcMemory &memory,
                                              const SharcCacheState &cache,
                                              const SharcFetchState &state,
                                              bool &forwarded);
void sharc_fetch_fill(SharcCacheState &cache, const SharcFetchState &state);
void sharc_fetch_flow_guard(const SharcState &cpu);
[[nodiscard]] bool sharc_fetch_branch_encoding(std::uint64_t opcode);
[[nodiscard]] std::uint32_t sharc_fetch_condition_mask(unsigned int code);
[[nodiscard]] bool sharc_fetch_interlock(const SharcState &cpu, const SharcFetchState &previous);
void sharc_fetch_branch_validate(const SharcFetchState &state);
[[nodiscard]] std::optional<bool> sharc_fetch_branch_retirement(const SharcState &cpu,
                                                                const SharcFetchWord &word);
void sharc_fetch_flow(const SharcState &cpu,
                      const SharcFetchState &previous,
                      SharcFetchState &next);

}
