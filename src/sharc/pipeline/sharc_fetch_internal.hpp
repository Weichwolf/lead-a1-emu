// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "sharc/core/sharc.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <cstdint>
#include <optional>

namespace lead_a1::detail {

inline constexpr std::uint32_t sharc_fetch_controls = 0x80010;
void sharc_fetch_validate(const SharcFetchState &state);
void sharc_fetch_address(std::uint32_t address);
void sharc_fetch_program_address(std::uint32_t address);
void sharc_fetch_visa_validate(const SharcFetchState &state);
void sharc_fetch_visa_loop_entry(const SharcFetchState &previous, const SharcFetchState &next);
[[nodiscard]] bool sharc_fetch_visa_loop_skip(const SharcFetchState &previous,
                                              const std::optional<SharcFetchWord> &address);
void sharc_fetch_visa_loop_response(const SharcFetchState &previous, SharcFetchState &next);
[[nodiscard]] bool sharc_fetch_visa_loop_request(const SharcFetchState &state);
[[nodiscard]] SharcFetchEvent sharc_fetch_visa_clock(SharcMemory &memory,
                                                     const SharcState &cpu,
                                                     const SharcFetchState &previous,
                                                     SharcFetchState &next,
                                                     std::optional<std::uint32_t> address,
                                                     SharcFetchBus bus,
                                                     SharcFetchRead read);
void sharc_fetch_multiplier_guard(const SharcFetchState &state);
void sharc_fetch_mode(const SharcFetchState &state, SharcKnown32 mode);
[[nodiscard]] SharcFetchLookup
sharc_fetch_lookup(SharcCacheState &cache, SharcKnown32 mode, std::uint32_t address);
void sharc_fetch_reserve(SharcCacheState &cache, SharcFetchState &state);
[[nodiscard]] SharcFetchWord sharc_fetch_read(const SharcMemory &memory,
                                              const SharcCacheState &cache,
                                              const SharcFetchState &state,
                                              bool &forwarded);
void sharc_fetch_fill(SharcCacheState &cache, const SharcFetchState &state);
void sharc_fetch_flow_guard(const SharcState &cpu, const SharcFetchState &transport);
[[nodiscard]] bool sharc_fetch_branch_encoding(std::uint64_t opcode);
[[nodiscard]] std::uint32_t sharc_fetch_condition_mask(unsigned int code);
enum class SharcFetchInterlock : std::uint8_t { none, condition, multiplier, fixed_compute };
[[nodiscard]] bool sharc_fetch_fixed_interlock(const SharcState &cpu, const SharcFetchState &state);
[[nodiscard]] SharcFetchInterlock sharc_fetch_interlock(const SharcState &cpu,
                                                        const SharcFetchState &previous);
[[nodiscard]] std::optional<unsigned int> sharc_fetch_destination(std::uint64_t opcode);
[[nodiscard]] bool
sharc_fetch_dag_stall(const SharcState &cpu, const SharcFetchState &state, SharcFetchBus bus);
void sharc_fetch_dag_prepare(const SharcState &cpu,
                             SharcFetchState &next,
                             const SharcControl *control);
void sharc_fetch_dag_validate(const SharcFetchState &state);
void sharc_fetch_branch_validate(const SharcFetchState &state);
[[nodiscard]] std::optional<bool> sharc_fetch_branch_retirement(const SharcState &cpu,
                                                                const SharcFetchWord &word);
[[nodiscard]] bool sharc_fetch_flow(const SharcState &cpu,
                                    const SharcFetchState &previous,
                                    SharcFetchState &next,
                                    bool address_accepted = true);
void sharc_fetch_quiet_effects(SharcState &cpu,
                               const SharcFetchState &transport,
                               const SharcControl *control);
void sharc_fetch_loop_validate(const SharcFetchState &transport);
void sharc_fetch_arithmetic_validate(const SharcFetchState &transport);
void sharc_fetch_arithmetic_guard(const SharcState &cpu, const SharcFetchState &transport);
void sharc_fetch_arithmetic_clock(SharcState &cpu,
                                  SharcFetchState &transport,
                                  const SharcFetchEvent &event);
[[nodiscard]] SharcLoopClock sharc_fetch_arithmetic_retirement(const SharcState &cpu,
                                                               const SharcFetchState &transport);
void sharc_fetch_loop_guard(const SharcState &cpu, const SharcFetchState &transport);
[[nodiscard]] bool sharc_fetch_loop_stall(const SharcFetchState &transport);
void sharc_fetch_loop_flow(const SharcState &cpu,
                           const SharcFetchState &previous,
                           SharcFetchState &next);
void sharc_fetch_loop_clock(SharcState &cpu,
                            const SharcFetchState &previous,
                            SharcFetchState &next,
                            bool startup_stall);
[[nodiscard]] SharcLoopClock sharc_fetch_loop_retirement(const SharcState &cpu,
                                                         SharcFetchState &transport);
void sharc_fetch_loop_finish(SharcFetchState &transport) noexcept;

}
