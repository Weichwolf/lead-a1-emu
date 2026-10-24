// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"

namespace lead_a1::detail {

[[nodiscard]] bool sharc_fetch_long_flow(const SharcState &cpu,
                                         const SharcFetchState &previous,
                                         SharcFetchState &next);
void sharc_fetch_long_validate(const SharcFetchState &transport);
void sharc_fetch_long_guard(const SharcState &cpu, const SharcFetchState &transport);
void sharc_fetch_long_retirement(const SharcState &cpu, SharcFetchState &transport);
void sharc_fetch_long_finish(SharcFetchState &transport) noexcept;
void sharc_fetch_long_clock(const SharcState &cpu, SharcFetchState &next);
[[nodiscard]] bool sharc_fetch_long_return_stall(const SharcFetchState &transport);

}
