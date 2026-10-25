// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_visa.hpp"

#include <cstdint>

namespace lead_a1::detail {

[[nodiscard]] bool sharc_visa_address(std::uint32_t address) noexcept;
void sharc_visa_validate(const SharcVisaState &state);
[[nodiscard]] std::uint64_t sharc_visa_decode(SharcVisaInstruction instruction);
void sharc_visa_execution_guard(const SharcState &state, SharcVisaInstruction instruction);

}
