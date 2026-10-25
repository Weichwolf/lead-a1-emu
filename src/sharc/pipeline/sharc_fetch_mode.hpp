// SPDX-License-Identifier: GPL-3.0-only
#pragma once

namespace lead_a1 {
struct SharcState;
struct SharcFetchState;
}

namespace lead_a1::detail {
// NESTM/IRPTEN cannot change captured register banks, operands or widths.
[[nodiscard]] bool sharc_fetch_interrupt_mode(const SharcState &cpu);
void sharc_fetch_interrupt_mode_guard(const SharcState &cpu, const SharcFetchState &transport);
// Other supported mode fields can mature when all already decoded consumers
// are unbanked register-only system operations. No data-mode operand is recaptured.
void sharc_fetch_unbanked_mode_guard(const SharcState &cpu, const SharcFetchState &transport);
}
