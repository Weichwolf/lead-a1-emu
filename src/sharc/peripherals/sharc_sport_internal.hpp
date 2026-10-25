// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "sharc/peripherals/sharc_sport.hpp"

#include <array>
#include <cstdint>

namespace lead_a1::sport_detail {

inline constexpr std::uint32_t control_mask = 0x03ffffff;
inline constexpr std::uint32_t multichannel_mask = 0x0080ffff;
inline constexpr std::uint32_t transmit_bit = 0x02000000;
inline constexpr std::uint32_t tdm_enables = 0x00800001;
inline constexpr unsigned int flush_clocks = 6;

enum class ActionKind : std::uint8_t { none, transmit_start, transmit_finish, receive };

struct Action {
  ActionKind kind{ActionKind::none};
  std::uint32_t value{};
};

using Actions = std::array<Action, 2>;
using DmaStates = std::array<SharcSportDmaState, 2>;

[[nodiscard]] std::array<std::uint32_t, 3> effective(const SharcSportState &state);
[[nodiscard]] bool enabled(const std::array<std::uint32_t, 3> &control, unsigned int channel);
[[nodiscard]] bool ready(const SharcSportState &state, unsigned int channel);
[[nodiscard]] unsigned int length(const SharcSportState &state);
[[nodiscard]] std::uint32_t buffer_status(const DmaStates &dma);
[[nodiscard]] SharcSportSignals outputs(const SharcSportState &state);
void validate(const SharcSportState &state);
[[nodiscard]] Actions
wire(SharcSportState &state, const SharcSportLines &lines, const DmaStates &dma);

}
