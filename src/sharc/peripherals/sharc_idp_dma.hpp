// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstdint>

namespace lead_a1 {

enum class SharcIdpDmaPhase : std::uint8_t { idle, standard, ping_a, ping_b, complete };

struct SharcIdpDmaChannel {
  // Native register groups: I, IA, M, IB, C, PC. HWR B-17/18 reset0.
  std::array<std::uint32_t, 6> parameters{};
  bool count_written{};
  bool ping_count_written{};
  SharcIdpDmaPhase phase{};
  std::uint32_t remaining{};
  bool interrupt_pending{};
  bool operator==(const SharcIdpDmaChannel &) const = default;
};

}
