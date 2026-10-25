// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace lead_a1 {

// Observed outer bundle geometry only. No processor or part role is inferred.
inline constexpr std::size_t bundle_header_bytes = 32;
inline constexpr std::size_t bundle_part_count = 2;
inline constexpr std::size_t maximum_bundle_bytes = std::size_t{4} * 1024 * 1024;

struct FirmwarePart {
  std::uint32_t offset{};
  std::uint32_t length{};
};

// Throws on unsupported geometry. Hash/OS identity verification is separate.
[[nodiscard]] std::array<FirmwarePart, bundle_part_count>
inspect_bundle(std::span<const std::byte> bytes);

}
