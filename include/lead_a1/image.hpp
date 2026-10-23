// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace lead_a1 {

inline constexpr std::size_t image_header_bytes = 20;
inline constexpr std::size_t descriptor_bytes = 44;
inline constexpr std::size_t descriptor_slots = 16;
inline constexpr std::size_t image_payload_offset =
    image_header_bytes + (descriptor_bytes * descriptor_slots);
inline constexpr std::size_t maximum_decoded_section = std::size_t{16} * 1024 * 1024;

struct ImageSection {
  std::array<char, 4> tag{};
  std::uint32_t load_address{};
  std::uint32_t stored_offset{};
  std::uint32_t stored_length{};
  bool compressed{};
  std::vector<std::byte> bytes;
};

struct FirmwareImage {
  std::uint16_t version{};
  std::uint16_t type{};
  // Observed header words retained without inventing meanings.
  std::array<std::uint32_t, 3> unresolved_header{};
  std::array<ImageSection, 3> sections;
};

[[nodiscard]] std::uint32_t section_checksum(std::span<const std::byte> bytes);
[[nodiscard]] FirmwareImage decode_image(std::span<const std::byte> bytes);

}
