// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::size_t word_bytes = 4;
constexpr unsigned int byte_bits = 8;

std::uint32_t read_big_endian(std::span<const std::byte> bytes, std::size_t offset) {
  std::uint32_t value{};
  for (std::size_t index = 0; index < word_bytes; ++index) {
    value = (value << byte_bits) | std::to_integer<std::uint32_t>(bytes[offset + index]);
  }
  return value;
}

}

std::array<FirmwarePart, bundle_part_count> inspect_bundle(std::span<const std::byte> bytes) {
  if (bytes.size() < bundle_header_bytes || bytes.size() > maximum_bundle_bytes) {
    throw std::invalid_argument("Unsupported firmware bundle size");
  }
  constexpr auto reserved_offset = bundle_part_count * word_bytes * 2;
  for (auto offset = reserved_offset; offset < bundle_header_bytes; offset += word_bytes) {
    if (read_big_endian(bytes, offset) != 0) {
      throw std::invalid_argument("Unsupported nonzero reserved bundle word");
    }
  }
  std::array<FirmwarePart, bundle_part_count> parts{};
  std::size_t expected_offset = bundle_header_bytes;
  for (std::size_t index = 0; index < parts.size(); ++index) {
    const auto header_offset = index * word_bytes * 2;
    const auto offset = read_big_endian(bytes, header_offset);
    const auto length = read_big_endian(bytes, header_offset + word_bytes);
    if (offset != expected_offset || length == 0 || length > bytes.size() - expected_offset) {
      throw std::invalid_argument("Noncontiguous, empty or out-of-bounds firmware part");
    }
    parts[index] = {.offset = offset, .length = length};
    expected_offset += length;
  }
  if (expected_offset != bytes.size()) {
    throw std::invalid_argument("Unaccounted trailing firmware bytes");
  }
  return parts;
}

}
