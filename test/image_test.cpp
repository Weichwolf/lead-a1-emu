// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/image.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

constexpr std::size_t word_bytes = 4;
constexpr unsigned int byte_bits = 8;
constexpr std::size_t checksum_field = 16;
constexpr std::size_t compressed_length_field = 20;
constexpr std::size_t compressed_checksum_field = 24;
constexpr std::size_t reserved_field = 28;
constexpr std::size_t fixture_bytes = lead_a1::image_payload_offset + 16;
constexpr std::uint32_t second_load_address = 0x2000;
constexpr std::uint32_t third_load_address = 0x3000;
constexpr std::uint32_t first_checksum = 0xfffffffe;
constexpr std::uint32_t code_checksum = 0xfffffe69;   // ~(108 + 101 + 97 + 100)
constexpr std::uint32_t stream_checksum = 0xfffffe43; // Includes 0x15 and end marker 0x11.
constexpr std::uint32_t third_checksum = 0xfffffff5;  // ~(1 + 2 + 3 + 4)
constexpr std::uint32_t short_checksum = 0xfffffffc;

void require(bool condition) {
  if (!condition) { throw std::runtime_error("Independent image expectation failed"); }
}

void put_word(std::span<std::byte> bytes, std::size_t offset, std::uint32_t value) {
  for (std::size_t index = 0; index < word_bytes; ++index) {
    const auto shift = static_cast<unsigned int>((word_bytes - index - 1) * byte_bits);
    bytes[offset + index] =
        static_cast<std::byte>((value >> shift) & std::numeric_limits<std::uint8_t>::max());
  }
}

std::vector<std::byte> fixture() {
  std::vector<std::byte> bytes(fixture_bytes);
  put_word(bytes, 0, 1U << 16);
  put_word(bytes, 16, 3);
  constexpr std::array<std::array<char, 4>, 3> tags{
      {{'S', 'R', 'A', 'M'}, {'C', 'O', 'D', 'E'}, {'A', 'R', 'M', '0'}}};
  constexpr std::array<std::uint32_t, 3> lengths{1, 4, 4};
  constexpr std::array<std::uint32_t, 3> addresses{0x1000, second_load_address, third_load_address};
  constexpr std::array<std::uint32_t, 3> offsets{lead_a1::image_payload_offset,
                                                 lead_a1::image_payload_offset + 4,
                                                 lead_a1::image_payload_offset + 12};
  constexpr std::array<std::uint32_t, 3> checksums{first_checksum, code_checksum, third_checksum};
  for (std::size_t index = 0; index < tags.size(); ++index) {
    const auto base = lead_a1::image_header_bytes + (index * lead_a1::descriptor_bytes);
    for (std::size_t byte = 0; byte < tags[index].size(); ++byte) {
      bytes[base + byte] = static_cast<std::byte>(tags[index][byte]);
    }
    put_word(bytes, base + 4, offsets[index]);
    put_word(bytes, base + 8, lengths[index]);
    put_word(bytes, base + 12, addresses[index]);
    put_word(bytes, base + checksum_field, checksums[index]);
  }
  const auto code_descriptor = lead_a1::image_header_bytes + lead_a1::descriptor_bytes;
  put_word(bytes, code_descriptor + compressed_length_field, 8);
  put_word(bytes, code_descriptor + compressed_checksum_field, stream_checksum);
  const auto payload = lead_a1::image_payload_offset;
  bytes[payload] = std::byte{1};
  // Owned literal-only LZO1X stream: four literals "lead", explicit end marker.
  constexpr std::array<std::byte, 8> stream{std::byte{0x15},
                                            std::byte{'l'},
                                            std::byte{'e'},
                                            std::byte{'a'},
                                            std::byte{'d'},
                                            std::byte{0x11},
                                            std::byte{0},
                                            std::byte{0}};
  for (std::size_t index = 0; index < stream.size(); ++index) {
    bytes[payload + 4 + index] = stream[index];
  }
  for (std::size_t index = 0; index < word_bytes; ++index) {
    bytes[payload + 12 + index] = static_cast<std::byte>(index + 1);
  }
  return bytes;
}

void reject(std::span<const std::byte> bytes) {
  try {
    static_cast<void>(lead_a1::decode_image(bytes));
  } catch (const std::invalid_argument &) { return; }
  throw std::runtime_error("Malformed image was accepted");
}

void check_image() {
  const auto bytes = fixture();
  const auto image = lead_a1::decode_image(bytes);
  require(image.version == 1 && image.type == 0);
  require(image.sections[1].compressed);
  constexpr std::array<std::byte, 4> expected{
      std::byte{'l'}, std::byte{'e'}, std::byte{'a'}, std::byte{'d'}};
  require(std::vector(expected.begin(), expected.end()) == image.sections[1].bytes);
  require(image.sections[1].load_address == second_load_address);
  require(lead_a1::section_checksum(std::array{std::byte{1}, std::byte{2}}) == short_checksum);
  // Every truncated prefix must fail, including streams/checksums and descriptor table.
  for (std::size_t size = 0; size < bytes.size(); ++size) { reject(std::span(bytes).first(size)); }
  auto changed = bytes;
  changed.push_back(std::byte{});
  reject(changed);
  for (const auto offset : std::array{std::size_t{16},
                                      lead_a1::image_header_bytes,
                                      lead_a1::image_header_bytes + reserved_field,
                                      lead_a1::image_header_bytes + checksum_field,
                                      lead_a1::image_header_bytes + (3 * lead_a1::descriptor_bytes),
                                      lead_a1::image_payload_offset + 1}) {
    changed = bytes;
    changed[offset] = std::byte{1};
    reject(changed);
  }
  const auto descriptor = lead_a1::image_header_bytes + lead_a1::descriptor_bytes;
  changed = bytes;
  put_word(
      changed, descriptor + 8, static_cast<std::uint32_t>(lead_a1::maximum_decoded_section + 1));
  reject(changed);
  changed = bytes;
  put_word(changed, descriptor + 12, 0x1000); // Loaded ranges overlap.
  reject(changed);
  changed = bytes;
  put_word(changed, descriptor + 12, std::numeric_limits<std::uint32_t>::max());
  reject(changed);
  // Corrupt stream with its new checksum retained: safe decoder, not checksum, must reject it.
  changed = bytes;
  changed[lead_a1::image_payload_offset + 4] = std::byte{0};
  const auto stored = std::span(changed).subspan(lead_a1::image_payload_offset + 4, 8);
  put_word(changed, descriptor + compressed_checksum_field, lead_a1::section_checksum(stored));
  reject(changed);
}

}

int main() {
  try {
    check_image();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
