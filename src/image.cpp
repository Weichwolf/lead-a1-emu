// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/image.hpp"

#include "minilzo.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <lzo/lzoconf.h>
#include <span>
#include <stdexcept>
#include <vector>

namespace lead_a1 {
namespace {

constexpr std::size_t word_bytes = 4;
constexpr unsigned int byte_bits = 8;
constexpr std::size_t descriptor_reserved_offset = 28;
constexpr std::size_t compressed_checksum_offset = 24;

struct PayloadIdentity {
  std::uint32_t length{};
  std::uint32_t decoded_checksum{};
  std::uint32_t compressed_checksum{};
};

std::uint32_t word(std::span<const std::byte> bytes, std::size_t offset) {
  std::uint32_t value{};
  for (std::size_t index = 0; index < word_bytes; ++index) {
    value = (value << byte_bits) | std::to_integer<std::uint32_t>(bytes[offset + index]);
  }
  return value;
}

bool all_zero(std::span<const std::byte> bytes) {
  return std::ranges::all_of(bytes, [](auto value) { return value == std::byte{}; });
}

void decode_payload(ImageSection &section,
                    std::span<const std::byte> stored,
                    PayloadIdentity expected) {
  if (expected.length == 0 || expected.length > maximum_decoded_section) {
    throw std::invalid_argument("Unsupported decoded section length");
  }
  if (section.compressed) {
    if (section_checksum(stored) != expected.compressed_checksum) {
      throw std::invalid_argument("Compressed section checksum mismatch");
    }
    section.bytes.resize(expected.length);
    lzo_uint result_length = expected.length;
    const auto result =
        lzo1x_decompress_safe(reinterpret_cast<const unsigned char *>(stored.data()),
                              static_cast<lzo_uint>(stored.size()),
                              reinterpret_cast<unsigned char *>(section.bytes.data()),
                              &result_length,
                              nullptr);
    if (result != LZO_E_OK || result_length != expected.length) {
      throw std::invalid_argument("Invalid LZO1X stream or decoded length mismatch");
    }
  } else {
    if (expected.compressed_checksum != 0) {
      throw std::invalid_argument("Unexpected checksum on an uncompressed stream");
    }
    section.bytes.assign(stored.begin(), stored.end());
  }
  if (section_checksum(section.bytes) != expected.decoded_checksum) {
    throw std::invalid_argument("Decoded section checksum mismatch");
  }
}

ImageSection decode_section(std::span<const std::byte> image,
                            std::span<const std::byte> descriptor,
                            std::size_t &expected_offset,
                            const std::array<char, 4> &expected_tag) {
  ImageSection section;
  for (std::size_t index = 0; index < section.tag.size(); ++index) {
    section.tag[index] = static_cast<char>(std::to_integer<unsigned char>(descriptor[index]));
  }
  if (section.tag != expected_tag || !all_zero(descriptor.subspan(descriptor_reserved_offset))) {
    throw std::invalid_argument("Unsupported image tag or reserved descriptor fields");
  }
  section.stored_offset = word(descriptor, 4);
  const auto decoded_length = word(descriptor, 8);
  section.load_address = word(descriptor, 12);
  const auto compressed_length = word(descriptor, 20);
  section.compressed = compressed_length != 0;
  section.stored_length = section.compressed ? compressed_length : decoded_length;
  if (section.stored_offset != expected_offset || section.stored_length == 0 ||
      section.stored_length > image.size() - expected_offset) {
    throw std::invalid_argument("Noncontiguous or out-of-bounds image section");
  }
  constexpr auto address_space_bytes = std::uint64_t{1} << 32;
  if (static_cast<std::uint64_t>(section.load_address) + decoded_length > address_space_bytes) {
    throw std::invalid_argument("Section load range exceeds 32-bit address space");
  }
  decode_payload(section,
                 image.subspan(expected_offset, section.stored_length),
                 {.length = decoded_length,
                  .decoded_checksum = word(descriptor, 16),
                  .compressed_checksum = word(descriptor, compressed_checksum_offset)});
  expected_offset += section.stored_length;
  const auto alignment = (word_bytes - expected_offset % word_bytes) % word_bytes;
  if (alignment > image.size() - expected_offset ||
      !all_zero(image.subspan(expected_offset, alignment))) {
    throw std::invalid_argument("Invalid section alignment padding");
  }
  expected_offset += alignment;
  return section;
}

void check_load_ranges(const FirmwareImage &image) {
  for (std::size_t first = 0; first < image.sections.size(); ++first) {
    const auto &left = image.sections[first];
    const auto left_end = static_cast<std::uint64_t>(left.load_address) + left.bytes.size();
    for (std::size_t second = first + 1; second < image.sections.size(); ++second) {
      const auto &right = image.sections[second];
      const auto right_end = static_cast<std::uint64_t>(right.load_address) + right.bytes.size();
      if (left.load_address < right_end && right.load_address < left_end) {
        throw std::invalid_argument("Overlapping decoded section load ranges");
      }
    }
  }
}

}

std::uint32_t section_checksum(std::span<const std::byte> bytes) {
  std::uint64_t sum{};
  for (auto value : bytes) { sum += std::to_integer<std::uint8_t>(value); }
  return ~static_cast<std::uint32_t>(sum & std::numeric_limits<std::uint32_t>::max());
}

FirmwareImage decode_image(std::span<const std::byte> bytes) {
  if (bytes.size() < image_payload_offset) {
    throw std::invalid_argument("Truncated firmware image header/descriptors");
  }
  FirmwareImage image;
  constexpr unsigned int halfword_bits = 16;
  image.version = static_cast<std::uint16_t>(word(bytes, 0) >> halfword_bits);
  image.type =
      static_cast<std::uint16_t>(word(bytes, 0) & std::numeric_limits<std::uint16_t>::max());
  for (std::size_t index = 0; index < image.unresolved_header.size(); ++index) {
    image.unresolved_header[index] = word(bytes, word_bytes + (index * word_bytes));
  }
  if (word(bytes, 16) != image.sections.size()) {
    throw std::invalid_argument("Unsupported image descriptor count");
  }
  constexpr std::array<std::array<char, 4>, 3> expected_tags{
      {{'S', 'R', 'A', 'M'}, {'C', 'O', 'D', 'E'}, {'A', 'R', 'M', '0'}}};
  std::size_t offset = image_payload_offset;
  for (std::size_t index = 0; index < image.sections.size(); ++index) {
    const auto descriptor =
        bytes.subspan(image_header_bytes + (index * descriptor_bytes), descriptor_bytes);
    image.sections[index] = decode_section(bytes, descriptor, offset, expected_tags[index]);
  }
  constexpr auto populated_end = image_header_bytes + (expected_tags.size() * descriptor_bytes);
  if (offset != bytes.size() ||
      !all_zero(bytes.subspan(populated_end, image_payload_offset - populated_end))) {
    throw std::invalid_argument("Trailing image bytes or populated unused descriptor");
  }
  check_load_ranges(image);
  return image;
}

}
