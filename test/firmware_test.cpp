// SPDX-License-Identifier: GPL-3.0-only
#include "firmware/firmware.hpp"

#include <array>
#include <cstddef>
#include <exception>
#include <iostream>
#include <span>
#include <stdexcept>

namespace {

constexpr std::size_t second_part_offset = 35;
constexpr auto overlapping_offset = std::byte{34};
constexpr auto gapped_offset = std::byte{36};
constexpr auto high_length_byte = std::byte{255};
// Owned fixture: 32-byte header, three bytes of part 0, five bytes of part 1.
constexpr std::array<std::byte, 40> synthetic_bundle{std::byte{0},
                                                     std::byte{0},
                                                     std::byte{0},
                                                     std::byte{32},
                                                     std::byte{0},
                                                     std::byte{0},
                                                     std::byte{0},
                                                     std::byte{3},
                                                     std::byte{0},
                                                     std::byte{0},
                                                     std::byte{0},
                                                     std::byte{35},
                                                     std::byte{0},
                                                     std::byte{0},
                                                     std::byte{0},
                                                     std::byte{5}};

void require(bool condition) {
  if (!condition) { throw std::runtime_error("Independent bundle expectation failed"); }
}

void expect_rejection(std::span<const std::byte> bytes) {
  try {
    static_cast<void>(lead_a1::inspect_bundle(bytes));
  } catch (const std::invalid_argument &) { return; }
  throw std::runtime_error("Malformed bundle accepted");
}

void check_boundaries() {
  const auto parts = lead_a1::inspect_bundle(synthetic_bundle);
  require(parts[0].offset == 32 && parts[0].length == 3);
  require(parts[1].offset == second_part_offset && parts[1].length == 5);
  for (std::size_t size = 0; size < synthetic_bundle.size(); ++size) {
    expect_rejection(std::span(synthetic_bundle).first(size));
  }
  auto changed = synthetic_bundle;
  changed[11] = overlapping_offset;
  expect_rejection(changed);
  changed[11] = gapped_offset;
  expect_rejection(changed);
  changed = synthetic_bundle;
  changed[15] = std::byte{0}; // Empty second part, trailing bytes.
  expect_rejection(changed);
  changed[15] = std::byte{4};
  expect_rejection(changed);
  changed = synthetic_bundle;
  changed[12] = high_length_byte; // Large length must fail before overflow/read.
  expect_rejection(changed);
  for (std::size_t index = 16; index < lead_a1::bundle_header_bytes; ++index) {
    changed = synthetic_bundle;
    changed[index] = std::byte{1};
    expect_rejection(changed);
  }
}

}

int main() {
  try {
    check_boundaries();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
