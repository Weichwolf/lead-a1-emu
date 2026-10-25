// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0x2000;
constexpr std::uint32_t initial_word = 0x12345678;
constexpr std::uint32_t initial_low_byte = 0x78;
constexpr std::uint32_t instruction_pc = 0x1000;
constexpr std::uint32_t overlay_word = 0x12aabb78;
constexpr std::uint32_t overlay_halfword = 0xaabb;
constexpr std::uint32_t unmapped_address = 0x3000;

void require(bool condition) {
  if (!condition) { throw std::runtime_error("Independent RAM expectation failed"); }
}

template <typename Function> void fault(Function function, const std::string &message) {
  try {
    function();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(message) != std::string::npos);
    return;
  }
  throw std::runtime_error("Expected RAM fault did not occur");
}

void check_ram() {
  lead_a1::Memory memory;
  memory.map({.address = base, .size = 16, .name = "unknown"});
  memory.set_pc(instruction_pc);
  require(!memory.peek(base));
  fault([&memory] { static_cast<void>(memory.read({.address = base, .size = 4})); },
        "address=0x2000 pc=0x1000 width=4: uninitialized");
  memory.write({.address = base, .size = 4}, initial_word);
  require(memory.read({.address = base}) == initial_low_byte);
  require(memory.read({.address = base, .size = 4}) == initial_word);
  fault([&memory] { memory.write({.address = base + 15, .size = 2}, 7); }, "boundary");
  require(!memory.peek(base + 15));
  memory.begin_instruction();
  memory.write({.address = base + 1, .size = 2}, overlay_halfword);
  require(memory.read({.address = base, .size = 4}) == overlay_word);
  memory.abort_instruction();
  require(memory.read({.address = base, .size = 4}) == initial_word);
  memory.begin_instruction();
  memory.write({.address = base + 4, .size = 4}, 11);
  require(memory.read({.address = base + 4, .size = 4}) == 11);
  memory.commit_instruction();
  require(memory.read({.address = base + 4, .size = 4}) == 11);
  fault([&memory] { memory.commit_instruction(); }, "No active");
  fault([&memory] { static_cast<void>(memory.read({.address = unmapped_address})); }, "unmapped");
  fault([&memory] { static_cast<void>(memory.read({.address = base, .size = 0})); }, "width");
  fault(
      [&memory] {
        static_cast<void>(
            memory.read({.address = std::numeric_limits<std::uint32_t>::max(), .size = 2}));
      },
      "wrap");
  fault([&memory] { memory.map({.address = base, .size = 4, .name = "overlap"}); }, "Overlapping");
  constexpr std::uint32_t read_only_base = 0x4000;
  constexpr std::array known{std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}};
  memory.map({.address = read_only_base,
              .size = 4,
              .name = "read only",
              .writable = false,
              .executable = false},
             known);
  fault([&memory] { memory.write({.address = read_only_base}, 0); }, "permission");
  fault(
      [&memory] {
        static_cast<void>(memory.read({.address = read_only_base}, lead_a1::AccessKind::fetch));
      },
      "permission");
  require(memory.read({.address = read_only_base}) == 1);
}

void offset_initialization() {
  lead_a1::Memory memory;
  constexpr std::size_t region_bytes = 16;
  constexpr std::size_t offset = 4;
  constexpr std::array known{std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}};
  memory.map({.address = base, .size = region_bytes, .name = "offset image"}, known, offset);
  constexpr std::uint32_t word = 0x04030201;
  require(memory.read({.address = base + offset, .size = 4}) == word);
  require(!memory.peek(base) && !memory.peek(base + offset - 1));
  require(!memory.peek(base + offset + known.size()));
  fault([&memory] { static_cast<void>(memory.read({.address = base + 3, .size = 4})); },
        "uninitialized");
  fault([&memory] { static_cast<void>(memory.read({.address = base + 5, .size = 4})); },
        "uninitialized");
  fault(
      [&memory, &known] {
        memory.map({.address = unmapped_address, .size = region_bytes, .name = "overflow"},
                   known,
                   region_bytes - known.size() + 1);
      },
      "Invalid RAM mapping");
  fault(
      [&memory] {
        memory.map({.address = unmapped_address, .size = region_bytes, .name = "overflow"},
                   {},
                   std::numeric_limits<std::size_t>::max());
      },
      "Invalid RAM mapping");
  memory.map(
      {.address = unmapped_address, .size = region_bytes, .name = "empty image"}, {}, region_bytes);
  require(!memory.peek(unmapped_address + region_bytes - 1));
}

}

int main() {
  try {
    check_ram();
    offset_initialization();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
