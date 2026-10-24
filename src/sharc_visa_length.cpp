// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_visa.hpp"

#include <cstdint>
#include <ios>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {
namespace {

std::optional<unsigned int> compute_suffix(std::span<const std::uint16_t> prefix) {
  if (prefix.size() < 2) { return {}; }
  constexpr unsigned int no_compute = 0x3f;
  constexpr unsigned int suffix_mask = 0x7f;
  return (prefix[1] & suffix_mask) == no_compute ? 2U : 3U;
}

unsigned int memory_prefix(std::uint16_t head) {
  constexpr unsigned int selector_shift = 2;
  const auto selector = (std::uint32_t{head} >> selector_shift) & 7U;
  if ((selector & 4U) != 0) { return 1; }           // Type3c, bit36=1.
  if (selector == 1 || selector == 2) { return 2; } // Type16b / Type15b.
  if (selector == 0) { return 3; }                  // Type16a.
  throw std::runtime_error("Reserved VISA memory length selector");
}

bool full_word(unsigned int type) {
  constexpr unsigned int direct_move = 0x10;
  constexpr unsigned int bit_operation = 0x14;
  constexpr unsigned int modify = 0x16;
  constexpr unsigned int stack = 0x17;
  constexpr unsigned int cjump = 0x18;
  return type == 2 || type == 6 || type == 7 || (type >= 12 && type <= 14) ||
         (type >= direct_move && type <= bit_operation) || type == modify || type == stack ||
         type == cjump;
}

std::optional<unsigned int> byte_prefix(std::span<const std::uint16_t> prefix) {
  constexpr unsigned int byte_shift = 8;
  const auto head = std::uint32_t{prefix[0]};
  const auto type = head >> byte_shift;
  constexpr unsigned int compact = 0x80;
  if (type == 1 || type == 15) { return (head & compact) != 0 ? 2U : 3U; }
  if (type == 10 || type == 11) { return (head & compact) != 0 ? 1U : 3U; }
  if (type == 4 || type == 8 || type == 9) { return compute_suffix(prefix); }
  if (type == 0) {
    constexpr unsigned int short_tag_mask = 0x3f;
    const auto tag = head & short_tag_mask;
    if (tag == 1) { return 1; } // Type21c / Type22c.
    if (tag == 0) { return 3; }
  }
  constexpr unsigned int rframe = 0x19;
  constexpr unsigned int compact_rframe = 0x1901;
  constexpr unsigned int full_rframe = 0x1900;
  if (type == rframe && head == compact_rframe) { return 1; }
  if (full_word(type) || head == full_rframe) { return 3; }
  std::ostringstream message;
  message << "Unsupported VISA length prefix 0x" << std::hex << head;
  throw std::runtime_error(message.str());
}

std::optional<unsigned int> length(std::span<const std::uint16_t> prefix) {
  constexpr unsigned int group_shift = 13;
  constexpr unsigned int nibble_shift = 12;
  constexpr unsigned int short_compute = 12;
  const auto head = std::uint32_t{prefix[0]};
  const auto group = head >> group_shift;
  const auto nibble = head >> nibble_shift;
  if (group == 1 || group == 2 || group == 3) { return compute_suffix(prefix); }
  if (nibble == 9) { return memory_prefix(prefix[0]); }
  if (nibble == short_compute) { return 1; }   // Type2c; Type10 is ISA only.
  if (nibble == 8 || group == 5) { return 3; } // Type6a / Type15a.
  return byte_prefix(prefix);
}

}

std::optional<unsigned int> sharc_visa_words(std::span<const std::uint16_t> prefix) {
  if (prefix.size() > 3) { throw std::invalid_argument("VISA prefix exceeds three short words"); }
  if (prefix.empty()) { return {}; }
  return length(prefix);
}

}
