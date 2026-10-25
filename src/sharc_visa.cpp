// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_visa.hpp"

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_memory_view.hpp"
#include "sharc_visa_internal.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <span>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t first_short = 0x100000;
constexpr std::uint32_t end_short = 0x200000;
constexpr unsigned int burst_words = 3;

}

void detail::sharc_visa_validate(const SharcVisaState &state) {
  if (state.decode < first_short || state.decode > end_short || state.fetch < state.decode ||
      state.fetch > end_short || state.count > state.data.size()) {
    throw std::invalid_argument("Invalid VISA buffer addresses/count");
  }
  const auto buffered_end = state.decode + state.count;
  const auto expected = buffered_end + (state.delay ? burst_words : 0U);
  if (state.fetch != expected || (state.delay && state.delay->address != buffered_end)) {
    throw std::invalid_argument("Invalid VISA buffer continuity/delay");
  }
  for (unsigned int index = state.count; index < state.data.size(); ++index) {
    if (state.data[index] != 0) { throw std::invalid_argument("Nonempty unused VISA buffer cell"); }
  }
}

namespace {

void consume(SharcVisaState &state, unsigned int words) {
  for (unsigned int index = words; index < state.count; ++index) {
    state.data[index - words] = state.data[index];
  }
  state.count -= words;
  state.decode += words;
  for (unsigned int index = state.count; index < state.data.size(); ++index) {
    state.data[index] = 0;
  }
}

}

SharcVisaBuffer::SharcVisaBuffer(SharcMemory &memory, SharcVisaState state)
    : memory_(memory), state_(state) {
  detail::sharc_visa_validate(state_);
}

void SharcVisaBuffer::capture() try {
  if (memory_.active()) { throw std::logic_error("VISA capture during a foreign transaction"); }
  if (state_.delay) { throw std::runtime_error("VISA delay register is occupied"); }
  const auto address = state_.fetch;
  const auto first = sharc_memory_view({.address = address, .width = SharcWidth::short16});
  const auto last =
      sharc_memory_view({.address = address + burst_words - 1, .width = SharcWidth::short16});
  if (first.bank != last.bank) {
    throw std::runtime_error("VISA cross-block burst arbitration not qualified");
  }
  SharcVisaBurst burst{.address = address};
  for (unsigned int index = 0; index < burst_words; ++index) {
    burst.data[index] = static_cast<std::uint16_t>(memory_.read(
        {.address = address + index, .width = SharcWidth::short16}, AccessKind::fetch));
  }
  auto next = state_;
  next.delay = burst;
  next.fetch += burst_words;
  detail::sharc_visa_validate(next);
  state_ = next;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC VISA F2 pc=0x" << std::hex << memory_.pc() << " short-address=0x"
          << state_.fetch << " operation=fetch words=3: " << error.what();
  throw std::runtime_error(message.str());
}

std::optional<SharcVisaInstruction> SharcVisaBuffer::instruction() const try {
  const auto available = std::min(state_.count, burst_words);
  const auto words = sharc_visa_words(std::span(state_.data).first(available));
  if (!words || state_.count < *words) { return {}; }
  if (*words == 0 || *words > burst_words) {
    throw std::logic_error("Invalid VISA instruction length");
  }
  SharcVisaInstruction result{.address = state_.decode, .words = *words};
  constexpr unsigned int word_bits = 16;
  for (unsigned int index = 0; index < burst_words; ++index) {
    if (index < *words) {
      result.opcode |= std::uint64_t{state_.data[index]} << ((2U - index) * word_bits);
    }
  }
  return result;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC VISA IAB pc=0x" << std::hex << memory_.pc() << " short-address=0x"
          << state_.decode << " operation=decode buffered=" << std::dec << state_.count << ": "
          << error.what();
  throw std::runtime_error(message.str());
}

std::optional<SharcVisaInstruction> SharcVisaBuffer::advance(bool accept) try {
  if (memory_.active()) {
    throw std::logic_error("VISA Decode clock during a foreign transaction");
  }
  std::optional<SharcVisaInstruction> decoded;
  if (accept) {
    decoded = instruction();
    if (!decoded) {
      throw std::runtime_error("VISA Decode requires a complete buffered instruction");
    }
  }
  auto next = state_;
  if (decoded) { consume(next, decoded->words); }
  const auto delay = next.delay;
  if (delay && next.count <= next.data.size() - burst_words) {
    for (unsigned int index = 0; index < burst_words; ++index) {
      next.data[next.count + index] = delay->data[index];
    }
    next.count += burst_words;
    next.delay.reset();
  }
  detail::sharc_visa_validate(next);
  state_ = next;
  return decoded;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC VISA IAB pc=0x" << std::hex << memory_.pc() << " short-address=0x"
          << state_.decode << " operation=advance: " << error.what();
  throw std::runtime_error(message.str());
}

}
