// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_fetch.hpp"

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t native_begin = 0x80000;
constexpr std::uint32_t native_end = 0x100000;
constexpr std::uint64_t native_bits = 0xffffffffffff;
constexpr std::uint32_t cache_disabled = 0x10;

void validate_address(std::uint32_t address) {
  if (address < native_begin || address >= native_end) {
    throw std::invalid_argument("SHARC fetch requires an internal native address");
  }
}

void validate(const SharcFetchState &state) {
  if (state.fetch1) { validate_address(*state.fetch1); }
  for (const auto &word : {state.fetch2, state.decode, state.address, state.execute}) {
    if (word) {
      validate_address(word->address);
      if ((word->opcode & ~native_bits) != 0) {
        throw std::invalid_argument("Invalid SHARC native fetched word");
      }
    }
  }
}

}

SharcFetch::SharcFetch(SharcMemory &memory, SharcFetchState initial)
    : memory_(memory), state_(initial) {
  validate(state_);
}

void SharcFetch::advance(SharcKnown32 effective_mode2,
                         std::optional<std::uint32_t> next_address) try {
  if (memory_.active()) {
    throw std::logic_error("SHARC fetch advance during a memory transaction");
  }
  if ((effective_mode2.value & ~effective_mode2.known) != 0) {
    throw std::invalid_argument("Invalid SHARC fetch MODE2 knowledge");
  }
  if ((effective_mode2.known & effective_mode2.value & cache_disabled) == 0) {
    throw std::runtime_error("SHARC fetch cache lookup/unknown CADIS not qualified");
  }
  if (state_.execute) { throw std::runtime_error("SHARC fetch Execute word has not retired"); }
  if (next_address) { validate_address(*next_address); }
  auto next = state_;
  next.execute = state_.address;
  next.address = state_.decode;
  next.decode = state_.fetch2;
  next.fetch2.reset();
  if (state_.fetch1) {
    // Table4-1: F1 selects an address; F2 captures the instruction data bus.
    const auto opcode =
        memory_.read({.address = *state_.fetch1, .width = SharcWidth::wide48}, AccessKind::fetch);
    next.fetch2 = SharcFetchWord{.address = *state_.fetch1, .opcode = opcode};
  }
  next.fetch1 = next_address;
  state_ = next;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC fetch transport F1=";
  if (state_.fetch1) {
    message << "0x" << std::hex << *state_.fetch1;
  } else {
    message << "bubble";
  }
  message << " next=";
  if (next_address) {
    message << "0x" << std::hex << *next_address;
  } else {
    message << "bubble";
  }
  message << " MODE2=0x" << std::hex << effective_mode2.value << "/0x" << effective_mode2.known
          << ": " << error.what();
  throw std::runtime_error(message.str());
}

std::uint64_t SharcFetch::for_execution(const SharcMemory &memory, std::uint32_t pc) const {
  if (&memory == &memory_ && state_.execute && state_.execute->address == pc) {
    return state_.execute->opcode;
  }
  std::ostringstream message;
  message << "SHARC fetched retirement pc=0x" << std::hex << pc << " Execute=";
  if (state_.execute) {
    message << "0x" << state_.execute->address << " opcode=0x" << state_.execute->opcode;
  } else {
    message << "bubble";
  }
  message << ": wrong memory binding, PC or missing fetched word";
  throw std::runtime_error(message.str());
}

SharcStep Sharc::step(SharcFetch &fetch) {
  const auto opcode = fetch.for_execution(memory_, state_.pc);
  const auto result = step_opcode(opcode);
  fetch.state_.execute.reset();
  return result;
}

}
