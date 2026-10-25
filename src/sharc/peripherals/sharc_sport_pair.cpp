// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/peripherals/sharc_sport.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1 {
namespace {

struct Register {
  unsigned int port;
  unsigned int word;
};

Register decode(MemoryAccess access) {
  constexpr unsigned int bytes = 4;
  constexpr unsigned int dma_first = 0x40;
  constexpr unsigned int dma_end = 0x50;
  constexpr unsigned int buffers_first = 0x60;
  constexpr unsigned int buffers_end = 0x68;
  constexpr unsigned int canonical_buffers = 24;
  constexpr unsigned int select_first = 5;
  constexpr unsigned int select_end = 13;
  constexpr unsigned int compand_end = 21;
  if (access.size != bytes || access.address % bytes != 0 ||
      access.address >= buffers_end * bytes) {
    throw std::runtime_error("SPORT pair requires aligned32-bit native register offsets");
  }
  const auto word = static_cast<unsigned int>(access.address / bytes);
  if (word < 4) { return {.port = word % 2, .word = word < 2 ? 0U : 3U}; }
  if (word == 4) { return {.port = 0, .word = 2}; }
  if (word >= select_first && word < select_end) {
    return {.port = (word - select_first) / 4, .word = 4 + ((word - select_first) % 4)};
  }
  if (word >= select_end && word < compand_end) {
    return {.port = (word - select_end) / 4, .word = 8 + ((word - select_end) % 4)};
  }
  constexpr unsigned int odd_multi = 0x17;
  constexpr unsigned int error_first = 0x18;
  constexpr unsigned int secondary_first = 0x1a;
  if (word == odd_multi) { return {.port = 1, .word = 2}; }
  if (word == error_first || word == error_first + 1) {
    return {.port = word - error_first, .word = 12};
  }
  if (word == secondary_first || word == secondary_first + 1) {
    return {.port = word - secondary_first, .word = 1};
  }
  if (word >= dma_first && word < dma_end) {
    return {.port = (word - dma_first) / 8, .word = 16 + ((word - dma_first) % 8)};
  }
  if (word >= buffers_first) {
    return {.port = (word - buffers_first) / 4,
            .word = canonical_buffers + ((word - buffers_first) % 4)};
  }
  throw std::runtime_error("Unimplemented SPORT pair register hole");
}

}

SharcSportPair::SharcSportPair(SharcSport &first, SharcSport &second) : ports_{&first, &second} {
  if (first.number() % 2 != 0 || second.number() != first.number() + 1) {
    throw std::invalid_argument("SPORT pair requires adjacent even/odd identities");
  }
}

std::uint64_t SharcSportPair::read(MemoryAccess access) {
  const auto target = decode(access);
  const auto value = ports_[target.port]->read({.address = target.word * 4, .size = 4});
  return target.word == 2 ? value | ports_[0]->dma_status() | ports_[1]->dma_status() : value;
}

void SharcSportPair::write(MemoryAccess access, std::uint64_t value) {
  const auto target = decode(access);
  ports_[target.port]->write({.address = target.word * 4, .size = 4}, value);
}

void SharcSportPair::begin_instruction() {
  if (active_) { throw std::logic_error("Nested SPORT pair instruction"); }
  ports_[0]->begin_instruction();
  try {
    ports_[1]->begin_instruction();
    active_ = true;
  } catch (...) {
    ports_[0]->abort_instruction();
    throw;
  }
}

void SharcSportPair::commit_instruction() noexcept {
  if (!active_) { return; }
  for (auto *port : ports_) { port->commit_instruction(); }
  active_ = false;
}

void SharcSportPair::abort_instruction() noexcept {
  if (!active_) { return; }
  for (auto *port : ports_) { port->abort_instruction(); }
  active_ = false;
}

}
