// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_iop_bridge.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1 {

SharcIopPort::SharcIopPort(SharcIopBridge &bridge, SharcIoMapping window)
    : bridge_(bridge), window_(window) {
  bridge_.idle();
  const auto span = bridge_.window_;
  if (window.words == 0 || window.first_word < span.first_word ||
      window.first_word - span.first_word >= span.words ||
      window.words > span.words - (window.first_word - span.first_word)) {
    throw std::invalid_argument("SHARC IOP port window outside its bridge");
  }
}

MemoryAccess SharcIopPort::translate(MemoryAccess access) const {
  constexpr unsigned int word_bytes = 4;
  if (access.size != word_bytes || access.address % word_bytes != 0 ||
      access.address / word_bytes >= window_.words) {
    throw std::out_of_range("SHARC IOP port requires aligned32-bit access within its window");
  }
  access.address += std::uint64_t{window_.first_word - bridge_.window_.first_word} * word_bytes;
  return access;
}

std::uint64_t SharcIopPort::read(MemoryAccess access) {
  return bridge_.read(translate(access));
}

void SharcIopPort::write(MemoryAccess access, std::uint64_t value) {
  bridge_.write(translate(access), value);
}

void SharcIopPort::begin_instruction() {
  bridge_.begin_instruction();
}

void SharcIopPort::commit_instruction() noexcept {
  bridge_.commit_instruction();
}

void SharcIopPort::abort_instruction() noexcept {
  bridge_.abort_instruction();
}

}
