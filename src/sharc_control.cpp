// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_control.hpp"

#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t all_bits = std::numeric_limits<std::uint32_t>::max();
constexpr std::uint32_t internal_vectors = 4;
constexpr std::uint32_t data_widths = 0x1e00;
constexpr std::uint32_t supported = internal_vectors | data_widths;

void validate(MemoryAccess access) {
  if (access.address != 0 || access.size != 4) {
    throw std::runtime_error("SHARC SYSCTL requires offset0/read-write32");
  }
}

}

SharcControl::SharcControl(std::optional<bool> valid_boot_mode) {
  current_.known = all_bits & ~internal_vectors;
  if (valid_boot_mode) {
    current_.known |= internal_vectors;
    if (*valid_boot_mode) { current_.control = internal_vectors; }
  }
  current_.effective = current_.control;
  current_.effective_known = current_.known;
}

SharcControlState &SharcControl::working() {
  return staged_ ? *staged_ : current_;
}

const SharcControlState &SharcControl::working() const {
  return staged_ ? *staged_ : current_;
}

SharcControlState SharcControl::state() const {
  return working();
}

SharcWidth SharcControl::data_width(std::uint32_t address) const {
  constexpr std::uint32_t normal_base = 0x80000;
  constexpr std::uint32_t short_base = 0x100000;
  constexpr std::uint32_t bank_span = 0x20000;
  if (address < normal_base || address >= short_base) {
    throw std::runtime_error("SHARC core data access outside normal-word RAM not implemented");
  }
  const auto bit = std::uint32_t{1} << (9 + ((address - normal_base) / bank_span));
  if ((working().effective_known & bit) == 0) {
    throw std::runtime_error("Unknown SHARC SYSCTL data-width control");
  }
  return (working().effective & bit) != 0 ? SharcWidth::extended40 : SharcWidth::normal32;
}

std::uint64_t SharcControl::read(MemoryAccess access) {
  validate(access);
  if (working().known != all_bits) {
    throw std::runtime_error("Unknown SHARC SYSCTL boot-mode input");
  }
  return working().control;
}

void SharcControl::write(MemoryAccess access, std::uint64_t value) {
  validate(access);
  if ((value & ~std::uint64_t{supported}) != 0) {
    throw std::runtime_error("SHARC SYSCTL reset/arbitration/mux/reserved write not implemented");
  }
  auto &next = working();
  next.control = static_cast<std::uint32_t>(value);
  next.known = all_bits;
  next.delayed = next.control;
}

void SharcControl::begin_core_instruction() {
  if (staged_) { throw std::logic_error("SHARC SYSCTL already owned by a transaction/core"); }
  staged_ = current_;
  due_ = staged_->delayed;
  staged_->delayed.reset();
  core_active_ = true;
}

void SharcControl::commit_core_instruction() noexcept {
  if (staged_) {
    if (due_) {
      staged_->effective = *due_;
      staged_->effective_known = all_bits;
    }
    current_ = *staged_;
  }
  abort_core_instruction();
}

void SharcControl::abort_core_instruction() noexcept {
  staged_.reset();
  due_.reset();
  core_active_ = false;
  joined_ = false;
}

void SharcControl::advance_internal_clocks(unsigned int core_cycles) {
  if (staged_ || core_active_) {
    throw std::logic_error("SHARC SYSCTL clock advance during a transaction");
  }
  const auto delayed = current_.delayed;
  if (core_cycles == 0 || !delayed) { return; }
  if (core_cycles < 2) {
    throw std::runtime_error("SHARC SYSCTL partial clock effect interval not qualified");
  }
  current_.effective = *delayed;
  current_.effective_known = all_bits;
  current_.delayed.reset();
}

void SharcControl::begin_instruction() {
  if (core_active_ && !joined_) {
    joined_ = true;
    return;
  }
  if (staged_) { throw std::logic_error("Nested SHARC SYSCTL transaction"); }
  staged_ = current_;
}

void SharcControl::commit_instruction() noexcept {
  if (core_active_) {
    joined_ = false;
    return;
  }
  if (staged_) { current_ = *staged_; }
  staged_.reset();
}

void SharcControl::abort_instruction() noexcept {
  if (core_active_) {
    joined_ = false;
    return;
  }
  staged_.reset();
}

}
