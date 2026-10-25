// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/peripherals/sharc_control.hpp"

#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t all_bits = std::numeric_limits<std::uint32_t>::max();
constexpr std::uint32_t internal_vectors = 4;
constexpr std::uint32_t data_widths = 0x1e00;
constexpr std::uint32_t dma_rotation_bit = 0x80;
constexpr std::uint32_t supported = internal_vectors | data_widths | dma_rotation_bit;

void validate_dma(const SharcControlState &state) {
  auto latest = state.dma_rotation;
  for (const auto command : state.dma_commands) {
    if (command) { latest = command; }
  }
  if (state.delayed) { latest = (*state.delayed & dma_rotation_bit) != 0; }
  const bool known = (state.known & dma_rotation_bit) != 0;
  if (latest.has_value() != known ||
      (latest && *latest != ((state.control & dma_rotation_bit) != 0))) {
    throw std::invalid_argument("Invalid SHARC SYSCTL DMA mode snapshot");
  }
}

void arm_dma(SharcControlState &state, std::uint32_t value) {
  const bool changed = (state.effective_known & dma_rotation_bit) == 0 ||
                       ((state.effective ^ value) & dma_rotation_bit) != 0;
  if (!changed) { return; }
  const bool rotating = (value & dma_rotation_bit) != 0;
  auto &command = state.dma_commands.back();
  if (command && *command != rotating) {
    throw std::runtime_error("SYSCTL DCPR effects in the same half-PCLK interval not qualified");
  }
  command = rotating;
}

void validate_state(const SharcControlState &state) {
  validate_dma(state);
  for (const auto fields :
       {SharcKnown32{.value = state.control, .known = state.known},
        SharcKnown32{.value = state.effective, .known = state.effective_known}}) {
    if ((fields.value & ~supported) != 0 || (fields.value & ~fields.known) != 0 ||
        (fields.known & ~supported) != ~supported) {
      throw std::invalid_argument("Invalid SHARC SYSCTL snapshot value/knowledge");
    }
  }
  if (state.delayed) {
    if (state.known != all_bits || state.delayed != state.control) {
      throw std::invalid_argument("Invalid SHARC SYSCTL snapshot pending effect");
    }
  } else if (state.control != state.effective || state.known != state.effective_known) {
    throw std::invalid_argument("Invalid SHARC SYSCTL snapshot without pending effect");
  }
}

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

SharcControl::SharcControl(SharcControlState initial, Restore restore) : current_(initial) {
  if (restore != Restore::snapshot) {
    throw std::invalid_argument("Invalid SHARC SYSCTL restore mode");
  }
  validate_state(current_);
}

SharcControl SharcControl::from_snapshot(SharcControlState initial) {
  return {initial, Restore::snapshot};
}

SharcControlState SharcControl::snapshot() const {
  if (staged_ || core_active_) {
    throw std::logic_error("SHARC SYSCTL snapshot during a transaction");
  }
  return current_;
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
    throw std::runtime_error("Unknown SHARC SYSCTL read-view bits");
  }
  return working().control;
}

void SharcControl::write(MemoryAccess access, std::uint64_t value) {
  validate(access);
  if ((value & ~std::uint64_t{supported}) != 0) {
    throw std::runtime_error("SHARC SYSCTL reset/arbitration/mux/reserved write not implemented");
  }
  auto next = working();
  next.control = static_cast<std::uint32_t>(value);
  next.known = all_bits;
  next.delayed = next.control;
  working() = next;
}

std::optional<bool> SharcControl::dma_rotation() const {
  if (staged_ || core_active_) {
    throw std::logic_error("SYSCTL DMA arbitration during an active transaction");
  }
  return current_.dma_rotation;
}

void SharcControl::advance_half_pclk(unsigned int half_periods) {
  if (staged_ || core_active_) {
    throw std::logic_error("SYSCTL DMA clock advance during an active transaction");
  }
  const auto steps =
      half_periods < current_.dma_commands.size() ? half_periods : current_.dma_commands.size();
  for (unsigned int step = 0; step < steps; ++step) {
    if (current_.dma_commands[0]) { current_.dma_rotation = current_.dma_commands[0]; }
    for (unsigned int index = 1; index < current_.dma_commands.size(); ++index) {
      current_.dma_commands[index - 1] = current_.dma_commands[index];
    }
    current_.dma_commands.back().reset();
  }
}

void SharcControl::begin_core_instruction() {
  if (staged_) { throw std::logic_error("SHARC SYSCTL already owned by a transaction/core"); }
  auto next = current_;
  if (next.delayed) { arm_dma(next, *next.delayed); }
  staged_ = next;
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
  // PGR A-32/A-45: one extra internal processor clock, not a retired instruction.
  auto next = current_;
  arm_dma(next, *delayed);
  next.effective = *delayed;
  next.effective_known = all_bits;
  next.delayed.reset();
  current_ = next;
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
