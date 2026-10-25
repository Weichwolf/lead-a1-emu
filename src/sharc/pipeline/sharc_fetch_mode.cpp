// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/pipeline/sharc_fetch_mode.hpp"

#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/pipeline/sharc_fetch_internal.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {
bool unbanked(unsigned int reg) {
  constexpr unsigned int first = 0x70;
  constexpr unsigned int end = 0x80;
  return reg >= first && reg < end;
}

bool unbanked_word(const std::optional<SharcFetchWord> &word) {
  if (!word || word->opcode == 0) { return true; }
  if (word->dag || word->pm_data == true) { return false; }
  const auto opcode = word->opcode;
  constexpr unsigned int class_shift = 40;
  constexpr unsigned int reg_shift = 32;
  constexpr unsigned int reg_mask = 127;
  constexpr unsigned int immediate = 15;
  constexpr unsigned int system_bit = 20;
  const auto type = opcode >> class_shift;
  if (type == system_bit) { return true; }
  if (type == immediate) {
    return unbanked(static_cast<unsigned int>((opcode >> reg_shift) & reg_mask));
  }
  constexpr std::uint64_t register_mask = 0xf00000000000;
  constexpr std::uint64_t register_move = 0x700000000000;
  constexpr std::uint64_t compute_mask = 0x7fffff;
  constexpr unsigned int source_high_shift = 38;
  constexpr unsigned int source_low_shift = 31;
  constexpr unsigned int destination_shift = 23;
  constexpr unsigned int source_mask = 31;
  if ((opcode & register_mask) != register_move || (opcode & compute_mask) != 0) { return false; }
  const auto source = static_cast<unsigned int>(
      (((opcode >> source_high_shift) & source_mask) << 2) | ((opcode >> source_low_shift) & 3));
  const auto destination = static_cast<unsigned int>((opcode >> destination_shift) & reg_mask);
  return unbanked(source) && unbanked(destination);
}

void mode_context_guard(const SharcState &cpu, const SharcFetchState &transport) {
  if (cpu.interrupt || cpu.branch || cpu.idle_pc || cpu.count_loop || !transport.flow ||
      transport.flow->loop || transport.flow->long_loop || transport.flow->discard_decode != 0 ||
      transport.pending_fetch2) {
    throw std::runtime_error("SHARC MODE1 interrupt-control stall context not qualified");
  }
  if (!cpu.arithmetic_loop) { return; }
  const auto &loop = *cpu.arithmetic_loop;
  if (loop.phase != SharcArithmeticPhase::deferred || (cpu.pc >= loop.body && cpu.pc <= loop.end) ||
      transport.fetch1 == loop.body) {
    throw std::runtime_error("SHARC MODE1 stall at active arithmetic-loop boundary not qualified");
  }
  sharc_fetch_arithmetic_guard(cpu, transport);
}
}

bool sharc_fetch_interrupt_mode(const SharcState &cpu) {
  const auto pending = cpu.pending[sharc_mode1];
  if (!pending || cpu.interrupt) { return false; }
  const auto previous = cpu.effective[sharc_mode1];
  constexpr std::uint32_t interrupt_controls = 0x1800;
  const auto changed = (pending->value ^ previous.value) | (pending->known ^ previous.known);
  return (changed & ~interrupt_controls) == 0 && (previous.known & sharc_simd) != 0 &&
         (previous.value & sharc_simd) == 0 &&
         (pending->known & interrupt_controls) == interrupt_controls;
}

void sharc_fetch_interrupt_mode_guard(const SharcState &cpu, const SharcFetchState &transport) {
  if (!sharc_fetch_interrupt_mode(cpu)) {
    throw std::runtime_error("SHARC MODE1 interrupt-control stall context not qualified");
  }
  mode_context_guard(cpu, transport);
}

void sharc_fetch_unbanked_mode_guard(const SharcState &cpu, const SharcFetchState &transport) {
  const auto pending = cpu.pending[sharc_mode1];
  const auto previous = cpu.effective[sharc_mode1];
  constexpr std::uint32_t fields = 0x0101fcff;
  if (!pending || !transport.flow ||
      (((pending->value ^ previous.value) | (pending->known ^ previous.known)) & ~fields) != 0 ||
      (previous.known & sharc_simd) == 0 || (previous.value & sharc_simd) != 0 ||
      !unbanked_word(transport.decode) || !unbanked_word(transport.address)) {
    throw std::runtime_error("SHARC quiet MODE1 pipeline effect interlock not qualified");
  }
  mode_context_guard(cpu, transport);
}
}
