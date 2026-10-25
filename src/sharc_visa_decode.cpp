// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_visa.hpp"
#include "sharc_internal.hpp"
#include "sharc_visa_internal.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int class_shift = 40;
constexpr unsigned int short_shift = 32;
constexpr std::uint64_t compute_mask = 0x7fffff;
constexpr std::uint64_t always_compute = 0x013e00000000;

std::uint64_t short_compute(std::uint64_t raw) {
  const auto head = static_cast<std::uint32_t>(raw >> short_shift);
  constexpr unsigned int operation_shift = 8;
  constexpr unsigned int register_shift = 4;
  const auto operation = (head >> operation_shift) & 15;
  const auto destination = (head >> register_shift) & 15;
  const auto source = head & 15;
  // PGR12-12 short OP -> PGR12-3/4 native compute field; zero entries
  // need arithmetic engines not yet qualified, rather than substitute results.
  constexpr std::array<unsigned int, 16> opcodes{0x001000,
                                                 0x002000,
                                                 0x021000,
                                                 0x00a000,
                                                 0x043000,
                                                 0x029000,
                                                 0x02a000,
                                                 0,
                                                 0x081000,
                                                 0x082000,
                                                 0,
                                                 0,
                                                 0x040000,
                                                 0x041000,
                                                 0x042000,
                                                 0x130000};
  const auto opcode = opcodes[operation];
  if (opcode == 0) { throw std::runtime_error("SHARC VISA short compute operation not qualified"); }
  const bool unary = operation == 2 || operation == 4 || operation == 5 || operation == 6;
  constexpr unsigned int native_destination_shift = 8;
  const auto operands = (destination << native_destination_shift) |
                        ((unary ? source : destination) << register_shift) | (unary ? 0U : source);
  return always_compute | opcode | operands;
}

std::uint64_t short_word(std::uint64_t raw) {
  const auto head = static_cast<std::uint32_t>(raw >> short_shift);
  constexpr unsigned int nibble_shift = 12;
  constexpr unsigned int short_compute_prefix = 12;
  if ((head >> nibble_shift) == short_compute_prefix) { return short_compute(raw); }
  if (head == 1) { return 0; } // Type21c NOP.
  const auto type = raw >> class_shift;
  if (type == 10) {
    if ((head & 1U) != 0) { throw std::runtime_error("SHARC VISA RTS loop reentry not qualified"); }
    constexpr std::uint64_t condition = 0x3e00000000;
    constexpr std::uint64_t delayed = 0x4000000000;
    constexpr unsigned int delay_shift = 12;
    return (std::uint64_t{10} << class_shift) | (raw & condition) |
           ((raw & delayed) >> delay_shift);
  }
  constexpr std::uint64_t dm_compact = 0x900000000000;
  constexpr std::uint64_t nibble_mask = 0xf00000000000;
  if ((raw & nibble_mask) == dm_compact) {
    constexpr std::uint64_t dag_fields = 0x0fc000000000;
    constexpr unsigned int direction_shift = 37;
    constexpr unsigned int native_direction_shift = 31;
    constexpr unsigned int native_register_shift = 23;
    constexpr std::uint64_t unconditional_dm_post = 0x503e00000000;
    return unconditional_dm_post | (raw & dag_fields) |
           (((raw >> direction_shift) & 1) << native_direction_shift) |
           ((std::uint64_t{head & 15U}) << native_register_shift);
  }
  throw std::runtime_error("SHARC VISA short instruction semantics not qualified");
}

std::uint64_t signed16(std::uint64_t raw) {
  constexpr unsigned int data_shift = 16;
  constexpr std::uint32_t sign = 0x8000;
  constexpr std::uint32_t upper = 0xffff0000;
  const auto value = static_cast<std::uint32_t>((raw >> data_shift) & 0xffffU);
  return (value & sign) != 0 ? value | upper : value;
}

std::uint64_t double_word(std::uint64_t raw) {
  const auto type = raw >> class_shift;
  constexpr unsigned int group_shift = 45;
  const auto group = raw >> group_shift;
  if (group == 1 || group == 2 || group == 3) { return raw & ~compute_mask; }
  if (type == 1) {
    constexpr unsigned int packed_compute_shift = 16;
    return always_compute | ((raw >> packed_compute_shift) & compute_mask);
  }
  if (type == 15) {
    constexpr std::uint64_t immediate_prefix = 0x0f0000000000;
    constexpr std::uint64_t ureg = 0x7f00000000;
    return immediate_prefix | (raw & ureg) | signed16(raw);
  }
  throw std::runtime_error("SHARC VISA double-word instruction semantics not qualified");
}

}

bool sharc_visa_address(std::uint32_t address) noexcept {
  constexpr std::uint32_t first = 0x100000;
  constexpr std::uint32_t end = 0x200000;
  return address >= first && address < end;
}

std::uint64_t sharc_visa_decode(SharcVisaInstruction instruction) {
  if (instruction.words == 0 || instruction.words > 3 || !sharc_visa_address(instruction.address)) {
    throw std::invalid_argument("Invalid captured VISA instruction shape/address");
  }
  constexpr unsigned int half_bits = 16;
  constexpr unsigned int opcode_bits = 48;
  const auto unused = (3U - instruction.words) * half_bits;
  if ((instruction.opcode >> opcode_bits) != 0 ||
      (instruction.opcode & ((std::uint64_t{1} << unused) - 1)) != 0) {
    throw std::invalid_argument("Invalid VISA opcode alignment/unused bits");
  }
  const std::array<std::uint16_t, 3> prefix{
      static_cast<std::uint16_t>(instruction.opcode >> short_shift),
      static_cast<std::uint16_t>(instruction.opcode >> half_bits),
      static_cast<std::uint16_t>(instruction.opcode)};
  if (sharc_visa_words(std::span(prefix).first(instruction.words)) != instruction.words) {
    throw std::invalid_argument("Captured VISA instruction length disagrees with encoding");
  }
  if (instruction.words == 1) { return short_word(instruction.opcode); }
  if (instruction.words == 2) { return double_word(instruction.opcode); }
  return instruction.opcode;
}

void sharc_visa_execution_guard(const SharcState &state, SharcVisaInstruction instruction) {
  if (state.pc != instruction.address || state.arithmetic_loop || state.interrupt) {
    throw std::runtime_error("SHARC VISA PC/arithmetic/interrupt context not qualified");
  }
  if (state.count_loop &&
      (state.count_loop->body != state.count_loop->end || instruction.words != 3)) {
    throw std::runtime_error("SHARC VISA loop requires a single uncompressed body");
  }
  const auto type = instruction.opcode >> class_shift;
  if (type == 12 || type == 13) {
    const auto end = static_cast<std::uint32_t>(
        (std::uint64_t{state.pc} + (instruction.opcode & sharc_pc_mask)) & sharc_pc_mask);
    if (instruction.words != 3 || end != state.pc + 3) {
      throw std::runtime_error("SHARC VISA multi-body counter-loop sequencing not qualified");
    }
  }
  constexpr std::uint64_t idle = 0x008000000000;
  if (type == 11 || type == 14 || instruction.opcode == idle) {
    throw std::runtime_error("SHARC VISA RTI/arithmetic-loop/IDLE timing not qualified");
  }
}

}
