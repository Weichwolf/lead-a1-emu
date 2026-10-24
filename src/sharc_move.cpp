// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <sstream>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint64_t write_bit = std::uint64_t{1} << 39;
constexpr std::uint64_t update_bit = std::uint64_t{1} << 38;
constexpr std::uint64_t program_bit = std::uint64_t{1} << 40;
constexpr std::uint64_t low_byte = 0xff;

std::uint32_t modified(SharcAccess access, std::uint64_t opcode) {
  constexpr unsigned int modifier_shift = 27;
  constexpr std::uint32_t sign = 32;
  constexpr std::uint32_t mask = 63;
  const auto immediate = static_cast<std::uint32_t>((opcode >> modifier_shift) & mask);
  const auto displacement =
      immediate < sign ? std::uint64_t{immediate} : (std::uint64_t{1} << 32) - (64 - immediate);
  return static_cast<std::uint32_t>(std::uint64_t{access.address} + displacement);
}

}

void sharc_addressing_guard(const SharcState &state, unsigned int index) {
  const auto mode = state.effective[sharc_mode1];
  unsigned int reversed{};
  if (index == 0) { reversed = 2; }
  if (index == 8) { reversed = 1; }
  constexpr std::uint32_t circular = 0x1000000;
  constexpr std::uint32_t broadcast = 0xc00000;
  const auto unsupported = reversed | circular | broadcast | sharc_simd;
  if ((mode.known & unsupported) != unsupported || (mode.value & unsupported) != 0) {
    throw std::runtime_error(
        "SHARC circular/reversed/broadcast/SIMD data addressing not implemented");
  }
}

bool sharc_data_move_condition(const SharcState &state, std::uint64_t opcode) {
  const auto mode = state.effective[sharc_mode1];
  if ((mode.known & sharc_simd) == 0 || (mode.value & sharc_simd) != 0) {
    throw std::runtime_error("SHARC SIMD data addressing not implemented");
  }
  constexpr std::uint64_t compute_mask = 0x7fffff;
  if ((opcode & compute_mask) != 0) {
    throw std::runtime_error("SHARC parallel compute/data move not implemented");
  }
  const auto condition = static_cast<unsigned int>((opcode >> 33) & 31);
  return sharc_condition(state, condition);
}

void sharc_ram_transfer(SharcMemory &memory,
                        SharcState &state,
                        SharcDataTransfer transfer,
                        const SharcControl *control) try {
  constexpr unsigned int complement_begin = 0x50;
  constexpr unsigned int complement_end = 0x60;
  const auto reg = transfer.ureg;
  constexpr unsigned int combined_px = 0x6b;
  if (reg == combined_px) {
    sharc_px_ram_transfer(memory, state, transfer);
    return;
  }
  if (reg >= 16 && (reg < complement_begin || reg >= complement_end)) {
    throw std::runtime_error("SHARC RAM non-data UREG transfer not implemented");
  }
  auto &data = sharc_data_register(state, reg);
  constexpr std::uint32_t short_begin = 0x100000;
  constexpr std::uint32_t short_end = 0x200000;
  if (transfer.access.address >= short_begin && transfer.access.address < short_end) {
    sharc_short_transfer(
        memory, data, state.effective[sharc_mode1], transfer.access, transfer.write);
    return;
  }
  if (control == nullptr) {
    throw std::runtime_error("SHARC RAM data-width access requires a bound SYSCTL reference");
  }
  const auto width = control->data_width(transfer.access.address);
  if (transfer.write) {
    const auto required =
        width == SharcWidth::normal32 ? sharc_data_mask & ~low_byte : sharc_data_mask;
    if ((data.known & required) != required) {
      throw std::runtime_error("Unknown SHARC data register store bits");
    }
    const auto stored_width = width == SharcWidth::normal32 ? width : SharcWidth::wide48;
    memory.write({.address = transfer.access.address, .width = stored_width},
                 width == SharcWidth::normal32 ? data.value >> 8 : data.value << 8);
  } else {
    const auto value = memory.read({.address = transfer.access.address, .width = width});
    data = {.value = width == SharcWidth::normal32 ? value << 8 : value, .known = sharc_data_mask};
  }
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC RAM word-address=0x" << std::hex << transfer.access.address << ": "
          << error.what();
  throw std::runtime_error(message.str());
}

void sharc_data_move(SharcMemory &memory,
                     SharcState &state,
                     std::uint64_t opcode,
                     const SharcControl *control) try {
  // SIMD data moves evaluate conditions per element, unlike branch AND rules.
  // Reject that mode before a false PEx condition could hide a PEy access.
  if (!sharc_data_move_condition(state, opcode)) { return; }
  if ((opcode & program_bit) != 0) { sharc_cache_program_access(state); }
  const auto index =
      static_cast<unsigned int>((opcode >> 41) & 7) + ((opcode & program_bit) != 0 ? 8U : 0U);
  sharc_addressing_guard(state, index);
  const auto current = sharc_value(sharc_register_read(state, 16 + index));
  const auto next = modified({.address = current}, opcode);
  const auto address = (opcode & update_bit) != 0 ? current : next;
  const auto reg = static_cast<unsigned int>((opcode >> 23) & 15);
  sharc_ram_transfer(
      memory,
      state,
      {.access = {.address = address}, .ureg = reg, .write = (opcode & write_bit) != 0},
      control);
  if ((opcode & update_bit) != 0) {
    constexpr std::uint32_t all_bits = 0xffffffff;
    sharc_register_write(state, 16 + index, {.value = next, .known = all_bits});
  }
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC indexed " << ((opcode & write_bit) != 0 ? "write" : "read") << ": "
          << error.what();
  throw std::runtime_error(message.str());
}

}
