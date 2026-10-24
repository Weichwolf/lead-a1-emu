// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "sharc_internal.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int complement_base = 0x50;
constexpr unsigned int complement_end = 0x60;
constexpr std::uint64_t low_byte = 0xff;

unsigned int selection(const SharcState &state, unsigned int bit) {
  const auto mode = state.effective[sharc_mode1];
  const auto mask = std::uint32_t{1} << bit;
  if ((mode.known & mask) == 0) {
    throw std::runtime_error("Unknown SHARC register bank selection");
  }
  return (mode.value & mask) != 0 ? 1U : 0U;
}

unsigned int data_bank(const SharcState &state, unsigned int ureg) {
  const auto mode = state.effective[sharc_mode1];
  if ((mode.known & sharc_simd) == 0 || (mode.value & sharc_simd) != 0) {
    throw std::runtime_error("SHARC SIMD data register transfer not implemented");
  }
  constexpr unsigned int low_select = 10;
  const auto bank = selection(state, (ureg & 15) < 8 ? low_select : 7U);
  return bank + (ureg >= complement_base ? 2U : 0U);
}

unsigned int dag_bank(const SharcState &state, unsigned int reg) {
  constexpr std::array<unsigned int, 4> bits{4, 3, 6, 5};
  return selection(state, bits[reg / 4]);
}

}

bool sharc_data_ureg(unsigned int ureg) {
  return ureg < complement_end;
}

SharcKnown32 sharc_register_read(const SharcState &state, unsigned int ureg) {
  const auto reg = ureg & 15;
  if (ureg < 16 || ureg >= complement_base) {
    const auto word = state.data[data_bank(state, ureg)][reg];
    return {.value = static_cast<std::uint32_t>(word.value >> 8),
            .known = static_cast<std::uint32_t>(word.known >> 8)};
  }
  const auto kind = (ureg / 16) - 1;
  return state.dag[dag_bank(state, reg)][kind][reg];
}

void sharc_register_write(SharcState &state, unsigned int ureg, SharcKnown32 value) {
  const auto reg = ureg & 15;
  if (ureg < 16 || ureg >= complement_base) {
    state.data[data_bank(state, ureg)][reg] = {.value = std::uint64_t{value.value} << 8,
                                               .known =
                                                   (std::uint64_t{value.known} << 8) | low_byte};
    return;
  }
  const auto kind = (ureg / 16) - 1;
  auto &bank = state.dag[dag_bank(state, reg)];
  bank[kind][reg] = value;
  if (kind == 3) { bank[0][reg] = value; } // Bx loads Ix, independent of CBUFEN.
}

SharcKnown40 &sharc_data_register(SharcState &state, unsigned int reg) {
  return state.data[data_bank(state, reg)][reg & 15];
}

SharcKnown40 &sharc_compute_register(SharcState &state, unsigned int reg, unsigned int element) {
  constexpr unsigned int low_select = 10;
  const auto bank = selection(state, reg < 8 ? low_select : 7U) + (element * 2);
  return state.data[bank][reg];
}

void sharc_validate_registers(const SharcState &state) {
  if ((state.px.value & ~state.px.known) != 0) {
    throw std::invalid_argument("Invalid SHARC PX register knowledge");
  }
  for (const auto &bank : state.data) {
    for (const auto word : bank) {
      if ((word.value & ~word.known) != 0 || (word.known & ~sharc_data_mask) != 0) {
        throw std::invalid_argument("Invalid SHARC 40-bit register knowledge");
      }
    }
  }
  for (const auto &bank : state.dag) {
    for (const auto &kind : bank) {
      for (const auto word : kind) {
        if ((word.value & ~word.known) != 0) {
          throw std::invalid_argument("Invalid SHARC DAG register knowledge");
        }
      }
    }
  }
}

void sharc_register_move(SharcState &state, std::uint64_t opcode) {
  constexpr std::uint64_t swap = std::uint64_t{1} << 43;
  constexpr std::uint64_t reserved = std::uint64_t{1} << 30;
  constexpr std::uint64_t compute = 0x7fffff;
  if ((opcode & (swap | reserved | compute)) != 0) {
    throw std::runtime_error("SHARC swap/reserved/parallel-compute register move not implemented");
  }
  const auto mode = state.effective[sharc_mode1];
  if ((mode.known & sharc_simd) == 0 || (mode.value & sharc_simd) != 0) {
    throw std::runtime_error("SHARC SIMD register transfer not implemented");
  }
  const auto condition = static_cast<unsigned int>((opcode >> 33) & 31);
  if (!sharc_condition(state, condition)) { return; }
  constexpr unsigned int source_high_shift = 38;
  const auto source =
      static_cast<unsigned int>((((opcode >> source_high_shift) & 31) << 2) | ((opcode >> 31) & 3));
  const auto destination = static_cast<unsigned int>((opcode >> 23) & 127);
  if (sharc_px_register_move(state, source, destination)) { return; }
  const auto data_file = [](unsigned int reg) {
    return reg < 16 || (reg >= complement_base && reg < complement_end);
  };
  if (data_file(source) && data_file(destination)) {
    throw std::runtime_error("SHARC data-to-data register transfer width not qualified");
  }
  // Capture before the destination write, including Bx's implicit Ix update.
  const auto value = sharc_ureg(state, source);
  sharc_ureg(state, destination, value);
}

}
