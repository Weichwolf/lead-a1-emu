// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/memory/sharc_memory.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int combined_px = 0x6b;
constexpr unsigned int px1 = 0x6c;
constexpr unsigned int px2 = 0x6d;
constexpr std::uint64_t word_mask = 0xffffffff;

void require_sisd(const SharcState &state) {
  const auto mode = state.effective[sharc_mode1];
  if ((mode.known & sharc_simd) == 0 || (mode.value & sharc_simd) != 0) {
    throw std::runtime_error("SHARC SIMD PX transfer not implemented");
  }
}

bool data_register(unsigned int reg) {
  constexpr unsigned int complement_begin = 0x50;
  constexpr unsigned int complement_end = 0x60;
  return reg < 16 || (reg >= complement_begin && reg < complement_end);
}

unsigned int half_shift(unsigned int reg) {
  if (!sharc_px_half(reg)) { throw std::logic_error("Invalid SHARC PX half register"); }
  constexpr unsigned int upper = 32;
  return reg == px1 ? 0U : upper;
}

}

bool sharc_px_half(unsigned int reg) {
  return reg == px1 || reg == px2;
}

SharcKnown32 sharc_px_half_read(const SharcState &state, unsigned int reg) {
  require_sisd(state);
  const auto shift = half_shift(reg);
  return {.value = static_cast<std::uint32_t>(state.px.value >> shift),
          .known = static_cast<std::uint32_t>(state.px.known >> shift)};
}

void sharc_px_half_write(SharcState &state, unsigned int reg, SharcKnown32 value) {
  require_sisd(state);
  const auto shift = half_shift(reg);
  const auto mask = word_mask << shift;
  state.px.value = (state.px.value & ~mask) | (std::uint64_t{value.value} << shift);
  state.px.known = (state.px.known & ~mask) | (std::uint64_t{value.known} << shift);
}

bool sharc_px_register_move(SharcState &state, unsigned int source, unsigned int destination) {
  if (source != combined_px && destination != combined_px) { return false; }
  require_sisd(state);
  constexpr unsigned int alignment = 24;
  constexpr std::uint64_t low_zero = 0xffffff;
  if (source == combined_px && data_register(destination)) {
    sharc_data_register(state, destination) = {.value = state.px.value >> alignment,
                                               .known = state.px.known >> alignment};
  } else if (destination == combined_px && data_register(source)) {
    const auto value = sharc_data_register(state, source);
    state.px = {.value = value.value << alignment, .known = (value.known << alignment) | low_zero};
  } else {
    throw std::runtime_error("SHARC combined PX/non-data register width not qualified");
  }
  return true;
}

void sharc_px_ram_transfer(SharcMemory &memory, SharcState &state, SharcDataTransfer transfer) {
  require_sisd(state);
  constexpr std::uint32_t normal_begin = 0x80000;
  constexpr std::uint32_t normal_end = 0x100000;
  if (transfer.access.address < normal_begin || transfer.access.address >= normal_end) {
    throw std::runtime_error("SHARC PX transfer outside normal-word RAM not qualified");
  }
  constexpr unsigned int alignment = 16;
  constexpr std::uint64_t low_zero = 0xffff;
  constexpr std::uint64_t required = ~low_zero;
  const SharcAccess access{.address = transfer.access.address, .width = SharcWidth::wide48};
  if (transfer.write) {
    if ((state.px.known & required) != required) {
      throw std::runtime_error("Unknown SHARC PX memory store bits");
    }
    memory.write(access, state.px.value >> alignment);
  } else {
    state.px = {.value = memory.read(access) << alignment, .known = ~std::uint64_t{0}};
  }
}

}
