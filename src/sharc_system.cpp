// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "sharc_internal.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {

void Sharc::latch_interrupt(std::uint32_t mask) {
  constexpr std::uint32_t maskable = 0xffffff7c;
  constexpr unsigned int irptl = 10;
  if (memory_.active() || (mask & ~maskable) != 0) {
    throw std::runtime_error(
        "SHARC interrupt event requires idle transaction and maskable IRPTL bits");
  }
  for (auto *latch : {&state_.system[irptl], &state_.effective[irptl]}) {
    latch->value |= mask;
    latch->known |= mask;
  }
  auto &pending = state_.pending[irptl];
  if (pending) {
    pending->value |= mask;
    pending->known |= mask;
  }
}

}

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t all_bits = std::numeric_limits<std::uint32_t>::max();
constexpr std::uint32_t interrupt_writable = 0xffffff7c;
constexpr std::uint32_t mode1_writable = 0x01e1fcff;
constexpr std::uint32_t global_interrupt_enable = 0x1000;
constexpr unsigned int sreg_base = 0x70;
constexpr unsigned int pc_ureg = 0x63;
constexpr unsigned int stack_ureg = 0x64;
constexpr unsigned int depth_ureg = 0x65;

SharcKnown32 merge(SharcKnown32 previous, SharcKnown32 value, std::uint32_t mask) {
  return {.value = (previous.value & ~mask) | (value.value & mask),
          .known = (previous.known & ~mask) | (value.known & mask)};
}

bool known_bit(SharcKnown32 value, std::uint32_t mask) {
  if ((value.known & mask) != mask) {
    throw std::runtime_error("Unknown SHARC control/status bits");
  }
  return (value.value & mask) != 0;
}

bool user_status(unsigned int index) {
  return index < 2 || index >= 14;
}

void require_sisd(const SharcState &state) {
  if (known_bit(state.effective[sharc_mode1], sharc_simd)) {
    throw std::runtime_error("SHARC SIMD register move/bit operation not implemented");
  }
}

void system_write(SharcState &state, unsigned int index, SharcKnown32 value) {
  if (index == sharc_flags) {
    sharc_flags_write(state, value, all_bits);
    return;
  }
  if (user_status(index)) {
    require_sisd(state);
    state.system[index] = value;
    state.effective[index] = value;
    return;
  }
  std::uint32_t mask{};
  if (index == sharc_mode1) {
    constexpr std::uint32_t family_bus_select = 0x60000;
    if ((value.value & family_bus_select) != 0) {
      throw std::runtime_error("SHARC family-specific MODE1 bus-select bits not qualified");
    }
    mask = mode1_writable;
  } else if (index == sharc_irptl || index == sharc_imask) {
    mask = interrupt_writable;
  } else {
    throw std::runtime_error("SHARC system register write semantics not implemented");
  }
  state.system[index] = merge(state.system[index], value, mask);
  state.pending[index] = state.system[index];
}

std::optional<bool> compare(SharcKnown32 value, std::uint32_t mask, bool equality) {
  if (equality) {
    if (((value.value ^ mask) & value.known) != 0) { return false; }
    if (value.known == all_bits) { return true; }
  } else {
    if ((~value.value & value.known & mask) != 0) { return false; }
    if ((value.known & mask) == mask) { return true; }
  }
  return {};
}

void test_flag(SharcState &state, std::optional<bool> result) {
  auto &flag = state.system[sharc_astat_x];
  flag.value &= ~sharc_btf;
  flag.known &= ~sharc_btf;
  if (result) {
    flag.known |= sharc_btf;
    if (*result) { flag.value |= sharc_btf; }
  }
  // The bit-test result is available to the immediately following TF branch.
  state.effective[sharc_astat_x] = merge(state.effective[sharc_astat_x], flag, sharc_btf);
}

bool element_condition(SharcKnown32 flags, unsigned int code) {
  constexpr std::array<std::uint32_t, 15> masks{
      1, 0, 0, 8, 2, 0x80, 0x40, 0x800, 0x1000, 0, 0, 0, 0, sharc_btf, 0x4000};
  const auto base = code & 15;
  if (base >= masks.size() || masks[base] == 0) {
    throw std::runtime_error("SHARC relational/pad/loop condition not implemented");
  }
  return known_bit(flags, masks[base]) != (code >= 16);
}

}

std::uint32_t sharc_value(SharcKnown32 word) {
  if (word.known != all_bits) { throw std::runtime_error("Unknown SHARC register operand bits"); }
  return word.value;
}

SharcKnown32 sharc_ureg(const SharcState &state, unsigned int ureg) {
  if (sharc_data_ureg(ureg)) { return sharc_register_read(state, ureg); }
  if (ureg >= sreg_base && ureg < sreg_base + state.system.size()) {
    const auto index = ureg - sreg_base;
    if (user_status(index) || (index >= sharc_astat_x && index <= 9)) { require_sisd(state); }
    if (index == sharc_flags) { return sharc_flags_read(state); }
    if (index == 12 || index == 13) {
      throw std::runtime_error("SHARC mask-pointer/link-register read not implemented");
    }
    return state.system[index];
  }
  if (ureg == pc_ureg) { return {.value = state.pc, .known = all_bits}; }
  if (ureg == depth_ureg) {
    if (!state.depth) { throw std::runtime_error("Unknown SHARC PC stack depth"); }
    return {.value = *state.depth, .known = all_bits};
  }
  if (ureg == stack_ureg) {
    if (!state.depth) { throw std::runtime_error("Unknown SHARC PC stack depth"); }
    if (*state.depth == 0) { return {.value = sharc_frame_mask, .known = all_bits}; }
    const auto &frame = state.pc_stack[*state.depth - 1];
    if (!frame) { return {.value = 0, .known = 0}; }
    return {.value = *frame, .known = all_bits};
  }
  throw std::runtime_error("SHARC data/DAG/PX or sequencer register not implemented");
}

void sharc_ureg(SharcState &state, unsigned int ureg, SharcKnown32 value) {
  if (sharc_data_ureg(ureg)) {
    sharc_register_write(state, ureg, value);
    return;
  }
  if (ureg >= sreg_base && ureg < sreg_base + state.system.size()) {
    system_write(state, ureg - sreg_base, value);
    return;
  }
  if (ureg == stack_ureg) {
    if (state.count_loop) {
      throw std::runtime_error("SHARC PCSTK modification inside a count loop not qualified");
    }
    if (!state.depth) { throw std::runtime_error("Unknown SHARC PC stack depth"); }
    if (*state.depth != 0) {
      state.pc_stack[*state.depth - 1] = sharc_value(value) & sharc_frame_mask;
    }
    return;
  }
  throw std::runtime_error("SHARC data/DAG/PX or sequencer register write not implemented");
}

void sharc_bit_operation(SharcState &state, std::uint64_t opcode) {
  const auto index = static_cast<unsigned int>((opcode >> 32) & 15);
  const auto operation = static_cast<unsigned int>((opcode >> 37) & 7);
  const auto mask = static_cast<std::uint32_t>(opcode);
  constexpr std::uint32_t flag_levels = 0x55555555;
  if (index == sharc_flags && operation == 2 && (mask & flag_levels) != 0) {
    throw std::runtime_error("SHARC FLAGS level toggle timing not qualified");
  }
  auto value = sharc_ureg(state, sreg_base + index);
  switch (operation) {
    case 0:
      value.value |= mask;
      value.known |= mask;
      break;
    case 1:
      value.value &= ~mask;
      value.known |= mask;
      break;
    case 2: value.value ^= mask & value.known; break;
    case 4:
    case 5:
      require_sisd(state);
      test_flag(state, compare(value, mask, operation == 5));
      return;
    default: throw std::runtime_error("Reserved SHARC system bit operation");
  }
  if (index == sharc_flags) {
    sharc_flags_write(state, value, mask);
  } else {
    sharc_ureg(state, sreg_base + index, value);
  }
}

bool sharc_condition(const SharcState &state, unsigned int code) {
  constexpr unsigned int always = 31;
  if (code == always) { return true; }
  const bool first = element_condition(state.effective[sharc_astat_x], code);
  if (!first) { return false; }
  if (!known_bit(state.effective[sharc_mode1], sharc_simd)) { return true; }
  return element_condition(state.effective[sharc_astat_y], code);
}

bool sharc_pe_condition(const SharcState &state, unsigned int code, unsigned int element) {
  constexpr unsigned int always = 31;
  return code == always || element_condition(state.effective[sharc_astat_x + element], code);
}

void sharc_interrupt_guard(const SharcState &state) {
  constexpr std::uint32_t timer_enable = 0x20;
  const auto mode2 = state.effective[4];
  if ((mode2.known & timer_enable) == 0 || (mode2.value & timer_enable) != 0) {
    throw std::runtime_error("SHARC active/unknown core timer not implemented");
  }
  const auto latch = state.effective[sharc_irptl];
  if ((latch.value & 3) != 0 || (latch.known & 3) != 3) {
    throw std::runtime_error("SHARC nonmaskable interrupt state/delivery unresolved");
  }
  if (latch.known == all_bits && latch.value == 0) { return; }
  if (!known_bit(state.effective[sharc_mode1], global_interrupt_enable)) { return; }
  const auto mask = state.effective[sharc_imask];
  const auto possible =
      (latch.value | ~latch.known) & (mask.value | ~mask.known) & interrupt_writable;
  if (possible != 0) {
    throw std::runtime_error("SHARC pending interrupt delivery not implemented");
  }
}

void sharc_stack_status(SharcState &state) {
  constexpr std::uint32_t full = 0x200000;
  constexpr std::uint32_t empty = 0x400000;
  SharcKnown32 flags;
  if (state.depth) {
    flags.known = full | empty;
    if (*state.depth == 0) { flags.value = empty; }
    if (*state.depth == state.pc_stack.size()) { flags.value = full; }
  }
  state.system[sharc_stky_x] = merge(state.system[sharc_stky_x], flags, full | empty);
  state.effective[sharc_stky_x] = merge(state.effective[sharc_stky_x], flags, full | empty);
  constexpr std::uint32_t loop_empty = 0x4000000;
  SharcKnown32 loop_flags;
  if (state.loops_empty) {
    loop_flags.known = loop_empty;
    if (*state.loops_empty) { loop_flags.value = loop_empty; }
  }
  state.system[sharc_stky_x] = merge(state.system[sharc_stky_x], loop_flags, loop_empty);
  state.effective[sharc_stky_x] = merge(state.effective[sharc_stky_x], loop_flags, loop_empty);
}

}
