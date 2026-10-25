// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t full = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t simd = 0x200000;
constexpr std::uint32_t shift_mask = 0x3800;
constexpr std::uint32_t zero = 0x1000;
constexpr std::uint32_t overflow = 0x800;
constexpr std::uint32_t preserved = 0xa50447ff;
constexpr std::uint64_t register_logical = 0x013e00200312; // R3=LSHIFT R1 BY R2.
constexpr std::uint64_t register_arithmetic = 0x013e00204312;
constexpr std::uint64_t immediate_logical = 0x023e00000031; // R3=LSHIFT R1 BY 0.
constexpr std::uint64_t immediate_arithmetic = 0x023e00010031;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC shift expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC shift rejection: " + reason);
}

void map(lead_a1::SharcMemory &memory, std::uint64_t opcode) {
  constexpr unsigned int cells = 96;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned shifter program"});
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, opcode);
}

lead_a1::SharcKnown40 word(std::uint32_t value) {
  constexpr std::uint64_t extension = 0xa7;
  return {.value = (std::uint64_t{value} << 8) | extension, .known = full40};
}

lead_a1::SharcState initial_state() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.system[6] = {.value = preserved | shift_mask, .known = full};
  state.effective = state.system;
  return state;
}

struct ShiftCase {
  std::uint64_t opcode;
  std::uint32_t input;
  std::uint32_t result;
  std::uint32_t flags;
  std::uint32_t count{};
  bool immediate{true};
};

constexpr std::array<ShiftCase, 24> cases{{
    {.opcode = 0x023e00000031, .input = 0x12345678, .result = 0x12345678, .flags = 0},
    {.opcode = 0x023e00000131, .input = 0x80000001, .result = 2, .flags = 0x800},
    {.opcode = 0x023e00001f31, .input = 1, .result = 0x80000000, .flags = 0x800},
    {.opcode = 0x023e00002031, .input = 0xffffffff, .result = 0, .flags = 0x1800},
    {.opcode = 0x023e00007f31, .input = 0xffffffff, .result = 0, .flags = 0x1800},
    {.opcode = 0x023e0000ff31, .input = 0x80000001, .result = 0x40000000, .flags = 0},
    {.opcode = 0x023e0000e131, .input = 0x80000001, .result = 1, .flags = 0},
    {.opcode = 0x023e0000e031, .input = 0xffffffff, .result = 0, .flags = 0x1000},
    {.opcode = 0x023e00008031, .input = 0xffffffff, .result = 0, .flags = 0x1000},
    {.opcode = 0x023e0001ff31, .input = 0x80000001, .result = 0xc0000000, .flags = 0},
    {.opcode = 0x023e0001e131, .input = 0x80000001, .result = 0xffffffff, .flags = 0},
    {.opcode = 0x023e0001e031, .input = 0x80000000, .result = 0xffffffff, .flags = 0},
    {.opcode = 0x023e00018031, .input = 0x80000000, .result = 0xffffffff, .flags = 0},
    {.opcode = 0x023e0001e031, .input = 0x7fffffff, .result = 0, .flags = 0x1000},
    {.opcode = 0x023e00010131, .input = 0x7fffffff, .result = 0xfffffffe, .flags = 0x800},
    {.opcode = 0x023e00000131, .input = 0, .result = 0, .flags = 0x1800},
    {.opcode = register_logical,
     .input = 1,
     .result = 0,
     .flags = 0x1800,
     .count = 0x100,
     .immediate = false},
    {.opcode = register_logical,
     .input = 1,
     .result = 0,
     .flags = 0x1800,
     .count = 0x7fffffff,
     .immediate = false},
    {.opcode = register_logical,
     .input = 0xffffffff,
     .result = 0,
     .flags = 0x1000,
     .count = 0x80000000,
     .immediate = false},
    {.opcode = register_logical,
     .input = 0xffffffff,
     .result = 0,
     .flags = 0x1000,
     .count = 0xffffff00,
     .immediate = false},
    {.opcode = register_arithmetic,
     .input = 0x80000000,
     .result = 0xffffffff,
     .flags = 0,
     .count = 0x80000000,
     .immediate = false},
    {.opcode = register_arithmetic,
     .input = 1,
     .result = 0,
     .flags = 0x1000,
     .count = 0xffffff00,
     .immediate = false},
    {.opcode = register_arithmetic,
     .input = 0xffffffff,
     .result = 0,
     .flags = 0x1800,
     .count = 0x7fffffff,
     .immediate = false},
    {.opcode = register_arithmetic,
     .input = 0x80000000,
     .result = 0xc0000000,
     .flags = 0,
     .count = 0xffffffff,
     .immediate = false},
}};

void literal_case(ShiftCase item) {
  lead_a1::SharcMemory memory;
  map(memory, item.opcode);
  auto initial = initial_state();
  initial.data[0][1] = word(item.input);
  if (!item.immediate) { initial.data[0][2] = word(item.count); }
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  require((cpu.state().data[0][3] ==
           lead_a1::SharcKnown40{.value = std::uint64_t{item.result} << 8, .known = full40}));
  require(cpu.state().system[6].value == (preserved | item.flags));
  require(cpu.state().system[6].known == full && cpu.state().effective[6] == cpu.state().system[6]);
  require(cpu.state().system[8] == initial.system[8] && cpu.state().system[9] == initial.system[9]);
  require(cpu.state().system[7] == initial.system[7] && cpu.state().data[2] == initial.data[2]);
}

// Independent per-destination-bit routing oracle, not a wide host shift/clamp.
struct Routing {
  std::uint32_t input;
  int count;
  bool arithmetic;
};

std::uint32_t routed(Routing request) {
  std::uint32_t result{};
  constexpr int bits = 32;
  constexpr std::uint32_t sign = 0x80000000;
  for (int target = 0; target < bits; ++target) {
    const auto source = target - request.count;
    bool one{};
    if (source >= 0 && source < bits) {
      one = ((request.input >> static_cast<unsigned int>(source)) & 1) != 0;
    } else if (source >= bits && request.arithmetic) {
      one = (request.input & sign) != 0;
    }
    if (one) { result |= std::uint32_t{1} << static_cast<unsigned int>(target); }
  }
  return result;
}

void immediate_count_matrix() {
  constexpr std::array<std::uint32_t, 4> inputs{0, 0xffffffff, 0x80000001, 0x12345678};
  constexpr int minimum = -128;
  constexpr int maximum = 127;
  for (const bool arithmetic : {false, true}) {
    for (int count = minimum; count <= maximum; ++count) {
      for (const auto input : inputs) {
        const auto result = routed({.input = input, .count = count, .arithmetic = arithmetic});
        const auto data = std::uint64_t{static_cast<std::uint8_t>(count)} << 8;
        const auto opcode = arithmetic ? immediate_arithmetic : immediate_logical;
        const auto flags = (result == 0 ? zero : 0) | (count > 0 ? overflow : 0);
        literal_case({.opcode = opcode | data, .input = input, .result = result, .flags = flags});
      }
    }
  }
}

struct PartialCase {
  std::uint64_t opcode;
  lead_a1::SharcKnown32 input;
  lead_a1::SharcKnown32 result;
  std::uint32_t flags;
  bool zero_known{};
};

constexpr std::array<PartialCase, 8> partial_cases{{
    {.opcode = 0x023e00000131, .input = {}, .result = {.known = 1}, .flags = overflow},
    {.opcode = 0x023e0000ff31, .input = {}, .result = {.known = 0x80000000}, .flags = 0},
    {.opcode = 0x023e0001ff31,
     .input = {.known = 0x80000000},
     .result = {.known = 0xc0000000},
     .flags = 0},
    {.opcode = 0x023e0001ff31,
     .input = {.value = 0x80000000, .known = 0x80000000},
     .result = {.value = 0xc0000000, .known = 0xc0000000},
     .flags = 0,
     .zero_known = true},
    {.opcode = 0x023e0001e031, .input = {}, .result = {}, .flags = 0},
    {.opcode = 0x023e00002031,
     .input = {},
     .result = {.known = full},
     .flags = zero | overflow,
     .zero_known = true},
    {.opcode = 0x023e00008031,
     .input = {},
     .result = {.known = full},
     .flags = zero,
     .zero_known = true},
    {.opcode = 0x023e0001e031,
     .input = {.value = 0x80000000, .known = 0x80000000},
     .result = {.value = full, .known = full},
     .flags = 0,
     .zero_known = true},
}};

void partial_knowledge() {
  for (const auto item : partial_cases) {
    lead_a1::SharcMemory memory;
    map(memory, item.opcode);
    auto initial = initial_state();
    initial.data[0][1] = {.value = std::uint64_t{item.input.value} << 8,
                          .known = std::uint64_t{item.input.known} << 8};
    lead_a1::Sharc cpu(memory, initial);
    static_cast<void>(cpu.step());
    constexpr std::uint64_t extension = 0xff;
    require((cpu.state().data[0][3] ==
             lead_a1::SharcKnown40{.value = std::uint64_t{item.result.value} << 8,
                                   .known = (std::uint64_t{item.result.known} << 8) | extension}));
    require(cpu.state().system[6].value == (preserved | item.flags));
    require(cpu.state().system[6].known == (item.zero_known ? full : full ^ zero));
  }
}

struct LaneResult {
  std::uint32_t value;
  std::uint32_t flags;
};

void check_lane(const lead_a1::SharcState &actual,
                const lead_a1::SharcState &initial,
                unsigned int element,
                LaneResult result) {
  const auto bank = element * 2;
  if ((initial.system[6 + element].value & zero) == 0) {
    require(actual.data[bank] == initial.data[bank]);
    require(actual.system[6 + element] == initial.system[6 + element]);
    return;
  }
  require(actual.data[bank][3].value == (std::uint64_t{result.value} << 8));
  require(actual.data[bank][3].known == full40);
  require(actual.system[6 + element].value == (preserved | result.flags));
  require(actual.effective[6 + element] == actual.system[6 + element]);
}

lead_a1::SharcState simd_state(unsigned int pattern, bool immediate) {
  auto initial = initial_state();
  initial.system[2].value = simd;
  for (unsigned int element = 0; element < 2; ++element) {
    const bool active = (pattern & (1U << element)) != 0;
    const auto bank = std::size_t{element} * 2;
    initial.system[6 + element] = {.value = preserved | (active ? zero : 0), .known = full};
    if (active) {
      constexpr std::uint32_t sign = 0x80000000;
      initial.data[bank][1] = word(element == 0 ? sign : 1);
      if (!immediate) { initial.data[bank][2] = word(element == 0 ? 1 : full); }
    }
  }
  initial.effective = initial.system;
  return initial;
}

void simd_conditions() {
  constexpr std::uint64_t if_sz_register = 0x011000200312;
  constexpr std::uint64_t if_sz_immediate = 0x02100000ff31;
  for (const bool immediate : {false, true}) {
    for (unsigned int pattern = 0; pattern < 4; ++pattern) {
      lead_a1::SharcMemory memory;
      map(memory, immediate ? if_sz_immediate : if_sz_register);
      const auto initial = simd_state(pattern, immediate);
      lead_a1::Sharc cpu(memory, initial);
      static_cast<void>(cpu.step());
      constexpr std::uint32_t half_sign = 0x40000000;
      check_lane(cpu.state(),
                 initial,
                 0,
                 {.value = immediate ? half_sign : 0, .flags = immediate ? 0 : zero | overflow});
      check_lane(cpu.state(), initial, 1, {.value = 0, .flags = zero});
      require(cpu.state().system[8] == initial.system[8] &&
              cpu.state().system[9] == initial.system[9]);
    }
  }
}

void banks_extension_and_count_alias() {
  for (unsigned int extension = 0; extension < 16; ++extension) {
    lead_a1::SharcMemory memory;
    constexpr std::uint64_t shift_high = 0x023e0000ffa9; // R10=LSHIFT R9 BY -1.
    constexpr unsigned int extension_shift = 27;
    map(memory, shift_high | (std::uint64_t{extension} << extension_shift));
    auto initial = initial_state();
    constexpr std::uint32_t high_secondary = 0x80;
    constexpr std::uint32_t low_bank = 0x400;
    initial.system[2] = {.value = high_secondary, .known = full ^ low_bank};
    initial.effective = initial.system;
    constexpr std::uint32_t input = 0x80000000;
    initial.data[1][9] = word(input);
    lead_a1::Sharc cpu(memory, initial);
    static_cast<void>(cpu.step());
    constexpr std::uint64_t expected = 0x4000000000;
    require(cpu.state().data[1][10].value == expected && cpu.state().data[0] == initial.data[0]);
  }
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t count_alias = 0x013e00200313;
  map(memory, count_alias);
  auto initial = initial_state();
  initial.data[0][1] = word(1);
  initial.data[0][3] = word(2);
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][3].value == (std::uint64_t{4} << 8));
}

void unknown_count_and_mode_rollback() {
  lead_a1::SharcMemory memory;
  map(memory, register_logical);
  auto initial = initial_state();
  initial.system[2].value = simd;
  initial.effective = initial.system;
  initial.pending[2] = initial.system[2];
  initial.data[0][1] = word(1);
  initial.data[0][2] = word(1);
  initial.data[2][1] = word(1); // PEy count unknown after tentative PEx result/flags.
  lead_a1::Sharc cpu(memory, initial);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Unknown SHARC register operand");
  require(cpu.state() == before && !memory.active());
  initial.system[2] = {.value = 0, .known = full ^ simd};
  initial.effective = initial.system;
  initial.pending[2].reset();
  lead_a1::Sharc unknown_mode(memory, initial);
  rejected([&] { static_cast<void>(unknown_mode.step()); }, "Unknown SHARC compute SIMD mode");
  require(unknown_mode.state() == initial && !memory.active());
}

void invalid_fields() {
  constexpr std::array<std::uint64_t, 3> reserved{0x023e80000131, 0x023e00400131, 0x023e04000131};
  for (const auto opcode : reserved) {
    lead_a1::SharcMemory memory;
    map(memory, opcode);
    lead_a1::Sharc cpu(memory, initial_state());
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(cpu.step()); }, "memory/reserved/VISA");
    require(cpu.state() == before && !memory.active());
  }
  constexpr std::array<std::uint64_t, 3> unsupported{
      0x023e00020131, 0x023e00080131, 0x013e00208312};
  for (const auto opcode : unsupported) {
    lead_a1::SharcMemory memory;
    map(memory, opcode);
    lead_a1::Sharc cpu(memory, initial_state());
    rejected([&] { static_cast<void>(cpu.step()); }, "other shifter operation");
  }
}

void shifts_in_short_loop() {
  constexpr std::array<std::uint64_t, 2> bodies{0x023e00000111, 0x013e00200112};
  for (const auto body : bodies) {
    lead_a1::SharcMemory memory;
    constexpr std::uint64_t loop = 0x0c0003000001;
    map(memory, loop);
    memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48}, body);
    auto initial = initial_state();
    initial.data[0][1] = word(1);
    initial.data[0][2] = word(1);
    lead_a1::Sharc cpu(memory, initial);
    static_cast<void>(cpu.step());
    for (const auto result : {2U, 4U, 8U}) {
      static_cast<void>(cpu.step());
      require(cpu.state().data[0][1].value == (std::uint64_t{result} << 8));
    }
    require(cpu.state().pc == base + 2 && !cpu.state().count_loop && cpu.state().depth == 0);
    require(cpu.state().system[6].value == (preserved | overflow));
  }
}

}

int main() {
  try {
    for (const auto item : cases) { literal_case(item); }
    immediate_count_matrix();
    partial_knowledge();
    simd_conditions();
    banks_extension_and_count_alias();
    unknown_count_and_mode_rollback();
    invalid_fields();
    shifts_in_short_loop();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
