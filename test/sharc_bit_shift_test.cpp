// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
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
constexpr std::uint64_t extension = 0xa7;
constexpr std::uint64_t low_zero = 0xff;
constexpr std::uint32_t flag_mask = 0x3800;
constexpr std::uint32_t zero = 0x1000;
constexpr std::uint32_t overflow = 0x800;
constexpr std::uint32_t preserved = 0xa50447ff;
enum class Operation : std::uint8_t { set = 0xc0, clear = 0xc4, toggle = 0xc8, test = 0xcc };
constexpr auto set = Operation::set;
constexpr auto clear = Operation::clear;
constexpr auto toggle = Operation::toggle;
constexpr auto test = Operation::test;
constexpr std::uint32_t simd = 0x200000;
constexpr std::uint64_t register_prefix = 0x013e00200000;
constexpr std::uint64_t immediate_prefix = 0x023e00000000;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC bit-operation expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) == std::string::npos) {
      throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
    }
    return;
  }
  throw std::runtime_error("Missing SHARC bit-operation rejection: " + reason);
}

lead_a1::SharcKnown40 word(std::uint32_t value, std::uint32_t known = full) {
  return {.value = (std::uint64_t{value} << 8) | extension,
          .known = (std::uint64_t{known} << 8) | low_zero};
}

void program(lead_a1::SharcMemory &memory, std::uint64_t opcode) {
  constexpr unsigned int cells = 96;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned bit-operation program"});
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, opcode);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.system[6] = {.value = preserved | flag_mask, .known = full};
  state.system[7] = state.system[6];
  state.effective = state.system;
  return state;
}

std::uint64_t instruction(Operation operation, std::uint32_t position, bool immediate) {
  constexpr unsigned int operation_shift = 12;
  constexpr unsigned int immediate_operation_shift = 16;
  constexpr std::uint64_t destinations = 0x312; // R3, R1, count R2.
  constexpr std::uint64_t immediate_registers = 0x31;
  return immediate
             ? immediate_prefix |
                   ((static_cast<std::uint64_t>(operation) / 4) << immediate_operation_shift) |
                   (std::uint64_t{position} << 8) | immediate_registers
             : register_prefix | (static_cast<std::uint64_t>(operation) << operation_shift) |
                   destinations;
}

struct Expectation {
  std::uint32_t result;
  std::uint32_t flags;
};

struct TestOperands {
  std::uint32_t input;
  std::uint32_t position;
};

Expectation expected(Operation operation, TestOperands operands) {
  constexpr unsigned int bit_count = 32;
  std::uint32_t mask{};
  if (operands.position < bit_count) {
    mask = 1;
    for (unsigned int bit = 0; bit < operands.position; ++bit) { mask *= 2; }
  }
  auto result = operands.input;
  if (operation == set) { result |= mask; }
  if (operation == clear) { result &= ~mask; }
  if (operation == toggle) { result ^= mask; }
  const bool is_zero = operation == test ? (operands.input & mask) == 0 : result == 0;
  return {.result = result, .flags = (mask == 0 ? overflow : 0) | (is_zero ? zero : 0)};
}

void routed_case(Operation operation, std::uint32_t position, bool immediate, std::uint32_t input) {
  lead_a1::SharcMemory memory;
  program(memory, instruction(operation, position, immediate));
  auto state = initial();
  constexpr std::uint32_t old = 0xdeadbeef;
  state.data[0][1] = word(input);
  state.data[0][2] = word(position);
  state.data[0][3] = word(old);
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  const auto result = expected(operation, {.input = input, .position = position});
  const auto data = cpu.state().data[0][3];
  if (operation == test) {
    require(cpu.state().data == state.data); // BTST has no destination or extension write.
  } else {
    require(data.value == std::uint64_t{result.result} * 256 && data.known == full40);
  }
  require(cpu.state().system[6].value == (preserved | result.flags));
  require(cpu.state().effective[6] == cpu.state().system[6]);
  require(cpu.state().system[7] == state.system[7]);
  require(cpu.state().system[8] == state.system[8] && cpu.state().system[9] == state.system[9]);
}

void literal_and_routing() {
  constexpr std::uint64_t immediate_set9 = 0x023e00300931;
  constexpr std::uint64_t register_clear = 0x013e002c4312;
  require(instruction(set, 9, true) == immediate_set9);
  require(instruction(clear, 9, false) == register_clear);
  for (const auto operation : {set, clear, toggle, test}) {
    constexpr unsigned int immediates = 256;
    for (unsigned int position = 0; position < immediates; ++position) {
      routed_case(operation, position, true, 0);
      routed_case(operation, position, true, full);
    }
    constexpr std::array<std::uint32_t, 13> positions{
        0, 1, 9, 31, 32, 127, 128, 255, 256, 65536, 0x7fffffff, 0x80000000, 0xffffffff};
    constexpr std::uint32_t pattern = 0x80000001;
    for (const auto position : positions) { routed_case(operation, position, false, pattern); }
  }
  lead_a1::SharcMemory memory;
  program(memory, immediate_set9);
  auto state = initial();
  state.data[0][1] = word(4);
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  constexpr std::uint64_t literal_result = 0x20400;
  require(cpu.state().data[0][3].value == literal_result);
  require(cpu.state().system[6].value == preserved);
}

void unknown_inputs() {
  constexpr std::uint32_t selected = 0x200;
  for (const auto operation : {set, clear, toggle, test}) {
    lead_a1::SharcMemory memory;
    program(memory, instruction(operation, 9, true));
    auto state = initial();
    state.data[0][1] = word(0, 0);
    state.data[0][3] = word(full);
    lead_a1::Sharc cpu(memory, state);
    static_cast<void>(cpu.step());
    const auto flags = cpu.state().system[6];
    const auto result = cpu.state().data[0][3];
    require((flags.value & flag_mask) == 0);
    require((flags.known & flag_mask) == (operation == set ? flag_mask : flag_mask & ~zero));
    if (operation == test) {
      require(result == state.data[0][3]);
    } else {
      require(result.value == (operation == set ? std::uint64_t{selected} * 256 : 0));
      const auto known =
          operation == toggle ? low_zero : (std::uint64_t{selected} * 256) | low_zero;
      require(result.known == known);
    }
  }
  for (const bool one : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory, instruction(test, 9, true));
    auto state = initial();
    state.data[0][1] = word(one ? selected : 0, selected);
    lead_a1::Sharc cpu(memory, state);
    static_cast<void>(cpu.step());
    require(cpu.state().data == state.data);
    require((cpu.state().system[6].known & flag_mask) == flag_mask);
    require((cpu.state().system[6].value & flag_mask) == (one ? 0 : zero));
  }
}

void aliased_operands() {
  struct AliasCase {
    std::uint64_t opcode;
    std::uint32_t input;
    std::uint32_t count;
    unsigned int destination;
    std::uint64_t result;
  };

  constexpr std::array<AliasCase, 4> cases{
      {{.opcode = 0x013e002c0212, .input = 0x20, .count = 9, .destination = 2, .result = 0x22000},
       {.opcode = 0x013e002c0311,
        .input = 9,
        .count = 0x7fffffff,
        .destination = 3,
        .result = 0x20900},
       {.opcode = 0x013e002c0112, .input = 0x20, .count = 9, .destination = 1, .result = 0x22000},
       {.opcode = 0x013e002c0111,
        .input = 9,
        .count = 0x7fffffff,
        .destination = 1,
        .result = 0x20900}}};
  for (const auto &item : cases) {
    lead_a1::SharcMemory memory;
    program(memory, item.opcode);
    auto state = initial();
    state.data[0][1] = word(item.input);
    state.data[0][2] = word(item.count);
    lead_a1::Sharc cpu(memory, state);
    static_cast<void>(cpu.step());
    auto expected_data = state.data;
    expected_data[0][item.destination] = {.value = item.result, .known = full40};
    require(cpu.state().data == expected_data);
    require(cpu.state().system[6].value == preserved);
  }
}

void simd_rollback_and_banks() {
  lead_a1::SharcMemory memory;
  program(memory, instruction(set, 0, false));
  auto state = initial();
  constexpr std::uint32_t low_bank = 0x400;
  state.system[2].value = simd | low_bank;
  state.effective = state.system;
  state.data[1][1] = word(0);
  state.data[1][2] = word(9);
  state.data[3][1] = word(0);
  state.data[3][2] = word(0, 0); // PEy fails after tentative PEx register/flag writes.
  lead_a1::Sharc cpu(memory, state);
  rejected([&] { static_cast<void>(cpu.step()); }, "Unknown SHARC register operand bits");
  require(cpu.state() == state && !memory.active());
  state.data[3][2] = word(1);
  lead_a1::Sharc valid(memory, state);
  static_cast<void>(valid.step());
  constexpr std::uint64_t x_result = 0x20000;
  constexpr std::uint64_t y_result = 0x200;
  require(valid.state().data[1][3].value == x_result);
  require(valid.state().data[3][3].value == y_result);
  require(valid.state().system[6].value == preserved && valid.state().system[7].value == preserved);
  require(valid.state().data[0] == state.data[0] && valid.state().data[2] == state.data[2]);
}

void predicates_and_count_failure() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t conditions = std::uint64_t{31} << 33;
  program(memory, instruction(test, 0, false) & ~conditions); // IF EQ.
  auto state = initial();
  state.system[6].value &= ~1U;
  state.effective = state.system;
  lead_a1::Sharc skipped(memory, state);
  static_cast<void>(skipped.step());
  require(skipped.state().data == state.data && skipped.state().system == state.system);
  memory.write({.address = base, .width = lead_a1::SharcWidth::wide48}, instruction(set, 0, false));
  state.data[0][1] = word(0);
  lead_a1::Sharc missing(memory, state);
  rejected([&] { static_cast<void>(missing.step()); }, "Unknown SHARC register operand bits");
  require(missing.state() == state && !memory.active());
}

void count_loop() {
  lead_a1::SharcMemory memory;
  constexpr std::uint64_t two_iterations = 0x0c0002000001;
  program(memory, two_iterations);
  constexpr std::uint64_t destination_field = 0xf0;
  constexpr std::uint64_t same_source_target = 0x10;
  memory.write({.address = base + 1, .width = lead_a1::SharcWidth::wide48},
               (instruction(toggle, 9, true) & ~destination_field) | same_source_target);
  auto state = initial();
  state.data[0][1] = word(0);
  lead_a1::Sharc cpu(memory, state);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(cpu.state().data[0][1].value == 0);
  require(cpu.state().system[6].value == (preserved | zero));
  require(!cpu.state().count_loop && cpu.state().pc == base + 2);
}

}

int main() {
  try {
    literal_and_routing();
    unknown_inputs();
    aliased_operands();
    simd_rollback_and_banks();
    predicates_and_count_failure();
    count_loop();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
