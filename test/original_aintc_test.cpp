// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/aintc.hpp"
#include "lead_a1/arm.hpp"
#include "lead_a1/arm_system.hpp"
#include "lead_a1/memory.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t image_base = 0xc0000000;
constexpr std::uint32_t controller_base = 0xfffee000;
constexpr std::uint32_t object = 0x6000;
constexpr std::uint32_t stack = 0x8000;
constexpr std::uint32_t sentinel = 0x1000;
constexpr std::uint32_t initial_status = 0xb80000df;
constexpr std::uint32_t individual_hold = 0x14;
constexpr std::uint32_t unmasked_channel = 0x103;
constexpr unsigned int binding_source = 2;
constexpr unsigned int test_source = 40;

struct Placement {
  std::uint32_t constructor;
  std::uint32_t table;
  std::uint32_t null_callback;
};

struct Inputs {
  std::filesystem::path specification;
  std::filesystem::path images;
};

constexpr std::array<Placement, 2> placements{
    {{.constructor = 0xc0172504, .table = 0xc0767e6c, .null_callback = 0xc01724f0},
     {.constructor = 0xc002e258, .table = 0xc00a8004, .null_callback = 0xc002e244}}};

struct Write {
  std::uint32_t offset;
  std::uint64_t value;
  bool operator==(const Write &) const = default;
};

class RecordedController final : public lead_a1::MemoryDevice {
public:
  explicit RecordedController(const lead_a1::AintcState &initial) : device(initial) {}

  std::uint64_t read(lead_a1::MemoryAccess access) override { return device.read(access); }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    device.write(access, value);
    writes.push_back({.offset = access.address, .value = value});
  }

  void begin_instruction() override {
    checkpoint = writes.size();
    device.begin_instruction();
  }

  void commit_instruction() noexcept override { device.commit_instruction(); }

  void abort_instruction() noexcept override {
    device.abort_instruction();
    writes.resize(checkpoint);
  }

  lead_a1::Aintc device;
  std::vector<Write> writes;
  std::size_t checkpoint{};
};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Original AINTC expectation at line " + std::to_string(where.line()));
  }
}

std::vector<std::byte> read(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  const std::vector<char> bytes{std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>()};
  if (!input || bytes.empty()) { throw std::runtime_error("Missing original AINTC input"); }
  const auto bytes_view = std::as_bytes(std::span(bytes));
  return {bytes_view.begin(), bytes_view.end()};
}

lead_a1::ArmSystemInitial profile() {
  constexpr std::uint32_t control = 0x50078;
  return {.control = control,
          .instruction_cache_bytes = {},
          .data_cache_bytes = {},
          .instruction_cache_invalid = true,
          .data_cache_invalid = true,
          .instruction_lockdown = {},
          .data_lockdown = {},
          .write_buffer = std::vector<lead_a1::ArmBufferedWrite>{},
          .writeback_buffer_empty = true,
          .mmu = {}};
}

lead_a1::ArmState call(lead_a1::Memory &memory,
                       const std::filesystem::path &spec,
                       std::uint32_t entry,
                       std::array<std::uint32_t, 4> arguments,
                       unsigned int expected_count = 0) {
  lead_a1::ArmState state;
  state.cpsr = initial_status;
  constexpr std::uint32_t marker = 0x10203000;
  for (unsigned int reg = 0; reg < 13; ++reg) { state.registers[reg] = marker + reg; }
  for (unsigned int reg = 0; reg < arguments.size(); ++reg) {
    state.registers[reg] = arguments[reg];
  }
  state.registers[13] = stack;
  state.registers[14] = sentinel;
  state.registers[15] = entry;
  lead_a1::Arm cpu(memory, spec, state, profile());
  constexpr unsigned int limit = 64;
  unsigned int count{};
  while (cpu.state().registers[15] != sentinel && count < limit) {
    require(!cpu.step().exception);
    ++count;
  }
  require(cpu.state().registers[15] == sentinel && count < limit);
  require(expected_count == 0 || count == expected_count);
  require(cpu.state().registers[13] == stack);
  constexpr std::uint32_t control_mask = 0xff;
  require((cpu.state().cpsr & control_mask) == (initial_status & control_mask));
  for (unsigned int reg = 4; reg <= 11; ++reg) {
    require(cpu.state().registers[reg] == state.registers[reg]);
  }
  return cpu.state();
}

void expect_writes(const RecordedController &controller, std::span<const Write> expected) {
  require(std::ranges::equal(controller.writes, expected));
}

void check_constructor(lead_a1::Memory &memory,
                       RecordedController &controller,
                       const std::filesystem::path &spec,
                       const Placement &where,
                       std::uint32_t old_control) {
  constexpr unsigned int constructor_instructions = 28;
  const auto after =
      call(memory, spec, where.constructor, {object, 0, 0, 0}, constructor_instructions);
  require(after.cpsr == initial_status && after.registers[0] == object);
  constexpr std::array<Write, 6> clears{{{.offset = 0x380, .value = 0xffffffff},
                                         {.offset = 0x384, .value = 0xffffffff},
                                         {.offset = 0x388, .value = 0x07ffffff},
                                         {.offset = 0x280, .value = 0xffffffff},
                                         {.offset = 0x284, .value = 0xffffffff},
                                         {.offset = 0x288, .value = 0x07ffffff}}};
  require(controller.writes.size() == 11);
  require(std::ranges::equal(std::span(controller.writes).first(clears.size()), clears));
  constexpr std::uint32_t hold_bit = 0x10;
  const std::array<Write, 5> configuration{{{.offset = 0x58, .value = where.null_callback},
                                            {.offset = 0x50, .value = where.table + 4},
                                            {.offset = 0x54, .value = 0},
                                            {.offset = 4, .value = (old_control & hold_bit) | 4},
                                            {.offset = 4, .value = individual_hold}}};
  require(
      std::ranges::equal(std::span(controller.writes).last(configuration.size()), configuration));
  constexpr std::array<std::uint32_t, 4> remaining{0, 0, 0xf8000000, 0x1f};
  const auto state = controller.device.state();
  require(state.control == individual_hold && state.pending == remaining &&
          state.enabled == remaining);
  require(state.global_enabled && state.host_enabled == 3);
  require(state.vector_base == where.table + 4 && state.vector_size == 0 &&
          state.vector_size_programmed);
  require(state.null_vector == where.null_callback);
  require(memory.read({.address = object, .size = 4}) == controller_base);
  require(memory.read({.address = where.table, .size = 4}) == object);
  require(!memory.peek(where.table + 4));
}

void check_helpers(lead_a1::Memory &memory,
                   RecordedController &controller,
                   const std::filesystem::path &spec,
                   const Placement &where) {
  constexpr std::uint32_t global_offset = 0x80;
  constexpr std::uint32_t bind_offset = 0xf8;
  constexpr std::uint32_t source_offset = 0x13c;
  constexpr std::uint32_t clear_offset = 0x150;
  constexpr std::uint32_t all_clear_offset = 0x15c;
  constexpr std::uint32_t nesting_offset = 0x18c;
  controller.writes.clear();
  static_cast<void>(call(memory, spec, where.constructor + global_offset, {object, 1, 0, 0}));
  constexpr std::array<Write, 3> enable{
      {{.offset = 0x34, .value = 0}, {.offset = 0x34, .value = 1}, {.offset = 0x10, .value = 1}}};
  expect_writes(controller, enable);
  require(controller.device.state().global_enabled && controller.device.state().host_enabled == 3);
  controller.writes.clear();
  static_cast<void>(call(memory, spec, where.constructor + global_offset, {object, 0, 0, 0}));
  constexpr std::array<Write, 3> disable{
      {{.offset = 0x10, .value = 0}, {.offset = 0x38, .value = 0}, {.offset = 0x38, .value = 1}}};
  expect_writes(controller, disable);
  require(!controller.device.state().global_enabled && controller.device.state().host_enabled == 0);
  static_cast<void>(call(memory,
                         spec,
                         where.constructor + bind_offset,
                         {object, binding_source, sentinel, unmasked_channel}));
  require(memory.read({.address = where.table + 12, .size = 4}) == sentinel);
  require(controller.device.state().channels[2] == 3 && controller.device.state().channels[3] == 1);
  static_cast<void>(
      call(memory, spec, where.constructor + source_offset, {object, test_source, 1, 0}));
  require(controller.device.state().enabled[1] == 0x100);
  controller.device.pulse(test_source);
  static_cast<void>(
      call(memory, spec, where.constructor + clear_offset, {object, test_source, 0, 0}));
  require(controller.device.state().pending[1] == 0);
  static_cast<void>(
      call(memory, spec, where.constructor + source_offset, {object, test_source, 0, 0}));
  require(controller.device.state().enabled[1] == 0);
  constexpr std::uint32_t board_limit = 100;
  const auto after = call(memory, spec, where.constructor + nesting_offset, {board_limit, 0, 0, 0});
  require(after.registers[0] == 0x100 && controller.device.state().host_nesting[1] == board_limit);
  require(controller.device.state().host_nesting_override[1]);
  static_cast<void>(call(memory, spec, where.constructor + all_clear_offset, {object, 0, 0, 0}));
  constexpr std::array<std::uint32_t, 4> remaining{0, 0, 0, 0x1f};
  require(controller.device.state().pending == remaining &&
          controller.device.state().enabled == remaining);
  require(!controller.device.state().global_enabled);
}

void execute(const Inputs &inputs, unsigned int part) {
  const auto code = read(inputs.images / (std::to_string(part) + "-CODE.bin"));
  const auto where = placements[part];
  constexpr std::array<std::uint32_t, 4> initial_controls{0, 0x10, 0x0c, 0x1c};
  for (const auto control : initial_controls) {
    auto initial = lead_a1::Aintc::reset_reference();
    initial.control = control;
    constexpr std::array<std::uint32_t, 4> pending_mask{0xffffffff, 0xffffffff, 0xffffffff, 0x1f};
    initial.pending = pending_mask;
    initial.enabled = initial.pending;
    initial.global_enabled = true;
    initial.host_enabled = 3;
    RecordedController controller(initial);
    lead_a1::Memory memory;
    memory.map(
        {.address = image_base, .size = code.size(), .name = "original CODE", .writable = false},
        code);
    memory.map_device(
        {.address = controller_base, .size = 0x2000, .name = "recorded AINTC", .executable = false},
        controller);
    memory.map({.address = object, .size = 4, .name = "owned controller object"});
    constexpr unsigned int stack_bytes = 128;
    const std::vector<std::byte> pattern(stack_bytes, std::byte{0xa5});
    memory.map({.address = stack - stack_bytes, .size = pattern.size(), .name = "owned stack"},
               pattern);
    constexpr unsigned int table_bytes = 512;
    memory.map({.address = where.table, .size = table_bytes, .name = "owned dispatch RAM"});
    check_constructor(memory, controller, inputs.specification, where, control);
    check_helpers(memory, controller, inputs.specification, where);
    constexpr unsigned int component_bytes = 428;
    for (unsigned int byte = 0; byte < component_bytes; ++byte) {
      require(memory.peek(where.constructor + byte) == code[where.constructor - image_base + byte]);
    }
  }
  std::cout << "Part " << part
            << ": original AINTC constructor/enable/bind/clear/nesting helpers "
               "preserve ordered writes, masked source tails and unmasked channel spill\n";
}

}

int main(int argc, char **argv) {
  if (argc != 3) { return 1; }
  try {
    const Inputs inputs{.specification = argv[1], .images = argv[2]};
    for (const unsigned int part : {0U, 1U}) { execute(inputs, part); }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
