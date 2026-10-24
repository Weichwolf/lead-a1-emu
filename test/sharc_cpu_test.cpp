// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t target = 0xc0010;
constexpr std::uint32_t full = std::numeric_limits<std::uint32_t>::max();
constexpr std::uint32_t call_frame = 0x1000000;
constexpr std::uint32_t btf = 0x40000;
constexpr std::uint32_t simd = 0x200000;
constexpr unsigned int mode1 = 2;
constexpr unsigned int astat_x = 6;
constexpr unsigned int astat_y = 7;
constexpr unsigned int irptl = 10;
constexpr unsigned int imask = 11;
constexpr unsigned int ustat3 = 14;
constexpr unsigned int ustat4 = 15;

// Owned native encodings with literal fields independent of the emulator decoder.
constexpr std::uint64_t load_ustat1_marker = 0x0f70a6b45c3e;
constexpr std::uint64_t toggle_nibble = 0x14400000000f;
constexpr std::uint64_t test_bit0 = 0x148000000001;
constexpr std::uint64_t jump_tf_target = 0x061a000c0010;
constexpr std::uint64_t enable_simd = 0x0f7200200000;
constexpr std::uint64_t load_ustat3_marker = 0x0f7e1234abcd;
constexpr std::uint64_t load_ustat3_other = 0x0f7e2345abcd;
constexpr std::uint64_t call_delayed = 0x06be040c0010;
constexpr std::uint64_t load_first = 0x0f7011112222;
constexpr std::uint64_t load_second = 0x0f7133334444;
constexpr std::uint64_t return_delayed = 0x0a3e04000000;
constexpr std::uint64_t load_third = 0x0f7e55556666;
constexpr std::uint64_t load_fourth = 0x0f7f77778888;
constexpr std::uint64_t jump_relative_wrap = 0x073e00f80000;
constexpr std::uint64_t jump_backward = 0x073e00fffffc;
constexpr std::uint64_t jump_other = 0x063e000c0020;
constexpr std::uint64_t jump_ne_delayed = 0x0620040c0010;
constexpr std::uint64_t jump_eq_delayed = 0x0600040c0020;
constexpr std::uint64_t set_bit0 = 0x140000000001;
constexpr std::uint64_t clear_bit1 = 0x142000000002;
constexpr std::uint64_t toggle_bits01 = 0x144000000003;
constexpr std::uint64_t compare_one = 0x14a000000001;
constexpr std::uint64_t test_bit2 = 0x148000000004;
constexpr std::uint64_t test_no_bits = 0x148000000000;
constexpr std::uint64_t compare_zero = 0x14a000000000;
constexpr std::uint64_t read_ustat1 = 0x107000001003;
constexpr std::uint64_t write_ustat1 = 0x117000001004;
constexpr std::uint64_t write_pc = 0x136300001005;
constexpr std::uint64_t read_unsupported_px = 0x106b00001003;
constexpr std::uint64_t read_ustat2 = 0x107100001003;
constexpr std::uint64_t clear_imask = 0x0f7b00000000;
constexpr std::uint64_t clear_irptl = 0x0f7a00000000;
constexpr std::uint64_t return_immediate = 0x0a3e00000000;
constexpr std::uint64_t call_immediate = 0x06be000c0010;
constexpr std::uint64_t reserved_jump = 0x063e080c0010;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC CPU expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC CPU rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void map(lead_a1::SharcMemory &memory) {
  constexpr std::uint32_t cells = 192;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned native instruction RAM"});
}

void linear_and_effects() {
  lead_a1::SharcMemory memory;
  map(memory);
  // Owned literals, independently encoded from Chapter10 field diagrams.
  code(memory, base, load_ustat1_marker);       // USTAT1 = immediate32.
  code(memory, base + 1, toggle_nibble);        // BIT TGL USTAT1, 15.
  code(memory, base + 2, test_bit0);            // BIT TST USTAT1, 1.
  code(memory, base + 3, jump_tf_target);       // IF TF JUMP target.
  code(memory, target, enable_simd);            // MODE1 = PEYEN.
  code(memory, target + 1, load_ustat3_marker); // USTAT3 = immediate32.
  code(memory, target + 2, load_ustat3_other);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  require(cpu.step().next_pc == base + 1);
  constexpr std::uint32_t initial = 0xa6b45c3e;
  constexpr std::uint32_t toggled = 0xa6b45c31;
  require(cpu.state().system[0].value == initial);
  static_cast<void>(cpu.step());
  require(cpu.state().system[0].value == toggled && cpu.state().system[0].known == full);
  static_cast<void>(cpu.step());
  require((cpu.state().effective[astat_x].value & btf) != 0);
  require(cpu.step().next_pc == target);
  static_cast<void>(cpu.step());
  require(cpu.state().system[mode1].value == simd && cpu.state().effective[mode1].value == 0);
  static_cast<void>(cpu.step());
  constexpr std::uint32_t supplied = 0x1234abcd;
  require(cpu.state().system[ustat3].value == supplied &&
          cpu.state().effective[mode1].value == simd);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "SIMD register");
  require(cpu.state() == before && !memory.active());
}

void delayed_call_return() {
  lead_a1::SharcMemory memory;
  map(memory);
  code(memory, base, call_delayed); // CALL target (DB).
  code(memory, base + 1, load_first);
  code(memory, base + 2, load_second);
  code(memory, target, return_delayed); // RTS (DB).
  code(memory, target + 1, load_third);
  code(memory, target + 2, load_fourth);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(cpu.step());
  require(cpu.state().depth == 1 && cpu.state().pc_stack[0] == (call_frame | (base + 3)));
  auto deferred = cpu.state().branch;
  require(deferred && deferred->slots == 2 && cpu.state().pc == base + 1);
  constexpr std::uint32_t stack_empty = 0x400000;
  require((cpu.state().system[8].value & stack_empty) == 0);
  static_cast<void>(cpu.step());
  deferred = cpu.state().branch;
  require(deferred && deferred->slots == 1 && cpu.state().pc == base + 2);
  static_cast<void>(cpu.step());
  require(!cpu.state().branch && cpu.state().pc == target);
  constexpr std::uint32_t first = 0x11112222;
  constexpr std::uint32_t second = 0x33334444;
  require(cpu.state().system[0].value == first && cpu.state().system[1].value == second);
  static_cast<void>(cpu.step());
  deferred = cpu.state().branch;
  require(cpu.state().depth == 0 && deferred && deferred->slots == 2);
  require(cpu.state().pc == target + 1 && (cpu.state().system[8].value & stack_empty) != 0);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(cpu.state().pc == base + 3 && !cpu.state().branch);
  constexpr std::uint32_t third = 0x55556666;
  constexpr std::uint32_t fourth = 0x77778888;
  require(cpu.state().system[ustat3].value == third && cpu.state().system[ustat4].value == fourth);
  require(cpu.state().pc_stack[0] == (call_frame | (base + 3))); // Pop retains physical storage.
}

void relative_and_delay_rollback() {
  lead_a1::SharcMemory memory;
  map(memory);
  code(memory, base, jump_relative_wrap); // PC-relative -0x80000, modulo24.
  lead_a1::Sharc relative(memory, lead_a1::Sharc::reset_reference_214xx(base));
  constexpr std::uint32_t relative_target = 0x40000;
  require(relative.step().next_pc == relative_target);
  code(memory, base, jump_backward); // PC-relative -4.
  lead_a1::Sharc backward(memory, lead_a1::Sharc::reset_reference_214xx(base));
  require(backward.step().next_pc == base - 4);
  code(memory, base, call_delayed);
  code(memory, base + 1, jump_other); // Taken branch inside a delay slot.
  code(memory, base + 2, 0);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(cpu.step());
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "taken branch in delay slot");
  require(cpu.state() == before && !memory.active());
  code(memory, base + 1, 0);
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require(cpu.state().pc == target);
  // Mutually exclusive false branches in a delay slot are allowed (4-23/24).
  code(memory, base, jump_ne_delayed);     // IF NE JUMP target (DB), AZ reset0.
  code(memory, base + 1, jump_eq_delayed); // IF EQ, false.
  lead_a1::Sharc exclusive(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(exclusive.step());
  static_cast<void>(exclusive.step());
  static_cast<void>(exclusive.step());
  require(exclusive.state().pc == target && exclusive.state().depth == 0);
}

struct Condition {
  unsigned int code;
  std::uint32_t bit;
};

void condition_case(Condition condition, unsigned int pattern) {
  const bool complement = (pattern & 1) != 0;
  const bool dual = (pattern & 2) != 0;
  const bool x = (pattern & 4) != 0;
  const bool y = (pattern & 8) != 0;
  lead_a1::SharcMemory memory;
  map(memory);
  const auto cond = condition.code + (complement ? 16U : 0U);
  constexpr std::uint64_t jump = 0x060000000000;
  constexpr unsigned int condition_shift = 33;
  code(memory, base, jump | (static_cast<std::uint64_t>(cond) << condition_shift) | target);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[astat_x].value = x ? condition.bit : 0;
  initial.system[astat_y].value = y ? condition.bit : 0;
  initial.system[mode1].value = dual ? simd : 0;
  initial.effective = initial.system;
  lead_a1::Sharc cpu(memory, initial);
  const bool expected = (x != complement) && (!dual || (y != complement));
  require(cpu.step().next_pc == (expected ? target : base + 1));
}

void condition_matrix() {
  constexpr std::array<Condition, 9> conditions{{{.code = 0, .bit = 1},
                                                 {.code = 3, .bit = 8},
                                                 {.code = 4, .bit = 2},
                                                 {.code = 5, .bit = 0x80},
                                                 {.code = 6, .bit = 0x40},
                                                 {.code = 7, .bit = 0x800},
                                                 {.code = 8, .bit = 0x1000},
                                                 {.code = 13, .bit = btf},
                                                 {.code = 14, .bit = 0x4000}}};
  for (const auto condition : conditions) {
    for (unsigned int pattern = 0; pattern < 16; ++pattern) { condition_case(condition, pattern); }
  }
}

void unsupported_condition_fields() {
  constexpr std::array<unsigned int, 9> unsupported{9, 10, 11, 12, 15, 25, 26, 27, 28};
  for (const auto condition : unsupported) {
    lead_a1::SharcMemory memory;
    map(memory);
    constexpr std::uint64_t jump = 0x060000000000;
    constexpr unsigned int condition_shift = 33;
    code(memory, base, jump | (static_cast<std::uint64_t>(condition) << condition_shift) | target);
    lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(cpu.step()); }, "condition not implemented");
    require(cpu.state() == before && !memory.active());
  }
}

void partial_bit_operations() {
  lead_a1::SharcMemory memory;
  map(memory);
  code(memory, base, set_bit0);          // SET USTAT1 bit0.
  code(memory, base + 1, clear_bit1);    // CLR bit1.
  code(memory, base + 2, toggle_bits01); // TGL bits0/1.
  code(memory, base + 3, test_bit0);     // TEST known-zero bit0, false.
  code(memory, base + 4, compare_one);   // XOR mismatches a known bit, false.
  code(memory, base + 5, test_bit2);     // TEST unknown bit2, unknown BTF.
  code(memory, base + 6, jump_tf_target);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[0] = {};
  initial.effective[0] = {};
  lead_a1::Sharc cpu(memory, initial);
  static_cast<void>(cpu.step());
  require((cpu.state().system[0] == lead_a1::SharcKnown32{.value = 1, .known = 1}));
  static_cast<void>(cpu.step());
  static_cast<void>(cpu.step());
  require((cpu.state().system[0] == lead_a1::SharcKnown32{.value = 2, .known = 3}));
  static_cast<void>(cpu.step());
  require((cpu.state().system[astat_x].known & btf) != 0 &&
          (cpu.state().system[astat_x].value & btf) == 0);
  static_cast<void>(cpu.step());
  require((cpu.state().system[astat_x].value & btf) == 0);
  static_cast<void>(cpu.step());
  require((cpu.state().system[astat_x].known & btf) == 0);
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "Unknown SHARC control/status");
  require(cpu.state() == before);
  // TEST zero is true for all values; XOR all32 equal requires all32 known.
  code(memory, base, test_no_bits);
  code(memory, base + 1, compare_zero);
  lead_a1::Sharc empty_mask(memory, initial);
  static_cast<void>(empty_mask.step());
  require((empty_mask.state().system[astat_x].value & btf) != 0);
  static_cast<void>(empty_mask.step());
  require((empty_mask.state().system[astat_x].known & btf) == 0);
}

class Port final : public lead_a1::MemoryDevice {
public:
  struct State {
    std::uint32_t value{};
    unsigned int reads{};
    unsigned int writes{};
    std::uint32_t offset{};
    bool operator==(const State &) const = default;
  };

  State current{.value = 0x13579bdf};
  State saved;
  bool pending{};

  std::uint64_t read(lead_a1::MemoryAccess access) override {
    require(access.size == 4);
    ++current.reads;
    current.offset = access.address;
    return current.value;
  }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    require(access.size == 4);
    ++current.writes;
    current.offset = access.address;
    current.value = static_cast<std::uint32_t>(value);
  }

  void begin_instruction() override {
    require(!pending);
    saved = current;
    pending = true;
  }

  void commit_instruction() noexcept override { pending = false; }

  void abort_instruction() noexcept override {
    current = saved;
    pending = false;
  }
};

void nested_calls_and_pc_stack() {
  Port port;
  lead_a1::SharcMemory memory;
  map(memory);
  memory.map_io({.first_word = 0x1000, .words = 1}, port);
  code(memory, base, call_immediate);
  constexpr std::uint64_t relative_call = 0x07be00000010;
  constexpr std::uint64_t replace_return = 0x0f64010c0005;
  constexpr std::uint64_t read_stack = 0x116400001000;
  constexpr std::uint64_t ignored_empty_write = 0x0f64ffffffff;
  code(memory, target, relative_call);
  code(memory, target + 1, replace_return);
  code(memory, target + 2, return_immediate);
  constexpr std::uint32_t nested = 0xc0020;
  code(memory, nested, return_immediate);
  code(memory, base + 5, read_stack);
  code(memory, base + 6, ignored_empty_write);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  require(cpu.step().next_pc == target && cpu.state().depth == 1);
  require(cpu.state().pc_stack[0] == (call_frame | (base + 1)));
  require(cpu.step().next_pc == nested && cpu.state().depth == 2);
  require(cpu.state().pc_stack[1] == (call_frame | (target + 1)));
  require(cpu.step().next_pc == target + 1 && cpu.state().depth == 1);
  static_cast<void>(cpu.step());
  require(cpu.state().pc_stack[0] == (call_frame | (base + 5)));
  require(cpu.step().next_pc == base + 5 && cpu.state().depth == 0);
  static_cast<void>(cpu.step());
  constexpr std::uint32_t empty_stack = 0x3ffffff;
  require(port.current.value == empty_stack);
  static_cast<void>(cpu.step());
  require(cpu.state().depth == 0 && cpu.state().pc_stack[0] == (call_frame | (base + 5)));
}

void failed_effect_and_input_guards() {
  lead_a1::SharcMemory memory;
  map(memory);
  code(memory, base, enable_simd);
  constexpr std::uint64_t load_unsupported_data = 0x0f6b12345678;
  code(memory, base + 1, load_unsupported_data);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(cpu.step());
  const auto before = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "data/DAG/PX");
  require(cpu.state() == before && cpu.state().effective[mode1].value == 0);
  code(memory, base + 1, 0);
  static_cast<void>(cpu.step());
  require(cpu.state().effective[mode1].value == simd);
  auto invalid = lead_a1::Sharc::reset_reference_214xx(base);
  invalid.system[0] = {.value = 1, .known = 0};
  rejected([&] { const lead_a1::Sharc bad(memory, invalid); }, "Invalid SHARC bit knowledge");
  invalid = lead_a1::Sharc::reset_reference_214xx(base);
  invalid.pending[0] = lead_a1::SharcKnown32{};
  rejected([&] { const lead_a1::Sharc bad(memory, invalid); },
           "Unsupported inherited SHARC system effect");
  invalid = lead_a1::Sharc::reset_reference_214xx(base);
  invalid.depth.reset();
  code(memory, base, call_immediate);
  lead_a1::Sharc unknown_stack(memory, invalid);
  const auto unknown_before = unknown_stack.state();
  rejected([&] { static_cast<void>(unknown_stack.step()); }, "Unknown SHARC PC stack depth");
  require(unknown_stack.state() == unknown_before);
  invalid = lead_a1::Sharc::reset_reference_214xx(base);
  invalid.loops_empty.reset();
  lead_a1::Sharc unknown_loop(memory, invalid);
  rejected([&] { static_cast<void>(unknown_loop.step()); }, "active/unknown loop");
}

void iop_transactions() {
  Port port;
  lead_a1::SharcMemory memory;
  map(memory);
  memory.map_io({.first_word = 0x1000, .words = 7}, port);
  code(memory, base, read_ustat1);             // USTAT1 = DM(0x1003).
  code(memory, base + 1, write_ustat1);        // DM(0x1004) = USTAT1.
  code(memory, base + 2, write_pc);            // PM(0x1005) = PC.
  code(memory, base + 3, read_unsupported_px); // Unsupported PX target, after device read.
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(cpu.step());
  constexpr std::uint32_t supplied = 0x13579bdf;
  require(cpu.state().system[0].value == supplied && port.current.reads == 1 &&
          port.current.offset == 12);
  static_cast<void>(cpu.step());
  require(port.current.writes == 1 && port.current.value == supplied && port.current.offset == 16);
  static_cast<void>(cpu.step());
  require(port.current.writes == 2 && port.current.value == base + 2);
  const auto before = cpu.state();
  const auto port_before = port.current;
  rejected([&] { static_cast<void>(cpu.step()); }, "data/DAG/PX");
  require(cpu.state() == before && port.current == port_before && !port.pending &&
          !memory.active());
  code(memory, base + 3, read_ustat2);
  static_cast<void>(cpu.step());
  require(cpu.state().system[1].value == base + 2 && port.current.reads == 2);
}

void masks_and_interrupts() {
  lead_a1::SharcMemory memory;
  map(memory);
  code(memory, base, clear_imask);     // IMASK=0 preserves read-only nonmaskable bits.
  code(memory, base + 1, clear_irptl); // IRPTL=0.
  code(memory, base + 2, 0);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(cpu.step());
  require(cpu.state().system[imask].value == 3 && cpu.state().pending[imask]);
  static_cast<void>(cpu.step());
  require(!cpu.state().pending[imask] && cpu.state().pending[irptl]);
  static_cast<void>(cpu.step());
  require(!cpu.state().pending[irptl] && cpu.state().effective[irptl].value == 0);
  auto pending = lead_a1::Sharc::reset_reference_214xx(base);
  pending.system[irptl].value = 4;
  pending.system[imask].value = 7;
  constexpr std::uint32_t global_enable = 0x1000;
  pending.system[mode1].value = global_enable;
  pending.effective = pending.system;
  lead_a1::Sharc blocked(memory, pending);
  rejected([&] { static_cast<void>(blocked.step()); }, "pending interrupt delivery");
  require(blocked.state() == pending && !memory.active());
  pending.system[irptl].value = 2;
  pending.effective = pending.system;
  lead_a1::Sharc nmi(memory, pending);
  rejected([&] { static_cast<void>(nmi.step()); }, "nonmaskable interrupt");
  auto timer = lead_a1::Sharc::reset_reference_214xx(base);
  constexpr std::uint32_t timer_enable = 0x20;
  timer.system[4].value |= timer_enable;
  timer.effective = timer.system;
  lead_a1::Sharc ticking(memory, timer);
  rejected([&] { static_cast<void>(ticking.step()); }, "core timer not implemented");
}

void repeated_control_writes() {
  lead_a1::SharcMemory memory;
  map(memory);
  constexpr std::uint64_t set_br8 = 0x0f7200000001;
  constexpr std::uint64_t set_br0 = 0x0f7200000002;
  code(memory, base, set_br8);
  code(memory, base + 1, set_br0);
  code(memory, base + 2, 0);
  lead_a1::Sharc cpu(memory, lead_a1::Sharc::reset_reference_214xx(base));
  static_cast<void>(cpu.step());
  require(cpu.state().system[mode1].value == 1 && cpu.state().effective[mode1].value == 0);
  static_cast<void>(cpu.step());
  require(cpu.state().system[mode1].value == 2 && cpu.state().effective[mode1].value == 1);
  static_cast<void>(cpu.step());
  require(cpu.state().effective[mode1].value == 2 && !cpu.state().pending[mode1]);
  code(memory, base, clear_imask);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  initial.system[imask] = {};
  initial.effective[imask] = {};
  lead_a1::Sharc unknown_ro(memory, initial);
  static_cast<void>(unknown_ro.step());
  constexpr std::uint32_t writable = 0xffffff7c;
  require(unknown_ro.state().system[imask].value == 0 &&
          unknown_ro.state().system[imask].known == writable);
}

void stack_and_decode_guards() {
  lead_a1::SharcMemory memory;
  map(memory);
  code(memory, base, return_immediate);
  auto initial = lead_a1::Sharc::reset_reference_214xx(base);
  lead_a1::Sharc empty(memory, initial);
  rejected([&] { static_cast<void>(empty.step()); }, "empty-stack RTS");
  initial.depth = 1;
  lead_a1::Sharc unknown(memory, initial);
  rejected([&] { static_cast<void>(unknown.step()); }, "Unknown SHARC return frame");
  constexpr std::uint32_t special_frame = 0x030c0020;
  initial.pc_stack[0] = special_frame;
  lead_a1::Sharc special(memory, initial);
  rejected([&] { static_cast<void>(special.step()); }, "loop/interrupt return frame");
  code(memory, base, call_immediate);
  constexpr unsigned int almost_full = 28;
  initial.depth = almost_full;
  lead_a1::Sharc overflow(memory, initial);
  const auto near_full = overflow.state();
  rejected([&] { static_cast<void>(overflow.step()); }, "stack-full interrupt boundary");
  require(overflow.state() == near_full);
  code(memory, base, reserved_jump); // Reserved direct-branch bit27.
  lead_a1::Sharc reserved(memory, lead_a1::Sharc::reset_reference_214xx(base));
  rejected([&] { static_cast<void>(reserved.step()); }, "Reserved SHARC direct-branch");
  code(memory, base, set_bit0);
  memory.begin_instruction();
  const auto before = reserved.state();
  rejected([&] { static_cast<void>(reserved.step()); }, "another memory transaction");
  require(memory.active() && reserved.state() == before);
  memory.abort_instruction();
  constexpr std::uint32_t visa_address = 0x100000;
  lead_a1::Sharc visa(memory, lead_a1::Sharc::reset_reference_214xx(visa_address));
  rejected([&] { static_cast<void>(visa.step()); }, "VISA execution");
}

}

int main() {
  try {
    linear_and_effects();
    delayed_call_return();
    relative_and_delay_rollback();
    condition_matrix();
    partial_bit_operations();
    unsupported_condition_fields();
    iop_transactions();
    nested_calls_and_pc_stack();
    failed_effect_and_input_guards();
    masks_and_interrupts();
    repeated_control_writes();
    stack_and_decode_guards();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
