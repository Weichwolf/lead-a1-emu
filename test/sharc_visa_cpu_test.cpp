// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_visa.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0x180040;
constexpr std::uint32_t native = 0x80000;
constexpr std::uint32_t data = 0x80040;
constexpr std::uint32_t all_bits = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr unsigned int cells = 512;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("VISA CPU expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing VISA CPU rejection: " + reason);
}

void program(lead_a1::SharcMemory &memory, bool writable) {
  std::array<lead_a1::SharcCell, cells> zero{};
  constexpr std::uint16_t full_half = 0xffff;
  for (auto &cell : zero) { cell.known = full_half; }
  memory.map({.bank = 0, .halfwords = cells, .name = "owned VISA data", .writable = writable},
             zero);
  memory.map({.bank = 2, .halfwords = cells, .name = "owned VISA program"});
  constexpr std::uint32_t short_start = 0x180000;
  for (unsigned int index = 0; index < cells; ++index) {
    memory.write({.address = short_start + index, .width = lead_a1::SharcWidth::short16}, 1);
  }
}

void code(lead_a1::SharcMemory &memory,
          std::uint32_t address,
          std::span<const std::uint16_t> words) {
  for (unsigned int index = 0; index < words.size(); ++index) {
    memory.write({.address = address + index, .width = lead_a1::SharcWidth::short16}, words[index]);
  }
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  for (auto &reg : state.data[0]) { reg = {.known = full40}; }
  return state;
}

struct Fixture {
  lead_a1::SharcMemory memory;
  lead_a1::SharcControl control{true};
  lead_a1::Sharc cpu;
  lead_a1::SharcVisaBuffer buffer;

  explicit Fixture(lead_a1::SharcState state = initial(), bool writable = true)
      : cpu(memory, state, &control), buffer(memory, {.decode = base, .fetch = base}) {
    program(memory, writable);
    constexpr std::uint32_t sysctl = 0x30024;
    memory.map_io({.first_word = sysctl, .words = 1}, control);
  }
};

void ready(lead_a1::SharcVisaBuffer &buffer) {
  for (unsigned int event = 0; event < 2 && !buffer.instruction(); ++event) {
    if (buffer.can_capture()) { buffer.capture(); }
    static_cast<void>(buffer.advance(false));
  }
  require(buffer.instruction().has_value());
}

lead_a1::SharcStep step(Fixture &fixture) {
  ready(fixture.buffer);
  return fixture.cpu.step(fixture.buffer);
}

void mixed_width_operations() {
  Fixture fixture;
  // R0=-1 (32), R0=R0+1 (16), NOP16, R1=0x12345678 (48), R2=PASS R1 (32).
  constexpr std::array<std::uint16_t, 9> words{
      0x0f80, 0xffff, 0xc500, 0x0001, 0x0f01, 0x1234, 0x5678, 0x0182, 0x1210};
  code(fixture.memory, base, words);
  constexpr std::array lengths{2U, 1U, 1U, 3U, 2U};
  constexpr std::array<std::uint64_t, 5> raw{
      0x0f80ffff0000, 0xc50000000000, 0x000100000000, 0x0f0112345678, 0x018212100000};
  constexpr std::array<std::uint32_t, 5> addresses{base, base + 2, base + 3, base + 4, base + 7};
  for (unsigned int index = 0; index < lengths.size(); ++index) {
    const auto result = step(fixture);
    require(result.pc == addresses[index] && result.opcode == raw[index] &&
            result.address_units == lengths[index] && result.minimum_core_cycles == 1);
  }
  constexpr std::uint64_t expected = 0x1234567800;
  require(fixture.cpu.state().pc == base + 9 && fixture.cpu.state().data[0][0].value == 0 &&
          fixture.cpu.state().data[0][1] ==
              (lead_a1::SharcKnown40{.value = expected, .known = full40}) &&
          fixture.cpu.state().data[0][2] == fixture.cpu.state().data[0][1]);
}

void signed_immediates() {
  struct Edge {
    std::uint16_t encoded;
    std::uint32_t expected;
  };

  constexpr std::array edges{Edge{.encoded = 0, .expected = 0},
                             Edge{.encoded = 0x7fff, .expected = 0x7fff},
                             Edge{.encoded = 0x8000, .expected = 0xffff8000},
                             Edge{.encoded = 0xffff, .expected = 0xffffffff}};
  constexpr std::uint16_t prefix = 0x0f80;
  for (unsigned int reg = 0; reg < 16; ++reg) {
    for (const auto edge : edges) {
      Fixture fixture;
      const std::array words{static_cast<std::uint16_t>(prefix | reg), edge.encoded};
      code(fixture.memory, base, words);
      require(step(fixture).address_units == 2);
      require(fixture.cpu.state().data[0][reg] ==
              (lead_a1::SharcKnown40{.value = std::uint64_t{edge.expected} << 8, .known = full40}));
      require(fixture.cpu.state().depth == 0 && fixture.cpu.state().pc == base + 2);
    }
  }
}

void short_compute_results() {
  struct Example {
    std::uint16_t encoded;
    std::uint32_t left;
    std::uint32_t right;
    std::uint32_t expected;
  };

  constexpr std::array examples{
      Example{.encoded = 0xc035, .left = 7, .right = 3, .expected = 10},
      Example{.encoded = 0xc135, .left = 7, .right = 3, .expected = 4},
      Example{.encoded = 0xc235, .left = 7, .right = 3, .expected = 3},
      Example{.encoded = 0xc335, .left = 7, .right = 3, .expected = 7},
      Example{.encoded = 0xc435, .left = 7, .right = 3, .expected = 0xfffffffc},
      Example{.encoded = 0xc535, .left = 7, .right = 3, .expected = 4},
      Example{.encoded = 0xc635, .left = 7, .right = 3, .expected = 2},
      Example{.encoded = 0xc835, .left = 0x3fc00000, .right = 0x3f000000, .expected = 0x40000000},
      Example{.encoded = 0xc935, .left = 0x3fc00000, .right = 0x3f000000, .expected = 0x3f800000},
      Example{.encoded = 0xcc35, .left = 7, .right = 3, .expected = 3},
      Example{.encoded = 0xcd35, .left = 7, .right = 3, .expected = 7},
      Example{.encoded = 0xce35, .left = 7, .right = 3, .expected = 4},
      Example{.encoded = 0xcf35, .left = 0x3fc00000, .right = 0x3f000000, .expected = 0x3f400000}};
  for (const auto example : examples) {
    auto state = initial();
    state.data[0][3].value = std::uint64_t{example.left} << 8;
    state.data[0][5].value = std::uint64_t{example.right} << 8;
    Fixture fixture(state);
    const std::array words{example.encoded};
    code(fixture.memory, base, words);
    require(step(fixture).address_units == 1);
    require(
        fixture.cpu.state().data[0][3] ==
        (lead_a1::SharcKnown40{.value = std::uint64_t{example.expected} << 8, .known = full40}));
    require(fixture.cpu.state().data[0][5].value == (std::uint64_t{example.right} << 8));
  }
}

void short_compare_flags() {
  struct Example {
    std::uint32_t left;
    std::uint32_t right;
    std::uint32_t expected;
  };

  constexpr std::array examples{Example{.left = 7, .right = 3, .expected = 0x80000000},
                                Example{.left = 3, .right = 7, .expected = 4},
                                Example{.left = 7, .right = 7, .expected = 1}};
  for (const auto example : examples) {
    auto state = initial();
    state.data[0][3].value = std::uint64_t{example.left} << 8;
    state.data[0][5].value = std::uint64_t{example.right} << 8;
    state.system[6] = {.known = all_bits};
    state.effective[6] = state.system[6];
    Fixture fixture(state);
    constexpr std::array<std::uint16_t, 1> compare{0xc335}; // COMP(R3,R5), Type2c.
    code(fixture.memory, base, compare);
    require(step(fixture).address_units == 1);
    require(fixture.cpu.state().system[6] ==
            (lead_a1::SharcKnown32{.value = example.expected, .known = all_bits}));
    require(fixture.cpu.state().effective[6] == fixture.cpu.state().system[6] &&
            fixture.cpu.state().data == state.data &&
            fixture.cpu.state().system[8] == state.system[8]);
  }
}

void call_and_captured_return() {
  Fixture fixture;
  constexpr std::uint32_t callee = base + 32;
  constexpr std::array<std::uint16_t, 3> call{0x06be, 0x0018, 0x0060};
  constexpr std::array<std::uint16_t, 3> body{0x0f84, 0x0009, 0x0abe};
  code(fixture.memory, base, call);
  code(fixture.memory, callee, body);
  require(step(fixture).next_pc == callee && fixture.cpu.state().depth == 1);
  constexpr std::uint32_t frame = 0x1180043;
  require(fixture.cpu.state().pc_stack[0] == frame && fixture.buffer.state().count == 0 &&
          fixture.buffer.state().decode == callee);
  require(step(fixture).address_units == 2 && fixture.cpu.state().pc == callee + 2);
  constexpr std::array<std::uint16_t, 1> corrupt{0xd000};
  code(fixture.memory, callee + 2, corrupt); // The already captured RTS remains actual input.
  const auto returned = step(fixture);
  constexpr std::uint64_t raw_rts = 0x0abe00000000;
  require(returned.opcode == raw_rts && returned.next_pc == base + 3 &&
          fixture.cpu.state().depth == 0 && fixture.cpu.state().data[0][4].value == (9U << 8));
  require(fixture.buffer.state().count == 0 && !fixture.buffer.state().delay &&
          fixture.buffer.state().decode == base + 3);
}

void redirect_to_sequential_address() {
  Fixture fixture;
  constexpr std::array<std::uint16_t, 3> call{0x06be, 0x0018, 0x0043};
  code(fixture.memory, base, call);
  ready(fixture.buffer);
  fixture.buffer.capture(); // Retained speculative NOPs after the actual CALL.
  constexpr std::array<std::uint16_t, 1> increment{0xc500};
  code(fixture.memory, base + 3, increment);
  require(fixture.cpu.step(fixture.buffer).next_pc == base + 3);
  require(fixture.buffer.state().count == 0 && !fixture.buffer.state().delay);
  require(step(fixture).pc == base + 3 && fixture.cpu.state().data[0][0].value == 256);
}

void delayed_return_mixed_slots() {
  auto state = initial();
  constexpr std::uint32_t target = base + 64;
  constexpr std::uint32_t frame = 0x1180080;
  state.depth = 1;
  state.pc_stack[0] = frame;
  Fixture fixture(state);
  constexpr std::array<std::uint16_t, 5> words{0x0afe, 0x0001, 0x0f80, 0x0007, 0xc500};
  code(fixture.memory, base, words);
  require(step(fixture).next_pc == base + 1 && fixture.cpu.state().depth == 0);
  const auto branch = fixture.cpu.state().branch;
  require(branch && branch->slots == 2 && branch->target == target);
  require(step(fixture).next_pc == base + 2);
  require(step(fixture).next_pc == target && !fixture.cpu.state().branch);
  require(fixture.cpu.state().data[0][0].value == (7U << 8) && fixture.buffer.state().count == 0);
}

void native_call_visa_return() {
  auto state = initial();
  state.pc = native;
  Fixture fixture(state);
  constexpr std::uint64_t call_db = 0x06be04000000;
  fixture.memory.write({.address = native, .width = lead_a1::SharcWidth::wide48}, call_db | base);
  constexpr std::array<std::uint16_t, 1> rts{0x0abe};
  code(fixture.memory, base, rts);
  require(fixture.cpu.step().next_pc == native + 1);
  require(fixture.cpu.step().next_pc == native + 2);
  require(fixture.cpu.step().next_pc == base);
  constexpr std::uint32_t frame = 0x1080003;
  require(fixture.cpu.state().pc_stack[0] == frame);
  require(step(fixture).next_pc == native + 3 && fixture.cpu.state().depth == 0);
  require(fixture.buffer.state().count == 0);
}

void single_uncompressed_loop() {
  Fixture fixture;
  constexpr std::array<std::uint16_t, 6> words{0x0c00, 0x0300, 0x0003, 0x0000, 0x0000, 0x0000};
  code(fixture.memory, base, words);
  require(step(fixture).next_pc == base + 3 && fixture.cpu.state().depth == 1);
  constexpr std::array<std::uint32_t, 3> next{base + 3, base + 3, base + 6};
  for (const auto address : next) { require(step(fixture).next_pc == address); }
  require(!fixture.cpu.state().count_loop && fixture.cpu.state().depth == 0 &&
          fixture.cpu.state().loops_empty == true);
  require(step(fixture).next_pc == base + 7);
}

void failures_retain_all_resources() {
  Fixture fixture;
  ready(fixture.buffer);
  const auto native_state = fixture.cpu.state();
  const auto input = fixture.buffer.state();
  const auto controller = fixture.control.snapshot();
  Fixture foreign;
  rejected([&] { static_cast<void>(fixture.cpu.step(foreign.buffer)); }, "idle bound memory");
  fixture.control.begin_instruction();
  rejected([&] { static_cast<void>(fixture.cpu.step(fixture.buffer)); }, "already owned");
  fixture.control.abort_instruction();
  require(fixture.cpu.state() == native_state && fixture.buffer.state() == input &&
          fixture.control.snapshot() == controller && !fixture.memory.active());
  require(step(fixture).next_pc == base + 1);
  constexpr std::array<std::uint16_t, 3> delayed_call{0x06be, 0x0418, 0x0060};
  Fixture delayed;
  code(delayed.memory, base, delayed_call);
  ready(delayed.buffer);
  const auto before = delayed.cpu.state();
  const auto pending = delayed.buffer.state();
  rejected([&] { static_cast<void>(delayed.cpu.step(delayed.buffer)); }, "captured slot lengths");
  require(delayed.cpu.state() == before && delayed.buffer.state() == pending);
}

void store_failure_and_independent_retry() {
  auto state = initial();
  constexpr std::uint64_t expected = 0x12345678;
  state.data[0][2].value = expected << 8;
  state.dag[0][0][1] = {.value = data, .known = all_bits};
  state.dag[0][1][2] = {.value = 3, .known = all_bits};
  Fixture fixture(state, false);
  constexpr std::array<std::uint16_t, 1> store{0x92b2}; // DM(I1,M2)=R2, Type3c.
  code(fixture.memory, base, store);
  ready(fixture.buffer);
  const auto before = fixture.cpu.state();
  const auto captured = fixture.buffer.state();
  const auto controller = fixture.control.snapshot();
  rejected([&] { static_cast<void>(fixture.cpu.step(fixture.buffer)); }, "Read-only SHARC region");
  require(fixture.cpu.state() == before && fixture.buffer.state() == captured &&
          fixture.control.snapshot() == controller && !fixture.memory.active());
  lead_a1::SharcMemory other_memory;
  program(other_memory, true); // Different instruction RAM cannot replace the captured store.
  auto other_control = lead_a1::SharcControl::from_snapshot(controller);
  lead_a1::Sharc other(other_memory, before, &other_control);
  lead_a1::SharcVisaBuffer restored(other_memory, captured);
  require(other.step(restored).next_pc == base + 1);
  require(other_memory.read({.address = data}) == expected &&
          other.state().dag[0][0][1].value == data + 3);
}

void double_word_dual_stores() {
  auto state = initial();
  constexpr std::uint32_t pm_data = 0xc0080;
  constexpr std::uint64_t left = 0x12345678;
  constexpr std::uint64_t right = 0x87654321;
  state.data[0][0].value = 7U << 8;
  state.data[0][6].value = left << 8;
  state.data[0][8].value = right << 8;
  state.dag[0][0][1] = {.value = data, .known = all_bits};
  state.dag[0][1][2] = {.value = 1, .known = all_bits};
  state.dag[0][0][10] = {.value = pm_data, .known = all_bits};
  state.dag[0][1][15] = {.value = all_bits, .known = all_bits};
  Fixture fixture(state);
  constexpr std::array<std::uint16_t, 2> dual{0x32ac, 0xbc3f};
  code(fixture.memory, base, dual);
  require(step(fixture).address_units == 2);
  require(fixture.memory.read({.address = data}) == left &&
          fixture.memory.read({.address = pm_data}) == right);
  require(fixture.cpu.state().dag[0][0][1].value == data + 1 &&
          fixture.cpu.state().dag[0][0][10].value == pm_data - 1 &&
          fixture.cpu.state().data[0][0].value == (7U << 8));
  require(fixture.cpu.state().system[6] == state.system[6]); // No COMPUTE in Type1b.
}

void unavailable_input_and_unqualified_forms() {
  Fixture missing;
  const auto before = missing.cpu.state();
  const auto empty = missing.buffer.state();
  rejected([&] { static_cast<void>(missing.cpu.step()); }, "captured IAB input");
  rejected([&] { static_cast<void>(missing.cpu.step(missing.buffer)); },
           "complete IAB instruction");
  require(missing.cpu.state() == before && missing.buffer.state() == empty);
  constexpr std::array<std::uint16_t, 1> multiply{0xc735};
  code(missing.memory, base, multiply);
  ready(missing.buffer);
  const auto held = missing.buffer.state();
  rejected([&] { static_cast<void>(missing.cpu.step(missing.buffer)); }, "short compute operation");
  require(missing.cpu.state() == before && missing.buffer.state() == held);
  constexpr std::array<std::uint16_t, 1> nop{0x0001};
  code(missing.memory, base, nop);
  rejected([&] { static_cast<void>(missing.cpu.step(missing.buffer)); }, "short compute operation");
  lead_a1::SharcVisaBuffer refetched(missing.memory, {.decode = base, .fetch = base});
  ready(refetched);
  require(missing.cpu.step(refetched).next_pc == base + 1);
  Fixture conditional;
  constexpr std::array<std::uint16_t, 3> false_call{0x0680, 0x0418, 0x0060};
  code(conditional.memory, base, false_call);
  require(step(conditional).next_pc == base + 3 && conditional.cpu.state().depth == 0);
}

void remaining_data_only_formats() {
  constexpr std::uint64_t payload = 0x12345678;
  auto state = initial();
  state.data[0][2].value = payload << 8;
  state.data[0][3].value = payload << 8;
  state.dag[0][0][1] = {.value = data, .known = all_bits};
  state.dag[0][1][2] = {.value = 2, .known = all_bits};
  Fixture indexed(state);
  indexed.memory.write({.address = data}, payload);
  constexpr std::array<std::uint16_t, 2> load{0x52be, 0x023f}; // R4=DM(I1,M2), Type3b.
  code(indexed.memory, base, load);
  require(step(indexed).address_units == 2 && indexed.cpu.state().pc == base + 2);
  require(indexed.cpu.state().data[0][4].value == (payload << 8) &&
          indexed.cpu.state().dag[0][0][1].value == data + 2);
  Fixture offset(state);
  constexpr std::array<std::uint16_t, 2> store{0x62be, 0x11bf}; // DM(2,I1)=R3, Type4b.
  code(offset.memory, base, store);
  require(step(offset).address_units == 2);
  require(offset.memory.read({.address = data + 2}) == payload &&
          offset.memory.read({.address = data}) == 0 &&
          offset.cpu.state().dag[0][0][1].value == data);
  Fixture registers(state);
  constexpr std::array<std::uint16_t, 2> move{0x703f, 0x023f}; // R4=R2, Type5b.
  code(registers.memory, base, move);
  require(step(registers).address_units == 2);
  require(registers.cpu.state().data[0][4] == state.data[0][2] &&
          registers.cpu.state().system[6] == state.system[6]);
}

void compressed_loop_body_stays_guarded() {
  Fixture fixture;
  constexpr std::array<std::uint16_t, 4> words{0x0c00, 0x0300, 0x0003, 0x0001};
  code(fixture.memory, base, words);
  require(step(fixture).next_pc == base + 3);
  ready(fixture.buffer);
  const auto before = fixture.cpu.state();
  const auto captured = fixture.buffer.state();
  rejected([&] { static_cast<void>(fixture.cpu.step(fixture.buffer)); },
           "single uncompressed body");
  require(fixture.cpu.state() == before && fixture.buffer.state() == captured);
}

}

int main() try {
  mixed_width_operations();
  signed_immediates();
  short_compute_results();
  short_compare_flags();
  call_and_captured_return();
  redirect_to_sequential_address();
  delayed_return_mixed_slots();
  native_call_visa_return();
  single_uncompressed_loop();
  failures_retain_all_resources();
  store_failure_and_independent_retry();
  double_word_dual_stores();
  unavailable_input_and_unqualified_forms();
  remaining_data_only_formats();
  compressed_loop_body_stays_guarded();
  std::cout << "Captured VISA instruction retirement, PC units and native frames pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
