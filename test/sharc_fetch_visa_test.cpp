// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "lead_a1/sharc_visa.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0x180080;
constexpr std::uint32_t native = 0xc0000;
constexpr unsigned int cells = 512;
constexpr unsigned int program_words = 48;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t jump = 0x063e00000000;
constexpr std::uint64_t call = 0x06be00000000;
constexpr std::uint64_t increment_r2 = 0x013e00029220;
constexpr std::uint16_t increment = 0xc500;
constexpr std::uint64_t raw_increment = 0xc50000000000;
using Word = std::optional<lead_a1::SharcFetchWord>;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Clocked VISA expectation at line " + std::to_string(where.line()));
  }
}

template <typename Optional> auto &present(Optional &value) {
  if (!value.has_value()) { throw std::runtime_error("Missing clocked VISA test state"); }
  return *value;
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing clocked VISA rejection: " + reason);
}

lead_a1::SharcState initial(std::uint32_t entry = base) {
  auto state = lead_a1::Sharc::reset_reference_214xx(entry);
  for (auto &reg : state.data[0]) { reg = {.known = full40}; }
  return state;
}

lead_a1::SharcFetchState sequenced(std::uint32_t entry) {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = entry};
  return state;
}

struct Fixture {
  lead_a1::SharcMemory memory;
  lead_a1::Sharc cpu;
  lead_a1::SharcFetch fetch;

  explicit Fixture(lead_a1::SharcState state = initial())
      : cpu(memory, state), fetch(memory, sequenced(state.pc)) {
    memory.map({.bank = 2, .halfwords = cells, .name = "owned clocked ISA/VISA program"});
    constexpr std::uint32_t first_short = 0x180000;
    for (unsigned int index = 0; index < cells; ++index) {
      memory.write({.address = first_short + index, .width = lead_a1::SharcWidth::short16}, 1);
    }
    constexpr unsigned int native_words = 32;
    for (unsigned int index = 0; index < native_words; ++index) {
      memory.write({.address = native + index, .width = lead_a1::SharcWidth::wide48}, 0);
    }
  }
};

void code(lead_a1::SharcMemory &memory,
          std::uint32_t address,
          std::span<const std::uint16_t> words) {
  for (unsigned int index = 0; index < words.size(); ++index) {
    memory.write({.address = address + index, .width = lead_a1::SharcWidth::short16}, words[index]);
  }
}

void wide(lead_a1::SharcMemory &memory, lead_a1::SharcVisaInstruction instruction) {
  const std::array words{static_cast<std::uint16_t>(instruction.opcode >> 32),
                         static_cast<std::uint16_t>(instruction.opcode >> 16),
                         static_cast<std::uint16_t>(instruction.opcode)};
  code(memory, instruction.address, words);
}

void phase(const Word &word, std::optional<std::uint32_t> address) {
  require(word.has_value() == address.has_value());
  if (word) { require(word->address == address && word->visa.has_value()); }
}

std::optional<lead_a1::SharcStep> tick(Fixture &fixture) {
  static_cast<void>(fixture.fetch.advance_sequenced(fixture.cpu));
  if (fixture.fetch.state().execute) { return fixture.cpu.step(fixture.fetch); }
  return {};
}

void linear_short() {
  Fixture fixture;
  std::array<std::uint16_t, program_words> program{};
  program.fill(increment);
  code(fixture.memory, base, program);
  // PGR4-9 Table4-3 gives cycles1..12; cycles13/14 continue its three-clock cadence.
  constexpr std::array<unsigned int, 14> fetch1{0, 3, 6, 9, 9, 9, 12, 12, 12, 15, 15, 15, 18, 18};
  constexpr std::array accepted{
      true, true, true, true, false, false, true, false, false, true, false, false, true, false};
  for (unsigned int cycle = 1; cycle <= fetch1.size(); ++cycle) {
    const auto event = fixture.fetch.advance_sequenced(fixture.cpu);
    const auto state = fixture.fetch.state();
    require(event.address_accepted == accepted[cycle - 1] && !event.stalled &&
            event.fetch_stall == !accepted[cycle - 1] && state.fetch1 == base + fetch1[cycle - 1] &&
            !state.fetch2);
    phase(state.decode, cycle >= 3 ? std::optional{base + cycle - 3} : std::nullopt);
    phase(state.address, cycle >= 4 ? std::optional{base + cycle - 4} : std::nullopt);
    phase(state.execute, cycle >= 5 ? std::optional{base + cycle - 5} : std::nullopt);
    if (state.execute) {
      const auto result = fixture.cpu.step(fixture.fetch);
      require(result.pc == base + cycle - 5 && result.address_units == 1 &&
              result.opcode == raw_increment && result.minimum_core_cycles == 1 &&
              fixture.cpu.state().data[0][0].value == (std::uint64_t{cycle - 4} << 8));
    }
    if (cycle == 4) {
      require(present(state.visa).count == 4 && present(state.visa).delay.has_value());
    }
  }
}

void linear_full() {
  Fixture fixture;
  constexpr unsigned int words = 48;
  for (unsigned int index = 0; index < words; ++index) {
    wide(fixture.memory, {.address = base + (index * 3), .opcode = 0});
  }
  // PGR4-8 Table4-2: five stages, one 48-bit instruction each clock.
  constexpr unsigned int clocks = 9;
  for (unsigned int cycle = 1; cycle <= clocks; ++cycle) {
    const auto event = fixture.fetch.advance_sequenced(fixture.cpu);
    const auto &state = fixture.fetch.state();
    require(event.address_accepted && !event.stalled && !event.fetch_stall &&
            state.fetch1 == base + ((cycle - 1) * 3) && !state.fetch2);
    phase(state.decode, cycle >= 3 ? std::optional{base + ((cycle - 3) * 3)} : std::nullopt);
    phase(state.address, cycle >= 4 ? std::optional{base + ((cycle - 4) * 3)} : std::nullopt);
    phase(state.execute, cycle >= 5 ? std::optional{base + ((cycle - 5) * 3)} : std::nullopt);
    if (state.execute) {
      const auto result = fixture.cpu.step(fixture.fetch);
      require(result.address_units == 3 && result.opcode == 0 &&
              result.pc == base + ((cycle - 5) * 3) && result.minimum_core_cycles == 1);
    }
  }
}

void mixed_width() {
  Fixture fixture;
  constexpr std::array<std::uint16_t, 9> program{
      0x0f80, 0xffff, 0xc500, 0x0001, 0x0f01, 0x1234, 0x5678, 0x0182, 0x1210};
  code(fixture.memory, base, program);
  constexpr std::array<unsigned int, 5> offsets{0, 2, 3, 4, 7};
  constexpr std::array<unsigned int, 5> lengths{2, 1, 1, 3, 2};
  constexpr std::array<std::uint64_t, 5> raw{
      0x0f80ffff0000, 0xc50000000000, 0x000100000000, 0x0f0112345678, 0x018212100000};
  constexpr unsigned int warmup = 4;
  for (unsigned int cycle = 0; cycle < warmup + offsets.size(); ++cycle) {
    const auto result = tick(fixture);
    if (cycle < warmup) {
      require(!result);
    } else {
      const auto index = cycle - warmup;
      require(result && result->pc == base + offsets[index] &&
              result->address_units == lengths[index] && result->opcode == raw[index]);
    }
  }
  constexpr std::uint64_t expected = 0x1234567800;
  require(fixture.cpu.state().pc == base + 9 && fixture.cpu.state().data[0][0].value == 0 &&
          fixture.cpu.state().data[0][1].value == expected &&
          fixture.cpu.state().data[0][2].value == expected);
}

void native_visa_calls() {
  Fixture fixture(initial(native));
  fixture.memory.write({.address = native, .width = lead_a1::SharcWidth::wide48}, jump | base);
  constexpr std::uint32_t callee = base + 32;
  wide(fixture.memory, {.address = base, .opcode = call | callee});
  constexpr std::array<std::uint16_t, 2> continuation{0x0f81, 9};
  code(fixture.memory, base + 3, continuation);
  wide(fixture.memory, {.address = base + 5, .opcode = jump | (native + 8)});
  constexpr std::array<std::uint16_t, 3> body{0x0f80, 7, 0x0abe};
  code(fixture.memory, callee, body);
  constexpr std::uint16_t invalid = 0xffff;
  fixture.memory.write({.address = callee + 3, .width = lead_a1::SharcWidth::short16}, invalid);
  fixture.memory.write({.address = base + 8, .width = lead_a1::SharcWidth::short16}, invalid);
  fixture.memory.write({.address = native + 8, .width = lead_a1::SharcWidth::wide48}, increment_r2);
  constexpr std::array<std::uint32_t, 7> pcs{
      native, base, callee, callee + 2, base + 3, base + 5, native + 8};
  constexpr std::array<unsigned int, 7> lengths{1, 3, 2, 1, 2, 3, 1};
  unsigned int retired{};
  constexpr unsigned int clock_limit = 64;
  for (unsigned int cycle = 0; cycle < clock_limit && retired < pcs.size(); ++cycle) {
    const auto result = tick(fixture);
    if (!result) { continue; }
    require(result->pc == pcs[retired] && result->address_units == lengths[retired]);
    if (retired == 1) {
      constexpr std::uint32_t call_frame = 0x1000000;
      require(fixture.cpu.state().depth == 1 &&
              fixture.cpu.state().pc_stack[0] == (call_frame | (base + 3)));
    }
    ++retired;
  }
  require(retired == pcs.size() && fixture.cpu.state().depth == 0 &&
          fixture.cpu.state().data[0][0].value == (7U << 8) &&
          fixture.cpu.state().data[0][1].value == (9U << 8) &&
          fixture.cpu.state().data[0][2].value == (1U << 8));
}

void full_iab_return() {
  auto state = initial();
  constexpr std::uint32_t frame = 0x10c0008;
  state.depth = 1;
  state.pc_stack[0] = frame;
  Fixture fixture(state);
  constexpr std::array<std::uint16_t, 6> program{0x0001, 0x0001, 0x0abe, 0xffff, 0xffff, 0xffff};
  code(fixture.memory, base, program);
  constexpr unsigned int decision_cycle = 6;
  for (unsigned int cycle = 1; cycle < decision_cycle; ++cycle) {
    static_cast<void>(tick(fixture));
  }
  require(present(fixture.fetch.state().visa).delay.has_value());
  const auto event = fixture.fetch.advance_sequenced(fixture.cpu);
  require(event.address_accepted && !event.fetch_stall && !fixture.fetch.state().visa &&
          fixture.fetch.state().visa_fetch_aborted &&
          present(fixture.fetch.state().flow).next == native + 8);
  require(fixture.cpu.step(fixture.fetch).pc == base + 1);
  const auto returned = tick(fixture);
  require(returned && returned->pc == base + 2 && returned->next_pc == native + 8 &&
          fixture.cpu.state().depth == 0);
  constexpr unsigned int target_clocks = 4;
  for (unsigned int cycle = 0; cycle < target_clocks; ++cycle) {
    const auto result = tick(fixture);
    if (cycle + 1 == target_clocks) {
      require(result && result->pc == native + 8);
    } else {
      require(!result);
    }
  }
}

void immutable_capture() {
  Fixture fixture;
  std::array<std::uint16_t, program_words> program{};
  program.fill(increment);
  code(fixture.memory, base, program);
  constexpr unsigned int clocks = 14;
  for (unsigned int cycle = 1; cycle <= clocks; ++cycle) {
    static_cast<void>(tick(fixture));
    if (cycle == 2 || cycle == 4) {
      const auto start = cycle == 2 ? base : base + 6;
      constexpr std::array<std::uint16_t, 3> invalid{0xffff, 0xffff, 0xffff};
      code(fixture.memory, start, invalid);
    }
  }
  require(fixture.cpu.state().data[0][0].value == (10U << 8));
}

void failure_and_snapshot() {
  Fixture fixture;
  constexpr std::array<std::uint16_t, 3> invalid{0xffff, 0xffff, 0xffff};
  code(fixture.memory, base, invalid);
  static_cast<void>(tick(fixture));
  static_cast<void>(tick(fixture));
  const auto core = fixture.cpu.state();
  const auto transport = fixture.fetch.state();
  rejected([&fixture] { static_cast<void>(fixture.fetch.advance_sequenced(fixture.cpu)); },
           "Unsupported VISA length prefix");
  require(core == fixture.cpu.state() && transport == fixture.fetch.state() &&
          !fixture.memory.active());
  auto malformed = transport;
  present(malformed.visa).fetch += 1;
  rejected([&fixture, malformed] { const lead_a1::SharcFetch bad(fixture.memory, malformed); },
           "Invalid VISA buffer continuity");
  malformed = transport;
  malformed.visa_fetch_aborted = true;
  malformed.fetch1 = native;
  rejected([&fixture, malformed] { const lead_a1::SharcFetch bad(fixture.memory, malformed); },
           "Invalid SHARC aborted VISA");
}

void dag_dependency() {
  auto state = initial();
  constexpr std::uint32_t data = 0x100080;
  constexpr std::uint32_t full32 = 0xffffffff;
  state.dag[0][0][2] = {.value = data, .known = full32};
  constexpr std::uint32_t payload = 0x1234;
  state.data[0][1] = {.value = std::uint64_t{payload} << 8, .known = full40};
  Fixture fixture(state);
  fixture.memory.map({.bank = 0, .halfwords = cells, .name = "owned VISA DAG data"});
  constexpr std::uint64_t load_m0 = 0x0f2000000001;
  constexpr std::uint64_t store = 0x543e80800000;
  wide(fixture.memory, {.address = base, .opcode = load_m0});
  wide(fixture.memory, {.address = base + 3, .opcode = store});
  constexpr unsigned int clocks = 8;
  unsigned int retired{};
  for (unsigned int cycle = 1; cycle <= clocks; ++cycle) {
    const auto before = fixture.fetch.state();
    const auto event = fixture.fetch.advance_sequenced(fixture.cpu);
    const bool stall = cycle == 4 || cycle == 5;
    require(event.dag_stall == stall && event.stalled == stall);
    if (stall) {
      require(fixture.fetch.state().visa == before.visa &&
              fixture.fetch.state().fetch1 == before.fetch1 && !fixture.fetch.state().decode);
    }
    if (fixture.fetch.state().execute) {
      const auto step = fixture.cpu.step(fixture.fetch);
      require(step.pc == base + (retired * 3) && step.address_units == 3);
      ++retired;
    }
  }
  require(retired == 2 &&
          fixture.memory.read({.address = data, .width = lead_a1::SharcWidth::short16}) ==
              payload &&
          fixture.cpu.state().dag[0][0][2].value == data + 1);
}

void restored_full_iab() {
  Fixture fixture;
  std::array<std::uint16_t, program_words> program{};
  program.fill(increment);
  code(fixture.memory, base, program);
  constexpr unsigned int fill_clocks = 4;
  for (unsigned int cycle = 0; cycle < fill_clocks; ++cycle) { static_cast<void>(tick(fixture)); }
  lead_a1::Sharc restored(fixture.memory, fixture.cpu.state());
  lead_a1::SharcFetch replay(fixture.memory, fixture.fetch.state());
  constexpr unsigned int clocks = 10;
  for (unsigned int cycle = 0; cycle < clocks; ++cycle) {
    const auto expected = fixture.fetch.advance_sequenced(fixture.cpu);
    require(replay.advance_sequenced(restored) == expected);
    const auto real = fixture.cpu.step(fixture.fetch);
    const auto copied = restored.step(replay);
    require(real.pc == base + cycle && real.address_units == 1 && copied.pc == real.pc &&
            copied.opcode == raw_increment && copied.address_units == 1 &&
            restored.state() == fixture.cpu.state() && replay.state() == fixture.fetch.state());
  }
  require(restored.state().data[0][0].value == (10U << 8));
}

void retained_failed_execute() {
  auto state = initial();
  state.data[0][0] = {};
  Fixture fixture(state);
  constexpr std::array<std::uint16_t, 1> program{increment};
  code(fixture.memory, base, program);
  constexpr unsigned int clocks = 5;
  for (unsigned int cycle = 0; cycle < clocks; ++cycle) {
    static_cast<void>(fixture.fetch.advance_sequenced(fixture.cpu));
  }
  const auto core = fixture.cpu.state();
  const auto transport = fixture.fetch.state();
  rejected([&fixture] { static_cast<void>(fixture.cpu.step(fixture.fetch)); }, "Unknown SHARC");
  require(fixture.cpu.state() == core && fixture.fetch.state() == transport &&
          !fixture.memory.active());
  constexpr std::array<std::uint16_t, 1> invalid{0xffff};
  code(fixture.memory, base, invalid);
  auto repaired = core;
  repaired.data[0][0] = {.known = full40};
  lead_a1::Sharc retry(fixture.memory, repaired);
  const auto step = retry.step(fixture.fetch);
  require(step.pc == base && step.opcode == raw_increment && step.address_units == 1 &&
          retry.state().data[0][0].value == (1U << 8) && !fixture.fetch.state().execute);
}

void explicit_boundaries() {
  constexpr std::array<std::uint64_t, 2> operations{0x0c0002000003, 0x06be041800a0};
  constexpr std::array reasons{"VISA single-body input alignment/length",
                               "clocked VISA delayed branch"};
  for (unsigned int index = 0; index < operations.size(); ++index) {
    Fixture fixture;
    wide(fixture.memory, {.address = base, .opcode = operations[index]});
    for (unsigned int cycle = 0; cycle < 3; ++cycle) { static_cast<void>(tick(fixture)); }
    const auto core = fixture.cpu.state();
    const auto transport = fixture.fetch.state();
    rejected([&fixture] { static_cast<void>(fixture.fetch.advance_sequenced(fixture.cpu)); },
             reasons[index]);
    require(fixture.cpu.state() == core && fixture.fetch.state() == transport);
  }
  Fixture fixture;
  constexpr std::uint64_t pm_read = 0x1200000c0010;
  wide(fixture.memory, {.address = base, .opcode = pm_read});
  for (unsigned int cycle = 0; cycle < 3; ++cycle) { static_cast<void>(tick(fixture)); }
  require(fixture.fetch.address_pm_request(fixture.cpu));
  const auto transport = fixture.fetch.state();
  rejected(
      [&fixture] {
        static_cast<void>(
            fixture.fetch.advance_sequenced(fixture.cpu, lead_a1::SharcFetchBus::pm_data));
      },
      "VISA cache/PM/deferred/loop");
  require(fixture.fetch.state() == transport);
}

void malformed_snapshots() {
  Fixture fixture;
  constexpr unsigned int clocks = 4;
  for (unsigned int cycle = 0; cycle < clocks; ++cycle) { static_cast<void>(tick(fixture)); }
  const auto transport = fixture.fetch.state();
  auto bad = transport;
  present(bad.decode).visa.reset();
  rejected([&fixture, bad] { const lead_a1::SharcFetch invalid(fixture.memory, bad); },
           "Missing or unexpected SHARC captured VISA");
  bad = transport;
  present(bad.decode).opcode = 1;
  rejected([&fixture, bad] { const lead_a1::SharcFetch invalid(fixture.memory, bad); },
           "Invalid SHARC captured VISA semantic");
  bad = transport;
  present(present(bad.decode).visa).words = 2;
  rejected([&fixture, bad] { const lead_a1::SharcFetch invalid(fixture.memory, bad); },
           "length disagrees with encoding");
  bad = transport;
  present(present(bad.decode).visa).address += 1;
  rejected([&fixture, bad] { const lead_a1::SharcFetch invalid(fixture.memory, bad); },
           "Invalid SHARC captured VISA semantic");
  bad = transport;
  present(bad.visa).data.back() = 1;
  rejected([&fixture, bad] { const lead_a1::SharcFetch invalid(fixture.memory, bad); },
           "Nonempty unused VISA buffer cell");
}

void native_delayed_call() {
  Fixture fixture(initial(native));
  constexpr std::uint64_t delayed = 0x4000000;
  fixture.memory.write({.address = native, .width = lead_a1::SharcWidth::wide48},
                       call | delayed | base);
  for (unsigned int slot = 1; slot <= 2; ++slot) {
    fixture.memory.write({.address = native + slot, .width = lead_a1::SharcWidth::wide48},
                         increment_r2);
  }
  constexpr std::array<std::uint16_t, 3> body{0x0abe, 0xffff, 0xffff};
  code(fixture.memory, base, body);
  constexpr std::array<std::uint32_t, 5> pcs{native, native + 1, native + 2, base, native + 3};
  constexpr unsigned int clock_limit = 32;
  unsigned int retired{};
  for (unsigned int cycle = 0; cycle < clock_limit && retired < pcs.size(); ++cycle) {
    const auto step = tick(fixture);
    if (!step) { continue; }
    require(step->pc == pcs[retired] && step->address_units == 1);
    ++retired;
  }
  constexpr std::uint32_t frame = 0x10c0003;
  require(retired == pcs.size() && fixture.cpu.state().pc_stack[0] == frame &&
          fixture.cpu.state().depth == 0 && fixture.cpu.state().data[0][2].value == (2U << 8));
}

}

int main() {
  try {
    linear_short();
    linear_full();
    mixed_width();
    native_visa_calls();
    full_iab_return();
    immutable_capture();
    failure_and_snapshot();
    dag_dependency();
    restored_full_iab();
    retained_failed_execute();
    explicit_boundaries();
    malformed_snapshots();
    native_delayed_call();
    std::cout << "Clocked VISA linear/branch/mixed-width/capture expectations passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
