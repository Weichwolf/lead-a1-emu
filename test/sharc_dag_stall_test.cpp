// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t data_base = 0x80000;
constexpr std::uint32_t full32 = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t load_m0 = 0x0f2000000001;
constexpr std::uint64_t store = 0x543e80800000; // DM(I2,M0)=R1, post, TRUE.
constexpr std::uint32_t payload = 0x12345678;
using Address = std::optional<std::uint32_t>;
using Stages = std::array<Address, 5>;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC DAG stall expectation at line " + std::to_string(where.line()));
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
  throw std::runtime_error("Missing SHARC DAG rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, unsigned int gap = 0) {
  constexpr unsigned int cells = 256;
  constexpr unsigned int instructions = 32;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned DAG stall program"});
  memory.map({.bank = 0, .halfwords = cells, .name = "owned DAG store data"});
  for (unsigned int index = 0; index < instructions; ++index) { code(memory, base + index, 0); }
  code(memory, base, load_m0);
  code(memory, base + gap + 1, store);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.dag[0][0][2] = {.value = data_base, .known = full32};
  state.data[0][1] = {.value = std::uint64_t{payload} << 8, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = base};
  return state;
}

Address address(const std::optional<lead_a1::SharcFetchWord> &word) {
  return word ? Address{word->address - base} : Address{};
}

Stages stages(const lead_a1::SharcFetch &fetch) {
  const auto &state = fetch.state();
  return {state.fetch1 ? Address{*state.fetch1 - base} : Address{},
          address(state.fetch2),
          address(state.decode),
          address(state.address),
          address(state.execute)};
}

void manual_tables() {
  const Address empty;
  // Tables4-51/52 begin when M0=1 is in Decode/Address respectively.
  const std::array<std::array<Stages, 6>, 2> expected{{
      {{{2, 1, 0, empty, empty},
        {2, 1, empty, 0, empty},
        {2, 1, empty, empty, 0},
        {3, 2, 1, empty, empty},
        {4, 3, 2, 1, empty},
        {5, 4, 3, 2, 1}}},
      {{{3, 2, 1, 0, empty},
        {3, 2, empty, 1, 0},
        {4, 3, 2, empty, 1},
        {5, 4, 3, 2, empty},
        {6, 5, 4, 3, 2},
        {7, 6, 5, 4, 3}}},
  }};
  for (const unsigned int gap : {0U, 1U}) {
    lead_a1::SharcMemory memory;
    program(memory, gap);
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, initial(), &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    for (unsigned int clock = 0; clock < gap + 2; ++clock) {
      static_cast<void>(fetch.advance_sequenced(cpu));
      require(!fetch.state().execute);
    }
    for (unsigned int column = 0; column < expected[gap].size(); ++column) {
      const auto event = fetch.advance_sequenced(cpu);
      require(stages(fetch) == expected[gap][column]);
      const bool stall = column == 1 || (gap == 0 && column == 2);
      require(event.stalled == stall && event.address_accepted != stall);
      require(event.dag_stall == stall);
      require(!event.loop_stall && !event.control_stall);
      if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
    }
    require(cpu.state().dag[0][0][2].value == data_base + 1);
    require(memory.read({.address = data_base}) == payload);
  }
}

class Input final : public lead_a1::MemoryDevice {
public:
  explicit Input(std::uint32_t input) : value(input) {}

  std::uint32_t value;
  unsigned int reads{};
  std::optional<unsigned int> staged;

  std::uint64_t read(lead_a1::MemoryAccess access) override {
    if (!staged || access.address != 0 || access.size != 4) {
      throw std::runtime_error("Invalid owned DAG input access");
    }
    ++*staged;
    return value;
  }

  void write(lead_a1::MemoryAccess /*access*/, std::uint64_t /*written*/) override {
    throw std::runtime_error("Read-only owned DAG input");
  }

  void begin_instruction() override {
    if (staged) { throw std::runtime_error("Owned DAG input already active"); }
    staged = reads;
  }

  void commit_instruction() noexcept override {
    if (staged) { reads = *staged; }
    staged.reset();
  }

  void abort_instruction() noexcept override { staged.reset(); }
};

enum class Producer : std::uint8_t { immediate, transfer, iop };

std::uint64_t load(Producer producer, unsigned int reg, std::uint32_t value) {
  constexpr unsigned int immediate_register_shift = 32;
  constexpr unsigned int move_register_shift = 23;
  constexpr std::uint64_t immediate = 0x0f0000000000;
  constexpr std::uint64_t from_r3 = 0x703f80000000;
  constexpr std::uint64_t from_iop = 0x100000001000;
  if (producer == Producer::immediate) {
    return immediate | (std::uint64_t{reg} << immediate_register_shift) | value;
  }
  if (producer == Producer::transfer) {
    return from_r3 | (std::uint64_t{reg} << move_register_shift);
  }
  return from_iop | (std::uint64_t{reg} << immediate_register_shift);
}

struct Dependency {
  unsigned int reg;
  std::uint32_t value;
  std::uint32_t target;
  std::uint32_t modifier;
  bool dependent;
};

void dependency_case(Dependency dependency, Producer producer, unsigned int gap, bool secondary) {
  lead_a1::SharcMemory memory;
  program(memory, gap);
  code(memory, base, load(producer, dependency.reg, dependency.value));
  constexpr std::uint32_t input_word = 0x1000;
  Input input(dependency.value);
  memory.map_io({.first_word = input_word, .words = 1}, input);
  constexpr unsigned int words = 16;
  constexpr std::uint32_t marker = 0xbadf00d;
  for (unsigned int index = 0; index < words; ++index) {
    memory.write({.address = data_base + index}, marker);
  }
  auto state = initial();
  const auto bank = secondary ? 1U : 0U;
  state.system[2].value = secondary ? 16U : 0U; // DAG0..3 bank bit4, including M0.
  state.effective = state.system;
  state.dag[bank][0][2] = {.value = data_base, .known = full32};
  state.dag[bank][1][0] = {.value = 2, .known = full32};
  state.data[0][3] = {.value = std::uint64_t{dependency.value} << 8, .known = full40};
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int stalls{};
  unsigned int retired{};
  constexpr unsigned int clocks = 12;
  for (unsigned int clock = 0; clock < clocks; ++clock) {
    const auto event = fetch.advance_sequenced(cpu);
    if (event.dag_stall) { ++stalls; }
    require(event.stalled == event.dag_stall && event.address_accepted != event.dag_stall);
    if (fetch.state().execute) {
      const auto step = cpu.step(fetch);
      require(step.pc == base + retired && step.minimum_core_cycles == 1);
      ++retired;
    }
  }
  const auto expected_stalls = dependency.dependent && gap < 2 ? 2 - gap : 0;
  require(stalls == expected_stalls && retired == clocks - 4 - stalls);
  require(cpu.state().dag[bank][0][2].value == dependency.target + dependency.modifier);
  require(cpu.state().dag[bank][0][2].known == full32);
  require(input.reads == (producer == Producer::iop ? 1U : 0U));
  for (unsigned int index = 0; index < words; ++index) {
    require(memory.read({.address = data_base + index}) ==
            (data_base + index == dependency.target ? payload : marker));
  }
}

void dependency_matrix() {
  const std::array<Dependency, 8> cases{{
      {.reg = 0x12,
       .value = data_base + 4,
       .target = data_base + 4,
       .modifier = 2,
       .dependent = true},
      {.reg = 0x42,
       .value = data_base + 4,
       .target = data_base + 4,
       .modifier = 2,
       .dependent = true},
      {.reg = 0x32, .value = 0, .target = data_base, .modifier = 2, .dependent = true},
      {.reg = 0x20, .value = 1, .target = data_base, .modifier = 1, .dependent = true},
      {.reg = 0x23, .value = 1, .target = data_base, .modifier = 2, .dependent = false},
      {.reg = 0x13, .value = data_base + 4, .target = data_base, .modifier = 2, .dependent = false},
      {.reg = 0x43, .value = data_base + 4, .target = data_base, .modifier = 2, .dependent = false},
      {.reg = 0x33, .value = 0, .target = data_base, .modifier = 2, .dependent = false},
  }};
  for (const auto &item : cases) {
    for (const auto producer : {Producer::immediate, Producer::transfer, Producer::iop}) {
      for (const unsigned int gap : {0U, 1U, 2U}) {
        dependency_case(item, producer, gap, false);
        dependency_case(item, producer, gap, true);
      }
    }
  }
}

void advance(lead_a1::SharcFetch &fetch, lead_a1::Sharc &cpu, unsigned int clocks) {
  for (unsigned int clock = 0; clock < clocks; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
  }
}

void snapshots_and_retained_data() {
  for (unsigned int checkpoint = 3; checkpoint <= 7; ++checkpoint) {
    lead_a1::SharcMemory memory;
    program(memory);
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, initial(), &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    advance(fetch, cpu, checkpoint - 1);
    static_cast<void>(fetch.advance_sequenced(cpu));
    lead_a1::SharcMemory other_memory;
    program(other_memory);
    lead_a1::SharcControl other_control(true);
    lead_a1::Sharc restored(other_memory, cpu.state(), &other_control);
    lead_a1::SharcFetch other(other_memory, fetch.state());
    constexpr unsigned int stop = 10;
    for (unsigned int clock = checkpoint; clock <= stop; ++clock) {
      if (fetch.state().execute) {
        const auto actual = cpu.step(fetch);
        const auto replay = restored.step(other);
        require(actual.pc == replay.pc && actual.opcode == replay.opcode);
      }
      require(cpu.state() == restored.state() && fetch.state() == other.state());
      if (clock != stop) {
        require(fetch.advance_sequenced(cpu) == other.advance_sequenced(restored));
      }
    }
    require(memory.read({.address = data_base}) == payload);
    require(other_memory.read({.address = data_base}) == payload);
  }
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  advance(fetch, cpu, 4); // First DAG stall; F2 retains the original store.
  code(memory, base + 1, 0);
  code(memory, base, 0);
  advance(fetch, cpu, 5);
  require(cpu.state().dag[0][1][0].value == 1 && cpu.state().dag[0][0][2].value == data_base + 1);
  require(memory.read({.address = data_base}) == payload);
}

void failed_producer_and_clock() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr unsigned int modifier0 = 0x20;
  code(memory, base, load(Producer::iop, modifier0, 1));
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  advance(fetch, cpu, 3);
  auto before = fetch.state();
  const auto old_cpu = cpu.state();
  control.begin_instruction();
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "during a transaction");
  require(fetch.state() == before && cpu.state() == old_cpu && !memory.active());
  rejected([&] { control.advance_internal_clocks(1); }, "during a transaction");
  control.abort_instruction();
  require(fetch.advance_sequenced(cpu).dag_stall);
  require(fetch.advance_sequenced(cpu).dag_stall);
  before = fetch.state();
  rejected([&] { static_cast<void>(cpu.step(fetch)); }, "Unmapped SHARC IOP");
  require(fetch.state() == before && cpu.state() == old_cpu && !memory.active());
  code(memory, base, 0);
  Input input(1);
  constexpr std::uint32_t input_word = 0x1000;
  memory.map_io({.first_word = input_word, .words = 1}, input);
  const auto actual = cpu.step(fetch);
  require(actual.opcode == load(Producer::iop, modifier0, 1) && input.reads == 1);
  advance(fetch, cpu, 4);
  require(cpu.state().dag[0][0][2].value == data_base + 1);
  require(memory.read({.address = data_base}) == payload);
}

void immediate_modifier_and_guards() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t immediate_store = 0x64fe08800000;
  code(memory, base + 1, immediate_store);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  for (unsigned int clock = 0; clock < 9; ++clock) {
    require(!fetch.advance_sequenced(cpu).stalled);
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
  }
  require(memory.read({.address = data_base}) == payload);
  require(cpu.state().dag[0][0][2].value == data_base + 1);
  for (const unsigned int reg : {0x12U, 0x32U, 0x42U}) {
    lead_a1::SharcMemory fixture;
    program(fixture);
    const bool length = reg == 0x32;
    code(fixture, base, load(Producer::immediate, reg, length ? 0 : data_base + 4));
    code(fixture, base + 1, immediate_store);
    lead_a1::Sharc other_cpu(fixture, initial(), &control);
    lead_a1::SharcFetch other(fixture, sequenced());
    unsigned int stalls{};
    for (unsigned int clock = 0; clock < 10; ++clock) {
      if (other.advance_sequenced(other_cpu).dag_stall) { ++stalls; }
      if (other.state().execute) { static_cast<void>(other_cpu.step(other)); }
    }
    const auto target = length ? data_base : data_base + 4;
    require(stalls == 2 && fixture.read({.address = target}) == payload);
    require(other_cpu.state().dag[0][0][2].value == target + 1);
  }
  constexpr std::array<std::uint64_t, 4> blocked{
      0x0c0005000001, 0x063e000c0010, 0x0f7200000010, 0x126b00080000};
  for (const auto opcode : blocked) {
    lead_a1::SharcMemory fixture;
    program(fixture, 1);
    code(fixture, base + 1, opcode);
    lead_a1::Sharc blocked_cpu(fixture, initial(), &control);
    lead_a1::SharcFetch blocked_fetch(fixture, sequenced());
    advance(blocked_fetch, blocked_cpu, 4);
    const auto old_fetch = blocked_fetch.state();
    const auto old_state = blocked_cpu.state();
    rejected([&] { static_cast<void>(blocked_fetch.advance_sequenced(blocked_cpu)); },
             "not qualified");
    require(blocked_fetch.state() == old_fetch && blocked_cpu.state() == old_state &&
            !fixture.active());
  }
}

void conditional_and_cache_guards() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  advance(fetch, cpu, 3);
  constexpr std::uint64_t condition_bits = std::uint64_t{31} << 33;
  constexpr std::uint64_t long_word = std::uint64_t{1} << 30;
  constexpr std::uint64_t dual = 0x240200000000;
  const std::array<std::uint64_t, 4> unsupported{
      store & ~condition_bits, store | 1, store | long_word, dual};
  for (const auto opcode : unsupported) {
    auto state = fetch.state();
    state.fetch2 = lead_a1::SharcFetchWord{.address = base + 1, .opcode = opcode};
    lead_a1::SharcFetch unqualified(memory, state);
    const auto old_cpu = cpu.state();
    rejected([&] { static_cast<void>(unqualified.advance_sequenced(cpu)); },
             "DAG stall not qualified");
    require(unqualified.state() == state && cpu.state() == old_cpu && !memory.active());
  }
  auto state = fetch.state();
  constexpr std::uint64_t if_eq_m0_from_r3 = 0x700190000000;
  state.decode = lead_a1::SharcFetchWord{.address = base, .opcode = if_eq_m0_from_r3};
  lead_a1::SharcFetch conditional(memory, state);
  rejected([&] { static_cast<void>(conditional.advance_sequenced(cpu)); },
           "conditional DAG producer");
  require(conditional.state() == state && cpu.state() == initial());

  lead_a1::SharcMemory other_memory;
  program(other_memory, 1);
  constexpr std::uint64_t load_px = 0x126b00080000;
  code(other_memory, base, load_px);
  code(other_memory, base + 1, load_m0);
  lead_a1::Sharc other_cpu(other_memory, initial());
  lead_a1::SharcFetch other(other_memory, sequenced());
  advance(other, other_cpu, 3);
  require(other.advance_sequenced(other_cpu, lead_a1::SharcFetchBus::pm_data).address_accepted);
  const auto old_cache = other_cpu.state();
  const auto old_fetch = other.state();
  rejected([&] { static_cast<void>(other.advance_sequenced(other_cpu)); },
           "DAG stall with loop/branch/PM/cache");
  require(other_cpu.state() == old_cache && other.state() == old_fetch && !other_memory.active());
}

struct DagProgram {
  std::uint64_t producer;
  std::uint64_t operation;
};

void dag2_cache_case(DagProgram instructions) {
  lead_a1::SharcMemory memory;
  program(memory);
  code(memory, base, instructions.producer);
  code(memory, base + 1, instructions.operation);
  auto state = initial();
  state.dag[0][0][10] = {.value = data_base, .known = full32};
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int dag_stalls{};
  unsigned int cache_stalls{};
  constexpr unsigned int clocks = 12;
  for (unsigned int clock = 0; clock < clocks; ++clock) {
    const auto decoded = fetch.state().decode;
    const auto bus = decoded && decoded->opcode == instructions.operation
                         ? lead_a1::SharcFetchBus::pm_data
                         : lead_a1::SharcFetchBus::available;
    const auto event = fetch.advance_sequenced(cpu, bus);
    if (event.dag_stall) { ++dag_stalls; }
    if (event.stalled && !event.dag_stall) { ++cache_stalls; }
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
  }
  require(dag_stalls == 2 && cache_stalls == 1);
  require(cpu.state().dag[0][0][10].value == data_base + 1);
  require(memory.read({.address = data_base}) == payload);
}

void dag2_then_cache_stall() {
  constexpr std::uint64_t load_m8 = 0x0f2800000001;
  constexpr std::uint64_t pm_register_store = 0x543f80800000;
  constexpr std::uint64_t load_i10 = 0x0f1a00080000;
  constexpr std::uint64_t pm_immediate_store = 0x65fe08800000;
  dag2_cache_case({.producer = load_m8, .operation = pm_register_store});
  dag2_cache_case({.producer = load_i10, .operation = pm_immediate_store});
}

}

int main() {
  try {
    manual_tables();
    dependency_matrix();
    snapshots_and_retained_data();
    failed_producer_and_clock();
    immediate_modifier_and_guards();
    conditional_and_cache_guards();
    dag2_then_cache_stall();
    std::cout << "SHARC Decode DAG load stalls passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
