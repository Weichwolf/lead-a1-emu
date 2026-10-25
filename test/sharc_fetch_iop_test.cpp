// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_iop_bridge.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t native = 0xc0000;
constexpr std::uint32_t visa = 0x180080;
constexpr std::uint32_t address = 0x2478;
constexpr std::uint32_t first_value = 0x11112222;
constexpr std::uint32_t second_value = 0x33334444;
constexpr std::uint32_t third_value = 0x55556666;
constexpr std::uint32_t sysctl = 0x30024;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint32_t all_source_bits = 0xffffffff;
constexpr lead_a1::SharcIoMapping window{.first_word = address, .words = 2};
using Kind = lead_a1::AccessKind;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Fetched IOP expectation at line " + std::to_string(where.line()));
  }
}

template <typename Optional> auto &present(Optional &value) {
  if (!value) { throw std::runtime_error("Missing fetched IOP test state"); }
  return *value;
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing fetched IOP rejection: " + reason);
}

struct RegisterState {
  std::array<std::uint32_t, 2> words{};
  unsigned int reads{};
  unsigned int writes{};
};

struct Registers final : lead_a1::MemoryDevice {
  RegisterState current;
  std::optional<RegisterState> staged;
  bool fail_read{};

  RegisterState &working() { return staged ? *staged : current; }

  std::uint64_t read(lead_a1::MemoryAccess access) override {
    auto &state = working();
    ++state.reads;
    const auto value = state.words.at(access.address / 4);
    if (fail_read) { throw std::runtime_error("owned IOP target read failure"); }
    return value;
  }

  void write(lead_a1::MemoryAccess access, std::uint64_t value) override {
    auto &state = working();
    ++state.writes;
    state.words.at(access.address / 4) = static_cast<std::uint32_t>(value);
  }

  void begin_instruction() override {
    if (staged) { throw std::logic_error("nested owned IOP target"); }
    staged = current;
  }

  void commit_instruction() noexcept override {
    if (staged) { current = *staged; }
    staged.reset();
  }

  void abort_instruction() noexcept override { staged.reset(); }
};

lead_a1::SharcState initial(std::uint32_t entry) {
  auto state = lead_a1::Sharc::reset_reference_214xx(entry);
  for (auto &reg : state.data[0]) { reg = {.known = full40}; }
  state.data[0][0] = {.value = std::uint64_t{first_value} << 8, .known = full40};
  state.data[0][1] = {.value = std::uint64_t{second_value} << 8, .known = full40};
  state.data[0][2] = {.value = 4U << 8, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced(std::uint32_t entry) {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = entry};
  return state;
}

std::uint64_t move(Kind operation, unsigned int reg, std::uint32_t word = address) {
  constexpr std::uint64_t read = 0x100000000000;
  constexpr std::uint64_t write = 0x110000000000;
  return (operation == Kind::read ? read : write) | (std::uint64_t{reg} << 32) | word;
}

struct Fixture {
  bool short_words;
  Registers target;
  lead_a1::SharcMemory peripheral;
  lead_a1::SharcMemory memory;
  lead_a1::SharcIopBridge bus;
  lead_a1::SharcControl control{true};
  lead_a1::Sharc cpu;
  lead_a1::SharcFetch fetch;

  explicit Fixture(bool short_mode = false)
      : Fixture(short_mode,
                {},
                initial(short_mode ? visa : native),
                sequenced(short_mode ? visa : native)) {}

  Fixture(bool short_mode,
          lead_a1::SharcIopState saved,
          lead_a1::SharcState core,
          lead_a1::SharcFetchState transport)
      : short_words(short_mode),
        bus(memory, peripheral, window, saved),
        cpu(memory, core, &control),
        fetch(memory, transport) {
    constexpr unsigned int cells = 512;
    constexpr std::uint32_t first_short = 0x180000;
    memory.map({.bank = 2, .halfwords = cells, .name = "owned fetched IOP code"});
    for (unsigned int index = 0; index < cells; ++index) {
      memory.write({.address = first_short + index, .width = lead_a1::SharcWidth::short16}, 0);
    }
    peripheral.map_io(window, target);
    memory.map_io(window, bus);
    memory.map_io({.first_word = sysctl, .words = 1}, control);
  }

  [[nodiscard]] unsigned int units() const { return short_words ? 3U : 1U; }

  [[nodiscard]] std::uint32_t entry() const { return short_words ? visa : native; }

  struct Instruction {
    unsigned int offset;
    std::uint64_t opcode;
  };

  void code(Instruction instruction) {
    const auto [offset, opcode] = instruction;
    const auto pc = entry() + (offset * units());
    if (!short_words) {
      memory.write({.address = pc, .width = lead_a1::SharcWidth::wide48}, opcode);
      return;
    }
    const std::array words{static_cast<std::uint16_t>(opcode >> 32),
                           static_cast<std::uint16_t>(opcode >> 16),
                           static_cast<std::uint16_t>(opcode)};
    for (unsigned int index = 0; index < words.size(); ++index) {
      memory.write({.address = pc + index, .width = lead_a1::SharcWidth::short16}, words[index]);
    }
  }

  void next() {
    constexpr unsigned int limit = 16;
    for (unsigned int clock = 0; clock < limit; ++clock) {
      static_cast<void>(fetch.advance_sequenced(cpu));
      if (fetch.state().execute) { return; }
      bus.tick();
    }
    throw std::runtime_error("Missing fetched IOP Execute");
  }

  void issue(unsigned int cycles) {
    const auto access = fetch.peripheral_access(cpu);
    if (!access) { throw std::runtime_error("Missing fetched IOP preview"); }
    require(bus.bound_to(memory, access->address));
    bus.issue({.pc = access->pc,
               .address = access->address,
               .operation = access->operation,
               .delivery_cycles = cycles});
  }

  lead_a1::SharcStep read(unsigned int cycles) {
    issue(cycles);
    const auto transport = fetch.state();
    const auto pc = cpu.state().pc;
    while (!bus.ready()) {
      bus.tick();
      if (!bus.ready()) { fetch.wait_iop(cpu, bus); }
      require(fetch.state() == transport && cpu.state().pc == pc);
    }
    return cpu.step(fetch); // The final response clock is also the retirement clock.
  }

  lead_a1::SharcStep write(unsigned int cycles) {
    issue(cycles);
    const auto transport = fetch.state();
    while (!bus.ready()) {
      fetch.wait_iop(cpu, bus);
      bus.tick();
      require(fetch.state() == transport);
    }
    const auto step = cpu.step(fetch);
    bus.tick(); // Write acceptance uses its own CCLK, even after a spacing hold.
    return step;
  }
};

std::uint32_t value(const lead_a1::Sharc &cpu, unsigned int reg) {
  return static_cast<std::uint32_t>(cpu.state().data[0].at(reg).value >> 8);
}

void isolated_read_and_retained_input() {
  for (const bool short_mode : {false, true}) {
    for (const unsigned int cycles : {7U, 8U}) {
      Fixture fixture(short_mode);
      fixture.target.current.words[0] = third_value;
      fixture.code({.offset = 0, .opcode = move(Kind::read, 0)});
      fixture.next();
      const auto start = fixture.bus.state().clocks;
      const auto transport = fixture.fetch.state();
      fixture.issue(cycles);
      constexpr std::uint64_t invalid = 0xffffffffffff;
      fixture.code(
          {.offset = 0, .opcode = invalid}); // Execute must use its real captured instruction.
      for (unsigned int clock = 1; clock < cycles; ++clock) {
        fixture.bus.tick();
        fixture.fetch.wait_iop(fixture.cpu, fixture.bus);
        require(fixture.fetch.state() == transport && fixture.cpu.state().pc == fixture.entry() &&
                value(fixture.cpu, 0) == first_value && fixture.target.current.reads == 0);
      }
      fixture.bus.tick();
      require(fixture.bus.ready() && !fixture.bus.waiting() && fixture.target.current.reads == 1);
      rejected([&fixture] { fixture.fetch.wait_iop(fixture.cpu, fixture.bus); },
               "exact unready bound request");
      const auto step = fixture.cpu.step(fixture.fetch);
      require(step.pc == fixture.entry() && step.address_units == fixture.units() &&
              step.opcode == move(Kind::read, 0) && value(fixture.cpu, 0) == third_value &&
              fixture.bus.state().clocks - start == cycles && !fixture.bus.state().request);
    }
  }
}

void stores_and_atomic_reads() {
  for (const bool short_mode : {false, true}) {
    for (const unsigned int write_cycles : {4U, 5U}) {
      for (const unsigned int read_cycles : {7U, 8U}) {
        Fixture fixture(short_mode);
        fixture.code({.offset = 0, .opcode = move(Kind::write, 0)});
        fixture.code({.offset = 1, .opcode = move(Kind::write, 1, address + 1)});
        fixture.code({.offset = 2, .opcode = move(Kind::read, 2)});
        fixture.code({.offset = 3, .opcode = move(Kind::read, 3, address + 1)});
        fixture.next();
        const auto start = fixture.bus.state().clocks;
        static_cast<void>(fixture.write(write_cycles));
        require(fixture.target.current.writes == 0);
        fixture.next();
        static_cast<void>(fixture.write(write_cycles));
        require(fixture.bus.state().clocks == start + 3 && fixture.target.current.writes == 0);
        fixture.next();
        static_cast<void>(fixture.read(read_cycles));
        require(fixture.bus.state().clocks == start + 2 + write_cycles + read_cycles &&
                fixture.target.current.writes == 2 && fixture.target.current.reads == 1 &&
                value(fixture.cpu, 2) == first_value);
        fixture.next();
        rejected([&fixture] { fixture.issue(7); }, "eight core clocks");
        static_cast<void>(fixture.read(8));
        require(fixture.target.current.reads == 2 && value(fixture.cpu, 3) == second_value &&
                fixture.bus.state().clocks == start + 2 + write_cycles + read_cycles + 8);
      }
    }
  }
}

void status_registers_and_posted_capture() {
  constexpr unsigned int ustat1 = 0x70;
  constexpr unsigned int ustat2 = 0x71;
  constexpr std::uint32_t original = 0x1234;
  constexpr std::uint32_t changed = 0x5678;
  constexpr std::uint32_t full32 = 0xffffffff;
  constexpr std::uint64_t load_original = 0x0f7000001234;
  constexpr std::uint64_t load_changed = 0x0f7000005678;
  for (const bool short_mode : {false, true}) {
    Fixture fixture(short_mode);
    fixture.code({.offset = 0, .opcode = load_original});
    fixture.code({.offset = 1, .opcode = move(Kind::write, ustat1)});
    fixture.code({.offset = 2, .opcode = load_changed});
    fixture.code({.offset = 3, .opcode = move(Kind::read, ustat2)});
    fixture.next();
    static_cast<void>(fixture.cpu.step(fixture.fetch));
    fixture.bus.tick();
    fixture.next();
    static_cast<void>(fixture.write(5));
    fixture.next();
    static_cast<void>(fixture.cpu.step(fixture.fetch));
    fixture.bus.tick();
    require(fixture.target.current.writes == 0 && fixture.cpu.state().system[0].value == changed);
    fixture.next();
    static_cast<void>(fixture.read(8));
    require(fixture.cpu.state().system[0].value == changed &&
            fixture.cpu.state().system[1] ==
                (lead_a1::SharcKnown32{.value = original, .known = full32}) &&
            fixture.target.current.words[0] == original && fixture.target.current.writes == 1 &&
            fixture.target.current.reads == 1);
  }
}

void unsupported_preview_contexts() {
  constexpr std::array<std::uint64_t, 4> opcodes{
      0x107200002478, 0x108000002478, 0x102e00002478, 0x105000002478};
  for (const auto opcode : opcodes) {
    Fixture fixture;
    fixture.code({.offset = 0, .opcode = opcode});
    fixture.next();
    const auto core = fixture.cpu.state();
    const auto transport = fixture.fetch.state();
    const auto bus = fixture.bus.snapshot();
    rejected([&fixture] { static_cast<void>(fixture.fetch.peripheral_access(fixture.cpu)); },
             "unconditional qualified scalar32 DM transfer");
    rejected([&fixture] { static_cast<void>(fixture.fetch.peripheral_access(fixture.cpu)); },
             "pc=0xc0000 word-address=0x2478 operation=read");
    require(fixture.cpu.state() == core && fixture.fetch.state() == transport &&
            fixture.bus.snapshot() == bus && fixture.target.current.reads == 0);
  }
  Fixture fixture;
  fixture.code({.offset = 0, .opcode = move(Kind::read, 0)});
  fixture.next();
  auto core = fixture.cpu.state();
  constexpr std::uint32_t monitor = 0x100000;
  core.effective[4].value |= monitor;
  lead_a1::Sharc monitored(fixture.memory, core, &fixture.control);
  rejected([&] { static_cast<void>(fixture.fetch.peripheral_access(monitored)); },
           "monitored IOP hold not qualified");
  require(monitored.state() == core && !fixture.bus.state().request);
}

void delayed_effects_and_failed_delivery() {
  for (const bool width : {false, true}) {
    Fixture fixture;
    constexpr std::uint64_t write_sysctl = 0x110200030024;
    constexpr std::uint64_t write_mask = 0x0f7b00001000;
    constexpr std::uint32_t expected_mask = 0x1003;
    fixture.code({.offset = 0, .opcode = width ? write_sysctl : write_mask});
    fixture.code({.offset = 1, .opcode = move(Kind::read, 0)});
    fixture.next();
    static_cast<void>(fixture.cpu.step(fixture.fetch));
    fixture.bus.tick();
    fixture.next();
    fixture.issue(8);
    fixture.target.fail_read = true;
    fixture.bus.tick();
    fixture.fetch.wait_iop(fixture.cpu, fixture.bus);
    if (width) {
      require(!fixture.control.state().delayed && fixture.control.state().effective == 4);
    } else {
      require(!fixture.cpu.state().pending[11] &&
              fixture.cpu.state().effective[11].value == expected_mask);
    }
    constexpr unsigned int remaining_holds = 6;
    for (unsigned int clock = 0; clock < remaining_holds; ++clock) {
      fixture.bus.tick();
      fixture.fetch.wait_iop(fixture.cpu, fixture.bus);
    }
    const auto transport = fixture.fetch.state();
    const auto core = fixture.cpu.state();
    rejected([&fixture] { fixture.bus.tick(); }, "owned IOP target read failure");
    const auto due = fixture.bus.state().clocks;
    require(!fixture.bus.waiting() && !fixture.bus.ready() && fixture.target.current.reads == 0);
    rejected([&fixture] { fixture.fetch.wait_iop(fixture.cpu, fixture.bus); },
             "exact unready bound request");
    require(transport == fixture.fetch.state() && core == fixture.cpu.state());
    fixture.target.fail_read = false;
    fixture.bus.retry_due();
    require(fixture.bus.state().clocks == due && fixture.target.current.reads == 1);
    static_cast<void>(fixture.cpu.step(fixture.fetch));
    require(fixture.target.current.reads == 1 && value(fixture.cpu, 0) == 0);
  }
}

void captured_response_survives_cpu_fault() {
  Fixture fixture(true);
  constexpr unsigned int mode1 = 2;
  constexpr std::uint32_t bank = 0x400;
  auto initial = fixture.cpu.state();
  initial.effective[mode1].known &= ~bank;
  lead_a1::Sharc cpu(fixture.memory, initial, &fixture.control);
  fixture.code({.offset = 0, .opcode = move(Kind::read, 1)});
  fixture.target.current.words[0] = first_value;
  while (!fixture.fetch.state().execute) {
    static_cast<void>(fixture.fetch.advance_sequenced(cpu));
    if (!fixture.fetch.state().execute) { fixture.bus.tick(); }
  }
  const auto access = fixture.fetch.peripheral_access(cpu);
  if (!access) { throw std::runtime_error("Missing real captured read request"); }
  fixture.bus.issue({.pc = access->pc,
                     .address = access->address,
                     .operation = access->operation,
                     .delivery_cycles = 8});
  while (!fixture.bus.ready()) {
    fixture.bus.tick();
    if (!fixture.bus.ready()) { fixture.fetch.wait_iop(cpu, fixture.bus); }
  }
  const auto core = cpu.state();
  const auto transport = fixture.fetch.state();
  const auto captured = fixture.bus.snapshot();
  rejected([&] { static_cast<void>(cpu.step(fixture.fetch)); },
           "Unknown SHARC register bank selection");
  require(cpu.state() == core && fixture.fetch.state() == transport &&
          fixture.bus.snapshot() == captured && fixture.target.current.reads == 1);
  auto repaired = core;
  repaired.effective[mode1].known |= bank;
  lead_a1::Sharc retry(fixture.memory, repaired, &fixture.control);
  fixture.target.current.words[0] = third_value;
  static_cast<void>(retry.step(fixture.fetch));
  require(value(retry, 1) == first_value && fixture.target.current.reads == 1 &&
          !fixture.bus.state().request);
}

void ownership_and_restore() {
  Fixture fixture(true);
  fixture.code({.offset = 0, .opcode = move(Kind::read, 1)});
  fixture.target.current.words[0] = first_value;
  fixture.next();
  const auto core = fixture.cpu.state();
  rejected([&fixture] { fixture.fetch.wait_iop(fixture.cpu, fixture.bus); },
           "exact unready bound request");
  fixture.issue(8);
  for (unsigned int clock = 0; clock < 3; ++clock) {
    fixture.bus.tick();
    fixture.fetch.wait_iop(fixture.cpu, fixture.bus);
  }
  const auto saved = fixture.fetch.state();
  Fixture copy(true, fixture.bus.snapshot(), fixture.cpu.state(), saved);
  copy.target.current = fixture.target.current;
  rejected([&fixture, &copy] { fixture.fetch.wait_iop(fixture.cpu, copy.bus); },
           "exact unready bound request");
  require(fixture.cpu.state() == core && fixture.fetch.state() == saved);
  while (!fixture.bus.ready()) {
    fixture.bus.tick();
    copy.bus.tick();
    if (!fixture.bus.ready()) {
      fixture.fetch.wait_iop(fixture.cpu, fixture.bus);
      copy.fetch.wait_iop(copy.cpu, copy.bus);
    }
  }
  static_cast<void>(fixture.cpu.step(fixture.fetch));
  static_cast<void>(copy.cpu.step(copy.fetch));
  require(fixture.cpu.state() == copy.cpu.state() && fixture.fetch.state() == copy.fetch.state() &&
          fixture.bus.snapshot() == copy.bus.snapshot() && fixture.target.current.reads == 1 &&
          copy.target.current.reads == 1);
}

struct SourceChoice {
  unsigned int reg;
  unsigned int bank_bit;
  bool alternate;
  bool short_mode;
  unsigned int write_cycles;
  unsigned int read_cycles;
};

void set_source(lead_a1::SharcState &state,
                SourceChoice choice,
                std::uint32_t word,
                std::uint32_t known = all_source_bits) {
  const auto bank = choice.alternate ? 1U : 0U;
  constexpr unsigned int dag_first = 0x10;
  constexpr unsigned int complement = 0x50;
  constexpr unsigned int user_status = 0x70;
  if (choice.reg < complement) {
    state.dag[bank][(choice.reg - dag_first) / 16][choice.reg & 15] = {.value = word,
                                                                       .known = known};
  } else if (choice.reg < user_status) {
    constexpr std::uint64_t low_byte = 0xff;
    state.data[bank + 2][choice.reg & 15] = {.value = std::uint64_t{word} << 8,
                                             .known = (std::uint64_t{known} << 8) | low_byte};
  } else {
    state.system[choice.reg - user_status] = {.value = word, .known = known};
    state.effective[choice.reg - user_status] = state.system[choice.reg - user_status];
  }
}

lead_a1::SharcState source_initial(SourceChoice choice) {
  auto state = initial(choice.short_mode ? visa : native);
  constexpr std::uint32_t bank_selectors = 0x4f8;
  for (auto &bank : state.dag) {
    for (auto &kind : bank) {
      for (auto &reg : kind) { reg = {.value = second_value, .known = all_source_bits}; }
    }
  }
  for (auto &bank : state.data) {
    for (auto &reg : bank) { reg = {.value = std::uint64_t{second_value} << 8, .known = full40}; }
  }
  constexpr unsigned int mode1 = 2;
  for (auto *mode : {&state.system[mode1], &state.effective[mode1]}) {
    mode->value &= ~bank_selectors;
    mode->known |= bank_selectors;
    if (choice.alternate) { mode->value |= 1U << choice.bank_bit; }
  }
  set_source(state, choice, first_value);
  return state;
}

void check_source(const lead_a1::SharcState &state, SourceChoice choice) {
  const auto bank = choice.alternate ? 1U : 0U;
  constexpr unsigned int complement = 0x50;
  constexpr unsigned int user_status = 0x70;
  if (choice.reg < complement) {
    const auto kind = (choice.reg - 16) / 16;
    require(state.dag[bank][kind][choice.reg & 15] ==
            (lead_a1::SharcKnown32{.value = third_value, .known = all_source_bits}));
    if (kind == 3) {
      require(state.dag[bank][0][choice.reg & 15] ==
              (lead_a1::SharcKnown32{.value = third_value, .known = all_source_bits}));
    }
  } else if (choice.reg < user_status) {
    require(state.data[bank + 2][choice.reg & 15] ==
            (lead_a1::SharcKnown40{.value = std::uint64_t{third_value} << 8, .known = full40}));
  } else {
    require(state.system[choice.reg - user_status] ==
            (lead_a1::SharcKnown32{.value = third_value, .known = all_source_bits}));
  }
}

void one_source(SourceChoice choice) {
  Fixture fixture(
      choice.short_mode, {}, source_initial(choice), sequenced(choice.short_mode ? visa : native));
  constexpr unsigned int ustat3 = 0x7e;
  constexpr unsigned int ustat4 = 0x7f;
  constexpr unsigned int user_status = 0x70;
  constexpr std::uint64_t immediate = 0x0f0000000000;
  const auto destination = (choice.reg & 1) != 0 ? ustat3 : ustat4;
  fixture.code({.offset = 0, .opcode = move(Kind::write, choice.reg)});
  fixture.code(
      {.offset = 1, .opcode = immediate | (std::uint64_t{choice.reg} << 32) | third_value});
  fixture.code({.offset = 2, .opcode = move(Kind::read, destination)});
  fixture.next();
  require(fixture.write(choice.write_cycles).pc == fixture.entry());
  require(fixture.target.current.writes == 0);
  fixture.next();
  require(!fixture.fetch.peripheral_access(fixture.cpu));
  require(fixture.cpu.step(fixture.fetch).pc == fixture.entry() + fixture.units());
  fixture.bus.tick();
  check_source(fixture.cpu.state(), choice);
  const auto posted = fixture.bus.snapshot().writes[0];
  require(posted && posted->value == first_value && fixture.target.current.writes == 0);
  fixture.next();
  require(fixture.read(choice.read_cycles).pc == fixture.entry() + (2 * fixture.units()));
  require(fixture.cpu.state().system[destination - user_status] ==
          (lead_a1::SharcKnown32{.value = first_value, .known = all_source_bits}));
  require(fixture.target.current.writes == 1 && fixture.target.current.reads == 1 &&
          fixture.target.current.words[0] == first_value && fixture.target.current.words[1] == 0);
}

void all_sources() {
  constexpr unsigned int dag_first = 0x10;
  constexpr unsigned int dag_end = 0x50;
  constexpr unsigned int complement_end = 0x60;
  constexpr unsigned int ustat3 = 0x7e;
  constexpr unsigned int ustat4 = 0x7f;
  // PGR A-4: DAG register quadrants select banks through these MODE1 bits.
  constexpr std::array<unsigned int, 4> dag_bits{4, 3, 6, 5};
  unsigned int cases{};
  for (unsigned int reg = dag_first; reg <= ustat4; ++reg) {
    if (reg >= complement_end && reg < ustat3) { continue; }
    auto bit = (reg & 15) < 8 ? 10U : 7U;
    if (reg < dag_end) { bit = dag_bits[(reg & 15) / 4]; }
    for (const bool alternate : {false, true}) {
      for (const bool short_mode : {false, true}) {
        for (const unsigned int writes : {4U, 5U}) {
          for (const unsigned int reads : {7U, 8U}) {
            one_source({.reg = reg,
                        .bank_bit = bit,
                        .alternate = alternate,
                        .short_mode = short_mode,
                        .write_cycles = writes,
                        .read_cycles = reads});
            ++cases;
          }
        }
      }
    }
  }
  constexpr unsigned int expected_cases = 1312;
  require(cases == expected_cases);
}

void failed_source(SourceChoice choice, bool unknown_bank) {
  auto state = source_initial(choice);
  constexpr unsigned int mode1 = 2;
  const auto mask = 1U << choice.bank_bit;
  if (unknown_bank) {
    state.effective[mode1].value &= ~mask;
    state.effective[mode1].known &= ~mask;
  } else {
    set_source(state, choice, 0, 0);
  }
  Fixture fixture(choice.short_mode, {}, state, sequenced(choice.short_mode ? visa : native));
  fixture.code({.offset = 0, .opcode = move(Kind::write, choice.reg)});
  fixture.next();
  fixture.issue(choice.write_cycles);
  const auto core = fixture.cpu.state();
  const auto transport = fixture.fetch.state();
  const auto bus = fixture.bus.snapshot();
  const auto control = fixture.control.snapshot();
  require(fixture.bus.ready());
  rejected([&fixture] { static_cast<void>(fixture.cpu.step(fixture.fetch)); },
           unknown_bank ? "Unknown SHARC register bank selection"
                        : "Unknown SHARC register operand bits");
  require(fixture.cpu.state() == core && fixture.fetch.state() == transport &&
          fixture.bus.snapshot() == bus && fixture.control.snapshot() == control &&
          !fixture.memory.active() && fixture.target.current.writes == 0 &&
          fixture.target.current.reads == 0);
  auto repaired = core;
  if (unknown_bank) {
    repaired.effective[mode1] = source_initial(choice).effective[mode1];
  } else {
    set_source(repaired, choice, first_value);
  }
  lead_a1::Sharc retry(fixture.memory, repaired, &fixture.control);
  lead_a1::SharcFetch rebound(fixture.memory, transport);
  require(retry.step(rebound).pc == fixture.entry());
  const auto posted = fixture.bus.snapshot().writes[0];
  require(posted && posted->value == first_value && fixture.bus.state().clocks == bus.clocks &&
          !fixture.bus.state().request && fixture.target.current.writes == 0);
  for (unsigned int clock = 0; clock < choice.write_cycles; ++clock) { fixture.bus.tick(); }
  require(fixture.target.current.writes == 1 && fixture.target.current.words[0] == first_value &&
          fixture.bus.state().write_count == 0 && fixture.target.current.reads == 0);
}

void unknown_sources_and_banks() {
  constexpr std::array<unsigned int, 8> registers{0x10, 0x24, 0x38, 0x4e, 0x50, 0x5f, 0x7e, 0x7f};
  constexpr std::array<unsigned int, 8> bits{4, 3, 6, 5, 10, 7, 0, 0};
  unsigned int cases{};
  for (unsigned int index = 0; index < registers.size(); ++index) {
    for (const bool alternate : {false, true}) {
      for (const bool short_mode : {false, true}) {
        for (const unsigned int cycles : {4U, 5U}) {
          for (const bool unknown_bank : {false, true}) {
            if (unknown_bank && bits[index] == 0) { continue; }
            failed_source({.reg = registers[index],
                           .bank_bit = bits[index],
                           .alternate = alternate,
                           .short_mode = short_mode,
                           .write_cycles = cycles,
                           .read_cycles = 8},
                          unknown_bank);
            ++cases;
          }
        }
      }
    }
  }
  constexpr unsigned int expected_cases = 112;
  require(cases == expected_cases);
}

}

int main() {
  try {
    all_sources();
    unknown_sources_and_banks();
    isolated_read_and_retained_input();
    stores_and_atomic_reads();
    status_registers_and_posted_capture();
    unsupported_preview_contexts();
    delayed_effects_and_failed_delivery();
    captured_response_survives_cpu_fault();
    ownership_and_restore();
    std::cout << "Fetched IOP request/hold/retirement/timing/fault/restore expectations passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
