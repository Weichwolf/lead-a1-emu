// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_fetch.hpp"
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
constexpr std::uint32_t data = 0x80040;
constexpr std::uint32_t all_bits = 0xffffffff;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t full64 = 0xffffffffffffffff;
constexpr std::uint64_t payload = 0x123456789abc;
constexpr std::uint64_t register_value = 0x1122334455;
constexpr std::uint64_t literal_store = 0xb36bfffffffd; // PM(-3,I9)=PX, Type15a.
constexpr std::uint64_t literal_load = 0xb26bfffffffd;  // PX=PM(-3,I9).
constexpr std::uint32_t negative_three = 0xfffffffd;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Immediate-indexed expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing immediate-indexed rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory, std::uint64_t opcode = literal_store) {
  constexpr unsigned int cells = 512;
  constexpr unsigned int words = 128;
  constexpr unsigned int code_words = 64;
  memory.map({.bank = 2, .halfwords = cells, .name = "owned Type15 program"});
  memory.map({.bank = 0, .halfwords = cells, .name = "owned Type15 data"});
  for (unsigned int index = 0; index < code_words; ++index) { code(memory, base + index, 0); }
  constexpr std::uint32_t data_base = 0x80000;
  for (unsigned int index = 0; index < words; ++index) {
    memory.write({.address = data_base + index}, 0);
  }
  code(memory, base, opcode);
}

lead_a1::SharcState initial(std::uint32_t pc = base) {
  auto state = lead_a1::Sharc::reset_reference_214xx(pc);
  state.dag[0][0][9] = {.value = data + 3, .known = all_bits};
  state.px = {.value = payload << 16, .known = full64};
  state.system[6] = {};
  state.system[7] = {};
  state.effective = state.system;
  return state;
}

lead_a1::SharcFetchState sequenced(std::uint32_t entry = base) {
  lead_a1::SharcFetchState state;
  state.flow = lead_a1::SharcFetchFlow{.next = entry};
  return state;
}

lead_a1::SharcFetchEvent advance(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  const auto bus = fetch.address_pm_request(cpu) ? lead_a1::SharcFetchBus::pm_data
                                                 : lead_a1::SharcFetchBus::available;
  return fetch.advance_sequenced(cpu, bus);
}

void literal_forms() {
  for (const auto opcode : {literal_store, literal_load}) {
    lead_a1::SharcMemory memory;
    program(memory, opcode);
    if (opcode == literal_load) { code(memory, data, payload); }
    auto state = initial();
    if (opcode == literal_load) { state.px = {}; }
    lead_a1::Sharc cpu(memory, state);
    require(cpu.step().next_pc == base + 1);
    require(cpu.state().dag == state.dag);
    require(cpu.state().system[6] == state.system[6] && cpu.state().system[7] == state.system[7]);
    require(cpu.state().px == (lead_a1::SharcKnown64{.value = payload << 16, .known = full64}));
    require(memory.read({.address = data, .width = lead_a1::SharcWidth::wide48}) == payload);
  }
}

struct NativeCase {
  bool program;
  bool write;
  bool px;
  bool secondary;
  bool extended;
  unsigned int index;
  std::uint32_t pointer;
  std::uint32_t modifier;
};

std::uint64_t encoding(NativeCase example) {
  const auto reg = example.px ? 0x6bU : 4U;
  constexpr std::uint64_t format = 0xa00000000000;
  constexpr unsigned int group_shift = 44;
  constexpr unsigned int index_shift = 41;
  constexpr unsigned int write_shift = 40;
  constexpr unsigned int reg_shift = 32;
  return format | (std::uint64_t{example.program} << group_shift) |
         (std::uint64_t{example.index} << index_shift) |
         (std::uint64_t{example.write} << write_shift) | (std::uint64_t{reg} << reg_shift) |
         example.modifier;
}

void native_case(NativeCase example) {
  lead_a1::SharcMemory memory;
  program(memory, encoding(example));
  lead_a1::SharcControl control(true);
  constexpr std::uint32_t extended = 0x204;
  if (example.extended) {
    control.write({.size = 4}, extended);
    control.advance_internal_clocks(1);
  }
  auto state = initial();
  const auto bank = example.secondary ? 1U : 0U;
  const auto index = example.index + (example.program ? 8U : 0U);
  if (example.secondary) {
    constexpr std::uint32_t alternate_dag_and_low_data = 0x478;
    state.system[2].value = alternate_dag_and_low_data;
    state.effective = state.system;
  }
  state.dag[bank][0][index] = {.value = example.pointer, .known = all_bits};
  state.data[bank][4] = {.value = register_value, .known = full40};
  const auto data_value = example.extended ? register_value << 8 : register_value >> 8;
  const auto stored = example.px ? payload : data_value;
  const auto width =
      example.px || example.extended ? lead_a1::SharcWidth::wide48 : lead_a1::SharcWidth::normal32;
  if (!example.write) { memory.write({.address = data, .width = width}, stored); }
  lead_a1::Sharc cpu(memory, state, &control);
  require(cpu.step().next_pc == base + 1 && cpu.state().dag == state.dag);
  require(memory.read({.address = data, .width = width}) == stored);
  if (example.px) {
    require(cpu.state().px == state.px);
  } else {
    const auto expected =
        example.write || example.extended ? register_value : register_value & ~std::uint64_t{255};
    require(cpu.state().data[bank][4] ==
            (lead_a1::SharcKnown40{.value = expected, .known = full40}));
  }
}

void native_matrix() {
  struct Offset {
    std::uint32_t pointer;
    std::uint32_t modifier;
  };

  constexpr std::array<Offset, 3> offsets{{
      {.pointer = data, .modifier = 0},
      {.pointer = data + 3, .modifier = negative_three},
      {.pointer = data - 5, .modifier = 5},
  }};
  constexpr unsigned int combinations = 32;
  for (unsigned int flags = 0; flags < combinations; ++flags) {
    for (unsigned int index = 0; index < 8; ++index) {
      for (const auto offset : offsets) {
        native_case({.program = (flags & 1U) != 0,
                     .write = (flags & 2U) != 0,
                     .px = (flags & 4U) != 0,
                     .secondary = (flags & 8U) != 0,
                     .extended = (flags & 16U) != 0,
                     .index = index,
                     .pointer = offset.pointer,
                     .modifier = offset.modifier});
      }
    }
  }
}

void native_rejections() {
  constexpr std::uint64_t long_word = std::uint64_t{1} << 39;
  constexpr std::uint64_t same_dag = 0xb319fffffffd;

  struct Failure {
    std::uint64_t opcode;
    unsigned int kind;
    const char *reason;
  };

  constexpr std::array<Failure, 7> cases{{
      {.opcode = literal_store | long_word, .kind = 0, .reason = "forced-LW immediate-indexed"},
      {.opcode = same_dag, .kind = 0, .reason = "same-DAG"},
      {.opcode = literal_store, .kind = 1, .reason = "Unknown SHARC"},
      {.opcode = literal_store, .kind = 2, .reason = "SIMD data addressing"},
      {.opcode = literal_store, .kind = 3, .reason = "Unknown SHARC PX memory store"},
      {.opcode = literal_store, .kind = 4, .reason = "Unmapped SHARC"},
      {.opcode = literal_store, .kind = 5, .reason = "pre-modify space crossing"},
  }};
  for (const auto failure : cases) {
    lead_a1::SharcMemory memory;
    program(memory, failure.opcode);
    auto state = initial();
    if (failure.kind == 1) { state.dag[0][0][9] = {}; }
    if (failure.kind == 2) {
      state.system[2].value = 0x200000;
      state.effective = state.system;
    }
    if (failure.kind == 3) { state.px = {}; }
    constexpr std::uint32_t unmapped_pointer = 0x90043;
    if (failure.kind == 4) { state.dag[0][0][9].value = unmapped_pointer; }
    constexpr std::uint32_t short_pointer = 0x100002;
    if (failure.kind == 5) { state.dag[0][0][9].value = short_pointer; }
    lead_a1::Sharc cpu(memory, state);
    const auto before = cpu.state();
    rejected([&] { static_cast<void>(cpu.step()); }, failure.reason);
    require(cpu.state() == before && !memory.active());
    require(memory.read({.address = data, .width = lead_a1::SharcWidth::wide48}) == 0);
  }
}

void index_producer_and_pm_stall() {
  constexpr std::uint64_t set_i9 = 0x0f1900080043;
  constexpr std::uint32_t wrong_pointer = 0x90043;
  lead_a1::SharcMemory memory;
  program(memory, set_i9);
  code(memory, base + 1, literal_store);
  auto state = initial();
  state.dag[0][0][9].value = wrong_pointer;
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  unsigned int dag_stalls{};
  unsigned int pm_stalls{};
  unsigned int retired{};
  constexpr unsigned int limit = 20;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc != base + 2; ++clock) {
    const auto event = advance(cpu, fetch);
    if (event.dag_stall) { ++dag_stalls; }
    if (event.stalled && !event.dag_stall) { ++pm_stalls; }
    if (fetch.state().execute) {
      require(cpu.step(fetch).pc == base + retired);
      ++retired;
    }
  }
  require(retired == 2 && dag_stalls == 2 && pm_stalls == 1 && cpu.state().pc == base + 2);
  require(cpu.state().dag[0][0][9] ==
          (lead_a1::SharcKnown32{.value = data + 3, .known = all_bits}));
  require(cpu.state().cache.valid.value != 0 && fetch.state().lookup);
  require(memory.read({.address = data, .width = lead_a1::SharcWidth::wide48}) == payload);
}

void px_read_and_source_precision() {
  for (const bool read : {false, true}) {
    lead_a1::SharcMemory memory;
    program(memory, read ? literal_load : literal_store);
    auto state = initial();
    if (read) {
      state.px = {};
      code(memory, data, payload);
    } else {
      constexpr std::uint64_t native_source = 0xffffffffffff0000;
      state.px.known = native_source;
    }
    lead_a1::Sharc cpu(memory, state);
    lead_a1::SharcFetch fetch(memory, sequenced());
    unsigned int partial_stalls{};
    constexpr unsigned int limit = 12;
    for (unsigned int clock = 0; clock < limit && cpu.state().pc == base; ++clock) {
      const auto event = advance(cpu, fetch);
      if (event.stalled) { ++partial_stalls; }
      if (fetch.state().execute) {
        require(cpu.step(fetch).opcode == (read ? literal_load : literal_store));
      }
    }
    require(cpu.state().pc == base + 1 && partial_stalls == 1 && cpu.state().dag == state.dag);
    const auto knowledge = read ? full64 : state.px.known;
    require(cpu.state().px == (lead_a1::SharcKnown64{.value = payload << 16, .known = knowledge}));
    require(memory.read({.address = data, .width = lead_a1::SharcWidth::wide48}) == payload);
  }
}

void finish_store(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  constexpr unsigned int limit = 24;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc == base; ++clock) {
    if (!fetch.state().execute) { static_cast<void>(advance(cpu, fetch)); }
    if (fetch.state().execute) { require(cpu.step(fetch).opcode == literal_store); }
  }
  require(cpu.state().pc == base + 1);
}

void rebound_capture(unsigned int checkpoint, bool occupied) {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  for (unsigned int clock = 0; clock < checkpoint && cpu.state().pc == base; ++clock) {
    static_cast<void>(advance(cpu, fetch));
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
  }
  if (occupied && cpu.state().pc == base) { static_cast<void>(advance(cpu, fetch)); }
  lead_a1::SharcMemory other_memory;
  program(other_memory);
  if (cpu.state().pc == base + 1) { code(other_memory, data, payload); }
  lead_a1::Sharc other(other_memory, cpu.state());
  lead_a1::SharcFetch transport(other_memory, fetch.state());
  finish_store(cpu, fetch);
  finish_store(other, transport);
  require(cpu.state() == other.state() && fetch.state() == transport.state());
  require(memory.read({.address = data, .width = lead_a1::SharcWidth::wide48}) == payload);
  require(other_memory.read({.address = data, .width = lead_a1::SharcWidth::wide48}) == payload);
  require(cpu.state().dag[0][0][9] ==
          (lead_a1::SharcKnown32{.value = data + 3, .known = all_bits}));
}

void captured_px_ignores_late_index() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  for (unsigned int clock = 0; clock < 3; ++clock) { static_cast<void>(advance(cpu, fetch)); }
  const auto word = fetch.state().decode;
  require(word && word->dag && word->dag->address == data &&
          word->dag->modifier == negative_three && word->dag->width == lead_a1::SharcWidth::wide48);
  auto state = cpu.state();
  state.dag[0][0][9] = {};
  constexpr std::uint32_t bank_bit = 64;
  state.system[2].known &= ~bank_bit;
  state.effective = state.system;
  lead_a1::Sharc restored(memory, state);
  lead_a1::SharcFetch transport(memory, fetch.state());
  finish_store(restored, transport);
  require(restored.state().dag[0][0][9].known == 0);
  require(memory.read({.address = data, .width = lead_a1::SharcWidth::wide48}) == payload);
  auto malformed = fetch.state();
  if (!malformed.decode || !malformed.decode->dag) { throw std::logic_error("Missing Type15 DAG"); }
  malformed.decode->dag->width = lead_a1::SharcWidth::normal32;
  rejected([&] { const lead_a1::SharcFetch invalid(memory, malformed); }, "PX width/domain");
}

void counted_dm_pre_modify() {
  for (const unsigned int count : {2U, 3U, 5U}) {
    lead_a1::SharcMemory memory;
    constexpr std::uint64_t do_count = 0x0c0000000001;
    constexpr std::uint64_t literal_dm_store = 0xa70400000002; // DM(2,I3)=R4.
    constexpr unsigned int count_shift = 24;
    program(memory, do_count | (std::uint64_t{count} << count_shift));
    code(memory, base + 1, literal_dm_store);
    auto state = initial();
    state.dag[0][0][3] = {.value = data - 2, .known = all_bits};
    state.data[0][4] = {.value = register_value, .known = full40};
    constexpr std::uint32_t parent = 0x10c0070;
    state.depth = 1;
    state.pc_stack[0] = parent;
    lead_a1::SharcControl control(true);
    lead_a1::Sharc cpu(memory, state, &control);
    lead_a1::SharcFetch fetch(memory, sequenced());
    unsigned int stores{};
    constexpr unsigned int limit = 32;
    for (unsigned int clock = 0; clock < limit && cpu.state().pc != base + 2; ++clock) {
      static_cast<void>(advance(cpu, fetch));
      if (fetch.state().execute) {
        const auto step = cpu.step(fetch);
        if (step.pc == base + 1) {
          require(step.opcode == literal_dm_store);
          ++stores;
        }
      }
    }
    require(stores == count && cpu.state().pc == base + 2 && cpu.state().dag == state.dag);
    require(cpu.state().depth == 1 && cpu.state().loops_empty == true && !cpu.state().count_loop);
    require(cpu.state().pc_stack[0] == parent);
    require(memory.read({.address = data}) == (register_value >> 8));
  }
}

void captured_complement_store() {
  constexpr std::uint64_t literal_complement = 0xb354fffffffd; // PM(-3,I9)=S4.
  lead_a1::SharcMemory memory;
  program(memory, literal_complement);
  auto state = initial();
  state.data[2][4] = {.value = register_value, .known = full40};
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  constexpr unsigned int limit = 16;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc == base; ++clock) {
    static_cast<void>(advance(cpu, fetch));
    if (fetch.state().execute) { require(cpu.step(fetch).opcode == literal_complement); }
  }
  require(cpu.state().pc == base + 1 && cpu.state().dag == state.dag);
  require(memory.read({.address = data}) == (register_value >> 8));
}

void px_quiet_width_and_owner() {
  constexpr std::uint32_t target = base + 16;
  constexpr std::uint32_t sysctl = 0x30024;
  constexpr std::uint32_t widened = 0x204;
  constexpr std::uint64_t jump_db = 0x063e04000000;
  constexpr std::uint64_t write_control = 0x117000030024;
  lead_a1::SharcMemory memory;
  program(memory, jump_db | target);
  code(memory, base + 2, write_control);
  code(memory, target, literal_store);
  lead_a1::SharcControl control(true);
  memory.map_io({.first_word = sysctl, .words = 1}, control);
  auto state = initial();
  state.system[0].value = widened;
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state, &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  for (unsigned int clock = 0; clock < 7; ++clock) {
    static_cast<void>(advance(cpu, fetch));
    if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
  }
  require(cpu.state().pc == target && control.state().effective == 4 &&
          control.state().delayed == widened);
  const auto before = cpu.state();
  const auto transport = fetch.state();
  const auto device = control.snapshot();
  control.begin_instruction();
  rejected([&] { static_cast<void>(advance(cpu, fetch)); }, "during a transaction");
  require(cpu.state() == before && fetch.state() == transport && !memory.active());
  control.abort_instruction();
  require(control.snapshot() == device);
  require(advance(cpu, fetch).address_accepted && !fetch.state().execute);
  require(control.state().effective == widened && !control.state().delayed);
  const auto decoded = fetch.state().address;
  require(decoded && decoded->dag && decoded->dag->width == lead_a1::SharcWidth::wide48 &&
          decoded->dag->address == data && cpu.state().pc == target);
  constexpr unsigned int limit = 12;
  for (unsigned int clock = 0; clock < limit && cpu.state().pc == target; ++clock) {
    static_cast<void>(advance(cpu, fetch));
    if (fetch.state().execute) { require(cpu.step(fetch).opcode == literal_store); }
  }
  require(cpu.state().pc == target + 1 && cpu.state().dag == state.dag);
  require(memory.read({.address = data, .width = lead_a1::SharcWidth::wide48}) == payload);
}

}

int main() try {
  literal_forms();
  native_matrix();
  native_rejections();
  index_producer_and_pm_stall();
  px_read_and_source_precision();
  captured_px_ignores_late_index();
  counted_dm_pre_modify();
  captured_complement_store();
  px_quiet_width_and_owner();
  constexpr unsigned int checkpoints = 10;
  for (unsigned int checkpoint = 0; checkpoint < checkpoints; ++checkpoint) {
    for (const bool occupied : {false, true}) { rebound_capture(checkpoint, occupied); }
  }
  std::cout << "Native Type15 offsets and captured PX/cache/quiet-width cases pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
