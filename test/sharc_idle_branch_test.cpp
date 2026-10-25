// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_boot.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t base = 0xc0000;
constexpr std::uint32_t target = base + 16;
constexpr std::uint32_t vector = base + 0x30;
constexpr std::uint64_t jump_db = 0x063e04000000;
constexpr std::uint64_t idle = 0x008000000000;
constexpr std::uint64_t clear_mask = 0x0f7b00000000;
constexpr std::uint32_t p1i = 0x1000;
constexpr std::uint64_t full40 = 0xffffffffff;
constexpr std::uint64_t invalid = 0xffffffffffff;
constexpr std::uint64_t rti = 0x0b3e00000000;
constexpr std::uint64_t increment = 0x013e00029770;
constexpr std::uint64_t decrement = 0x013e0002a770;
constexpr std::uint32_t parent = 0x1000000 | (base + 63);
constexpr unsigned int cells = 192;
constexpr unsigned int instructions = 64;
constexpr unsigned int limit = 32;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC delayed IDLE expectation at line " +
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
  throw std::runtime_error("Missing SHARC delayed IDLE rejection: " + reason);
}

void code(lead_a1::SharcMemory &memory, std::uint32_t address, std::uint64_t opcode) {
  memory.write({.address = address, .width = lead_a1::SharcWidth::wide48}, opcode);
}

void program(lead_a1::SharcMemory &memory) {
  memory.map({.bank = 2, .halfwords = cells, .name = "owned delayed IDLE program"});
  for (unsigned int index = 0; index < cells / 3; ++index) { code(memory, base + index, 0); }
  code(memory, base, jump_db | target);
  code(memory, base + 1, idle);
  code(memory, base + 2, clear_mask);
  code(memory, target, decrement);
  code(memory, vector, invalid);
}

lead_a1::SharcState initial() {
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  state.system[2].value = p1i;
  state.system[11].value |= p1i;
  state.effective = state.system;
  state.depth = 1;
  state.pc_stack[0] = parent;
  state.data[0][7] = {.value = 7ULL * 256, .known = full40};
  return state;
}

lead_a1::SharcFetchState sequenced() {
  lead_a1::SharcFetchState transport;
  transport.flow = lead_a1::SharcFetchFlow{.next = base};
  return transport;
}

void retire(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  if (fetch.state().execute) { static_cast<void>(cpu.step(fetch)); }
}

void enter_idle(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  for (unsigned int clock = 0; clock < limit; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    retire(cpu, fetch);
    if (!cpu.state().idle_pc) { continue; }
    require(cpu.state().pc == base + 2 && cpu.state().idle_pc == base + 1);
    const auto branch = cpu.state().branch;
    require(branch && branch->target == target && branch->slots == 1);
    const auto held = fetch.state().address;
    require(held && held->address == base + 2 && held->opcode == clear_mask);
    require(cpu.state().idle_delay_word == clear_mask);
    const auto prefetched = fetch.state().fetch2;
    require(prefetched && prefetched->address == target && prefetched->opcode == decrement);
    return;
  }
  throw std::runtime_error("Missing owned delayed IDLE state");
}

void send_word(lead_a1::SharcSpi &spi, lead_a1::SharcMemory &memory, std::uint32_t value) {
  spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
  for (unsigned int bit = 0; bit < 32; ++bit) {
    spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
    spi.observe(memory, {.selected = true, .clock = true, .mosi = ((value >> bit) & 1) != 0});
  }
}

std::uint32_t replacement_word(unsigned int index) {
  // Owned 64-instruction image; actual serial/DMA performs every RAM replacement.
  std::array<std::uint64_t, instructions> image{};
  image[0] = invalid;
  image[1] = invalid;
  image[2] = invalid;
  image[target - base] = increment;
  image[vector - base] = rti;
  image[vector - base + 1] = invalid;
  image[vector - base + 2] = invalid;
  image[vector - base + 3] = invalid;
  std::uint32_t value{};
  constexpr unsigned int instruction_bytes = 6;
  constexpr unsigned int word_bytes = 4;
  constexpr std::uint64_t byte = 255;
  for (unsigned int offset = 0; offset < word_bytes; ++offset) {
    const auto position = (index * word_bytes) + offset;
    const auto data =
        (image.at(position / instruction_bytes) >> ((position % instruction_bytes) * 8)) & byte;
    value |= static_cast<std::uint32_t>(data) << (offset * 8);
  }
  return value;
}

void dma_over_held_pipeline(lead_a1::SharcMemory &memory, lead_a1::Sharc &cpu) {
  constexpr std::uint32_t loaded_kernel_end = 0x92180;
  lead_a1::SharcSpi spi(lead_a1::SharcBootState{.clock = true, .next_word = loaded_kernel_end});
  constexpr unsigned int words = 96;
  spi.write({.address = 0, .size = 4}, base);
  spi.write({.address = 8, .size = 4}, words);
  const auto held = cpu.state();
  unsigned int written{};
  for (unsigned int index = 0; index < words; ++index) {
    send_word(spi, memory, replacement_word(index));
    written += spi.service_dma(memory);
    require(cpu.state() == held && !spi.state().overflow);
    require(spi.state().interrupt_pending == (index == words - 1));
  }
  require(written == words && spi.state().serial.remaining == 0);
  require(memory.read({.address = base + 2, .width = lead_a1::SharcWidth::wide48}) == invalid);
  require(memory.read({.address = target, .width = lead_a1::SharcWidth::wide48}) == increment);
  require(memory.read({.address = vector, .width = lead_a1::SharcWidth::wide48}) == rti);
  cpu.latch_interrupt(p1i);
  spi.acknowledge_interrupt();
  require(!spi.state().interrupt_pending);
}

void resume_last_slot(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  const auto held = cpu.state();
  const auto held_fetch = fetch.state();
  require(!cpu.service_interrupt(fetch, {.vector_base = base}));
  require(cpu.state() == held && fetch.state() == held_fetch);
  require(cpu.wake_idle() && !cpu.state().idle_pc && cpu.state().idle_delay_word == clear_mask);
  require(cpu.state().branch == held.branch && cpu.state().pc == base + 2);
  require((cpu.state().system[10].value & p1i) != 0 && cpu.state().depth == 1);
  const auto awake = cpu.state();
  rejected([&] { static_cast<void>(cpu.step()); }, "retained IDLE delay word required");
  require(cpu.state() == awake && fetch.state() == held_fetch);
  require(fetch.advance_sequenced(cpu).address_accepted);
  require(cpu.step(fetch).opcode == clear_mask);
  require(cpu.state().pc == target && !cpu.state().branch && !cpu.state().idle_delay_word);
  require(cpu.state().system[11].value == 3 && cpu.state().effective[11].value == (p1i | 3));
  const auto stale = fetch.state().decode;
  require(stale && stale->address == target && stale->opcode == decrement);
}

void recognize_and_finish(lead_a1::Sharc &cpu, lead_a1::SharcFetch &fetch) {
  const auto event = cpu.service_interrupt(fetch, {.vector_base = base});
  require(event && event->return_pc == target && event->vector == vector);
  require(cpu.state().pc == vector && cpu.state().depth == 2 && cpu.state().interrupt);
  require(cpu.state().pc_stack[1] == (0x2000000 | target));
  require(cpu.state().effective[11].value == 3 && !cpu.state().pending[11]);
  const auto transport = fetch.state();
  require(!transport.fetch1 && !transport.fetch2 && !transport.decode && !transport.address &&
          !transport.execute);
  require(transport.flow && transport.flow->next == vector);
  unsigned int retired{};
  for (unsigned int clock = 0; clock < limit; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    if (fetch.state().execute) {
      const auto step = cpu.step(fetch);
      require(step.pc == (retired == 0 ? vector : target));
      require(step.opcode == (retired == 0 ? rti : increment));
      ++retired;
    }
    if (cpu.state().pc != target + 1) { continue; }
    require(retired == 2 && !cpu.state().interrupt && cpu.state().depth == 1);
    require(cpu.state().pc_stack[0] == parent && cpu.state().data[0][7].value == 8ULL * 256);
    require((cpu.state().system[10].value & p1i) == 0);
    return;
  }
  throw std::runtime_error("Missing owned IRQ/RTI/refetched target continuation");
}

void real_dma_and_independent_restore() {
  lead_a1::SharcMemory memory;
  lead_a1::SharcMemory rebound;
  program(memory);
  program(rebound);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  enter_idle(cpu, fetch);
  const auto held = cpu.state();
  const auto held_fetch = fetch.state();
  require(!cpu.wake_idle() && cpu.state() == held);
  rejected([&] { static_cast<void>(cpu.step()); }, "IDLE core requires an owner event");
  rejected([&] { static_cast<void>(fetch.advance_sequenced(cpu)); }, "IDLE fetch requires");
  require(cpu.state() == held && fetch.state() == held_fetch);
  lead_a1::Sharc restored(rebound, held);
  lead_a1::SharcFetch continuation(rebound, held_fetch);
  dma_over_held_pipeline(memory, cpu);
  dma_over_held_pipeline(rebound, restored);
  resume_last_slot(cpu, fetch);
  resume_last_slot(restored, continuation);
  recognize_and_finish(cpu, fetch);
  recognize_and_finish(restored, continuation);
  require(cpu.state() == restored.state() && fetch.state() == continuation.state());
}

void masked_and_unknown_wake() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  enter_idle(cpu, fetch);
  for (const bool global : {false, true}) {
    auto state = cpu.state();
    state.system[global ? 2 : 11].value = global ? 0U : 3U;
    state.effective = state.system;
    lead_a1::Sharc masked(memory, state);
    masked.latch_interrupt(p1i);
    const auto before = masked.state();
    require(!masked.wake_idle() && masked.state() == before);
  }
  auto state = cpu.state();
  state.system[11].value &= ~p1i;
  state.system[11].known &= ~p1i;
  state.effective = state.system;
  lead_a1::Sharc unknown(memory, state);
  unknown.latch_interrupt(p1i);
  const auto before = unknown.state();
  rejected([&] { static_cast<void>(unknown.wake_idle()); }, "Unknown SHARC IDLE wake control");
  require(unknown.state() == before);
}

void recognition_ownership_and_binding() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::SharcControl control(true);
  lead_a1::Sharc cpu(memory, initial(), &control);
  lead_a1::SharcFetch fetch(memory, sequenced());
  enter_idle(cpu, fetch);
  cpu.latch_interrupt(p1i);
  resume_last_slot(cpu, fetch);
  const auto before = cpu.state();
  const auto before_fetch = fetch.state();
  const auto before_control = control.snapshot();
  rejected([&] { static_cast<void>(cpu.service_interrupt(fetch, {.vector_base = base + 1})); },
           "explicit internal native IVT");
  require(cpu.state() == before && fetch.state() == before_fetch);
  lead_a1::SharcMemory other;
  program(other);
  lead_a1::SharcFetch foreign(other, sequenced());
  rejected([&] { static_cast<void>(cpu.service_interrupt(foreign, {.vector_base = base})); },
           "idle bound memory");
  control.begin_instruction();
  rejected([&] { static_cast<void>(cpu.service_interrupt(fetch, {.vector_base = base})); },
           "during a transaction");
  require(cpu.state() == before && fetch.state() == before_fetch);
  require(control.state() == before_control);
  control.abort_instruction();
  require(cpu.service_interrupt(fetch, {.vector_base = base}).has_value());
}

void retained_rti_decision_and_changed_frame() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t if_ne_rti = 0x0b2000000000;
  code(memory, vector, if_ne_rti);
  auto state = initial();
  state.pc = target;
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  cpu.latch_interrupt(p1i);
  require(cpu.service_interrupt(fetch, {.vector_base = base}).has_value());
  for (unsigned int clock = 0; clock < 4; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
  }
  const auto addressed = fetch.state().address;
  require(addressed && addressed->branch && addressed->branch->target == target);
  auto changed = cpu.state();
  if (!changed.interrupt) { throw std::runtime_error("Missing owned IRQ frame"); }
  changed.interrupt->return_pc = target + 1;
  changed.pc_stack[1] = 0x2000000 | (target + 1);
  lead_a1::Sharc different(memory, changed);
  static_cast<void>(fetch.advance_sequenced(different));
  const auto before_fetch = fetch.state();
  const auto before_cpu = different.state();
  rejected([&] { static_cast<void>(different.step(fetch)); }, "interrupt frame changed");
  require(different.state() == before_cpu && fetch.state() == before_fetch);
  auto unknown = cpu.state();
  unknown.system[6] = {};
  unknown.effective[6] = {};
  lead_a1::Sharc restored(memory, unknown);
  require(restored.step(fetch).opcode == if_ne_rti && restored.state().pc == target);
}

void false_rti_does_not_read_frame() {
  lead_a1::SharcMemory memory;
  program(memory);
  constexpr std::uint64_t if_ne_rti = 0x0b2000000000;
  code(memory, base, if_ne_rti);
  auto state = initial();
  state.depth.reset();
  state.pc_stack[0].reset();
  state.system[6].value = 1;
  state.effective = state.system;
  lead_a1::Sharc cpu(memory, state);
  lead_a1::SharcFetch fetch(memory, sequenced());
  for (unsigned int clock = 0; clock < 5; ++clock) {
    static_cast<void>(fetch.advance_sequenced(cpu));
    retire(cpu, fetch);
  }
  require(cpu.state().pc == base + 1 && !cpu.state().depth && !cpu.state().interrupt);
}

void invalid_retained_word_states() {
  lead_a1::SharcMemory memory;
  program(memory);
  lead_a1::Sharc cpu(memory, initial());
  lead_a1::SharcFetch fetch(memory, sequenced());
  enter_idle(cpu, fetch);
  auto state = cpu.state();
  state.idle_delay_word.reset();
  rejected([&] { const lead_a1::Sharc invalid_cpu(memory, state); }, "IDLE/resume state");
  state = cpu.state();
  constexpr unsigned int native_bits = 48;
  state.idle_delay_word = std::uint64_t{1} << native_bits;
  rejected([&] { const lead_a1::Sharc invalid_cpu(memory, state); }, "retained IDLE delay word");
  state = cpu.state();
  state.branch = lead_a1::SharcBranch{.target = target, .slots = 2};
  rejected([&] { const lead_a1::Sharc invalid_cpu(memory, state); }, "retained IDLE delay word");
  state = cpu.state();
  state.pending[11] = state.system[11];
  rejected([&] { const lead_a1::Sharc invalid_cpu(memory, state); }, "effect before retained IDLE");
}

}

int main() try {
  real_dma_and_independent_restore();
  masked_and_unknown_wake();
  recognition_ownership_and_binding();
  retained_rti_decision_and_changed_frame();
  false_rti_does_not_read_frame();
  invalid_retained_word_states();
  std::cout << "SHARC delayed IDLE, real DMA, deferred IRQ and retained RTI passed\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
