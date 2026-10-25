// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_power.hpp"
#include "sharc/peripherals/sharc_sport.hpp"
#include "sharc/peripherals/sharc_sport_dma.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using Channel = lead_a1::SharcSportChannel;
using Width = lead_a1::SharcWidth;
constexpr std::uint32_t late_rx8 = 0x00023071;
constexpr std::uint32_t late_tx8 = 0x02023071;
constexpr std::uint32_t dual_rx8 = 0x01023071;
constexpr std::string_view byte_bits = "10010110";
constexpr std::uint32_t byte_value = 0x96;
constexpr std::uint32_t inverse_byte = 0x69;
constexpr std::uint32_t ram_address = 0x80020;
constexpr unsigned int rx_a_word = 25;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SPORT expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing SPORT rejection: " + reason);
}

template <typename Function> void check(std::string_view name, const Function &action) {
  try {
    action();
  } catch (const std::exception &error) {
    throw std::runtime_error(std::string(name) + ": " + error.what());
  }
}

struct Fixture {
  lead_a1::SharcPower power;
  lead_a1::SharcMemory memory;
  lead_a1::SharcSport first{power, memory, 0};
  lead_a1::SharcSport second{power, memory, 1};
  lead_a1::SharcSportPair pair{first, second};
  lead_a1::SharcMemoryBus bus{memory, lead_a1::SharcPeripheralPhase::active};
  lead_a1::SharcSportLines lines{.clock = false, .frame = false};

  Fixture() : Fixture(lead_a1::SharcSportSnapshot{}) {
    lines = {.clock = false, .frame = false};
    first.observe(lines);
  }

  explicit Fixture(const lead_a1::SharcSportSnapshot &initial)
      : first(power, memory, 0, initial), lines(initial.serial.lines) {
    constexpr unsigned int cells = 256;
    memory.map({.bank = 0, .halfwords = cells, .name = "owned SPORT data RAM"});
  }

  void store(unsigned int word, std::uint32_t value) {
    first.write({.address = word * 4, .size = 4}, value);
  }

  std::uint32_t load(unsigned int word) {
    return static_cast<std::uint32_t>(first.read({.address = word * 4, .size = 4}));
  }

  void configure(std::uint32_t control) {
    store(0, control);
    first.advance_sclk(3);
  }

  void frame(bool value) {
    lines.frame = value;
    first.observe(lines);
  }

  void edge(bool clock, std::optional<bool> data = {}, std::optional<bool> other = {}) {
    lines.clock = clock;
    lines.data = {data, other};
    first.observe(lines);
  }

  void sample(bool bit, std::optional<bool> other = {}) {
    edge(true, bit, other);
    edge(false);
  }

  void dma_parameters(unsigned int count) {
    constexpr unsigned int index = 16;
    store(index, ram_address);
    store(index + 1, 1);
    store(index + 2, count);
    store(index + 3, 0);
  }

  void transfer() {
    auto &dma = first.dma(Channel::a);
    const auto request = dma.request();
    if (!request) { throw std::runtime_error("Missing actual SPORT DMA request"); }
    bus.request(lead_a1::SharcMemoryMaster::io0, *request);
    while (!bus.grant(lead_a1::SharcMemoryMaster::io0)) {
      bus.advance();
      first.advance_cclk(1);
    }
    dma.service(bus);
  }
};

void raw_controls_effect_and_readiness() {
  Fixture f;
  require(f.load(0) == 0 && f.load(1) == 0 && f.load(2) == 0);
  f.store(0, late_rx8);
  require(f.load(0) == late_rx8);
  f.first.advance_sclk(1);
  f.frame(true);
  f.edge(true, true);
  require(!f.first.snapshot().serial.word_active);
  f.first.advance_sclk(1);
  f.edge(false);
  f.edge(true, true);
  require(!f.first.snapshot().serial.word_active);
  f.first.advance_sclk(1);
  f.edge(false);
  f.edge(true, true);
  require(f.first.snapshot().serial.word_active && f.first.snapshot().serial.bit == 1);
  const auto before = f.first.snapshot();
  f.load(1); // Register reads supply no serial or peripheral clock.
  require(f.first.snapshot() == before);
}

void dual_receive_readback_and_abort() {
  Fixture f;
  f.configure(dual_rx8);
  f.frame(true);
  for (unsigned int index = 0; index < byte_bits.size(); ++index) {
    const bool bit = byte_bits[index] == '1';
    f.edge(true, bit, !bit);
    if (index == 0) {
      rejected([&] { static_cast<void>(f.load(0)); }, "partial RX");
    }
    f.edge(false);
  }
  require(f.first.dma(Channel::a).snapshot().receive_buffer == byte_value &&
          f.first.dma(Channel::b).snapshot().receive_buffer == inverse_byte &&
          f.first.interrupt_request());
  constexpr std::uint32_t complete_status = 0x91023071;
  require(f.load(0) == complete_status);
  const auto before = f.first.snapshot();
  f.first.begin_instruction();
  constexpr unsigned int receive_a = 25;
  constexpr unsigned int receive_b = 27;
  require(f.load(receive_a) == byte_value && f.load(receive_b) == inverse_byte);
  f.first.abort_instruction();
  require(f.first.snapshot() == before);
  require(f.load(receive_a) == byte_value && f.load(receive_b) == inverse_byte);
  rejected([&] { static_cast<void>(f.load(receive_a)); }, "empty");
  constexpr std::uint32_t hang_disabled = 0x01823071;
  f.store(0, hang_disabled);
  f.first.advance_sclk(2);
  require(f.load(receive_a) == byte_value && f.load(receive_b) == inverse_byte);
  require(!f.first.dma(Channel::a).snapshot().receive_buffer);
  f.first.acknowledge_interrupt();
  require(!f.first.interrupt_request());
}

struct WordCase {
  std::uint32_t value;
  std::uint32_t signed_value;
  std::string_view bits;
};

constexpr std::array words{
    WordCase{.value = 5, .signed_value = 0xfffffffd, .bits = "101"},
    WordCase{.value = 0x96, .signed_value = 0xffffff96, .bits = "10010110"},
    WordCase{.value = 0x8001, .signed_value = 0xffff8001, .bits = "1000000000000001"},
    WordCase{.value = 0x800001, .signed_value = 0xff800001, .bits = "100000000000000000000001"},
    WordCase{.value = 0x80000001,
             .signed_value = 0x80000001,
             .bits = "10000000000000000000000000000001"}};

struct Format {
  bool lsb{};
  bool rising{};
  bool sign{};
  bool transmit{};
};

void linear_word(const WordCase &word, Format format) {
  Fixture f;
  const auto count = static_cast<unsigned int>(word.bits.size());
  constexpr std::uint32_t late_required = 0x22001;
  constexpr std::uint32_t transmit = 0x02000000;
  constexpr std::uint32_t rising = 0x1000;
  const auto control = late_required | ((count - 1) << 4) | (format.lsb ? 8U : 0U) |
                       (format.sign ? 2U : 0U) | (format.rising ? rising : 0U) |
                       (format.transmit ? transmit : 0U);
  f.lines.clock = !format.rising;
  f.first.observe(f.lines);
  f.configure(control);
  constexpr unsigned int transmit_a = 24;
  constexpr unsigned int receive_a = 25;
  if (format.transmit) { f.store(transmit_a, word.value); }
  f.frame(true);
  for (unsigned int index = 0; index < count; ++index) {
    const auto position = format.lsb ? count - index - 1 : index;
    const bool bit = word.bits[position] == '1';
    if (format.transmit) {
      const auto signal = f.first.signals();
      require(signal.enable[0] == true && signal.data[0] == bit);
    }
    f.edge(format.rising, format.transmit ? std::nullopt : std::optional<bool>{bit});
    if (!format.transmit || index + 1 != count) { f.edge(!format.rising); }
  }
  if (format.transmit) {
    require(!f.first.dma(Channel::a).snapshot().transmit_shift);
  } else {
    require(f.load(receive_a) == (format.sign ? word.signed_value : word.value));
  }
}

void independent_linear_formats() {
  unsigned int cases{};
  for (const auto &word : words) {
    for (const bool lsb : {false, true}) {
      for (const bool rising : {false, true}) {
        for (const bool sign : {false, true}) {
          for (const bool tx : {false, true}) {
            linear_word(word, {.lsb = lsb, .rising = rising, .sign = sign, .transmit = tx});
            ++cases;
          }
        }
      }
    }
  }
  constexpr unsigned int expected = 80;
  require(cases == expected);
}

void async_transmit_first_bit_and_buffers() {
  Fixture f;
  f.configure(late_tx8);
  constexpr unsigned int transmit_a = 24;
  f.store(transmit_a, byte_value);
  require(f.load(transmit_a) == byte_value);
  f.frame(true); // No SCLK transition: the first late-FS bit is already driven.
  require(f.first.signals().data[0] == true && f.first.snapshot().serial.bit == 0 &&
          f.first.dma(Channel::a).snapshot().transmit_shift && f.first.interrupt_request());
  f.first.acknowledge_interrupt();
  f.store(transmit_a, inverse_byte);
  constexpr std::uint32_t two_words = 0xc2023071;
  require(f.load(0) == two_words);
  for (const char bit : byte_bits) {
    require(f.first.signals().data[0] == (bit == '1'));
    f.edge(true);
    // Keep FS high for late-framed back-to-back words.
    f.edge(false);
  }
  constexpr std::string_view second_bits = "01101001";
  for (unsigned int index = 0; index < second_bits.size(); ++index) {
    require(f.first.signals().data[0] == (second_bits[index] == '1'));
    f.edge(true);
    if (index + 1 != second_bits.size()) { f.edge(false); }
  }
  require(!f.first.dma(Channel::a).snapshot().transmit_shift &&
          !f.first.dma(Channel::a).snapshot().transmit_buffer);
}

void actual_serial_dma_and_delayed_status() {
  Fixture f;
  f.dma_parameters(1);
  constexpr std::uint32_t receive_dma8 = 0x00063071;
  f.configure(receive_dma8);
  require(f.first.dma(Channel::a).snapshot().active && f.load(2) == 0);
  f.first.advance_half_pclk(1);
  require(f.load(2) == 0);
  f.first.advance_half_pclk(1);
  constexpr std::uint32_t active_a = 0x01000000;
  require(f.load(2) == active_a);
  constexpr unsigned int odd_multichannel = 0x17;
  require(f.pair.read({.address = odd_multichannel * 4, .size = 4}) == active_a);
  f.frame(true);
  for (const char bit : byte_bits) { f.sample(bit == '1'); }
  require(!f.first.interrupt_request());
  const auto request = f.first.dma(Channel::a).request();
  require(request && request->access.address == ram_address && request->value == byte_value &&
          request->kind == lead_a1::AccessKind::write);
  f.transfer();
  require(f.memory.read({.address = ram_address, .width = Width::normal32}) == byte_value &&
          f.first.interrupt_request() && f.load(2) == active_a);
  f.first.advance_half_pclk(1);
  require(f.load(2) == active_a);
  f.first.advance_half_pclk(1);
  require(f.load(2) == 0);
}

void clock_disable_flush_and_atomic_faults() {
  Fixture f;
  f.configure(late_tx8);
  constexpr unsigned int transmit_a = 24;
  f.store(transmit_a, byte_value);
  f.frame(true);
  f.first.acknowledge_interrupt();
  f.store(transmit_a, inverse_byte);
  f.store(0, 0);
  require(f.first.snapshot().serial.flush[0] == 6);
  f.first.advance_cclk(5);
  require(f.first.dma(Channel::a).snapshot().transmit_shift &&
          f.first.dma(Channel::a).snapshot().transmit_buffer);
  const auto before = f.first.snapshot();
  rejected([&] { f.edge(true); }, "disable flush");
  require(f.first.snapshot() == before);
  f.first.advance_cclk(1);
  require(!f.first.dma(Channel::a).snapshot().transmit_shift &&
          !f.first.dma(Channel::a).snapshot().transmit_buffer && !f.first.interrupt_request());
  f.first.advance_sclk(3);
  require(f.first.signals().enable[0] == false);
  Fixture rx;
  rx.configure(dual_rx8);
  rx.frame(true);
  const auto saved = rx.first.snapshot();
  rejected([&] { rx.edge(true, true); }, "Unknown selected");
  require(rx.first.snapshot() == saved && !rx.first.dma(Channel::a).snapshot().receive_buffer);
}

void tdm_sparse_receive_and_frame_edge() {
  Fixture f;
  // Three3-bit slots, only0/2 selected, A enabled, frame delay0, FSED1.
  constexpr std::uint32_t rx3 = 0x1020;
  f.store(0, rx3);
  f.store(1, 4);
  constexpr std::uint32_t three_slots = 0x41;
  f.store(2, three_slots);
  f.store(4, 5);
  f.first.advance_sclk(3);
  f.frame(true);
  constexpr std::string_view bits = "101???011";
  for (unsigned int index = 0; index < bits.size(); ++index) {
    f.edge(true, bits[index] == '?' ? std::nullopt : std::optional<bool>{bits[index] == '1'});
    if (index == 2) { require(f.load(rx_a_word) == 5); }
    if (index == 5) { require(!f.first.dma(Channel::a).snapshot().receive_buffer); }
    f.lines.frame = false;
    f.edge(false);
  }
  require(f.load(rx_a_word) == 3 && !f.first.snapshot().serial.frame_active);
  Fixture waiting;
  waiting.frame(true); // Known active FS while still disabled.
  waiting.store(0, rx3);
  waiting.store(1, 4);
  waiting.store(2, 1);
  waiting.store(4, 1);
  waiting.first.advance_sclk(3);
  waiting.edge(true, true);
  require(!waiting.first.snapshot().serial.word_active);
  waiting.edge(false);
  waiting.frame(false);
  waiting.frame(true);
  waiting.sample(true);
  waiting.sample(false);
  waiting.edge(true, true);
  require(waiting.load(rx_a_word) == 5);
}

void delayed_tdm_boundaries(unsigned int delay) {
  Fixture f;
  constexpr unsigned int selection_last = 7;
  constexpr unsigned int last_slot = 127;
  constexpr unsigned int select_boundary = 0x80000000;
  constexpr unsigned int two_lanes = 0x800001;
  constexpr unsigned int slots = 128;
  constexpr std::string_view a_bits = "101";
  constexpr std::string_view b_bits = "010";
  constexpr unsigned int rx3 = 0x1020;
  f.store(0, rx3); // Three-bit RX, sample on rising SCLK.
  f.store(1, 4);   // A new frame edge is required.
  f.store(2, two_lanes | (last_slot << 5) | (delay << 1));
  f.store(4, select_boundary);              // Slot31.
  f.store(5, 1);                            // Slot32.
  f.store(selection_last, select_boundary); // Slot127.
  f.first.advance_sclk(3);
  f.frame(true);
  for (unsigned int clock = 0; clock < delay; ++clock) {
    f.sample(false, false);
    f.lines.frame = false;
    require(!f.first.snapshot().serial.word_active && !f.first.interrupt_request());
  }
  for (unsigned int slot = 0; slot < slots; ++slot) {
    const bool selected = slot == 31 || slot == 32 || slot == last_slot;
    for (unsigned int bit = 0; bit < a_bits.size(); ++bit) {
      f.edge(true,
             selected ? std::optional<bool>{a_bits[bit] == '1'} : std::nullopt,
             selected ? std::optional<bool>{b_bits[bit] == '1'} : std::nullopt);
      f.lines.frame = false;
      f.edge(false);
    }
    constexpr unsigned int receive_b = 27;
    if (selected) {
      require(f.load(rx_a_word) == 5 && f.load(receive_b) == 2);
      require(f.first.interrupt_request());
      f.first.acknowledge_interrupt();
    } else {
      require(!f.first.dma(Channel::a).snapshot().receive_buffer &&
              !f.first.dma(Channel::b).snapshot().receive_buffer && !f.first.interrupt_request());
    }
    require(f.first.snapshot().serial.slot == (slot + 1) % slots);
  }
  require(!f.first.snapshot().serial.frame_active);
}

void restored_wire_and_control_pipeline() {
  Fixture original;
  original.store(0, dual_rx8);
  original.first.advance_sclk(1);
  Fixture sampling(original.first.snapshot());
  sampling.first.advance_sclk(2);
  sampling.frame(true);
  for (unsigned int index = 0; index < 3; ++index) {
    sampling.sample(byte_bits[index] == '1', byte_bits[index] == '0');
  }
  const auto captured = sampling.first.snapshot();
  Fixture restored(captured);
  // The rebound owner needs neither the old object's RAM nor its signal cache.
  for (unsigned int index = 3; index < byte_bits.size(); ++index) {
    restored.sample(byte_bits[index] == '1', byte_bits[index] == '0');
  }
  constexpr unsigned int receive_b = 27;
  require(restored.load(rx_a_word) == byte_value && restored.load(receive_b) == inverse_byte &&
          restored.first.interrupt_request());
  require(sampling.first.snapshot() == captured);
  require(original.first.snapshot().serial.sampled[1][0] == 0 &&
          !original.first.interrupt_request());
}

void dma_transmit_terminal_serial_completion() {
  Fixture f;
  constexpr unsigned int dma_enable = 0x40000;
  constexpr unsigned int external_completion = 2;
  constexpr unsigned int transmit_a = 24;
  f.memory.write({.address = ram_address, .width = Width::normal32}, byte_value);
  f.dma_parameters(1);
  f.store(1, external_completion);
  f.configure(late_tx8 | dma_enable);
  require(f.first.dma(Channel::a).request().has_value());
  f.transfer();
  require(f.load(transmit_a) == byte_value && !f.first.interrupt_request());
  f.frame(true);
  require(!f.first.interrupt_request()); // Loading the shifter is not the last bit.
  for (unsigned int index = 0; index < byte_bits.size(); ++index) {
    require(f.first.signals().data[0] == (byte_bits[index] == '1'));
    f.edge(true);
    require(f.first.interrupt_request() == (index + 1 == byte_bits.size()));
    if (index + 1 < byte_bits.size()) { f.edge(false); }
  }
  require(!f.first.dma(Channel::a).snapshot().transmit_shift &&
          !f.first.dma(Channel::a).snapshot().transmit_buffer);
  f.first.acknowledge_interrupt();
  require(!f.first.interrupt_request());
}

void malformed_and_owner_guards() {
  Fixture f;
  const auto before = f.first.snapshot();
  rejected([&] { static_cast<void>(f.first.read({.address = 1, .size = 4})); }, "aligned");
  rejected([&] { f.first.write({.address = 0, .size = 4}, std::uint64_t{1} << 32); }, "Oversized");
  rejected([&] { static_cast<void>(f.first.read({.address = 13 * 4, .size = 4})); }, "hole");
  require(f.first.snapshot() == before);
  f.first.begin_instruction();
  f.store(0, late_rx8);
  rejected([&] { f.first.advance_sclk(1); }, "active instruction");
  rejected([&] { f.first.observe(f.lines); }, "active instruction");
  rejected([&] { static_cast<void>(f.first.snapshot()); }, "active instruction");
  f.first.abort_instruction();
  require(f.first.snapshot() == before);
  auto malformed = before;
  malformed.serial.bit = 1;
  rejected([&] { const Fixture invalid(malformed); }, "Malformed SPORT serial phase");
  malformed = before;
  malformed.serial.sampled[0][0] = 0x80000000;
  rejected([&] { const Fixture invalid(malformed); }, "Malformed SPORT sampled control");
}

void early_continuous_receive_and_frame_restore() {
  Fixture f;
  constexpr unsigned int early_continuous_rx = 0x1071;
  f.configure(early_continuous_rx);
  f.frame(true);
  f.sample(false); // Early frame consumes one clock before bit0.
  f.frame(false);
  for (const auto bit : byte_bits) { f.sample(bit == '1'); }
  require(f.load(rx_a_word) == byte_value);
  for (const auto bit : byte_bits) { f.sample(bit != '1'); }
  require(f.load(rx_a_word) == inverse_byte && f.first.snapshot().serial.frame_active);
  Fixture delayed;
  constexpr unsigned int rx3 = 0x1020;
  constexpr unsigned int tdm_delay15 = 31;
  delayed.store(0, rx3);
  delayed.store(1, 4);
  delayed.store(2, tdm_delay15);
  delayed.store(4, 1);
  delayed.first.advance_sclk(3);
  delayed.frame(true);
  delayed.sample(false);
  require(delayed.first.snapshot().serial.pending_frame == 14);
  Fixture restored(delayed.first.snapshot());
  restored.frame(false);
  for (unsigned int index = 0; index < 14; ++index) {
    restored.sample(false);
    require(!restored.first.interrupt_request());
  }
  restored.sample(true);
  restored.sample(false);
  restored.sample(true);
  require(restored.load(rx_a_word) == 5 && !restored.first.snapshot().serial.frame_active);
}

void early_transmit_overlapping_frame() {
  Fixture f;
  constexpr unsigned int early_tx = 0x02003071;
  constexpr unsigned int tx_a = 24;
  f.configure(early_tx);
  f.store(tx_a, byte_value);
  f.frame(true);
  require(!f.first.signals().data[0]);
  f.edge(true);
  f.frame(false);
  f.edge(false); // First data bit of the early-framed word.
  f.store(tx_a, inverse_byte);
  for (unsigned int index = 0; index < byte_bits.size(); ++index) {
    require(f.first.signals().data[0] == (byte_bits[index] == '1'));
    if (index + 1 == byte_bits.size()) { f.frame(true); }
    f.edge(true);
    f.edge(false);
  }
  f.frame(false);
  for (unsigned int index = 0; index < byte_bits.size(); ++index) {
    require(f.first.signals().data[0] == (byte_bits[index] != '1'));
    f.edge(true);
    if (index + 1 < byte_bits.size()) { f.edge(false); }
  }
  require(!f.first.dma(Channel::a).snapshot().transmit_shift &&
          !f.first.snapshot().serial.frame_active);
}

void unknown_frame_during_transfer() {
  Fixture f;
  f.configure(dual_rx8);
  f.frame(true);
  f.sample(true, false);
  const auto before = f.first.snapshot();
  f.lines.frame.reset();
  rejected([&] { f.edge(true, false, true); }, "Unknown required SPORT frame");
  require(f.first.snapshot() == before);
  Fixture tdm;
  constexpr unsigned int rx3 = 0x1020;
  tdm.store(0, rx3);
  tdm.store(1, 4);
  tdm.store(2, 1);
  tdm.store(4, 1);
  tdm.first.advance_sclk(3);
  tdm.frame(true);
  tdm.sample(true);
  const auto tdm_before = tdm.first.snapshot();
  tdm.lines.frame.reset();
  rejected([&] { tdm.edge(true, false); }, "Unknown required SPORT frame");
  require(tdm.first.snapshot() == tdm_before);
}

void physical_pair_registers_and_clock_gate() {
  Fixture f;
  constexpr std::array<unsigned int, 2> select_bases{5, 9};
  constexpr std::array<unsigned int, 2> compand_bases{13, 17};
  constexpr std::array<unsigned int, 2> secondary_words{0x1a, 0x1b};
  for (unsigned int port = 0; port < select_bases.size(); ++port) {
    for (unsigned int index = 0; index < 4; ++index) {
      f.pair.write({.address = (select_bases[port] + index) * 4, .size = 4}, 1U << index);
      f.pair.write({.address = (compand_bases[port] + index) * 4, .size = 4}, 1U << (index + 4));
    }
    f.pair.write({.address = secondary_words[port] * 4, .size = 4}, port + 1);
  }
  const auto a = f.first.snapshot();
  const auto b = f.second.snapshot();
  constexpr std::array<std::uint32_t, 4> selected{1, 2, 4, 8};
  constexpr std::array<std::uint32_t, 4> companded{16, 32, 64, 128};
  require(a.serial.select == selected && b.serial.select == selected &&
          a.serial.compand == companded && b.serial.compand == companded &&
          a.serial.control[1] == 1 && b.serial.control[1] == 2);
  constexpr unsigned int hole = 0x15;
  rejected([&] { f.pair.write({.address = hole * 4, .size = 4}, 0); }, "hole");
  f.pair.begin_instruction();
  f.pair.write({.address = select_bases[1] * 4, .size = 4}, 0);
  f.pair.abort_instruction();
  require(f.first.snapshot() == a && f.second.snapshot() == b);
  constexpr unsigned int stop_pair01 = 0x40;
  f.power.write({.address = 0, .size = 4}, stop_pair01);
  f.power.advance_pclk(2);
  rejected([&] { f.first.advance_sclk(1); }, "stopped peripheral clock");
  rejected([&] { f.first.observe(f.lines); }, "stopped peripheral clock");
  rejected([&] { f.second.advance_half_pclk(1); }, "stopped peripheral clock");
  require(f.first.snapshot() == a && f.second.snapshot() == b);
}

}

int main() try {
  check("raw_controls_effect_and_readiness", raw_controls_effect_and_readiness);
  check("dual_receive_readback_and_abort", dual_receive_readback_and_abort);
  check("independent_linear_formats", independent_linear_formats);
  check("async_transmit_first_bit_and_buffers", async_transmit_first_bit_and_buffers);
  check("actual_serial_dma_and_delayed_status", actual_serial_dma_and_delayed_status);
  check("clock_disable_flush_and_atomic_faults", clock_disable_flush_and_atomic_faults);
  check("tdm_sparse_receive_and_frame_edge", tdm_sparse_receive_and_frame_edge);
  for (const unsigned int delay : {0U, 1U, 15U}) { delayed_tdm_boundaries(delay); }
  check("restored_wire_and_control_pipeline", restored_wire_and_control_pipeline);
  check("dma_transmit_terminal_serial_completion", dma_transmit_terminal_serial_completion);
  check("malformed_and_owner_guards", malformed_and_owner_guards);
  check("early_continuous_receive_and_frame_restore", early_continuous_receive_and_frame_restore);
  check("early_transmit_overlapping_frame", early_transmit_overlapping_frame);
  check("unknown_frame_during_transfer", unknown_frame_during_transfer);
  check("physical_pair_registers_and_clock_gate", physical_pair_registers_and_clock_gate);
  std::cout << "SPORT control/linear-wire/DMA/frame/flush expectations pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
