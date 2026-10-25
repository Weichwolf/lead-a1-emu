// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_idp_dma.hpp"
#include "sharc/peripherals/sharc_pdap.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {
using Phase = lead_a1::SharcIdpDmaPhase;
using Master = lead_a1::SharcMemoryMaster;
constexpr std::uint32_t base = 0x80000;
constexpr std::uint32_t pdap_configuration = 0x800ff000;
constexpr std::uint32_t all_pins = 0xfffff;
constexpr std::uint32_t payload = 0x78563412;
constexpr std::uint32_t global = 0xa0;
constexpr unsigned int index_a = 8;
constexpr unsigned int modify = 16;
constexpr unsigned int index_b = 24;
constexpr unsigned int count = 32;
constexpr unsigned int reload = 40;
constexpr std::uint32_t ping_enabled = 0x10101;
constexpr std::uint32_t standard_enabled = 0x101;
constexpr std::uint32_t status_word = 0x20;
constexpr std::uint32_t dma_status = 0x20000;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("IDP DMA expectation line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw;
  }
  throw std::runtime_error("Missing IDP DMA rejection: " + reason);
}

template <typename T> T must(const std::optional<T> &value) {
  if (!value) { throw std::runtime_error("Missing IDP DMA owned value"); }
  return *value;
}

struct Fixture {
  lead_a1::SharcPower power;
  lead_a1::SharcMemory memory;
  lead_a1::SharcPdap pdap;
  lead_a1::SharcIdpDmaPort parameters{pdap};
  lead_a1::SharcMemoryBus bus{memory, lead_a1::SharcPeripheralPhase::active};

  explicit Fixture(lead_a1::SharcPdapState initial = {}) : pdap(power, memory, initial) {
    constexpr unsigned int cells = 256;
    memory.map({.bank = 0, .halfwords = cells, .name = "owned IDP DMA RAM"});
  }

  void write(unsigned int word, std::uint32_t value) {
    parameters.write({.address = word * 4, .size = 4}, value);
  }

  std::uint64_t read(unsigned int word) {
    return parameters.read({.address = word * 4, .size = 4});
  }

  void control(unsigned int word, std::uint32_t value) {
    pdap.write({.address = word * 4, .size = 4}, value);
  }

  void enable(bool ping) {
    control(1, pdap_configuration);
    control(2, ping ? ping_enabled : standard_enabled);
    control(0, global);
    pdap.advance_sclk(1);
  }

  void byte(std::uint32_t value) {
    pdap.observe(lead_a1::SharcPdapPort::dai, {.clock = false});
    pdap.observe(lead_a1::SharcPdapPort::dai,
                 {.clock = true, .hold = false, .data = value << 12, .known = all_pins});
  }

  void word() {
    constexpr std::array<std::uint32_t, 4> bytes{0x12, 0x34, 0x56, 0x78};
    for (const auto value : bytes) { byte(value); }
  }

  void transfer() {
    const auto request = pdap.dma_request();
    require(request.has_value());
    bus.request(Master::io0, must(request));
    for (unsigned int tick = 0; tick < 3 && !bus.grant(Master::io0); ++tick) { bus.advance(); }
    require(bus.grant(Master::io0) == request);
    pdap.service_dma(bus);
  }

  void standard(unsigned int transfer_count, unsigned int stride = 1) {
    constexpr unsigned int start = 0x10;
    write(0, base | start);
    write(modify, stride);
    write(count, transfer_count);
    enable(false);
  }
};

void reset_widths_and_transactions() {
  Fixture f;
  constexpr unsigned int registers = 48;
  constexpr std::uint32_t all = 0xffffffff;
  constexpr std::array<std::uint32_t, 6> masks{0x7ffff, 0x7ffff, 0x3f, 0x7ffff, 0xffff, 0xffff};
  for (unsigned int word = 0; word < registers; ++word) { require(f.read(word) == 0); }
  const auto before = f.pdap.snapshot();
  f.parameters.begin_instruction();
  for (unsigned int word = 0; word < registers; ++word) {
    f.write(word, all);
    require(f.read(word) == masks[word / 8]);
  }
  rejected([&] { static_cast<void>(f.pdap.dma_request()); }, "active instruction");
  f.parameters.abort_instruction();
  require(f.pdap.snapshot() == before);
  f.parameters.begin_instruction();
  f.write(reload, 2);
  f.parameters.commit_instruction();
  require(f.read(reload) == 2 && f.pdap.snapshot().dma[0].ping_count_written);
  rejected([&] { static_cast<void>(f.parameters.read({.address = 1, .size = 4})); }, "aligned32");
  constexpr unsigned int span = 192;
  rejected([&] { f.parameters.write({.address = span, .size = 4}, 0); }, "aligned32");
  rejected([&] { f.parameters.write({.address = 0, .size = 2}, 0); }, "aligned32");
  rejected([&] { f.parameters.write({.address = 0, .size = 4}, std::uint64_t{1} << 32); },
           "Oversized");
}

void standard_completion_and_drop() {
  Fixture f;
  f.standard(3);
  require(f.pdap.read({.address = status_word, .size = 4}) == dma_status && !f.pdap.dma_request());
  constexpr std::array<std::uint32_t, 3> addresses{0x80010, 0x80011, 0x80012};
  for (unsigned int index = 0; index < addresses.size(); ++index) {
    f.word();
    const auto snapshot = f.pdap.snapshot();
    const auto request = f.pdap.dma_request();
    require(request == (lead_a1::SharcMemoryRequest{.access = {.address = addresses[index]},
                                                    .kind = lead_a1::AccessKind::write,
                                                    .value = payload}));
    require(f.pdap.snapshot() == snapshot && !f.pdap.interrupts().threshold);
    f.transfer();
    require(f.memory.read({.address = addresses[index]}) == payload);
    require(f.pdap.interrupts().dma == (index == 2 ? 1 : 0));
  }
  constexpr unsigned int following = 0x13;
  require(f.read(0) == following && f.read(count) == 0 &&
          f.pdap.snapshot().dma[0].phase == Phase::complete);
  require(f.pdap.read({.address = status_word, .size = 4}) == 0);
  f.word();
  require(!f.pdap.dma_request() && f.pdap.dma_discard_ready());
  f.pdap.discard_dma_word();
  require(f.pdap.snapshot().count == 0 && f.pdap.interrupts().dma == 1);
  f.pdap.acknowledge_dma(2);
  require(f.pdap.interrupts().dma == 1);
  f.pdap.acknowledge_dma(1);
  require(f.pdap.interrupts().dma == 0);
}

void signed_six_bit_strides() {
  struct Case {
    unsigned int modifier;
    std::array<std::uint32_t, 3> addresses;
  };

  constexpr std::array cases{Case{.modifier = 0x1f, .addresses = {0x80010, 0x8002f, 0x8004e}},
                             Case{.modifier = 0x20, .addresses = {0x80040, 0x80020, 0x80000}},
                             Case{.modifier = 0x3f, .addresses = {0x80010, 0x8000f, 0x8000e}},
                             Case{.modifier = 0, .addresses = {0x80010, 0x80010, 0x80010}}};
  for (const auto &choice : cases) {
    Fixture f;
    f.write(0, choice.addresses[0]);
    f.write(modify, choice.modifier);
    f.write(count, 3);
    f.enable(false);
    for (const auto address : choice.addresses) {
      f.word();
      require(must(f.pdap.dma_request()).access.address == address);
      f.transfer();
      require(f.memory.read({.address = address}) == payload);
    }
    require(f.pdap.interrupts().dma == 1);
  }
}

void ping_pong_reload_and_independent_restore() {
  Fixture f;
  constexpr unsigned int first = 0x10;
  constexpr unsigned int second = 0x20;
  constexpr unsigned int negative_one = 0x3f;
  f.write(index_a, base | first);
  f.write(index_b, base | second);
  f.write(modify, negative_one);
  f.write(reload, 2);
  f.enable(true);
  constexpr std::array<std::uint32_t, 6> addresses{
      0x80010, 0x8000f, 0x80020, 0x8001f, 0x80010, 0x8000f};
  for (unsigned int index = 0; index < addresses.size(); ++index) {
    f.word();
    require(must(f.pdap.dma_request()).access.address == addresses[index]);
    f.transfer();
    require(f.memory.read({.address = addresses[index]}) == payload && f.read(index_a) == first &&
            f.read(index_b) == second && f.read(reload) == 2 &&
            f.read(count) == (index % 2 == 0 ? 1U : 2U));
    require(f.pdap.interrupts().dma == (index % 2 == 0 ? 0 : 1));
    f.pdap.acknowledge_dma(1);
  }
  require(f.pdap.snapshot().dma[0].phase == Phase::ping_b && f.read(0) == second);
  constexpr std::array<std::uint32_t, 4> bytes{0x12, 0x34, 0x56, 0x78};
  f.byte(bytes[0]);
  f.byte(bytes[1]);
  Fixture restored(f.pdap.snapshot());
  restored.byte(bytes[2]);
  restored.byte(bytes[3]);
  restored.transfer();
  constexpr unsigned int next_index = 0x1f;
  require(restored.memory.read({.address = addresses[2]}) == payload &&
          restored.read(0) == next_index && restored.read(count) == 1 &&
          restored.pdap.interrupts().dma == 0 && f.pdap.snapshot().samples == 2);
}

void pause_and_live_count_abort() {
  Fixture f;
  f.standard(4);
  f.word();
  f.transfer();
  f.word();
  constexpr unsigned int acquisition_only = 0x80;
  f.control(0, acquisition_only);
  const auto paused = f.pdap.snapshot();
  require(!f.pdap.dma_request() && paused.count == 1 && paused.dma[0].remaining == 3);
  rejected([&] { f.write(modify, 2); }, "retained transfer");
  require(f.pdap.snapshot() == paused);
  Fixture restored(paused);
  constexpr std::uint32_t next_word = 0x80011;
  restored.control(0, global);
  restored.write(count, 1); // HWR3-28/45: actual final transfer, no write-generated IRQ.
  require(restored.pdap.interrupts().dma == 0 &&
          must(restored.pdap.dma_request()).access.address == next_word);
  restored.transfer();
  require(restored.memory.read({.address = next_word}) == payload &&
          restored.pdap.interrupts().dma == 1 &&
          restored.pdap.snapshot().dma[0].phase == Phase::complete);
}

void failed_grant_and_separate_ram_repair() {
  Fixture f;
  constexpr unsigned int missing_index = 0x100;
  f.write(0, missing_index);
  f.write(count, 1);
  f.enable(false);
  f.word();
  const auto request = f.pdap.dma_request();
  require(request.has_value());
  f.bus.request(Master::io0, must(request));
  f.bus.advance();
  const auto state = f.pdap.snapshot();
  const auto grants = f.bus.state();
  rejected([&] { f.pdap.service_dma(f.bus); }, "word-address=0x80100");
  require(f.pdap.snapshot() == state && f.bus.state() == grants && f.pdap.interrupts().dma == 0);
  Fixture restored(state);
  lead_a1::SharcMemoryBus rebound(restored.memory, grants);
  rejected([&] { f.pdap.service_dma(rebound); }, "exact bound");
  constexpr unsigned int first_halfword = 0x200;
  restored.memory.map({.bank = 0,
                       .first_halfword = first_halfword,
                       .halfwords = 2,
                       .name = "owned repaired IDP destination"});
  restored.pdap.service_dma(rebound);
  require(restored.memory.read({.address = base | missing_index}) == payload &&
          restored.pdap.interrupts().dma == 1 && restored.pdap.snapshot().count == 0 &&
          !rebound.grant(Master::io0));
  require(f.pdap.snapshot() == state && f.bus.state() == grants);
}

void explicit_zero_count_and_wrap() {
  Fixture f;
  f.enable(false);
  require(!f.pdap.dma_request() && f.pdap.snapshot().dma[0].phase == Phase::idle);
  f.write(count, 0);
  constexpr unsigned int full = 65536;
  require(f.pdap.snapshot().dma[0].remaining == full && f.pdap.interrupts().dma == 0);
  for (unsigned int word = 1; word < full; ++word) {
    f.word();
    f.transfer();
  }
  require(f.read(count) == 1 && f.pdap.interrupts().dma == 0);
  f.word();
  f.transfer();
  require(f.read(count) == 0 && f.pdap.interrupts().dma == 1 &&
          f.memory.read({.address = base}) == payload);

  Fixture wrap;
  constexpr unsigned int maximum = 0x7ffff;
  constexpr unsigned int final_cells = 0x3fffe;
  wrap.memory.map({.bank = 3,
                   .first_halfword = final_cells,
                   .halfwords = 2,
                   .name = "owned IDP final address"});
  wrap.write(0, maximum);
  wrap.write(modify, 1);
  wrap.write(count, 2);
  wrap.enable(false);
  wrap.word();
  wrap.transfer();
  wrap.word();
  require(wrap.read(0) == 0 && must(wrap.pdap.dma_request()).access.address == base);
  wrap.transfer();
  require(wrap.memory.read({.address = base | maximum}) == payload &&
          wrap.memory.read({.address = base}) == payload && wrap.pdap.interrupts().dma == 1);
}

void malformed_state_and_gates() {
  Fixture f;
  auto state = f.pdap.snapshot();
  state.dma[0].remaining = 1;
  rejected([&] { const Fixture bad(state); }, "active/count");
  state = f.pdap.snapshot();
  constexpr unsigned int outside_modify = 64;
  state.dma[0].parameters[2] = outside_modify;
  rejected([&] { const Fixture bad(state); }, "parameter width");
  f.standard(2);
  f.word();
  const auto before = f.pdap.snapshot();
  f.power.write({.address = 0, .size = 4}, 16);
  f.power.advance_pclk(2);
  rejected([&] { static_cast<void>(f.pdap.dma_request()); }, "stopped DAI");
  rejected([&] { static_cast<void>(f.read(0)); }, "stopped DAI");
  require(f.pdap.snapshot() == before);
}

void partial_configuration_is_not_a_default_count() {
  Fixture f;
  constexpr unsigned int first = 0x10;
  f.write(0, first);
  f.control(1, pdap_configuration);
  f.control(2, standard_enabled);
  const auto partial = f.pdap.snapshot();
  rejected([&] { f.control(0, global); }, "no explicit transfer count");
  require(f.pdap.snapshot() == partial);
  f.write(count, 0);
  f.control(0, global);
  constexpr unsigned int full = 65536;
  require(f.pdap.snapshot().dma[0].remaining == full && f.pdap.interrupts().dma == 0);
}

void dma_arm_is_separate_from_receiver_enable() {
  Fixture f;
  f.write(count, 1);
  f.control(1, pdap_configuration);
  f.control(2, standard_enabled);
  constexpr unsigned int dma_only = 0x20;
  f.control(0, dma_only);
  require(f.pdap.read({.address = status_word, .size = 4}) == dma_status);
  f.word();
  require(f.pdap.snapshot().count == 0 && !f.pdap.dma_request() && f.pdap.interrupts().dma == 0);
  f.control(0, global);
  f.pdap.advance_sclk(1);
  f.word();
  f.transfer();
  require(f.memory.read({.address = base}) == payload && f.pdap.interrupts().dma == 1);
}
}

int main() try {
  reset_widths_and_transactions();
  standard_completion_and_drop();
  signed_six_bit_strides();
  ping_pong_reload_and_independent_restore();
  pause_and_live_count_abort();
  failed_grant_and_separate_ram_repair();
  explicit_zero_count_and_wrap();
  malformed_state_and_gates();
  partial_configuration_is_not_a_default_count();
  dma_arm_is_separate_from_receiver_enable();
  std::cout << "IDP actual FIFO/RAM grants, ping-pong, completion/restore expectations pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
