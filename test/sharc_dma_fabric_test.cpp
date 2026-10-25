// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/board/sharc_dma_fabric.hpp"
#include "sharc/memory/sharc_dma_arbiter.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_boot.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/peripherals/sharc_pdap.hpp"
#include "sharc/peripherals/sharc_power.hpp"
#include "sharc/peripherals/sharc_spi.hpp"
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
using Group = lead_a1::SharcDmaGroup;
using Channel = lead_a1::SharcSportChannel;
constexpr std::uint32_t pdap_data = 0x78563412;
constexpr std::uint32_t spi_data = 0x12345678;
constexpr std::uint32_t sport_data = 0x96;
constexpr std::uint32_t sport_address = 0x80010;
constexpr std::uint32_t pdap_address = 0x80020;
constexpr std::uint32_t spi_address = 0x80030;
constexpr std::uint32_t completed_boot_index = 0x92180;
constexpr lead_a1::MemoryAccess control_word{.size = 4};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("DMA fabric expectation line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw;
  }
  throw std::runtime_error("Missing DMA fabric rejection: " + reason);
}

template <typename T> T must(const std::optional<T> &value) {
  if (!value) { throw std::runtime_error("Missing DMA fabric value"); }
  return *value;
}

struct Fixture {
  lead_a1::SharcControl control{true};
  lead_a1::SharcPower power;
  lead_a1::SharcMemory memory;
  lead_a1::SharcSpi spi{lead_a1::SharcBootState{.clock = true, .next_word = completed_boot_index}};
  lead_a1::SharcPdap pdap;
  std::array<std::optional<lead_a1::SharcSport>, 8> sports;
  std::optional<lead_a1::SharcDmaFabric> fabric;

  explicit Fixture(lead_a1::SharcPdapState initial = {}) : pdap(power, memory, initial) {
    constexpr unsigned int cells = 256;
    memory.map({.bank = 0, .halfwords = cells, .name = "owned DMA fabric RAM"});
    for (unsigned int port = 0; port < sports.size(); ++port) {
      sports[port].emplace(power, memory, port);
    }
  }

  lead_a1::SharcSport &sport(unsigned int index) {
    auto &port = sports[index];
    if (!port) { throw std::runtime_error("Missing owned SPORT"); }
    return *port;
  }

  void bind(lead_a1::SharcDmaFabricState initial = {}) {
    std::array<lead_a1::SharcSport *, 8> ports{};
    for (unsigned int index = 0; index < ports.size(); ++index) { ports[index] = &sport(index); }
    fabric.emplace(control, memory, spi, pdap, ports, initial);
  }

  lead_a1::SharcDmaFabric &owner() {
    if (!fabric) { throw std::runtime_error("Missing owned DMA fabric"); }
    return *fabric;
  }

  void pdap_ready(unsigned int words, std::uint32_t address = pdap_address) {
    constexpr unsigned int count_offset = 32;
    pdap.dma_write({.address = 0, .size = 4}, address);
    pdap.dma_write({.address = 64, .size = 4}, 1);
    pdap.dma_write({.address = count_offset * 4, .size = 4}, words);
    constexpr std::uint32_t configuration = 0x800ff000;
    constexpr std::uint32_t channels = 0x101;
    constexpr std::uint32_t global = 0xa0;
    pdap.write({.address = 4, .size = 4}, configuration);
    pdap.write({.address = 8, .size = 4}, channels);
    pdap.write(control_word, global);
    pdap.advance_sclk(1);
    constexpr std::array<std::uint32_t, 4> bytes{0x12, 0x34, 0x56, 0x78};
    constexpr std::uint32_t pins = 0xfffff;
    for (unsigned int index = 0; index < words; ++index) {
      for (const auto byte : bytes) {
        pdap.observe(lead_a1::SharcPdapPort::dai, {.clock = false});
        pdap.observe(lead_a1::SharcPdapPort::dai,
                     {.clock = true, .hold = false, .data = byte << 12, .known = pins});
      }
    }
  }

  void spi_ready() {
    spi.write(control_word, spi_address);
    spi.write({.address = 8, .size = 4}, 1);
    spi.observe(memory, {.selected = true, .clock = true, .mosi = {}});
    constexpr unsigned int bits = 32;
    for (unsigned int bit = 0; bit < bits; ++bit) {
      spi.observe(memory, {.selected = true, .clock = false, .mosi = {}});
      spi.observe(memory, {.selected = true, .clock = true, .mosi = ((spi_data >> bit) & 1) != 0});
    }
  }

  void sport_ready() {
    auto &port = sport(1); // HWR3-39: SPORT1A is ordinal0 in the highest group.
    auto &dma = port.dma(Channel::a);
    dma.write(control_word, sport_address);
    dma.write({.address = 4, .size = 4}, 1);
    dma.write({.address = 8, .size = 4}, 1);
    constexpr std::uint32_t rx8_dma = 0x00063071;
    lead_a1::SharcSportLines lines{.clock = false, .frame = false};
    port.observe(lines);
    port.write(control_word, rx8_dma);
    port.advance_sclk(3);
    lines.frame = true;
    port.observe(lines);
    for (const auto bit : std::string_view("10010110")) {
      lines.clock = true;
      lines.data[0] = bit == '1';
      port.observe(lines);
      lines.clock = false;
      lines.data[0].reset();
      port.observe(lines);
    }
  }

  void pair_ticks() {
    owner().tick();
    owner().tick();
  }
};

Group selected(Fixture &f) {
  return must(f.owner().snapshot().arbiter.transfer).candidate.group;
}

void real_producers_and_mode_changes_order() {
  for (const bool rotating : {false, true}) {
    Fixture f;
    f.pdap_ready(2);
    f.spi_ready();
    f.sport_ready();
    if (rotating) {
      constexpr std::uint32_t mode = 0x84;
      f.control.write(control_word, mode);
      f.control.advance_internal_clocks(1);
      f.control.advance_half_pclk(4);
    }
    lead_a1::SharcDmaFabricState initial;
    initial.arbiter.mode = rotating;
    initial.arbiter.heads = 1; // Explicit owned cyclic pointer reference.
    f.bind(initial);
    f.owner().tick();
    require(selected(f) == Group::sport01);
    f.pair_ticks();
    require(f.memory.read({.address = sport_address}) == sport_data &&
            f.sport(1).interrupt_request() && selected(f) == Group::idp &&
            f.pdap.interrupts().dma == 0 && !f.spi.interrupt_request());
    f.pair_ticks();
    require(f.memory.read({.address = pdap_address}) == pdap_data &&
            f.pdap.snapshot().dma[0].remaining == 1 &&
            selected(f) == (rotating ? Group::spi : Group::idp));
    f.pair_ticks();
    require(selected(f) == (rotating ? Group::idp : Group::spi));
    require(f.pdap.interrupts().dma == (rotating ? 0 : 1) && f.spi.interrupt_request() == rotating);
    f.pair_ticks();
    require(!f.owner().snapshot().arbiter.transfer &&
            f.memory.read({.address = pdap_address + 1}) == pdap_data &&
            f.memory.read({.address = spi_address}) == spi_data && f.pdap.interrupts().dma == 1 &&
            f.spi.interrupt_request());
  }
}

void failure_and_independent_owner_repair() {
  Fixture f;
  constexpr std::uint32_t missing = 0x80100;
  f.pdap_ready(1, missing);
  f.bind();
  f.owner().tick();
  f.owner().tick();
  const auto receiver = f.pdap.snapshot();
  rejected([&] { f.owner().tick(); }, "word-address=0x80100");
  const auto held = f.owner().snapshot();
  require(f.pdap.snapshot() == receiver && held.memory_bus.clocks == 3 &&
          held.memory_bus.granted[0].has_value() && f.pdap.interrupts().dma == 0);
  const auto control = f.control.snapshot();
  rejected([&] { f.owner().tick(); }, "retry/consume");
  require(f.owner().snapshot() == held && f.control.snapshot() == control);
  Fixture restored(receiver);
  restored.bind(held);
  constexpr unsigned int repaired_cell = 0x200;
  restored.memory.map({.bank = 0,
                       .first_halfword = repaired_cell,
                       .halfwords = 2,
                       .name = "owned repaired DMA fabric RAM"});
  restored.owner().retry();
  require(restored.memory.read({.address = missing}) == pdap_data &&
          restored.pdap.interrupts().dma == 1 && !restored.owner().snapshot().arbiter.transfer &&
          restored.owner().snapshot().memory_bus.clocks == 3 && f.owner().snapshot() == held);
}

void tcb_lock_is_explicit() {
  Fixture f;
  auto &dma = f.sport(0).dma(Channel::a);
  constexpr std::uint32_t chain = 0x80030;
  dma.write({.address = 12, .size = 4}, chain);
  dma.configure({.enabled = true, .chained = true});
  f.bind();
  rejected([&] { f.owner().tick(); },
           "pc=0x0 word-address=0x80030 operation=read SPORT0 lane=0: TCB bus lock");
  require(!f.owner().snapshot().arbiter.transfer && dma.snapshot().load.has_value());
}

void malformed_bus_owner_pair() {
  Fixture f;
  lead_a1::SharcDmaFabricState missing;
  missing.arbiter.transfer =
      lead_a1::SharcDmaTransfer{.candidate = {.group = Group::idp,
                                              .request = {.access = {.address = pdap_address},
                                                          .kind = lead_a1::AccessKind::write,
                                                          .value = pdap_data}},
                                .remaining_half_pclk = 0,
                                .submitted = true};
  rejected([&] { f.bind(missing); }, "no matching submitted request");
}
}

int main() try {
  real_producers_and_mode_changes_order();
  failure_and_independent_owner_repair();
  tcb_lock_is_explicit();
  malformed_bus_owner_pair();
  std::cout << "Real SPI/PDAP/SPORT arbitration, RAM/IRQ/order/fault/restore expectations pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
