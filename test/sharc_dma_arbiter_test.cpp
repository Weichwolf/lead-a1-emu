// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/memory/sharc_dma_arbiter.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/memory/sharc_memory_bus.hpp"
#include "sharc/peripherals/sharc_control.hpp"

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
using Group = lead_a1::SharcDmaGroup;
using Candidate = lead_a1::SharcDmaCandidate;
constexpr auto io0 = lead_a1::SharcMemoryMaster::io0;
constexpr std::uint32_t destination = 0x80010;
constexpr std::uint32_t payload = 0x78563412;
constexpr lead_a1::MemoryAccess word{.size = 4};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("DMA arbiter expectation line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw;
  }
  throw std::runtime_error("Missing DMA arbiter rejection: " + reason);
}

template <typename T> T must(const std::optional<T> &value) {
  if (!value) { throw std::runtime_error("Missing owned DMA value"); }
  return *value;
}

Candidate request(Group group, unsigned int channel = 0) {
  return {.group = group,
          .channel = channel,
          .request = {.access = {.address = destination},
                      .kind = lead_a1::AccessKind::write,
                      .value = payload}};
}

struct Fixture {
  lead_a1::SharcControl control{true};
  lead_a1::SharcMemory memory;
  lead_a1::SharcMemoryBus bus{memory, lead_a1::SharcPeripheralPhase::active};
  std::optional<lead_a1::SharcDmaArbiter> arbiter;

  explicit Fixture(bool rotating = false) {
    constexpr unsigned int cells = 128;
    memory.map({.bank = 0, .halfwords = cells, .name = "owned group arbitration RAM"});
    if (rotating) {
      constexpr unsigned int mode = 0x84;
      control.write(word, mode);
      control.advance_internal_clocks(1);
      control.advance_half_pclk(4);
    }
    arbiter.emplace(control, memory, lead_a1::SharcDmaArbiterState{.heads = 1, .mode = rotating});
  }

  void finish() {
    auto &owner = must_arbiter();
    owner.advance_half_pclk(2);
    owner.launch(bus);
    for (unsigned int tick = 0; tick < 3 && !bus.grant(io0); ++tick) { bus.advance(); }
    require(bus.grant(io0).has_value());
    static_cast<void>(bus.execute(io0));
    owner.retire(bus);
    require(memory.read({.address = destination}) == payload && !owner.snapshot().transfer);
  }

  lead_a1::SharcDmaArbiter &must_arbiter() {
    if (!arbiter) { throw std::runtime_error("Missing owned arbiter"); }
    return *arbiter;
  }
};

void fixed_groups_and_channel_ordinals() {
  Fixture f;
  constexpr std::array groups{Group::sport01,
                              Group::sport23,
                              Group::sport45,
                              Group::idp,
                              Group::spi,
                              Group::spib,
                              Group::memory_write,
                              Group::memory_read,
                              Group::uart,
                              Group::sport67,
                              Group::accelerator_in,
                              Group::accelerator_out};
  std::array<Candidate, groups.size()> reversed{};
  for (unsigned int index = 0; index < groups.size(); ++index) {
    reversed[index] = request(groups[groups.size() - 1 - index]);
  }
  for (unsigned int index = 0; index < groups.size(); ++index) {
    const auto active = std::span(reversed).first(groups.size() - index);
    require(must(f.must_arbiter().select(active)).group == groups[index]);
    f.finish();
  }
  constexpr std::array fixed_channels{
      Group::sport01, Group::sport23, Group::sport45, Group::idp, Group::uart, Group::sport67};
  for (const auto group : fixed_channels) {
    const auto sport_or_uart = group == Group::uart ? 2U : 4U;
    const auto limit = group == Group::idp ? 8U : sport_or_uart;
    for (unsigned int higher = 0; higher + 1 < limit; ++higher) {
      for (unsigned int lower = higher + 1; lower < limit; ++lower) {
        const std::array candidates{request(group, lower), request(group, higher)};
        require(f.must_arbiter().select(candidates) == candidates[1]);
        f.finish();
      }
    }
  }
  const std::array two_stage{request(Group::spi), request(Group::idp), request(Group::sport01, 3)};
  require(f.must_arbiter().select(two_stage) == two_stage[2]);
  f.finish();
}

void literal_rotation_and_idle() {
  Fixture f(true);
  const std::array candidates{request(Group::spi), request(Group::idp), request(Group::sport01)};
  constexpr std::array winners{
      Group::sport01, Group::idp, Group::spi, Group::sport01, Group::idp, Group::spi};
  for (const auto expected : winners) {
    require(must(f.must_arbiter().select(candidates)).group == expected);
    f.finish();
    const auto idle = f.must_arbiter().snapshot();
    require(!f.must_arbiter().select({}) && f.must_arbiter().snapshot() == idle);
  }
}

void unknown_cursor_and_mode_transition() {
  Fixture f;
  constexpr unsigned int mode = 0x84;
  f.control.write(word, mode);
  f.control.advance_internal_clocks(1);
  f.control.advance_half_pclk(4);
  const std::array competing{request(Group::spi), request(Group::sport01)};
  const auto before = f.must_arbiter().snapshot();
  rejected([&] { static_cast<void>(f.must_arbiter().select(competing)); },
           "Unknown rotating DMA priority");
  require(f.must_arbiter().snapshot() == before);
  const std::array one{request(Group::idp)};
  require(f.must_arbiter().select(one) == one[0]);
  f.finish();
  require(must(f.must_arbiter().select(competing)).group == Group::spi);
  f.finish();
  auto partial = f.control.snapshot();
  constexpr unsigned int bit = 0x80;
  partial.control &= ~bit;
  partial.effective &= ~bit;
  partial.known &= ~bit;
  partial.effective_known &= ~bit;
  partial.dma_rotation.reset();
  auto unknown = lead_a1::SharcControl::from_snapshot(partial);
  lead_a1::SharcDmaArbiter arbiter(unknown, f.memory);
  rejected([&] { static_cast<void>(arbiter.select(competing)); }, "Unknown rotating DMA priority");
}

void captured_latency_and_independent_restore() {
  Fixture f;
  const std::array one{request(Group::idp)};
  require(f.must_arbiter().select(one) == one[0]);
  const auto captured = f.must_arbiter().snapshot();
  rejected([&] { f.must_arbiter().launch(f.bus); }, "latency");
  require(f.must_arbiter().snapshot() == captured);
  f.must_arbiter().advance_half_pclk(1);
  lead_a1::SharcDmaArbiter restored(f.control, f.memory, f.must_arbiter().snapshot());
  rejected([&] { restored.launch(f.bus); }, "latency");
  restored.advance_half_pclk(1);
  constexpr unsigned int rotating = 0x84;
  f.control.write(word, rotating);
  f.control.advance_internal_clocks(1);
  f.control.advance_half_pclk(4);
  restored.launch(f.bus);
  f.bus.advance();
  rejected([&] { restored.retire(f.bus); }, "consumed bound");
  static_cast<void>(f.bus.execute(io0));
  restored.retire(f.bus);
  require(!restored.snapshot().transfer && f.must_arbiter().snapshot().transfer.has_value());
}

void malformed_requests_and_ownership() {
  Fixture f;
  const auto before = f.must_arbiter().snapshot();
  const std::array duplicate{request(Group::idp), request(Group::idp)};
  rejected([&] { static_cast<void>(f.must_arbiter().select(duplicate)); }, "Duplicate");
  const std::array absent{request(Group::mlb)};
  rejected([&] { static_cast<void>(f.must_arbiter().select(absent)); }, "MLB/link");
  auto bad = request(Group::sport01);
  bad.channel = 4;
  const std::array invalid{bad};
  rejected([&] { static_cast<void>(f.must_arbiter().select(invalid)); }, "channel ordinal");
  require(f.must_arbiter().snapshot() == before);
  f.memory.begin_instruction();
  rejected([&] { static_cast<void>(f.must_arbiter().select({})); }, "memory transaction");
  f.memory.abort_instruction();
  auto state = before;
  state.heads = 0;
  rejected([&] { const lead_a1::SharcDmaArbiter invalid_owner(f.control, f.memory, state); },
           "possibilities");
}
}

int main() try {
  fixed_groups_and_channel_ordinals();
  literal_rotation_and_idle();
  unknown_cursor_and_mode_transition();
  captured_latency_and_independent_restore();
  malformed_requests_and_ownership();
  std::cout << "DMA literal group/channel/rotation/unknown/latency/restore expectations pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
