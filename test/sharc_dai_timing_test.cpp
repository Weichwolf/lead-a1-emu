// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/memory/sharc_iop_bridge.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_dai_timing.hpp"
#include "sharc/peripherals/sharc_power.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

using lead_a1::SharcDaiTiming;
using lead_a1::SharcDaiTimingGroup;

constexpr std::array groups{SharcDaiTimingGroup::clock, SharcDaiTimingGroup::frame};
constexpr std::uint32_t repeat_field = 0x02108421;
constexpr std::uint32_t all_low = 0x3def7bde;
constexpr std::uint32_t all_high = 0x3fffffff;
constexpr unsigned int source_count = 32;
constexpr unsigned int dynamic_sources = 30;
constexpr unsigned int reserved_frame = 27;
constexpr unsigned int words = 6;
constexpr unsigned int fields = 6;
constexpr std::uint32_t base = 0x80000;
constexpr std::uint32_t clock_address = 0x2432;
constexpr std::uint32_t frame_address = 0x2452;
constexpr std::uint32_t idp0_clock_pin11 = 0x3de57bde;
constexpr std::uint32_t idp0_frame_pin12 = 0x3de5fbde;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("DAI timing expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(reason) != std::string::npos) { return; }
    throw std::runtime_error("Expected '" + reason + "', received '" + error.what() + "'");
  }
  throw std::runtime_error("Missing DAI timing rejection: " + reason);
}

void write(SharcDaiTiming &router, unsigned int word, std::uint32_t value) {
  router.write({.address = word * 4, .size = 4}, value);
}

std::uint32_t read(SharcDaiTiming &router, unsigned int word) {
  return static_cast<std::uint32_t>(router.read({.address = word * 4, .size = 4}));
}

unsigned int valid_fields(SharcDaiTimingGroup group, unsigned int word) {
  // Independent destination topology from HWR A-126..128/A-136..138.
  constexpr std::array<unsigned int, words> clocks{63, 63, 63, 63, 59, 63};
  constexpr std::array<unsigned int, words> frames{63, 63, 63, 31, 3, 0};
  return group == SharcDaiTimingGroup::clock ? clocks.at(word) : frames.at(word);
}

unsigned int verify_default_topology(SharcDaiTiming &router, SharcDaiTimingGroup group) {
  unsigned int destinations{};
  for (unsigned int word = 0; word < words; ++word) {
    for (unsigned int field = 0; field < fields; ++field) {
      if ((valid_fields(group, word) & (1U << field)) == 0) {
        rejected([&] { static_cast<void>(router.output({.word = word, .field = field})); },
                 "destination");
      } else {
        ++destinations;
        const auto level = router.output({.word = word, .field = field});
        require(word == 0 ? !level.has_value() : level == false);
      }
    }
  }
  return destinations;
}

void reset_and_destination_topology() {
  constexpr std::array<std::uint32_t, words> clock_reset{
      0x252630c2, 0x3def7bde, 0x3def7bde, 0x3def7bde, 0x3def7bde, 0x3def7bde};
  constexpr std::array<std::uint32_t, words> frame_reset{
      0x2736b4e3, 0x3def7bde, 0x3def7bde, 0x01ef7bde, 0x000003de, 0};
  for (const auto group : groups) {
    lead_a1::SharcPower power;
    SharcDaiTiming router(power, group);
    require(router.snapshot().routes ==
            (group == SharcDaiTimingGroup::clock ? clock_reset : frame_reset));
    const auto destinations = verify_default_topology(router, group);
    constexpr unsigned int clock_destinations = 35;
    constexpr unsigned int frame_destinations = 25;
    require(destinations ==
            (group == SharcDaiTimingGroup::clock ? clock_destinations : frame_destinations));
    const auto registers = group == SharcDaiTimingGroup::clock ? 6U : 5U;
    for (unsigned int word = 0; word < registers; ++word) {
      require(read(router, word) ==
              (group == SharcDaiTimingGroup::clock ? clock_reset[word] : frame_reset[word]));
    }
  }
}

void independent_reset_sources() {
  constexpr std::array<unsigned int, fields> clock_sources{2, 6, 12, 12, 18, 18};
  constexpr std::array<unsigned int, fields> frame_sources{3, 7, 13, 13, 19, 19};
  for (const auto group : groups) {
    lead_a1::SharcPower power;
    SharcDaiTiming router(power, group);
    for (unsigned int source = 0; source < dynamic_sources; ++source) {
      if (group == SharcDaiTimingGroup::frame && source == reserved_frame) { continue; }
      router.source(source, false);
    }
    for (unsigned int source = 0; source < dynamic_sources; ++source) {
      if (group == SharcDaiTimingGroup::frame && source == reserved_frame) { continue; }
      router.source(source, true);
      for (unsigned int field = 0; field < fields; ++field) {
        const auto expected =
            group == SharcDaiTimingGroup::clock ? clock_sources[field] : frame_sources[field];
        require(router.output({.field = field}) == (source == expected));
      }
      router.source(source, false);
    }
  }
}

void set_repeated_routes(SharcDaiTiming &router, SharcDaiTimingGroup group, unsigned int source) {
  const auto repeated = source * repeat_field;
  for (unsigned int word = 0; word < words; ++word) {
    if (valid_fields(group, word) == 0) { continue; }
    constexpr std::uint32_t pll_mask = 0x7c00;
    const auto target = group == SharcDaiTimingGroup::clock && word == 4
                            ? (repeated & ~pll_mask) | (all_low & pll_mask)
                            : repeated;
    write(router, word, target);
  }
}

void verify_repeated_routes(SharcDaiTiming &router,
                            SharcDaiTimingGroup group,
                            bool high,
                            unsigned int source) {
  for (unsigned int word = 0; word < words; ++word) {
    for (unsigned int field = 0; field < fields; ++field) {
      if ((valid_fields(group, word) & (1U << field)) == 0) { continue; }
      constexpr unsigned int a_clock = 28;
      constexpr unsigned int b_clock = 29;
      const bool tied_low =
          group == SharcDaiTimingGroup::clock && word == 4 &&
          ((field == 0 && source == a_clock) || (field == 1 && source == b_clock));
      require(router.output({.word = word, .field = field}) == (high && !tied_low));
    }
  }
}

void all_sources_and_independent_words() {
  for (const auto group : groups) {
    lead_a1::SharcPower power;
    SharcDaiTiming router(power, group);
    for (unsigned int source = 0; source < source_count; ++source) {
      if (group == SharcDaiTimingGroup::frame && source == reserved_frame) { continue; }
      set_repeated_routes(router, group, source);
      router.advance_pclk(3);
      const bool dynamic = source < dynamic_sources;
      if (dynamic) { router.source(source, true); }
      verify_repeated_routes(router, group, dynamic || source == source_count - 1, source);
      if (!dynamic) { continue; }
      router.source(source, std::nullopt);
      require(!router.output({.word = 2, .field = 3}).has_value());
      router.source(source, false);
      require(router.output({.word = 2, .field = 3}) == false);
    }
  }
}

void pcg_self_inputs_are_tied_low() {
  lead_a1::SharcPower power;
  SharcDaiTiming router(power, SharcDaiTimingGroup::clock);
  constexpr std::uint32_t self_inputs = 0x3def7bbc;
  constexpr std::uint32_t cross_inputs = 0x3def7b9d;
  constexpr unsigned int pcg_a = 28;
  constexpr unsigned int pcg_b = 29;
  write(router, 4, self_inputs);
  router.advance_pclk(3);
  require(router.output({.word = 4, .field = 0}) == false &&
          router.output({.word = 4, .field = 1}) == false);
  router.source(pcg_a, true);
  router.source(pcg_b, true);
  require(router.output({.word = 4, .field = 0}) == false &&
          router.output({.word = 4, .field = 1}) == false);
  write(router, 4, cross_inputs);
  router.advance_half_pclk(3);
  require(router.output({.word = 4, .field = 0}) == false);
  router.advance_half_pclk(1);
  require(!router.output({.word = 4, .field = 0}) && !router.output({.word = 4, .field = 1}));
  SharcDaiTiming restored(power, SharcDaiTimingGroup::clock, router.snapshot());
  restored.advance_half_pclk(2);
  require(restored.output({.word = 4, .field = 0}) == true &&
          restored.output({.word = 4, .field = 1}) == true);
  write(restored, 4, self_inputs);
  restored.advance_half_pclk(4);
  require(!restored.output({.word = 4, .field = 0}));
  restored.advance_half_pclk(2);
  require(restored.output({.word = 4, .field = 0}) == false &&
          restored.output({.word = 4, .field = 1}) == false);
}

void exact_intervals_and_restore() {
  for (const auto group : groups) {
    lead_a1::SharcPower power;
    SharcDaiTiming router(power, group);
    write(router, 2, all_high);
    router.advance_half_pclk(3);
    require(router.output({.word = 2, .field = 3}) == false);
    const auto before = router.snapshot();
    router.advance_half_pclk(0);
    require(router.snapshot() == before);
    router.advance_half_pclk(1);
    require(!router.output({.word = 2, .field = 3}).has_value());
    const auto uncertain = router.snapshot();
    SharcDaiTiming restored(power, group, uncertain);
    restored.advance_half_pclk(1);
    require(!restored.output({.word = 2, .field = 3}).has_value());
    rejected([&] { static_cast<void>(read(restored, 2)); }, "unsettled");
    restored.advance_half_pclk(1);
    require(restored.output({.word = 2, .field = 3}) == true && read(restored, 2) == all_high);
    router.advance_pclk(1);
    require(router.snapshot() == restored.snapshot());
    write(router, 2, all_low);
    router.advance_half_pclk(std::numeric_limits<std::uint32_t>::max());
    require(router.output({.word = 2, .field = 3}) == false);
    write(router, 2, all_low); // Equal known source levels remain known mid-interval.
    router.advance_pclk(2);
    require(router.output({.word = 2, .field = 3}) == false);
    router.advance_pclk(std::numeric_limits<std::uint32_t>::max());
    write(router, 0, all_high);
    router.advance_half_pclk(1);
    write(router, 1, all_high);
    router.advance_half_pclk(5);
    require(router.output({.field = 0}) == true);
    require(!router.output({.word = 1, .field = 0}).has_value());
    router.advance_half_pclk(1);
    require(router.output({.word = 1, .field = 0}) == true);
  }
}

void live_inputs_during_uncertain_switch() {
  for (const auto group : groups) {
    lead_a1::SharcPower power;
    SharcDaiTiming router(power, group);
    router.source(10, false);
    write(router, 2, idp0_clock_pin11);
    router.advance_pclk(2);
    require(router.output({.word = 2, .field = 3}) == false);
    router.source(10, true);
    require(!router.output({.word = 2, .field = 3}).has_value());
    router.source(10, std::nullopt);
    require(!router.output({.word = 2, .field = 3}).has_value());
    router.source(10, false);
    require(router.output({.word = 2, .field = 3}) == false);
    router.advance_pclk(1);
    router.source(10, true);
    require(router.output({.word = 2, .field = 3}) == true);
    router.source(11, true);
    write(router, 2, idp0_frame_pin12);
    router.advance_pclk(2);
    require(router.output({.word = 2, .field = 3}) == true);
    router.source(10, std::nullopt);
    require(!router.output({.word = 2, .field = 3}).has_value());
    router.source(10, false);
    require(!router.output({.word = 2, .field = 3}).has_value());
    router.advance_pclk(1);
    require(router.output({.word = 2, .field = 3}) == true);
  }
}

void malformed_and_unqualified_inputs() {
  for (const auto group : groups) {
    lead_a1::SharcPower power;
    SharcDaiTiming router(power, group);
    const auto before = router.snapshot();
    rejected([&] { router.write({.address = 2, .size = 4}, 0); }, "aligned32-bit");
    rejected([&] { static_cast<void>(router.read({.size = 2})); }, "aligned32-bit");
    const auto end = group == SharcDaiTimingGroup::clock ? 24U : 20U;
    rejected([&] { static_cast<void>(router.read({.address = end, .size = 4})); }, "aligned32-bit");
    constexpr std::uint64_t oversized = 0x100000000;
    rejected([&] { router.write({.size = 4}, oversized); }, "Oversized");
    rejected([&] { router.source(dynamic_sources, true); }, "source input");
    rejected([&] { static_cast<void>(router.output({.word = words})); }, "register index");
    rejected([&] { static_cast<void>(router.output({.field = fields})); }, "destination");
    if (group == SharcDaiTimingGroup::clock) {
      rejected([&] { write(router, 4, all_high); }, "PLL destination conflict");
    } else {
      rejected([&] { write(router, 2, reserved_frame * repeat_field); }, "Reserved");
      rejected([&] { router.source(reserved_frame, false); }, "reserved");
    }
    require(router.snapshot() == before);
    write(router, 2, all_high);
    const auto pending = router.snapshot();
    rejected([&] { write(router, 2, all_low); }, "overlapping");
    require(router.snapshot() == pending);
    auto invalid = pending;
    invalid.pending[2] = lead_a1::SharcDaiTimingPending{.target = all_high, .half_periods = 6};
    rejected([&] { const SharcDaiTiming other(power, group, invalid); }, "pending interval");
    if (group == SharcDaiTimingGroup::frame) {
      invalid = before;
      invalid.sources[reserved_frame] = false;
      rejected([&] { const SharcDaiTiming other(power, group, invalid); }, "Reserved");
      invalid = before;
      invalid.routes[5] = 1;
      rejected([&] { const SharcDaiTiming other(power, group, invalid); }, "Nonexistent");
      constexpr std::uint32_t unused_fields = 0xfffffc00;
      write(router, 4, unused_fields | 1U); // Reserved fields stay raw, not fake destinations.
      router.advance_pclk(3);
      require(read(router, 4) == (unused_fields | 1U));
    }
  }
}

void ownership_and_stopped_clock() {
  lead_a1::SharcPower power;
  SharcDaiTiming router(power, SharcDaiTimingGroup::clock);
  const auto before = router.snapshot();
  router.begin_instruction();
  write(router, 2, all_high);
  require(router.output({.word = 2, .field = 3}) == false);
  rejected([&] { router.begin_instruction(); }, "Nested");
  rejected([&] { router.source(0, true); }, "active instruction");
  rejected([&] { router.advance_pclk(1); }, "active instruction");
  rejected([&] { router.advance_half_pclk(1); }, "active instruction");
  rejected([&] { static_cast<void>(router.snapshot()); }, "active instruction");
  router.abort_instruction();
  require(router.snapshot() == before);
  router.begin_instruction();
  write(router, 2, all_high);
  router.commit_instruction();
  router.advance_pclk(3);
  require(router.output({.word = 2, .field = 3}) == true);
  constexpr std::uint32_t stop_dai = 0x10;
  power.write({.size = 4}, stop_dai);
  power.advance_pclk(2);
  const auto gated = router.snapshot();
  rejected([&] { static_cast<void>(read(router, 2)); }, "stopped DAI clock");
  rejected([&] { write(router, 2, all_low); }, "stopped DAI clock");
  rejected([&] { router.advance_pclk(1); }, "stopped DAI clock");
  rejected([&] { static_cast<void>(router.output({.word = 2, .field = 3})); }, "stopped DAI clock");
  require(router.snapshot() == gated);
}

void actual_core_bridge_composition() {
  lead_a1::SharcPower power;
  SharcDaiTiming clocks(power, SharcDaiTimingGroup::clock);
  SharcDaiTiming frames(power, SharcDaiTimingGroup::frame);
  lead_a1::SharcMemory peripheral;
  std::optional<lead_a1::SharcIopBridge> bridge;
  lead_a1::SharcMemory memory;
  auto state = lead_a1::Sharc::reset_reference_214xx(base);
  constexpr std::uint64_t full40 = 0xffffffffff;
  state.data[0][0] = {.value = std::uint64_t{idp0_clock_pin11} << 8, .known = full40};
  state.data[0][1] = {.value = std::uint64_t{idp0_frame_pin12} << 8, .known = full40};
  lead_a1::Sharc cpu(memory, state);
  peripheral.map_io({.first_word = clock_address - 2, .words = 6}, clocks);
  peripheral.map_io({.first_word = frame_address - 2, .words = 5}, frames);
  constexpr lead_a1::SharcIoMapping window{.first_word = 0x2400, .words = 0x100};
  auto &bus = bridge.emplace(memory, peripheral, window);
  memory.map_io(window, bus);
  memory.map({.bank = 0, .first_halfword = 0, .halfwords = 128, .name = "owned timing code"});
  constexpr std::array<std::uint64_t, 4> program{
      0x110000002432, 0x110100002452, 0x100200002432, 0x100300002452};
  for (unsigned int index = 0; index < program.size(); ++index) {
    memory.write({.address = base + index, .width = lead_a1::SharcWidth::wide48}, program[index]);
  }
  const auto tick = [&] {
    power.advance_half_pclk(1);
    clocks.advance_half_pclk(1);
    frames.advance_half_pclk(1);
    bus.tick();
  };
  for (unsigned int index = 0; index < program.size(); ++index) {
    const bool store = index < 2;
    bus.issue({.pc = cpu.state().pc,
               .address = index % 2 == 0 ? clock_address : frame_address,
               .operation = store ? lead_a1::AccessKind::write : lead_a1::AccessKind::read,
               .delivery_cycles = store ? 5U : 8U});
    while (!bus.ready()) { tick(); }
    require(cpu.step().next_pc == base + index + 1);
    if (store) { tick(); }
  }
  require((cpu.state().data[0][2].value >> 8) == idp0_clock_pin11);
  require((cpu.state().data[0][3].value >> 8) == idp0_frame_pin12);
  require(!clocks.output({.word = 2, .field = 3}).has_value());
  require(!frames.output({.word = 2, .field = 3}).has_value());
  clocks.source(10, true);  // DAI_PB11_O, not PBEN11 or an assumed pad voltage.
  frames.source(11, false); // DAI_PB12_O.
  require(clocks.output({.word = 2, .field = 3}) == true);
  require(frames.output({.word = 2, .field = 3}) == false);
  clocks.source(10, false);
  frames.source(11, true);
  require(clocks.output({.word = 2, .field = 3}) == false);
  require(frames.output({.word = 2, .field = 3}) == true);
}

}

int main() try {
  reset_and_destination_topology();
  independent_reset_sources();
  all_sources_and_independent_words();
  pcg_self_inputs_are_tied_low();
  exact_intervals_and_restore();
  live_inputs_during_uncertain_switch();
  malformed_and_unqualified_inputs();
  ownership_and_stopped_clock();
  actual_core_bridge_composition();
  std::cout
      << "DAI clock/frame routes, sources, intervals, snapshots and real IOP transfers pass\n";
  return 0;
} catch (const std::exception &error) {

  std::cerr << error.what() << '\n';
  return 1;
}
