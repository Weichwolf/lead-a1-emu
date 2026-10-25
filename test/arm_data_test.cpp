// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm_data_internal.hpp"
#include "arm/core/arm_mmu.hpp"
#include "arm/core/arm_mmu_internal.hpp"
#include "arm/core/arm_system.hpp"
#include "common/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using lead_a1::detail::ArmDataPath;
using lead_a1::detail::DataMaintenance;
using lead_a1::detail::DataPolicy;
using lead_a1::detail::MmuTranslation;
constexpr std::uint32_t physical_base = 0x2000;
constexpr std::uint32_t alternate_base = 0x4000;
constexpr std::uint32_t virtual_base = 0x8000;
constexpr std::uint32_t marker = 0x11223344;
constexpr std::uint32_t other_marker = 0xaabbccdd;
constexpr std::uint32_t initial_word = 0x03020100;
constexpr std::uint32_t initial_second_half = 0x13121110;
constexpr std::uint32_t alternate_word = 0x55555555;
constexpr std::uint32_t single_way = 0xfffe;
constexpr std::uint32_t unlocked = 0xfff0;
constexpr std::uint32_t all_locked = 0xffff;
constexpr std::uint32_t way_three = 0xc0000000;
constexpr std::size_t line_bytes = 32;
constexpr DataPolicy cached{.lockdown = single_way, .enabled = true, .round_robin = false};

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Independent ARM data expectation failed at line " +
                             std::to_string(where.line()));
  }
}

lead_a1::ArmSystemInitial profile() {
  return {.control = std::nullopt,
          .instruction_cache_bytes = std::nullopt,
          .data_cache_bytes = 128,
          .instruction_cache_invalid = false,
          .data_cache_invalid = true,
          .instruction_lockdown = std::nullopt,
          .data_lockdown = single_way,
          .write_buffer = std::vector<lead_a1::ArmBufferedWrite>{},
          .writeback_buffer_empty = true,
          .mmu = std::nullopt};
}

std::vector<std::byte> pattern() {
  std::vector<std::byte> bytes(512);
  constexpr unsigned int mask = 255;
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    bytes[index] = static_cast<std::byte>(index & mask);
  }
  return bytes;
}

struct Fixture {
  lead_a1::Memory memory;
  ArmDataPath data;

  Fixture() : data(memory, profile()) {
    const auto bytes = pattern();
    memory.map({.address = physical_base, .size = bytes.size(), .name = "owned data RAM"}, bytes);
    const std::vector<std::byte> alternate(bytes.size(), std::byte{0x55});
    memory.map({.address = alternate_base, .size = alternate.size(), .name = "owned alternate RAM"},
               alternate);
  }
};

MmuTranslation address(unsigned int offset = 0) {
  return {.physical = {.address = physical_base + offset, .size = 4},
          .modified = virtual_base + offset,
          .cacheable = true,
          .bufferable = true};
}

template <typename Function>
void transaction(lead_a1::Memory &memory, ArmDataPath &data, Function function) {
  memory.begin_instruction();
  data.begin_instruction();
  try {
    function();
    memory.commit_instruction();
    data.commit_instruction();
  } catch (...) {
    memory.abort_instruction();
    data.abort_instruction();
    throw;
  }
}

std::uint64_t read(Fixture &fixture, MmuTranslation translation = address()) {
  std::uint64_t result{};
  transaction(
      fixture.memory, fixture.data, [&] { result = fixture.data.read(translation, cached); });
  return result;
}

void write(Fixture &fixture, std::uint32_t value, MmuTranslation translation = address()) {
  transaction(
      fixture.memory, fixture.data, [&] { fixture.data.write(translation, cached, value); });
}

void maintenance(Fixture &fixture, DataMaintenance operation) {
  transaction(fixture.memory, fixture.data, [&] { fixture.data.maintain(operation); });
}

void drain(Fixture &fixture) {
  transaction(fixture.memory, fixture.data, [&] { fixture.data.drain(); });
}

template <typename Function> void rejected(Function function, const std::string &reason) {
  try {
    function();
  } catch (const std::runtime_error &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Expected ARM data-path rejection did not occur");
}

void stale_and_bypass() {
  Fixture fixture;
  require(read(fixture) == initial_word);
  fixture.memory.write({.address = physical_base, .size = 4}, marker);
  require(read(fixture) == initial_word);
  auto uncached = address();
  uncached.cacheable = false;
  require(read(fixture, uncached) == marker);
  require(fixture.data.state().read_hits == 1 && fixture.data.state().read_misses == 1);
  transaction(fixture.memory, fixture.data, [&] { fixture.data.invalidate_all(); });
  require(read(fixture) == marker);
  const auto before = fixture.data.state();
  transaction(fixture.memory, fixture.data, [&] {
    fixture.data.write(
        address(), {.lockdown = {}, .enabled = false, .round_robin = false}, other_marker);
  });
  require(fixture.data.state().cached_lines == before.cached_lines);
  require(read(fixture) == marker);
  require(fixture.memory.read({.address = physical_base, .size = 4}) == marker);
  drain(fixture);
  require(fixture.memory.read({.address = physical_base, .size = 4}) == other_marker);
}

void dirty_halves() {
  Fixture fixture;
  require(read(fixture) == initial_word);
  auto byte = address(1);
  byte.physical.size = 1;
  constexpr std::uint32_t byte_value = 0xab;
  constexpr std::uint32_t changed_word = 0x0302ab00;
  write(fixture, byte_value, byte);
  require(read(fixture) == changed_word && fixture.data.state().dirty_halves == 1);
  constexpr std::uint32_t untouched_offset = 20;
  fixture.memory.write({.address = physical_base + untouched_offset, .size = 4}, other_marker);
  maintenance(fixture, {.operation = 10, .selector = 1, .operand = virtual_base});
  require(fixture.data.state().dirty_halves == 0 && fixture.data.state().writeback_words == 4);
  require(fixture.memory.read({.address = physical_base, .size = 4}) == initial_word);
  drain(fixture);
  require(fixture.memory.read({.address = physical_base, .size = 4}) == changed_word);
  require(fixture.memory.read({.address = physical_base + untouched_offset, .size = 4}) ==
          other_marker);
  write(fixture, marker);
  write(fixture, other_marker, address(16));
  require(fixture.data.state().dirty_halves == 2);
  maintenance(fixture, {.operation = 14, .selector = 2, .operand = 0});
  require(fixture.data.state().cached_lines == 0 && fixture.data.state().writeback_words == 8);
  drain(fixture);
  require(fixture.memory.read({.address = physical_base, .size = 4}) == marker);
  require(fixture.memory.read({.address = physical_base + 16, .size = 4}) == other_marker);
  require(read(fixture) == marker);
  write(fixture, initial_word);
  maintenance(fixture, {.operation = 6, .selector = 1, .operand = virtual_base});
  drain(fixture);
  require(fixture.memory.read({.address = physical_base, .size = 4}) == marker);
}

void no_write_allocate_and_write_through() {
  Fixture fixture;
  write(fixture, marker);
  require(fixture.data.state().cached_lines == 0 && fixture.data.state().write_misses == 1);
  require(fixture.data.state().buffered_stores == 1);
  require(fixture.memory.read({.address = physical_base, .size = 4}) == initial_word);
  require(read(fixture) == marker);
  require(fixture.data.state().buffered_stores == 0 && fixture.data.state().bus_drains == 1);
  auto through = address();
  through.bufferable = false;
  write(fixture, other_marker, through);
  require(read(fixture) == other_marker && fixture.data.state().dirty_halves == 0);
  require(fixture.memory.read({.address = physical_base, .size = 4}) == marker);
  through.cacheable = false;
  require(read(fixture, through) == other_marker);
  require(fixture.memory.read({.address = physical_base, .size = 4}) == other_marker);
}

void physical_writeback_tag_and_eviction() {
  Fixture fixture;
  require(read(fixture) == initial_word);
  auto remapped = address();
  remapped.physical.address = alternate_base;
  write(fixture, marker, remapped);
  maintenance(fixture, {.operation = 10, .selector = 1, .operand = virtual_base});
  drain(fixture);
  require(fixture.memory.read({.address = physical_base, .size = 4}) == marker);
  require(fixture.memory.read({.address = alternate_base, .size = 4}) == alternate_word);
  write(fixture, other_marker, address(16));
  constexpr std::uint32_t next_word = 0x23222120;
  require(read(fixture, address(line_bytes)) == next_word);
  require(fixture.data.state().evictions == 1 && fixture.data.state().writeback_words == 4);
  require(fixture.memory.read({.address = physical_base + 16, .size = 4}) == initial_second_half);
  drain(fixture);
  require(fixture.memory.read({.address = physical_base + 16, .size = 4}) == other_marker);
}

void capacity_and_lockdown() {
  Fixture fixture;
  for (unsigned int index = 0; index < 5; ++index) {
    write(fixture, marker + index, address(index * 4));
  }
  require(fixture.data.state().buffered_stores == 1 && fixture.data.state().cached_lines == 0);
  for (unsigned int index = 0; index < 4; ++index) {
    require(fixture.memory.read({.address = physical_base + (index * 4), .size = 4}) ==
            marker + index);
  }
  drain(fixture);
  constexpr std::array<unsigned int, 16> available{4, 3, 3, 2, 3, 2, 2, 1, 3, 2, 2, 1, 2, 1, 1, 1};
  for (std::uint32_t mask = 0; mask < available.size(); ++mask) {
    Fixture candidate;
    const DataPolicy policy{.lockdown = unlocked | mask, .enabled = true, .round_robin = false};
    for (unsigned int index = 0; index < available[mask]; ++index) {
      transaction(candidate.memory, candidate.data, [&] {
        static_cast<void>(candidate.data.read(address(index * line_bytes), policy));
      });
    }
    const auto state = candidate.data.state();
    if (available[mask] > 1) {
      rejected(
          [&] {
            transaction(candidate.memory, candidate.data, [&] {
              static_cast<void>(candidate.data.read(address(available[mask] * line_bytes), policy));
            });
          },
          "random replacement");
      require(candidate.data.state() == state);
    } else {
      transaction(candidate.memory, candidate.data, [&] {
        static_cast<void>(candidate.data.read(address(line_bytes), policy));
      });
      require(candidate.data.state().evictions == 1);
    }
  }
  Fixture fallback;
  transaction(fallback.memory, fallback.data, [&] {
    static_cast<void>(fallback.data.read(
        address(), {.lockdown = all_locked, .enabled = true, .round_robin = false}));
  });
  maintenance(fallback, {.operation = 6, .selector = 2, .operand = way_three});
  require(fallback.data.state().cached_lines == 0);
}

void failed_eviction_and_drain() {
  lead_a1::Memory memory;
  const auto bytes = pattern();
  memory.map({.address = physical_base, .size = 2 * line_bytes, .name = "owned partial fill"},
             std::span(bytes).first(line_bytes + 4));
  ArmDataPath data(memory, profile());
  transaction(memory, data, [&] {
    require(data.read(address(), cached) == initial_word);
    data.write(address(), cached, marker);
  });
  const auto before = data.state();
  rejected(
      [&] {
        transaction(
            memory, data, [&] { static_cast<void>(data.read(address(line_bytes), cached)); });
      },
      "uninitialized");
  require(data.state() == before);
  transaction(memory, data, [&] { require(data.read(address(), cached) == marker); });
  require(memory.read({.address = physical_base, .size = 4}) == initial_word);
  lead_a1::Memory protected_memory;
  protected_memory.map({.address = physical_base, .size = 16, .name = "owned writable half"},
                       std::span(bytes).first(16));
  protected_memory.map({.address = physical_base + 16,
                        .size = 16,
                        .name = "owned protected half",
                        .writable = false},
                       std::span(bytes).subspan(16, 16));
  ArmDataPath protected_data(protected_memory, profile());
  transaction(protected_memory, protected_data, [&] {
    static_cast<void>(protected_data.read(address(), cached));
    protected_data.write(address(), cached, marker);
    protected_data.write(address(16), cached, other_marker);
    protected_data.maintain({.operation = 10, .selector = 1, .operand = virtual_base});
  });
  const auto buffered = protected_data.state();
  rejected([&] { transaction(protected_memory, protected_data, [&] { protected_data.drain(); }); },
           "permission denied");
  require(protected_data.state() == buffered);
  require(protected_memory.read({.address = physical_base, .size = 4}) == initial_word);
}

void buffered_page_table_walk() {
  Fixture fixture;
  constexpr std::uint32_t table_base = 0x10000;
  constexpr std::uint32_t descriptor_address = 0x12000;
  constexpr std::uint32_t descriptor = 0x20000c12;
  const std::vector<std::byte> zero(0x4000);
  fixture.memory.map({.address = table_base, .size = zero.size(), .name = "owned MMU table"}, zero);
  auto table_store = address();
  table_store.physical.address = descriptor_address;
  table_store.cacheable = false;
  write(fixture, descriptor, table_store);
  require(fixture.memory.read({.address = descriptor_address, .size = 4}) == 0);
  lead_a1::detail::ArmMmu mmu(fixture.memory,
                              lead_a1::ArmMmuInitial{.translation_table = table_base,
                                                     .domain_access = 3,
                                                     .fcse_pid = 0,
                                                     .context_id = {},
                                                     .tlb_lockdown = 0,
                                                     .tlb_invalid = true,
                                                     .locked_tlb_invalid = true},
                              &fixture.data);
  transaction(fixture.memory, fixture.data, [&] {
    mmu.begin_instruction();
    try {
      constexpr std::uint32_t virtual_address = 0x80000004;
      constexpr std::uint32_t physical_address = 0x20000004;
      constexpr std::uint32_t control = 0x50079;
      const auto result = mmu.translate({.access = {.address = virtual_address, .size = 4},
                                         .kind = lead_a1::AccessKind::read,
                                         .control = control,
                                         .user = false,
                                         .retain = true});
      require(result.physical.address == physical_address);
      mmu.commit_instruction();
    } catch (...) {
      mmu.abort_instruction();
      throw;
    }
  });
  require(fixture.data.state().buffered_stores == 0);
  require(fixture.memory.read({.address = descriptor_address, .size = 4}) == descriptor);
}

void widths_and_pending_writeback() {
  constexpr std::array<unsigned int, 4> widths{1, 2, 4, 8};
  constexpr std::array<std::uint64_t, 4> initial{0x1f, 0x1f1e, 0x1f1e1d1c, 0x1f1e1d1c1b1a1918};
  constexpr std::uint64_t stored = 0x8877665544332211;
  constexpr std::array<std::uint64_t, 4> truncated{0x11, 0x2211, 0x44332211, stored};
  for (std::size_t index = 0; index < widths.size(); ++index) {
    Fixture fixture;
    auto translation = address(static_cast<unsigned int>(line_bytes) - widths[index]);
    translation.physical.size = widths[index];
    require(read(fixture, translation) == initial[index]);
    transaction(
        fixture.memory, fixture.data, [&] { fixture.data.write(translation, cached, stored); });
    require(read(fixture, translation) == truncated[index]);
    require(fixture.data.state().dirty_halves == 1);
    require(fixture.memory.read(translation.physical) == initial[index]);
    maintenance(fixture, {.operation = 10, .selector = 2, .operand = 0});
    transaction(fixture.memory, fixture.data, [&] { fixture.data.invalidate_all(); });
    require(fixture.data.state().cached_lines == 0 && fixture.data.state().writeback_words == 4);
    drain(fixture);
    require(fixture.memory.read(translation.physical) == truncated[index]);
  }
}

void modified_tags_and_set_index() {
  Fixture fixture;
  auto modified = address();
  constexpr std::uint32_t process_address = 0x04008000;
  modified.modified = process_address;
  require(read(fixture, modified) == initial_word);
  write(fixture, marker, modified);
  maintenance(fixture, {.operation = 10, .selector = 1, .operand = virtual_base});
  require(fixture.data.state().dirty_halves == 1 && fixture.data.state().writebacks == 0);
  maintenance(fixture, {.operation = 10, .selector = 1, .operand = process_address});
  drain(fixture);
  require(fixture.memory.read({.address = physical_base, .size = 4}) == marker);
  auto system = profile();
  system.data_cache_bytes = 256;
  ArmDataPath two_sets(fixture.memory, system);
  transaction(fixture.memory, two_sets, [&] {
    static_cast<void>(two_sets.read(address(line_bytes), cached));
    two_sets.write(address(line_bytes), cached, other_marker);
    two_sets.maintain({.operation = 14, .selector = 2, .operand = line_bytes});
  });
  require(two_sets.state().cached_lines == 0 && two_sets.state().writeback_words == 4);
  transaction(fixture.memory, two_sets, [&] { two_sets.drain(); });
  require(fixture.memory.read({.address = physical_base + line_bytes, .size = 4}) == other_marker);
  Fixture protected_line;
  require(read(protected_line) == initial_word);
  const DataPolicy protect_way_zero{
      .lockdown = unlocked | 1, .enabled = true, .round_robin = false};
  transaction(protected_line.memory, protected_line.data, [&] {
    require(protected_line.data.read(address(), protect_way_zero) == initial_word);
    static_cast<void>(protected_line.data.read(address(line_bytes), protect_way_zero));
  });
  require(protected_line.data.state().read_hits == 1 &&
          protected_line.data.state().cached_lines == 2);
}

void unknown_and_invalid_inputs() {
  Fixture fixture;
  auto system = profile();
  system.data_cache_invalid = false;
  ArmDataPath unknown(fixture.memory, system);
  const auto before = unknown.state();
  rejected(
      [&] {
        transaction(
            fixture.memory, unknown, [&] { static_cast<void>(unknown.read(address(), cached)); });
      },
      "data cache validity");
  require(unknown.state() == before);
  transaction(fixture.memory, unknown, [&] {
    unknown.invalidate_all();
    require(unknown.read(address(), cached) == initial_word);
  });
  system.data_cache_invalid = true;
  system.data_cache_bytes.reset();
  ArmDataPath no_geometry(fixture.memory, system);
  rejected(
      [&] {
        transaction(fixture.memory, no_geometry, [&] {
          static_cast<void>(no_geometry.read(address(), cached));
        });
      },
      "data cache geometry");
  system = profile();
  system.writeback_buffer_empty = false;
  ArmDataPath unknown_writeback(fixture.memory, system);
  rejected(
      [&] { transaction(fixture.memory, unknown_writeback, [&] { unknown_writeback.drain(); }); },
      "write-back buffer");
  system = profile();
  system.write_buffer.reset();
  ArmDataPath unknown_stores(fixture.memory, system);
  rejected(
      [&] {
        transaction(fixture.memory, unknown_stores, [&] {
          static_cast<void>(unknown_stores.physical_read(address().physical));
        });
      },
      "Unknown initial ARM write buffer");
  constexpr std::array<DataMaintenance, 4> invalid{
      {{.operation = 6, .selector = 1, .operand = virtual_base + 4},
       {.operation = 6, .selector = 2, .operand = line_bytes},
       {.operation = 6, .selector = 3, .operand = 0},
       {.operation = 5, .selector = 1, .operand = virtual_base}}};
  for (const auto operation : invalid) {
    const auto state = fixture.data.state();
    rejected([&] { maintenance(fixture, operation); }, "ARM data-cache maintenance");
    require(fixture.data.state() == state);
  }
  auto unaligned = address(1);
  rejected([&] { static_cast<void>(read(fixture, unaligned)); }, "alignment");
  unaligned = address();
  ++unaligned.modified;
  rejected([&] { static_cast<void>(read(fixture, unaligned)); }, "alignment");
  const DataPolicy no_lockdown{.lockdown = {}, .enabled = true, .round_robin = false};
  rejected(
      [&] {
        transaction(fixture.memory, fixture.data, [&] {
          static_cast<void>(fixture.data.read(address(), no_lockdown));
        });
      },
      "lockdown");
}

void critical_word_first() {
  lead_a1::Memory memory;
  memory.map({.address = physical_base, .size = line_bytes, .name = "owned unknown line"});
  ArmDataPath data(memory, profile());
  const auto before = data.state();
  rejected(
      [&] {
        transaction(memory, data, [&] { static_cast<void>(data.read(address(16), cached)); });
      },
      "address=0x2010");
  require(data.state() == before);
}

}

int main() {
  try {
    stale_and_bypass();
    dirty_halves();
    no_write_allocate_and_write_through();
    physical_writeback_tag_and_eviction();
    capacity_and_lockdown();
    failed_eviction_and_drain();
    buffered_page_table_walk();
    widths_and_pending_writeback();
    modified_tags_and_set_index();
    unknown_and_invalid_inputs();
    critical_word_first();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
