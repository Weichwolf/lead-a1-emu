// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/aintc.hpp"
#include "lead_a1/memory.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t host_enable_offset = 0x1500;
constexpr std::uint32_t vector_base_offset = 0x50;
constexpr std::uint32_t vector_size_offset = 0x54;
constexpr std::uint32_t global_nesting_offset = 0x1c;
constexpr std::uint32_t irq_nesting_offset = 0x1104;
constexpr std::uint32_t fiq_vector_offset = 0x1600;
constexpr std::uint32_t status_enabled_offset = 0x280;
constexpr std::uint32_t enable_set_offset = 0x28;
constexpr std::uint32_t fiq_index_offset = 0x900;
constexpr std::uint32_t host_set_offset = 0x34;
constexpr std::uint32_t irq_index_offset = 0x904;
constexpr std::uint32_t host_clear_offset = 0x38;
constexpr std::uint32_t enable_clear_offset = 0x2c;
constexpr std::uint32_t status_clear_offset = 0x24;
constexpr std::uint32_t enable_bank_offset = 0x300;
constexpr std::uint32_t disable_bank_offset = 0x380;
constexpr std::uint32_t priority_source_ninety = 90;
constexpr std::uint32_t priority_source_forty_five = 45;
constexpr std::uint32_t priority_source_hundred = 100;
constexpr std::uint32_t null_vector_offset = 0x58;
constexpr std::uint32_t source_five_vector = 0x10014;
constexpr std::uint32_t source_one_vector = 0x20004;
constexpr std::uint32_t larger_stride_vector = 0x200a0;
constexpr std::uint32_t wrapping_vector_base = 0xfffffff0;
constexpr std::uint32_t wrapped_vector = 0x7ffffff0;
constexpr std::uint32_t invalid_channel_word = 0x01022003;
constexpr std::uint32_t last_channel_word = 0x464;
constexpr std::uint32_t channel_one_offset = 0x401;
constexpr std::uint32_t initial_channel_word = 0x03020100;
constexpr std::uint32_t channel_two_offset = 0x402;
constexpr std::uint32_t initial_upper_channels = 0x302;
constexpr std::uint32_t updated_upper_channels = 0x1f1e;
constexpr std::uint32_t updated_channel_word = 0x1f1e0100;
constexpr std::uint32_t irq_vector_offset = 0x1604;
constexpr std::uint32_t nesting_override_thirty = 0x8000001e;
constexpr std::uint32_t nesting_override_initial = 0x80000100;
constexpr std::uint32_t nesting_thirty = 30;

constexpr std::uint32_t no_interrupt = 0x80000000;
constexpr std::uint32_t vector_base = 0x10000;
constexpr std::uint32_t hold_mode = 0x10;
constexpr std::uint32_t mmio_base = 0xfffee000;
constexpr std::uint32_t mmio_size = 0x2000;
constexpr std::uint32_t ram_base = 0x1000;
constexpr unsigned int source_count = 101;
constexpr unsigned int channel_count = 32;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Independent AINTC expectation at line " +
                             std::to_string(where.line()));
  }
}

template <typename Function> void rejected(Function function, const std::string &reason) {
  try {
    function();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Expected AINTC/MMIO failure did not occur");
}

std::uint32_t read(lead_a1::Aintc &device, std::uint32_t offset) {
  return static_cast<std::uint32_t>(device.read({.address = offset, .size = 4}));
}

void write(lead_a1::Aintc &device, std::uint32_t offset, std::uint32_t value) {
  device.write({.address = offset, .size = 4}, value);
}

void channel(lead_a1::Aintc &device, unsigned int source, unsigned int priority) {
  constexpr std::uint32_t channels = 0x400;
  device.write({.address = channels + source, .size = 1}, priority);
}

void configure(lead_a1::Aintc &device, std::uint32_t control = 0) {
  write(device, 4, control);
  write(device, 0x10, 1);
  write(device, host_enable_offset, 3);
  write(device, vector_base_offset, vector_base);
  write(device, vector_size_offset, 0);
}

void map(lead_a1::Memory &memory, lead_a1::Aintc &device, std::uint32_t address = mmio_base) {
  memory.map_device(
      {.address = address, .size = mmio_size, .name = "candidate AINTC", .executable = false},
      device);
}

void reset_and_gates() {
  lead_a1::Aintc device(lead_a1::Aintc::reset_reference());
  constexpr std::uint32_t revision = 0x4e82a900;
  require(read(device, 0) == revision && read(device, 4) == 0);
  require(read(device, global_nesting_offset) == 0x100 &&
          read(device, irq_nesting_offset) == 0x100);
  require(read(device, 0x80) == no_interrupt && read(device, fiq_vector_offset) == 0);
  device.pulse(7);
  require(read(device, 0x200) == 0x80 && read(device, status_enabled_offset) == 0);
  require(device.outputs() == (std::array{false, false}));
  write(device, enable_set_offset, 7);
  require(read(device, status_enabled_offset) == 0x80 && read(device, fiq_index_offset) == 7);
  require(device.outputs() == (std::array{false, false}));
  write(device, 0x10, 1);
  write(device, host_set_offset, 0);
  require(device.outputs() == (std::array{true, false}));
  channel(device, 7, 2);
  require(read(device, fiq_index_offset) == no_interrupt && read(device, irq_index_offset) == 7);
  require(device.outputs() == (std::array{false, false}));
  write(device, host_set_offset, 1);
  require(device.outputs() == (std::array{false, true}));
  write(device, host_clear_offset, 1);
  require(device.outputs() == (std::array{false, false}));
  require(read(device, 0x80) == 7 && read(device, irq_index_offset) == 7);
  write(device, enable_clear_offset, 7);
  require(read(device, 0x200) == 0x80 && read(device, status_enabled_offset) == 0);
  for (const auto offset : {0x20U, 0x24U, 0x28U, 0x2cU, 0x34U, 0x38U}) {
    require(read(device, offset) == 0);
  }
}

void source_channel_matrix() {
  for (unsigned int source = 0; source < source_count; ++source) {
    for (unsigned int priority = 0; priority < channel_count; ++priority) {
      lead_a1::Aintc device(lead_a1::Aintc::reset_reference());
      configure(device);
      channel(device, source, priority);
      write(device, enable_set_offset, source);
      device.pulse(source);
      const unsigned int host = priority < 2 ? 0 : 1;
      require(read(device, fiq_index_offset + (host * 4)) == source);
      require(read(device, fiq_index_offset + ((1 - host) * 4)) == no_interrupt);
      require(read(device, fiq_vector_offset + (host * 4)) == vector_base + (source * 4));
      require(read(device, 0x80) == source);
      require(device.outputs() == (std::array{priority < 2, priority >= 2}));
      write(device, status_clear_offset, source);
      require(read(device, 0x80) == no_interrupt);
      require(device.outputs() == (std::array{false, false}));
    }
  }
}

void banks_and_priority() {
  lead_a1::Aintc device(lead_a1::Aintc::reset_reference());
  configure(device);
  constexpr std::array<std::uint32_t, 4> masks{0x80000001, 0x80000001, 0x80000001, 0x11};
  for (unsigned int index = 0; index < masks.size(); ++index) {
    const auto displacement = index * 4;
    write(device, 0x200 + displacement, masks[index]);
    write(device, enable_bank_offset + displacement, masks[index]);
    require(read(device, status_enabled_offset + displacement) == masks[index]);
    require(read(device, disable_bank_offset + displacement) == masks[index]);
    write(device, disable_bank_offset + displacement, masks[index]);
    require(read(device, enable_bank_offset + displacement) == 0);
    require(read(device, 0x200 + displacement) == masks[index]);
    write(device, status_enabled_offset + displacement, masks[index]);
    require(read(device, 0x200 + displacement) == 0);
  }
  constexpr std::array<unsigned int, 5> sources{7, 90, 3, 100, 45};
  constexpr std::array<unsigned int, 5> priorities{5, 2, 5, 1, 0};
  for (unsigned int index = 0; index < sources.size(); ++index) {
    channel(device, sources[index], priorities[index]);
    write(device, enable_set_offset, sources[index]);
    device.pulse(sources[index]);
  }
  require(read(device, irq_index_offset) == priority_source_ninety &&
          read(device, fiq_index_offset) == priority_source_forty_five &&
          read(device, 0x80) == priority_source_forty_five);
  write(device, status_clear_offset, priority_source_ninety);
  require(read(device, irq_index_offset) == 3);
  write(device, status_clear_offset, priority_source_forty_five);
  require(read(device, fiq_index_offset) == priority_source_hundred &&
          read(device, 0x80) == priority_source_hundred);
  write(device, status_clear_offset, priority_source_hundred);
  require(read(device, 0x80) == 3);
}

void holding_and_vectors() {
  lead_a1::Aintc device(lead_a1::Aintc::reset_reference());
  configure(device, hold_mode);
  constexpr std::uint32_t null_address = 0x12345678;
  write(device, null_vector_offset, null_address);
  require(read(device, fiq_index_offset) == no_interrupt);
  write(device, enable_set_offset, 5);
  device.pulse(5);
  require(device.outputs()[0] && read(device, fiq_vector_offset) == null_address);
  write(device, fiq_index_offset, 0);
  require(read(device, fiq_vector_offset) == source_five_vector);
  write(device, vector_base_offset, 0x20000);
  require(read(device, fiq_vector_offset) == source_five_vector &&
          read(device, fiq_index_offset) == 5);
  write(device, enable_set_offset, 1);
  device.pulse(1);
  require(read(device, 0x80) == 1 && read(device, fiq_index_offset) == 5);
  write(device, host_set_offset, 0);
  require(read(device, fiq_index_offset) == 1 &&
          read(device, fiq_vector_offset) == source_one_vector);
  write(device, host_clear_offset, 0);
  write(device, status_clear_offset, 1);
  require(read(device, fiq_index_offset) == 5);
  write(device, fiq_vector_offset, 0);
  write(device, status_clear_offset, 5);
  require(read(device, fiq_vector_offset) == null_address);
  write(device, 4, 0);
  write(device, vector_size_offset, 3);
  write(device, 0x20, 5);
  require(read(device, fiq_vector_offset) == larger_stride_vector);
  write(device, vector_base_offset, wrapping_vector_base);
  constexpr unsigned int largest_size = 29;
  write(device, vector_size_offset, largest_size);
  require(read(device, fiq_vector_offset) == wrapped_vector);
}

void rejected_registers() {
  lead_a1::Aintc device(lead_a1::Aintc::reset_reference());
  constexpr std::array<std::uint32_t, 12> offsets{
      4, 4, 4, 0x10, 0x20, 0x34, 0x54, 0x20c, 0x1500, 0x1100, 0, 0x1608};
  constexpr std::array<std::uint32_t, 12> values{1, 2, 0x80, 2, 101, 2, 30, 32, 4, 0x200, 0, 0};
  constexpr std::array<const char *, 12> reasons{"Reserved",
                                                 "Reserved",
                                                 "Reserved",
                                                 "Reserved",
                                                 "source",
                                                 "Reserved",
                                                 "size",
                                                 "Reserved",
                                                 "Reserved",
                                                 "Reserved",
                                                 "Unsupported",
                                                 "Unsupported"};
  for (unsigned int index = 0; index < offsets.size(); ++index) {
    const auto before = device.state();
    rejected([&] { write(device, offsets[index], values[index]); }, reasons[index]);
    require(device.state() == before);
  }
  rejected([&] { device.write({.address = 0x400, .size = 4}, invalid_channel_word); },
           "channel index");
  require(read(device, 0x400) == 0);
  rejected([&] { device.write({.address = last_channel_word, .size = 4}, 0x100); }, "Reserved");
  rejected([&] { static_cast<void>(device.read({.address = channel_one_offset, .size = 2})); },
           "channel-map");
  rejected([&] { static_cast<void>(device.read({.address = 4, .size = 1})); }, "word");
  rejected([&] { static_cast<void>(read(device, 8)); }, "Unsupported");
  write(device, enable_set_offset, 0);
  device.pulse(0);
  require(read(device, fiq_index_offset) == 0);
  rejected([&] { static_cast<void>(read(device, fiq_vector_offset)); }, "not been programmed");
  write(device, 4, hold_mode);
  const auto before = device.state();
  rejected([&] { static_cast<void>(read(device, fiq_index_offset)); }, "not been programmed");
  require(device.state() == before);
  auto unknown = lead_a1::Aintc::reset_reference();
  unknown.revision.reset();
  lead_a1::Aintc no_revision(unknown);
  rejected([&] { static_cast<void>(read(no_revision, 0)); }, "Unknown");
}

void channel_lanes_and_hold_release() {
  lead_a1::Aintc device(lead_a1::Aintc::reset_reference());
  configure(device, hold_mode);
  write(device, 0x400, initial_channel_word);
  require(device.read({.address = channel_one_offset, .size = 1}) == 1);
  require(device.read({.address = channel_two_offset, .size = 2}) == initial_upper_channels);
  device.write({.address = channel_two_offset, .size = 2}, updated_upper_channels);
  require(read(device, 0x400) == updated_channel_word);
  device.pulse(0);
  device.pulse(2);
  write(device, enable_bank_offset, 5);
  require(read(device, fiq_index_offset) == 0 && read(device, irq_index_offset) == 2);
  write(device, status_enabled_offset, 5);
  write(device, host_enable_offset, 1);
  require(read(device, fiq_index_offset) == no_interrupt && read(device, irq_index_offset) == 2);
  write(device, irq_vector_offset, 0);
  require(read(device, irq_index_offset) == no_interrupt);
  write(device, irq_nesting_offset, nesting_override_thirty);
  write(device, global_nesting_offset, nesting_override_initial);
  require(read(device, irq_nesting_offset) == nesting_thirty &&
          read(device, global_nesting_offset) == 0x100);
  write(device, 4, 12);
  require(read(device, 4) == 12);
}

void memory_transactions() {
  lead_a1::Aintc first(lead_a1::Aintc::reset_reference());
  lead_a1::Aintc second(lead_a1::Aintc::reset_reference());
  configure(first, hold_mode);
  configure(second, hold_mode);
  lead_a1::Memory memory;
  map(memory, first);
  constexpr std::uint32_t second_base = 0x4000;
  map(memory, second, second_base);
  memory.map({.address = ram_base, .size = 4, .name = "owned RAM"});
  memory.write({.address = ram_base, .size = 4}, 7);
  memory.set_pc(ram_base);
  const auto before = first.state();
  require(!memory.peek(mmio_base + fiq_index_offset) && first.state() == before);
  const auto other_before = second.state();
  memory.begin_instruction();
  memory.write({.address = ram_base, .size = 4}, 8);
  memory.write({.address = mmio_base + 0x20, .size = 4}, 5);
  require(memory.read({.address = mmio_base + fiq_index_offset, .size = 4}) == no_interrupt);
  require(memory.read({.address = second_base + fiq_index_offset, .size = 4}) == no_interrupt);
  require(first.state().held[0] && second.state().held[0]);
  rejected([&] { static_cast<void>(memory.read({.address = mmio_base + 8, .size = 4})); },
           "address=0xfffee008 pc=0x1000 width=4: candidate AINTC: Unsupported");
  memory.abort_instruction();
  require(first.state() == before && second.state() == other_before);
  require(memory.read({.address = ram_base, .size = 4}) == 7);
  memory.begin_instruction();
  memory.write({.address = ram_base, .size = 4}, 8);
  memory.write({.address = mmio_base + enable_set_offset, .size = 4}, 5);
  memory.write({.address = mmio_base + 0x20, .size = 4}, 5);
  require(memory.read({.address = mmio_base + fiq_index_offset, .size = 4}) == 5);
  memory.commit_instruction();
  require(first.state().held[0] && first.outputs()[0]);
  require(memory.read({.address = ram_base, .size = 4}) == 8);
  rejected(
      [&] {
        static_cast<void>(
            memory.read({.address = mmio_base, .size = 4}, lead_a1::AccessKind::fetch));
      },
      "fetch denied");
  rejected([&] { memory.map({.address = mmio_base, .size = 4, .name = "overlap"}); },
           "Overlapping");
  rejected([&] { map(memory, first, 0x8000); }, "already mapped");
  rejected([&] { map(memory, second, ram_base); }, "Overlapping");
}

void invalid_state_and_lifecycle() {
  auto initial = lead_a1::Aintc::reset_reference();
  initial.channels.back() = channel_count;
  rejected([&] { const lead_a1::Aintc invalid(initial); }, "Invalid");
  initial = lead_a1::Aintc::reset_reference();
  initial.enabled.back() = channel_count;
  rejected([&] { const lead_a1::Aintc invalid(initial); }, "Invalid");
  initial = lead_a1::Aintc::reset_reference();
  initial.held[0] = lead_a1::AintcPriority{};
  rejected([&] { const lead_a1::Aintc invalid(initial); }, "without priority");
  lead_a1::Aintc device(lead_a1::Aintc::reset_reference());
  configure(device, hold_mode);
  const auto before = device.state();
  {
    lead_a1::Memory memory;
    map(memory, device);
    memory.begin_instruction();
    static_cast<void>(memory.read({.address = mmio_base + fiq_index_offset, .size = 4}));
    require(device.state().held[0].has_value());
    rejected([&] { device.pulse(0); }, "active instruction");
    rejected([&] { device.begin_instruction(); }, "Nested");
    rejected([&] { memory.begin_instruction(); }, "Nested");
  }
  require(device.state() == before);
  lead_a1::Memory read_only;
  read_only.map_device({.address = mmio_base,
                        .size = mmio_size,
                        .name = "read-only AINTC",
                        .writable = false,
                        .executable = false},
                       device);
  rejected([&] { read_only.write({.address = mmio_base, .size = 4}, 0); }, "permission");
  rejected([&] { static_cast<void>(read_only.read({.address = mmio_base, .size = 0})); }, "width");
  require(device.state() == before);
}

}

int main() {
  try {
    reset_and_gates();
    source_channel_matrix();
    banks_and_priority();
    holding_and_vectors();
    rejected_registers();
    channel_lanes_and_hold_release();
    memory_transactions();
    invalid_state_and_lifecycle();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
