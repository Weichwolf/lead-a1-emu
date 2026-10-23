// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc_memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

using lead_a1::SharcWidth;
constexpr std::uint32_t short_base = 0x124000;
constexpr std::uint32_t normal_base = 0x92000;
constexpr std::uint32_t wide_base = 0x8c000;
constexpr std::uint32_t long_base = 0x49000;
constexpr std::uint64_t first_instruction = 0x112233445566;
constexpr std::uint64_t second_instruction = 0x7788aabbccdd;
constexpr std::uint32_t first_word = 0x33445566;
constexpr std::uint32_t second_word = 0xccdd1122;
constexpr std::uint32_t third_word = 0x7788aabb;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("SHARC memory expectation at line " + std::to_string(where.line()));
  }
}

template <typename Function> void rejected(const Function &action, const std::string &reason) {
  try {
    action();
  } catch (const std::exception &error) {
    require(std::string(error.what()).find(reason) != std::string::npos);
    return;
  }
  throw std::runtime_error("Missing SHARC memory rejection: " + reason);
}

std::uint64_t read(const lead_a1::SharcMemory &memory, std::uint32_t address, SharcWidth width) {
  return memory.read({.address = address, .width = width});
}

void golden_boot_packing() {
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  // HWR Figure 24-4, independent 3x32 -> 2x48 boot-packing example.
  memory.write({.address = normal_base}, first_word);
  memory.write({.address = normal_base + 1}, second_word);
  memory.write({.address = normal_base + 2}, third_word);
  require(read(memory, wide_base, SharcWidth::wide48) == first_instruction);
  require(read(memory, wide_base + 1, SharcWidth::wide48) == second_instruction);
  constexpr std::uint64_t first_long = 0xccdd112233445566;
  require(read(memory, long_base, SharcWidth::long64) == first_long);
  require(read(memory, normal_base, SharcWidth::long64) == first_long);
  // Forced LW discards address bit0; a future core must implement MODE2.U64MAE.
  require(read(memory, normal_base + 1, SharcWidth::long64) == first_long);
  constexpr std::array<std::uint16_t, 6> halfwords{0x5566, 0x3344, 0x1122, 0xccdd, 0xaabb, 0x7788};
  for (std::uint32_t index = 0; index < halfwords.size(); ++index) {
    require(read(memory, short_base + index, SharcWidth::short16) == halfwords[index]);
  }
  memory.write({.address = short_base + 3, .width = SharcWidth::short16}, 0);
  constexpr std::uint64_t changed_instruction = 0x7788aabb0000;
  require(read(memory, wide_base + 1, SharcWidth::wide48) == changed_instruction);
}

void extended_knowledge_and_rotations() {
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  constexpr std::uint64_t extended = 0x123456789a;
  memory.write({.address = wide_base, .width = SharcWidth::extended40}, extended);
  require(read(memory, wide_base, SharcWidth::extended40) == extended);
  rejected([&] { static_cast<void>(read(memory, wide_base, SharcWidth::wide48)); }, "Unknown");
  rejected([&] { static_cast<void>(read(memory, short_base, SharcWidth::short16)); }, "Unknown");
  constexpr std::uint16_t low_halfword = 0x9abc;
  memory.write({.address = short_base, .width = SharcWidth::short16}, low_halfword);
  memory.write({.address = wide_base, .width = SharcWidth::extended40}, extended);
  constexpr std::uint64_t preserved_low_byte = 0x123456789abc;
  require(read(memory, wide_base, SharcWidth::wide48) == preserved_low_byte);
  // Four successive 48-bit words rotate over all four physical columns.
  constexpr std::array<std::uint16_t, 12> halves{
      0x102, 0x304, 0x506, 0x708, 0x90a, 0xb0c, 0xd0e, 0xf10, 0x1112, 0x1314, 0x1516, 0x1718};
  for (std::uint32_t index = 0; index < halves.size(); ++index) {
    memory.write({.address = short_base + index, .width = SharcWidth::short16}, halves[index]);
  }
  constexpr std::array<std::uint64_t, 4> wide_words{
      0x050603040102, 0x0b0c090a0708, 0x11120f100d0e, 0x171815161314};
  for (std::uint32_t index = 0; index < wide_words.size(); ++index) {
    require(read(memory, wide_base + index, SharcWidth::wide48) == wide_words[index]);
  }
}

void geometry() {
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  // Datasheet Rev.I Table 5; literal endpoints for all four bank views.
  constexpr std::array<std::array<std::uint32_t, 4>, 4> first{
      {{0x124000, 0x92000, 0x8c000, 0x49000},
       {0x164000, 0xb2000, 0xac000, 0x59000},
       {0x180000, 0xc0000, 0xc0000, 0x60000},
       {0x1c0000, 0xe0000, 0xe0000, 0x70000}}};
  constexpr std::array<std::array<std::uint32_t, 4>, 4> last{
      {{0x13bfff, 0x9dfff, 0x93fff, 0x4efff},
       {0x17bfff, 0xbdfff, 0xb3fff, 0x5efff},
       {0x18ffff, 0xc7fff, 0xc5554, 0x63fff},
       {0x1cffff, 0xe7fff, 0xe5554, 0x73fff}}};
  constexpr std::array<SharcWidth, 4> widths{
      SharcWidth::short16, SharcWidth::normal32, SharcWidth::wide48, SharcWidth::long64};
  for (std::size_t bank = 0; bank < first.size(); ++bank) {
    for (std::size_t width = 0; width < widths.size(); ++width) {
      const lead_a1::SharcAccess start{.address = first[bank][width], .width = widths[width]};
      const lead_a1::SharcAccess end{.address = last[bank][width], .width = widths[width]};
      memory.write(start, 1);
      memory.write(end, 1);
      require(memory.read(start) == 1 && memory.read(end) == 1);
      const lead_a1::SharcAccess after{.address = end.address + 1, .width = end.width};
      rejected([&] { memory.write(after, 0); }, "Unmapped");
      if (bank < 2 || width == 0 || width == 3) {
        const lead_a1::SharcAccess before{.address = start.address - 1, .width = start.width};
        rejected([&] { memory.write(before, 0); }, "Unmapped");
      }
    }
  }
  rejected([&] { memory.map_2148x_5mbit_ram(); }, "empty memory");
  constexpr std::uint32_t physical_ram_start = 0x24000;
  rejected(
      [&] {
        memory.map(
            {.bank = 0, .first_halfword = physical_ram_start, .halfwords = 1, .name = "overlap"});
      },
      "Overlapping");
}

class Register final : public lead_a1::MemoryDevice {
public:
  std::uint64_t value{};
  std::uint64_t saved{};
  std::uint32_t offset{};
  unsigned int begins{};
  bool pending{};

  std::uint64_t read(lead_a1::MemoryAccess access) override {
    require(access.size == 4);
    offset = access.address;
    return value;
  }

  void write(lead_a1::MemoryAccess access, std::uint64_t next) override {
    require(access.size == 4);
    offset = access.address;
    value = next;
  }

  void begin_instruction() override {
    require(!pending);
    ++begins;
    saved = value;
    pending = true;
  }

  void commit_instruction() noexcept override { pending = false; }

  void abort_instruction() noexcept override {
    value = saved;
    pending = false;
  }
};

void transactions_and_iop() {
  Register device;
  lead_a1::SharcMemory memory;
  memory.map_2148x_5mbit_ram();
  memory.map_io({.first_word = 0x1000, .words = 7}, device);
  memory.write({.address = normal_base}, first_word);
  memory.begin_instruction();
  memory.write({.address = wide_base, .width = SharcWidth::wide48}, first_instruction);
  require(read(memory, normal_base, SharcWidth::normal32) == first_word);
  constexpr std::uint32_t txspi = 0x1003;
  memory.write({.address = txspi}, 7);
  require(read(memory, txspi, SharcWidth::normal32) == 7 && device.offset == 12);
  require(device.begins == 1);
  rejected([&] { static_cast<void>(read(memory, normal_base + 2, SharcWidth::normal32)); },
           "Unknown");
  rejected([&] { memory.begin_instruction(); }, "Nested");
  rejected([&] { memory.map_io({.first_word = 0, .words = 1}, device); }, "Invalid");
  memory.abort_instruction();
  require(device.value == 0 && !device.pending && !memory.active());
  rejected([&] { static_cast<void>(read(memory, wide_base, SharcWidth::wide48)); }, "Unknown");
  memory.begin_instruction();
  memory.write({.address = wide_base, .width = SharcWidth::wide48}, second_instruction);
  memory.write({.address = 0x1000}, 9);
  memory.commit_instruction();
  require(device.value == 9 && !device.pending);
  require(read(memory, wide_base, SharcWidth::wide48) == second_instruction);
  rejected([&] { memory.commit_instruction(); }, "Missing");
  rejected([&] { memory.map_io({.first_word = 0x1000, .words = 1}, device); }, "Overlapping");
  rejected([&] { static_cast<void>(read(memory, 0x1000, SharcWidth::short16)); }, "32-bit");
  rejected([&] { static_cast<void>(memory.read({.address = 0x1000}, lead_a1::AccessKind::fetch)); },
           "fetch from IOP");
  device.value = std::numeric_limits<std::uint64_t>::max();
  rejected([&] { static_cast<void>(read(memory, 0x1000, SharcWidth::normal32)); }, "Oversized");
}

void mapping_guards() {
  lead_a1::SharcMemory memory;
  constexpr std::array<lead_a1::SharcCell, 2> initial{
      {{.value = 1, .known = 0xffff}, {.value = 2, .known = 0xffff}}};
  memory.map({.bank = 0, .first_halfword = 0, .halfwords = 2, .name = "writable"}, initial);
  memory.map({.bank = 0, .first_halfword = 2, .halfwords = 2, .name = "ROM", .writable = false},
             initial);
  constexpr std::uint32_t nw = 0x80000;
  constexpr std::uint32_t sw = 0x100000;
  constexpr std::uint64_t old = 0x20001;
  rejected([&] { memory.write({.address = nw, .width = SharcWidth::wide48}, 0); }, "Read-only");
  require(read(memory, nw, SharcWidth::normal32) == old);
  rejected([&] { memory.write({.address = nw + 2}, 0); }, "Unmapped");
  memory.map({.bank = 1, .first_halfword = 0, .halfwords = 2, .name = "data", .executable = false},
             initial);
  constexpr std::uint32_t data_address = 0xa0000;
  rejected(
      [&] {
        static_cast<void>(memory.read({.address = data_address}, lead_a1::AccessKind::fetch));
      },
      "Non-executable");
  memory.set_pc(wide_base);
  rejected([&] { static_cast<void>(read(memory, sw, SharcWidth::normal32)); },
           "read pc=0x8c000 word-address=0x100000 bits=32");
  rejected([&] { memory.write({.address = sw, .width = SharcWidth::short16}, first_word); },
           "write pc=0x8c000");
  rejected([&] { static_cast<void>(memory.read({.address = nw}, lead_a1::AccessKind::write)); },
           "Invalid SHARC read kind");
  rejected([&] { memory.map({.bank = 4, .halfwords = 1, .name = "bad bank"}); }, "Invalid");
  constexpr std::array<lead_a1::SharcCell, 1> unknown_bits{{{.value = 1, .known = 0}}};
  rejected([&] { memory.map({.bank = 2, .halfwords = 1, .name = "bad bits"}, unknown_bits); },
           "knowledge");
  memory.begin_instruction();
  constexpr unsigned int journal_cells = 32;
  for (unsigned int cell = 0; cell < journal_cells; ++cell) {
    memory.write({.address = sw, .width = SharcWidth::short16}, cell);
  }
  rejected([&] { memory.write({.address = nw}, 0); }, "journal full");
  memory.abort_instruction();
  require(read(memory, nw, SharcWidth::normal32) == old);
}

}

int main() {
  try {
    golden_boot_packing();
    extended_knowledge_and_rotations();
    geometry();
    transactions_and_iop();
    mapping_guards();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
