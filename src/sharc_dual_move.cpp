// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <sstream>
#include <stdexcept>
#include <string>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t all_bits = 0xffffffff;
constexpr std::uint32_t iop_end = 0x40000;
constexpr std::uint64_t compute_mask = 0x7fffff;

struct DualTransfer {
  unsigned int index{};
  unsigned int reg{};
  std::uint32_t address{};
  std::uint32_t next{};
  bool write{};
  SharcKnown40 value;
};

DualTransfer decode(SharcState &state, std::uint64_t opcode, bool program) {
  constexpr unsigned int dm_index_shift = 41;
  constexpr unsigned int pm_index_shift = 30;
  constexpr unsigned int dm_modifier_shift = 38;
  constexpr unsigned int pm_modifier_shift = 27;
  constexpr unsigned int dm_reg_shift = 33;
  constexpr unsigned int pm_reg_shift = 23;
  constexpr unsigned int dm_write_shift = 44;
  constexpr unsigned int pm_write_shift = 37;
  const auto index =
      static_cast<unsigned int>((opcode >> (program ? pm_index_shift : dm_index_shift)) & 7) +
      (program ? 8U : 0U);
  const auto modifier =
      static_cast<unsigned int>((opcode >> (program ? pm_modifier_shift : dm_modifier_shift)) & 7) +
      (program ? 8U : 0U);
  sharc_addressing_guard(state, index);
  const auto address = sharc_value(sharc_register_read(state, 16 + index));
  const auto delta = sharc_value(sharc_register_read(state, 32 + modifier));
  const auto reg =
      static_cast<unsigned int>((opcode >> (program ? pm_reg_shift : dm_reg_shift)) & 15);
  return {.index = index,
          .reg = reg,
          .address = address,
          .next = static_cast<std::uint32_t>(std::uint64_t{address} + delta),
          .write = ((opcode >> (program ? pm_write_shift : dm_write_shift)) & 1) != 0,
          .value = sharc_data_register(state, reg)};
}

void iop_transfer(SharcMemory &memory, SharcState &captured, DualTransfer transfer) {
  constexpr std::uint32_t sysctl = 0x30024;
  constexpr std::uint32_t monitor = 0x100000;
  // PGR7-5/8: peripheral IOP dual access is prohibited; only this core
  // control device has been qualified. Do not infer domains from an address range.
  if (transfer.address != sysctl) {
    throw std::runtime_error("SHARC Type1 IOP target/domain not qualified");
  }
  const auto mode = captured.effective[4];
  if ((mode.known & monitor) == 0 || (mode.value & monitor) != 0) {
    throw std::runtime_error("SHARC IOP access monitor/interrupt not implemented");
  }
  if (transfer.write) {
    memory.write({.address = transfer.address},
                 sharc_value(sharc_register_read(captured, transfer.reg)));
  } else {
    const auto value = static_cast<std::uint32_t>(memory.read({.address = transfer.address}));
    sharc_register_write(captured, transfer.reg, {.value = value, .known = all_bits});
  }
}

void execute_transfer(SharcMemory &memory,
                      SharcState &captured,
                      DualTransfer &transfer,
                      const SharcControl *control) try {
  // Every store reads its pre-instruction source, even after a DM load aliases it.
  sharc_data_register(captured, transfer.reg) = transfer.value;
  if (transfer.address < iop_end) {
    iop_transfer(memory, captured, transfer);
  } else {
    sharc_ram_transfer(
        memory,
        captured,
        {.access = {.address = transfer.address}, .ureg = transfer.reg, .write = transfer.write},
        control);
  }
  if (!transfer.write) { transfer.value = sharc_data_register(captured, transfer.reg); }
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "word-address=0x" << std::hex << transfer.address << ' '
          << (transfer.write ? "write" : "read") << ": " << error.what();
  throw std::runtime_error(message.str());
}

}

void sharc_dual_move(SharcMemory &memory,
                     SharcState &state,
                     std::uint64_t opcode,
                     const SharcControl *control) try {
  auto dm = decode(state, opcode, false);
  auto pm = decode(state, opcode, true);
  auto captured = state;
  // PGR4-110: on a same-block conflict the DM access completes first.
  execute_transfer(memory, captured, dm, control);
  execute_transfer(memory, captured, pm, control);
  const auto compute = opcode & compute_mask;
  if (compute != 0) {
    constexpr std::uint64_t unconditional_compute = 0x013e00000000;
    sharc_compute(state, unconditional_compute | compute);
  }
  // PGR2-6/7: DM load > PM load > compute; flags still come from compute.
  if (!pm.write) { sharc_data_register(state, pm.reg) = pm.value; }
  if (!dm.write) { sharc_data_register(state, dm.reg) = dm.value; }
  sharc_register_write(state, 16 + dm.index, {.value = dm.next, .known = all_bits});
  sharc_register_write(state, 16 + pm.index, {.value = pm.next, .known = all_bits});
  sharc_cache_program_access(state);
} catch (const std::exception &error) {

  throw std::runtime_error("SHARC dual DM/PM transfer: " + std::string(error.what()));
}

}
