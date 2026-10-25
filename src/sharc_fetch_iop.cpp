// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/memory.hpp"
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "lead_a1/sharc_iop_bridge.hpp"
#include "lead_a1/sharc_memory.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace lead_a1 {
namespace {

void context_guard(const SharcState &core, const SharcFetchState &fetch) {
  if (core.branch || core.count_loop || core.arithmetic_loop || core.interrupt || core.idle_pc ||
      core.pending[detail::sharc_mode1] || fetch.lookup || fetch.fill || fetch.pending_fetch2 ||
      (fetch.flow &&
       (fetch.flow->loop || fetch.flow->long_loop || fetch.flow->discard_decode != 0))) {
    throw std::runtime_error(
        "SHARC IOP hold with mode/branch/cache/loop/IRQ context not qualified");
  }
  const auto mode1 = core.effective[detail::sharc_mode1];
  if ((mode1.known & detail::sharc_simd) == 0 || (mode1.value & detail::sharc_simd) != 0) {
    throw std::runtime_error("SHARC SIMD IOP hold not qualified");
  }
  constexpr std::uint32_t monitor = 0x100000;
  const auto mode2 = core.effective[4];
  const auto pending = core.pending[4];
  if ((mode2.known & monitor) == 0 || (mode2.value & monitor) != 0 ||
      (pending && ((pending->known & monitor) == 0 || (pending->value & monitor) != 0))) {
    throw std::runtime_error("SHARC monitored IOP hold not qualified");
  }
}

}

std::optional<SharcFetchIopAccess> SharcFetch::peripheral_access(const Sharc &cpu) const try {
  if (&cpu.memory_ != &memory_ || memory_.active()) {
    throw std::logic_error("SHARC IOP preview requires its idle bound memory");
  }
  if (!state_.execute) { return {}; }
  const auto opcode = for_execution(memory_, cpu.state_.pc);
  constexpr unsigned int class_shift = 40;
  constexpr unsigned int direct_read = 0x10;
  constexpr unsigned int direct_write = 0x11;
  constexpr unsigned int program_write = 0x13;
  const auto type = opcode >> class_shift;
  constexpr std::uint32_t peripheral_end = 0x10000;
  const auto address = static_cast<std::uint32_t>(opcode);
  if (type < direct_read || type > program_write || address >= peripheral_end) { return {}; }
  constexpr std::uint64_t forced_long = 0x8000000000;
  constexpr unsigned int register_shift = 32;
  constexpr unsigned int register_mask = 127;
  constexpr unsigned int data_registers = 16;
  constexpr unsigned int ustat1 = 0x70;
  constexpr unsigned int ustat2 = 0x71;
  const auto reg = (opcode >> register_shift) & register_mask;
  const bool scalar = reg < data_registers || reg == ustat1 || reg == ustat2;
  if (type > direct_write || (opcode & forced_long) != 0 || !scalar || state_.execute->dag ||
      state_.execute->pm_data != false) {
    throw std::runtime_error(
        "SHARC IOP preview requires an unconditional DM/DREG/USTAT32 transfer");
  }
  context_guard(cpu.state_, state_);
  return SharcFetchIopAccess{.pc = cpu.state_.pc,
                             .address = address,
                             .operation =
                                 type == direct_read ? AccessKind::read : AccessKind::write};
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC fetched IOP preview pc=0x" << std::hex << cpu.state_.pc;
  if (state_.execute) {
    constexpr unsigned int class_shift = 40;
    constexpr unsigned int first_transfer = 0x10;
    constexpr unsigned int last_transfer = 0x13;
    const auto type = state_.execute->opcode >> class_shift;
    const auto *operation = "preview";
    if (type >= first_transfer && type <= last_transfer) {
      operation = (type & 1) != 0 ? "write" : "read";
    }
    message << " word-address=0x" << static_cast<std::uint32_t>(state_.execute->opcode)
            << " operation=" << operation << " opcode=0x" << state_.execute->opcode;
  }
  message << ": " << error.what();
  throw std::runtime_error(message.str());
}

void SharcFetch::wait_iop(Sharc &cpu, const SharcIopBridge &bridge) try {
  const auto access = peripheral_access(cpu);
  if (!access) { throw std::runtime_error("SHARC IOP wait requires an actual peripheral Execute"); }
  const auto bus = bridge.snapshot();
  const auto request = bus.request;
  if (!request || request->pc != access->pc || request->address != access->address ||
      request->operation != access->operation || !bridge.bound_to(memory_, access->address) ||
      !bridge.waiting()) {
    throw std::runtime_error("SHARC IOP wait requires its exact unready bound request");
  }
  auto core = cpu.state_;
  auto held = state_;
  held.execute.reset(); // This clock retires no instruction, including the held real word.
  detail::sharc_fetch_quiet_effects(core, held, cpu.control_);
  detail::sharc_validate_cache(core.cache);
  static_assert(std::is_nothrow_copy_assignable_v<SharcState>);
  if (cpu.control_ != nullptr) { cpu.control_->advance_internal_clocks(1); }
  cpu.state_ = core;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC fetched IOP wait pc=0x" << std::hex << cpu.state_.pc;
  if (state_.execute) {
    message << " operation=hold word-address=0x"
            << static_cast<std::uint32_t>(state_.execute->opcode) << " opcode=0x"
            << state_.execute->opcode;
  }
  message << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
