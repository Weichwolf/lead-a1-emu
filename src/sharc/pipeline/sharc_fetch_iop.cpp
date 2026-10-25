// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/memory/sharc_iop_bridge.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/pipeline/sharc_fetch_internal.hpp"
#include "sharc/pipeline/sharc_fetch_long.hpp"
#include "sharc/pipeline/sharc_fetch_mode.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace lead_a1 {
namespace {

bool count_loop_context(const SharcState &core, const SharcFetchState &fetch) {
  if (!core.count_loop) { return !fetch.flow || !fetch.flow->long_loop; }
  if (!fetch.flow || !fetch.flow->long_loop || !fetch.flow->long_loop->counter) { return false; }
  const auto &native = *core.count_loop;
  const auto depth = native.control ? native.control->depth : core.depth;
  const auto floor = native.saved_depth + (native.frame_live ? 1U : 0U);
  // Only a callee outside the loop body: no iteration or pre-pop retires on
  // these extra clocks. Count1 startup keeps its functional CALL bookkeeping
  // private; a scalar IOP transfer must not expose it as architectural state.
  if (!depth || (core.pc >= native.body && core.pc <= native.end) || *depth <= floor) {
    return false;
  }
  detail::sharc_fetch_long_guard(core, fetch);
  return true;
}

bool arithmetic_setup_context(const SharcState &core, const SharcFetchState &fetch) {
  if (!core.arithmetic_loop) { return true; }
  const auto &loop = *core.arithmetic_loop;
  if (loop.phase != SharcArithmeticPhase::deferred ||
      (core.pc >= loop.body && core.pc <= loop.end)) {
    return false;
  }
  detail::sharc_fetch_arithmetic_guard(core, fetch);
  return true;
}

void context_guard(const SharcState &core, const SharcFetchState &fetch) {
  // A captured callee branch or deferred-loop entry may suppress Decode. A
  // full IOP hold retains that decision and mask until the next real advance.
  if (core.branch || !count_loop_context(core, fetch) || !arithmetic_setup_context(core, fetch) ||
      core.interrupt || core.idle_pc ||
      (core.pending[detail::sharc_mode1] && !detail::sharc_fetch_interrupt_mode(core)) ||
      fetch.lookup || fetch.fill || fetch.pending_fetch2 ||
      (fetch.flow && (fetch.flow->loop || (fetch.flow->discard_decode != 0 && !core.count_loop &&
                                           !core.arithmetic_loop)))) {
    throw std::runtime_error(
        "SHARC IOP hold with mode/branch/cache/loop/IRQ context not qualified");
  }
  if (core.pending[detail::sharc_mode1]) { detail::sharc_fetch_interrupt_mode_guard(core, fetch); }
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
  constexpr unsigned int ustat3 = 0x7e;
  constexpr unsigned int ustat4 = 0x7f;
  constexpr unsigned int complement_end = 0x60;
  const auto reg = (opcode >> register_shift) & register_mask;
  const bool scalar = reg < data_registers || reg == ustat1 || reg == ustat2 || reg == ustat3 ||
                      reg == ustat4 || (type == direct_write && reg < complement_end);
  if (type > direct_write || (opcode & forced_long) != 0 || !scalar || state_.execute->dag ||
      state_.execute->pm_data != false) {
    throw std::runtime_error(
        "SHARC IOP preview requires an unconditional qualified scalar32 DM transfer");
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
