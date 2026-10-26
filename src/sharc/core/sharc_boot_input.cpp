// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_control.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace lead_a1 {
namespace {

constexpr std::uint32_t reset_vector = 0x8c004;
constexpr std::uint32_t first_program = reset_vector + 1;
constexpr std::uint32_t spi_vector = 0x8c030;
constexpr std::uint32_t spi_latch = 0x1000;
constexpr std::uint64_t boot_idle = 0x008000000000;
constexpr std::uint64_t initial_rti = 0x0b3e00000000;

bool empty_context(const SharcState &state) {
  return state.depth == 0 && !state.interrupt && !state.idle_pc;
}

}

namespace detail {

void sharc_validate_spi_boot(const SharcState &state) {
  if (!state.spi_boot) { return; }
  if (state.branch || state.count_loop || state.arithmetic_loop || state.loops_empty != true ||
      state.idle_delay_word || state.cache.pending_flush) {
    throw std::invalid_argument("Invalid SPI boot sequencer context");
  }
  for (const auto &pending : state.pending) {
    if (pending) { throw std::invalid_argument("Invalid pending SPI boot core effect"); }
  }
  switch (*state.spi_boot) {
    case SharcSpiBootPhase::input:
      if (state.pc == reset_vector && empty_context(state)) { return; }
      break;
    case SharcSpiBootPhase::idle:
      if (state.pc == first_program && state.idle_pc == reset_vector && state.depth == 0 &&
          !state.interrupt) {
        return;
      }
      break;
    case SharcSpiBootPhase::interrupt:
      if (state.pc == spi_vector && !state.idle_pc && state.depth == 1 && state.interrupt) {
        const auto frame = *state.interrupt;
        if (frame.bank == SharcInterruptBank::irptl && frame.source == spi_latch &&
            frame.vector == spi_vector && frame.return_pc == first_program &&
            frame.saved_depth == 0) {
          return;
        }
      }
      break;
    default: break;
  }
  throw std::invalid_argument("Invalid SPI boot input/IDLE/interrupt phase");
}

void sharc_boot_input_guard(const SharcState &state, bool boot_input) {
  sharc_validate_spi_boot(state);
  if (boot_input) {
    if (state.spi_boot != SharcSpiBootPhase::input) {
      throw std::runtime_error("SPI boot input requires its selected initial channel");
    }
  } else if (state.spi_boot && state.spi_boot != SharcSpiBootPhase::interrupt) {
    throw std::runtime_error("SPI boot requires its boot-channel IDLE, not program execution");
  }
}

void sharc_boot_opcode_guard(const SharcState &state, std::uint64_t opcode) {
  if (!state.spi_boot) { return; }
  if ((state.spi_boot == SharcSpiBootPhase::input && opcode == boot_idle) ||
      (state.spi_boot == SharcSpiBootPhase::interrupt && opcode == initial_rti)) {
    return;
  }
  throw std::runtime_error(
      "SPI boot input/initial vector requires its actual IDLE/RTI instruction");
}

}

SharcState Sharc::spi_slave_boot_reference_214xx() {
  auto state = reset_reference_214xx(reset_vector);
  // PGR4-29/30 establishes automatic boot-source unmasking, but does not
  // specify its architectural MODE1/IMASK readback. Preserve that uncertainty.
  for (const unsigned int reg : {detail::sharc_mode1, detail::sharc_imask}) {
    state.system[reg].known &= ~spi_latch;
    state.system[reg].value &= ~spi_latch;
    state.effective[reg] = state.system[reg];
  }
  state.spi_boot = SharcSpiBootPhase::input;
  return state;
}

SharcStep Sharc::step_boot_input() {
  // PGR8-19's dedicated pipeline source supplies IDLE while initial DMA loads
  // executable SRAM. This does not read, seed or patch any instruction RAM.
  return step_opcode(boot_idle,
                     false,
                     std::nullopt,
                     SharcLoopClock::functional,
                     nullptr,
                     nullptr,
                     false,
                     Input::boot);
}

std::optional<SharcInterruptStep> Sharc::service_boot_interrupt() try {
  if (memory_.active()) { throw std::logic_error("Active SHARC transaction at boot recognition"); }
  auto next = state_;
  const auto event = detail::sharc_service_boot_interrupt(next);
  detail::sharc_validate_spi_boot(next);
  detail::sharc_validate_interrupt(next);
  detail::sharc_validate_idle(next);
  if (event && control_ != nullptr) {
    control_->advance_internal_clocks(event->minimum_core_cycles);
  }
  static_assert(std::is_nothrow_copy_assignable_v<SharcState>);
  state_ = next;
  return event;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC SPI boot recognition pc=0x" << std::hex << state_.pc << " vector=0x"
          << spi_vector << " operation=boot-interrupt IRPTL=0x" << state_.effective[10].value
          << "/0x" << state_.effective[10].known << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
