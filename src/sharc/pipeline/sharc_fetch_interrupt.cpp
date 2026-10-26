// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/pipeline/sharc_fetch_internal.hpp"

#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace lead_a1 {

std::optional<SharcInterruptStep> Sharc::service_boot_interrupt(SharcFetch &fetch) try {
  if (&fetch.memory_ != &memory_ || memory_.active()) {
    throw std::logic_error("SHARC fetched boot interrupt requires its idle bound memory");
  }
  SharcFetchState empty;
  if (fetch.state_.flow) {
    const auto cursor = fetch.state_.flow->next;
    if (cursor != state_.pc && cursor != state_.idle_pc) {
      throw std::runtime_error("SHARC boot fetch cursor must address reset/queued IDLE");
    }
    empty.flow = SharcFetchFlow{.next = cursor};
  }
  if (fetch.state_ != empty) {
    throw std::runtime_error("SHARC boot recognition requires an empty fetch transport");
  }
  auto core = state_;
  const auto event = detail::sharc_service_boot_interrupt(core);
  if (!event) { return {}; }
  SharcFetchState redirected;
  if (fetch.state_.flow) { redirected.flow = SharcFetchFlow{.next = core.pc}; }
  detail::sharc_validate_spi_boot(core);
  detail::sharc_validate_interrupt(core);
  detail::sharc_validate_idle(core);
  detail::sharc_validate_cache(core.cache);
  detail::sharc_fetch_validate(redirected);
  if (control_ != nullptr) { control_->advance_internal_clocks(event->minimum_core_cycles); }
  static_assert(std::is_nothrow_copy_assignable_v<SharcState>);
  static_assert(std::is_nothrow_copy_assignable_v<SharcFetchState>);
  state_ = core;
  fetch.state_ = redirected;
  return event;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC fetched SPI boot recognition pc=0x" << std::hex << state_.pc
          << " operation=boot-interrupt IRPTL=0x" << state_.effective[10].value << "/0x"
          << state_.effective[10].known << ": " << error.what();
  throw std::runtime_error(message.str());
}

std::optional<SharcInterruptStep> Sharc::service_interrupt(SharcFetch &fetch,
                                                           SharcInterruptReference reference) try {
  if (&fetch.memory_ != &memory_ || memory_.active()) {
    throw std::logic_error("SHARC fetched interrupt requires its idle bound memory");
  }
  if (fetch.state_.execute) {
    throw std::runtime_error("SHARC interrupt requires retired fetch Execute");
  }
  if (state_.arithmetic_loop && !fetch.state_.flow) {
    throw std::runtime_error("SHARC arithmetic interrupt requires its sequenced fetch transport");
  }
  if (fetch.state_.lookup || fetch.state_.fill || fetch.state_.pending_fetch2 ||
      (fetch.state_.flow && (fetch.state_.flow->loop || fetch.state_.flow->long_loop))) {
    throw std::runtime_error("SHARC interrupt with pending cache/loop transport not qualified");
  }
  auto core = state_;
  const auto event = detail::sharc_service_interrupt(core, reference, true);
  if (!event) { return {}; }
  SharcFetchState redirected;
  if (fetch.state_.flow) { redirected.flow = SharcFetchFlow{.next = core.pc}; }
  detail::sharc_validate_interrupt(core);
  detail::sharc_validate_arithmetic_loop(core);
  detail::sharc_validate_idle(core);
  detail::sharc_validate_cache(core.cache);
  detail::sharc_fetch_validate(redirected);
  if (control_ != nullptr) { control_->advance_internal_clocks(event->minimum_core_cycles); }
  static_assert(std::is_nothrow_copy_assignable_v<SharcState>);
  static_assert(std::is_nothrow_copy_assignable_v<SharcFetchState>);
  state_ = core;
  fetch.state_ = redirected;
  return event;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC fetched interrupt pc=0x" << std::hex << state_.pc << " IVT=0x"
          << reference.vector_base << " IRPTL=0x" << state_.effective[10].value << "/0x"
          << state_.effective[10].known << " IMASK=0x" << state_.effective[11].value << "/0x"
          << state_.effective[11].known << " LIRPTL=0x" << state_.effective[13].value << "/0x"
          << state_.effective[13].known << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
