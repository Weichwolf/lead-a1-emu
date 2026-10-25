// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/pipeline/sharc_fetch.hpp"

#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/pipeline/sharc_fetch_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint32_t native_begin = 0x80000;
constexpr std::uint32_t native_end = 0x100000;
constexpr std::uint64_t native_bits = 0xffffffffffff;
constexpr std::uint32_t cache_disabled = 0x10;

void cache_metadata(const SharcFetchState &state) {
  constexpr unsigned int entries = 32;
  constexpr unsigned int sets = 15;
  if ((state.fetch2 && state.fetch2->pm_data) || (state.decode && state.decode->pm_data)) {
    throw std::invalid_argument("Invalid SHARC PM request before Address");
  }
  if (state.lookup) {
    const auto lookup = *state.lookup;
    if (!state.fetch1 || lookup.mode.known != detail::sharc_fetch_controls ||
        (lookup.mode.value & ~lookup.mode.known) != 0 ||
        (lookup.hit && (!lookup.entry || lookup.wait)) ||
        (lookup.wait && (lookup.entry || lookup.hit)) ||
        (lookup.hit && (lookup.mode.value & cache_disabled) != 0) ||
        (!lookup.hit && lookup.entry && lookup.mode.value != 0) ||
        (!lookup.hit && !lookup.wait && !lookup.entry && lookup.mode.value == 0)) {
      throw std::invalid_argument("Invalid SHARC inherited cache lookup");
    }
    if (lookup.entry && (*lookup.entry >= entries || *lookup.entry / 2 != (*state.fetch1 & sets))) {
      throw std::invalid_argument("Invalid SHARC inherited cache lookup entry");
    }
  }
  if (state.fill) {
    const auto fill = *state.fill;
    if (!state.fetch2 || fill.entry >= entries ||
        fill.entry / 2 != (state.fetch2->address & sets) ||
        fill.mode.known != detail::sharc_fetch_controls || fill.mode.value != 0) {
      throw std::invalid_argument("Invalid SHARC inherited cache fill");
    }
  }
}

}

namespace detail {

void sharc_fetch_address(std::uint32_t address) {
  if (address < native_begin || address >= native_end) {
    throw std::invalid_argument("SHARC fetch requires an internal native address");
  }
}

void sharc_fetch_program_address(std::uint32_t address) {
  constexpr std::uint32_t visa_end = 0x200000;
  if (address < native_begin || address >= visa_end) {
    throw std::invalid_argument("SHARC fetch requires an internal native or VISA address");
  }
}

void sharc_fetch_validate(const SharcFetchState &state) {
  if (state.pending_fetch2) {
    sharc_fetch_address(*state.pending_fetch2);
    if (state.fetch2 || state.fill) {
      throw std::invalid_argument("Invalid SHARC pending F2 memory response");
    }
  }
  if (state.fetch1) { sharc_fetch_program_address(*state.fetch1); }
  for (const auto &word : {state.fetch2, state.decode, state.address, state.execute}) {
    if (word) {
      sharc_fetch_program_address(word->address);
      if ((word->opcode & ~native_bits) != 0) {
        throw std::invalid_argument("Invalid SHARC native fetched word");
      }
    }
  }
  sharc_fetch_visa_validate(state);
  cache_metadata(state);
  sharc_fetch_branch_validate(state);
  sharc_fetch_loop_validate(state);
  sharc_fetch_arithmetic_validate(state);
  sharc_fetch_dag_validate(state);
  if (state.flow) {
    sharc_fetch_program_address(state.flow->next);
    if (state.flow->discard_decode > 3) {
      throw std::invalid_argument("Invalid SHARC inherited Decode suppression");
    }
  }
}

}

SharcFetch::SharcFetch(SharcMemory &memory, SharcFetchState initial)
    : memory_(memory), state_(initial) {
  detail::sharc_fetch_validate(state_);
}

void SharcFetch::advance(SharcKnown32 effective_mode2,
                         std::optional<std::uint32_t> next_address) try {
  if (memory_.active()) {
    throw std::logic_error("SHARC fetch advance during a memory transaction");
  }
  if (state_.flow) {
    throw std::runtime_error("SHARC sequenced fetch requires its sequenced clock");
  }
  if (state_.visa || state_.visa_fetch_aborted) {
    throw std::runtime_error("SHARC VISA fetch requires its bound sequencer");
  }
  if (state_.lookup || state_.fill) {
    throw std::runtime_error("SHARC pending cache fetch requires its bound owner");
  }
  if (state_.pending_fetch2) {
    throw std::runtime_error("SHARC pending F2 memory response requires its bound owner");
  }
  if ((effective_mode2.value & ~effective_mode2.known) != 0) {
    throw std::invalid_argument("Invalid SHARC fetch MODE2 knowledge");
  }
  if ((effective_mode2.known & effective_mode2.value & cache_disabled) == 0) {
    throw std::runtime_error("SHARC fetch cache lookup/unknown CADIS not qualified");
  }
  if (state_.execute) { throw std::runtime_error("SHARC fetch Execute word has not retired"); }
  if (next_address) { detail::sharc_fetch_address(*next_address); }
  auto next = state_;
  next.execute = state_.address;
  next.address = state_.decode;
  next.decode = state_.fetch2;
  next.fetch2.reset();
  if (state_.fetch1) {
    // Table4-1: F1 selects an address; F2 captures the instruction data bus.
    const auto opcode =
        memory_.read({.address = *state_.fetch1, .width = SharcWidth::wide48}, AccessKind::fetch);
    next.fetch2 = SharcFetchWord{.address = *state_.fetch1, .opcode = opcode};
  }
  next.fetch1 = next_address;
  state_ = next;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC fetch transport F1=";
  if (state_.fetch1) {
    message << "0x" << std::hex << *state_.fetch1;
  } else {
    message << "bubble";
  }
  message << " next=";
  if (next_address) {
    message << "0x" << std::hex << *next_address;
  } else {
    message << "bubble";
  }
  message << " MODE2=0x" << std::hex << effective_mode2.value << "/0x" << effective_mode2.known
          << ": " << error.what();
  throw std::runtime_error(message.str());
}

std::uint64_t SharcFetch::for_execution(const SharcMemory &memory, std::uint32_t pc) const {
  if (&memory == &memory_ && state_.execute && state_.execute->address == pc) {
    return state_.execute->opcode;
  }
  std::ostringstream message;
  message << "SHARC fetched retirement pc=0x" << std::hex << pc << " Execute=";
  if (state_.execute) {
    message << "0x" << state_.execute->address << " opcode=0x" << state_.execute->opcode;
  } else {
    message << "bubble";
  }
  message << ": wrong memory binding, PC or missing fetched word";
  throw std::runtime_error(message.str());
}

SharcStep Sharc::step(SharcFetch &fetch) try {
  if (fetch.state_.flow) { detail::sharc_fetch_flow_guard(state_, fetch.state_); }
  const auto opcode = fetch.for_execution(memory_, state_.pc);
  std::optional<std::uint64_t> idle_delay;
  constexpr std::uint64_t idle = 0x008000000000;
  if (opcode == idle && state_.branch) {
    const auto following = fetch.state_.address;
    if (!following || following->address != state_.pc + 1 || following->branch ||
        following->loop_body || following->loop_pop || following->dag) {
      throw std::runtime_error("SHARC delayed IDLE requires its captured following slot");
    }
    idle_delay = following->opcode;
  }
  bool preserve_cache{};
  const auto word = fetch.state_.execute;
  if (word && word->pm_data) {
    const bool actual = detail::sharc_cache_pm_request(state_, opcode);
    if (actual != *word->pm_data) {
      throw std::runtime_error("SHARC Address-stage PM request disagrees with retirement");
    }
    preserve_cache = actual;
  }
  const auto branch_taken =
      word ? detail::sharc_fetch_branch_retirement(state_, *word) : std::nullopt;
  auto transport = fetch.state_;
  const auto loop_clock = detail::sharc_fetch_loop_retirement(state_, transport);
  const auto *dag = word && word->dag ? &*word->dag : nullptr;
  const auto *visa = word && word->visa ? &*word->visa : nullptr;
  const auto result =
      step_opcode(opcode, preserve_cache, branch_taken, loop_clock, dag, visa, true);
  if (idle_delay) { state_.idle_delay_word = idle_delay; }
  transport.execute.reset();
  detail::sharc_fetch_loop_finish(transport);
  fetch.state_ = transport;
  return result;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC captured execution pc=0x" << std::hex << state_.pc;
  if (fetch.state_.execute) { message << " opcode=0x" << fetch.state_.execute->opcode; }
  message << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
