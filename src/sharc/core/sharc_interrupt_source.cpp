// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc_interrupt_source.hpp"

#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int programmable_first = 11;
constexpr unsigned int programmable_count = 19;
constexpr unsigned int vector_count = 42;
constexpr unsigned int mask_shift = 10;
constexpr std::uint32_t primary_maskable = 0xffffff7c;

SharcInterruptSource vector_source(unsigned int number) {
  if (number >= programmable_first && number < programmable_first + programmable_count) {
    return sharc_programmable_source(number - programmable_first);
  }
  constexpr unsigned int after_programmable = 30;
  const auto bit = number < after_programmable ? number : number - mask_shift;
  return {.mask = std::uint32_t{1} << bit, .offset = number * 4};
}

}

unsigned int sharc_latch_register(SharcInterruptBank bank) {
  switch (bank) {
    case SharcInterruptBank::irptl: return sharc_irptl;
    case SharcInterruptBank::lirptl: return sharc_lirptl;
    default: throw std::invalid_argument("Invalid SHARC interrupt bank");
  }
}

SharcInterruptSource sharc_programmable_source(unsigned int priority) {
  if (priority >= programmable_count) {
    throw std::invalid_argument("SHARC programmable priority must be P0..18");
  }
  constexpr unsigned int first_secondary = 6;
  constexpr unsigned int secondary_gap = 14;
  constexpr unsigned int last_secondary = 17;
  const bool secondary =
      priority >= first_secondary && (priority < secondary_gap || priority >= last_secondary);
  unsigned int bit = priority + programmable_first;
  if (secondary) {
    bit = priority - (priority < secondary_gap ? first_secondary : 9);
  } else if (priority >= secondary_gap) {
    bit = priority + 3;
  }
  return {.bank = secondary ? SharcInterruptBank::lirptl : SharcInterruptBank::irptl,
          .mask = std::uint32_t{1} << bit,
          .offset = (priority + programmable_first) * 4};
}

bool sharc_programmable_source(SharcInterruptSource source) {
  if (source.offset % 4 != 0 || source.offset < programmable_first * 4 ||
      source.offset >= (programmable_first + programmable_count) * 4) {
    return false;
  }
  const auto expected = sharc_programmable_source((source.offset / 4) - programmable_first);
  return source.bank == expected.bank && source.mask == expected.mask;
}

std::optional<SharcInterruptSource> sharc_pending_interrupt(const SharcState &state) {
  const auto primary = state.effective[sharc_irptl];
  const auto secondary = state.effective[sharc_lirptl];
  if (((primary.value | ~primary.known) & primary_maskable) == 0 &&
      ((secondary.value | ~secondary.known) & sharc_lirptl_latches) == 0) {
    return {};
  }
  constexpr std::uint32_t global = 0x1000;
  const auto mode = state.effective[sharc_mode1];
  if ((mode.known & global) == 0) { throw std::runtime_error("Unknown SHARC interrupt control"); }
  if ((mode.value & global) == 0) { return {}; }
  for (unsigned int number = 0; number < vector_count; ++number) {
    const auto source = vector_source(number);
    const bool extended = source.bank == SharcInterruptBank::lirptl;
    if (!extended && (source.mask & primary_maskable) == 0) { continue; }
    const auto latch = extended ? secondary : primary;
    auto mask = extended ? secondary : state.effective[sharc_imask];
    if (extended) {
      mask.value >>= mask_shift;
      mask.known >>= mask_shift;
    }
    if ((((~latch.value & latch.known) | (~mask.value & mask.known)) & source.mask) != 0) {
      continue;
    }
    if ((latch.known & mask.known & source.mask) == 0) {
      throw std::runtime_error("SHARC unknown interrupt eligibility");
    }
    return source;
  }
  return {};
}

void sharc_latch_source(SharcState &state, SharcInterruptBank bank, std::uint32_t mask, bool set) {
  const auto index = sharc_latch_register(bank);
  for (auto *latch : {&state.system[index], &state.effective[index]}) {
    latch->value = set ? latch->value | mask : latch->value & ~mask;
    latch->known |= mask;
  }
  auto &pending = state.pending[index];
  if (pending) {
    pending->value = set ? pending->value | mask : pending->value & ~mask;
    pending->known |= mask;
  }
}

}
