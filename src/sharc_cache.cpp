// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t all_bits = 0xffffffff;
constexpr std::uint32_t lru_bits = 0xffff;
constexpr std::uint64_t instruction_bits = 0xffffffffffff;
constexpr std::uint64_t flush = 0x170200000000;

bool program_data(std::uint64_t opcode) {
  constexpr unsigned int class_shift = 40;
  const auto type = opcode >> class_shift;
  constexpr std::uint64_t type4_mask = 0xf00000000000;
  constexpr std::uint64_t type4 = 0x600000000000;
  constexpr std::uint64_t type4_program = std::uint64_t{1} << 40;
  constexpr std::uint64_t type3_mask = 0xe00000000000;
  constexpr std::uint64_t type3 = 0x400000000000;
  constexpr std::uint64_t type3_program = std::uint64_t{1} << 32;
  constexpr unsigned int direct_pm_read = 18;
  constexpr unsigned int direct_pm_write = 19;
  constexpr std::uint64_t dual_mask = 0xe00000000000;
  constexpr std::uint64_t dual_class = 0x200000000000;
  return (opcode & dual_mask) == dual_class || type == direct_pm_read || type == direct_pm_write ||
         ((opcode & type4_mask) == type4 && (opcode & type4_program) != 0) ||
         ((opcode & type3_mask) == type3 && (opcode & type3_program) != 0);
}

}

void sharc_validate_cache(const SharcCacheState &cache) {
  if ((cache.valid.value & ~cache.valid.known) != 0 || (cache.lru.value & ~cache.lru.known) != 0 ||
      ((cache.lru.value | cache.lru.known) & ~lru_bits) != 0) {
    throw std::invalid_argument("Invalid SHARC cache validity/LRU knowledge");
  }
  for (unsigned int entry = 0; entry < cache.addresses.size(); ++entry) {
    const auto address = cache.addresses[entry];
    const auto instruction = cache.instructions[entry];
    constexpr unsigned int ways = 2;
    constexpr unsigned int set_mask = 15;
    if ((address.value & ~address.known) != 0 ||
        ((address.value | address.known) & ~sharc_pc_mask) != 0 ||
        (((address.value ^ (entry / ways)) & address.known & set_mask) != 0) ||
        (instruction.value & ~instruction.known) != 0 ||
        ((instruction.value | instruction.known) & ~instruction_bits) != 0) {
      throw std::invalid_argument("Invalid SHARC cache address/instruction knowledge");
    }
  }
}

void sharc_cache_flush(SharcState &state, std::uint64_t opcode) {
  if (opcode != flush) {
    throw std::runtime_error("SHARC cache/stack push-pop/reserved fields not implemented");
  }
  constexpr std::uint32_t normal_begin = 0x80000;
  constexpr std::uint32_t normal_end = 0x100000;
  if (state.pc < normal_begin || state.pc >= normal_end) {
    throw std::runtime_error("SHARC non-internal cache-flush effect latency not implemented");
  }
  // PGR4-86/9-70: independent of CADIS/CAFRZ, effective after one follower.
  state.cache.pending_flush = true;
}

void sharc_cache_guard(const SharcState &state, std::uint64_t opcode) {
  if (state.cache.pending_flush && program_data(opcode)) {
    throw std::runtime_error("SHARC PM-data instruction immediately after FLUSH CACHE prohibited");
  }
}

void sharc_cache_program_access(SharcState &state) {
  constexpr unsigned int mode2 = 4;
  constexpr std::uint32_t disabled = 0x10;
  constexpr std::uint32_t frozen = 0x80000;
  const auto mode = state.effective[mode2];
  if ((mode.known & mode.value & disabled) != 0) { return; }
  if ((mode.known & mode.value & frozen) != 0) {
    // Frozen contents cannot fill; hit-driven replacement history is unqualified.
    state.cache.lru = {};
    return;
  }
  // Fetch1/2/Decode fills are not qualified by this instruction-only reference.
  // Forget their possible effects, rather than exposing a fabricated empty cache.
  const bool pending = state.cache.pending_flush;
  state.cache = {};
  state.cache.pending_flush = pending;
}

void sharc_cache_finish(const SharcState &previous, SharcState &next) {
  if (previous.cache.pending_flush) { next.cache.valid = {.known = all_bits}; }
}

}
