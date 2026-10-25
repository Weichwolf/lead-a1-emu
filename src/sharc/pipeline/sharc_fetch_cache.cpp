// SPDX-License-Identifier: GPL-3.0-only
#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/pipeline/sharc_fetch_internal.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t disabled = 0x10;
constexpr std::uint32_t frozen = 0x80000;
constexpr std::uint64_t native_bits = 0xffffffffffff;
constexpr unsigned int set_mask = 15;
constexpr unsigned int ways = 2;

SharcKnown32 controls(SharcKnown32 mode) {
  if ((mode.value & ~mode.known) != 0 ||
      (mode.known & sharc_fetch_controls) != sharc_fetch_controls) {
    throw std::runtime_error("Unknown/invalid SHARC cache controls at F1");
  }
  return {.value = mode.value & sharc_fetch_controls, .known = sharc_fetch_controls};
}

void recent(SharcCacheState &cache, unsigned int entry) {
  const auto bit = std::uint32_t{1} << (entry / ways);
  cache.lru.value = (cache.lru.value & ~bit) | (entry % ways == 0 ? bit : 0);
  cache.lru.known |= bit;
}

bool matches(unsigned int entry, const SharcCacheState &cache, std::uint32_t address) {
  const auto bit = std::uint32_t{1} << entry;
  if ((cache.valid.known & bit) != 0 && (cache.valid.value & bit) == 0) { return false; }
  const auto tag = cache.addresses[entry];
  if (((tag.value ^ address) & tag.known) != 0) { return false; }
  if ((cache.valid.known & bit) == 0 || tag.known != sharc_pc_mask) {
    throw std::runtime_error("Unknown SHARC cache membership/tag at F1");
  }
  return true;
}

void bound_tag(unsigned int entry, const SharcCacheState &cache, std::uint32_t address) {
  if (cache.addresses[entry] != (SharcKnown32{.value = address, .known = sharc_pc_mask})) {
    throw std::runtime_error("SHARC pending cache tag changed/unknown");
  }
}

}

void sharc_fetch_mode(const SharcFetchState &state, SharcKnown32 mode) {
  for (const auto expected : {state.lookup ? std::optional{state.lookup->mode} : std::nullopt,
                              state.fill ? std::optional{state.fill->mode} : std::nullopt}) {
    if (expected && *expected != controls(mode)) {
      throw std::runtime_error("SHARC cache control change during pending fetch not qualified");
    }
  }
}

SharcFetchLookup
sharc_fetch_lookup(SharcCacheState &cache, SharcKnown32 mode, std::uint32_t address) {
  const auto selected = controls(mode);
  SharcFetchLookup lookup{.mode = selected, .wait = true};
  if ((selected.value & disabled) != 0) { return lookup; }
  const auto first = (address & set_mask) * ways;
  for (unsigned int entry = first; entry < first + ways; ++entry) {
    if (matches(entry, cache, address)) {
      if (lookup.entry) { throw std::runtime_error("Ambiguous duplicate SHARC cache tags"); }
      lookup.entry = entry;
    }
  }
  if (lookup.entry) {
    recent(cache, *lookup.entry);
    lookup.hit = true;
    lookup.wait = false;
  }
  return lookup;
}

void sharc_fetch_reserve(SharcCacheState &cache, SharcFetchState &state) {
  if (!state.lookup || !state.lookup->wait || !state.fetch1) {
    throw std::logic_error("Missing SHARC cache miss reservation");
  }
  auto &lookup = *state.lookup;
  if ((lookup.mode.value & (disabled | frozen)) == 0) {
    const auto set = *state.fetch1 & set_mask;
    const auto bit = std::uint32_t{1} << set;
    if ((cache.lru.known & bit) == 0) {
      throw std::runtime_error("Unknown SHARC cache replacement at F1 tag load");
    }
    const auto entry = (set * ways) + ((cache.lru.value & bit) != 0 ? 1U : 0U);
    if (state.fill && state.fill->entry == entry) {
      throw std::runtime_error("SHARC overlapping pending cache replacement not qualified");
    }
    lookup.entry = entry;
    cache.addresses[entry] = {.value = *state.fetch1, .known = sharc_pc_mask};
    const auto valid = std::uint32_t{1} << entry;
    cache.valid.value |= valid;
    cache.valid.known |= valid;
    recent(cache, entry); // Logical request recency; no architectural LRU read timing claim.
  }
  lookup.wait = false;
}

SharcFetchWord sharc_fetch_read(const SharcMemory &memory,
                                const SharcCacheState &cache,
                                const SharcFetchState &state,
                                bool &forwarded) {
  if (!state.fetch1) { throw std::logic_error("Missing SHARC F1 address"); }
  const auto address = *state.fetch1;
  if (state.lookup && state.lookup->entry) {
    const auto entry = *state.lookup->entry;
    bound_tag(entry, cache, address);
    if (state.lookup->hit) {
      if (state.fill && state.fill->entry == entry) {
        if (!state.fetch2 || state.fetch2->address != address) {
          throw std::runtime_error("SHARC cache load-bus forwarding context changed");
        }
        forwarded = true; // PGR4-84/85 Table4-33: instruction input bus, before Decode fill.
        return {.address = address, .opcode = state.fetch2->opcode};
      }
      const auto word = cache.instructions[entry];
      if (word.known != native_bits) {
        throw std::runtime_error("Unknown SHARC cached instruction at F2");
      }
      return {.address = address, .opcode = word.value};
    }
  }
  return {.address = address,
          .opcode =
              memory.read({.address = address, .width = SharcWidth::wide48}, AccessKind::fetch)};
}

void sharc_fetch_fill(SharcCacheState &cache, const SharcFetchState &state) {
  if (!state.fill) { return; }
  if (!state.fetch2) { throw std::logic_error("Missing SHARC cache input data at Decode"); }
  const auto entry = state.fill->entry;
  bound_tag(entry, cache, state.fetch2->address);
  // Valid/tag were loaded in F1. Do not resurrect a validity bit cleared by FLUSH.
  cache.instructions[entry] = {.value = state.fetch2->opcode, .known = native_bits};
}

}
