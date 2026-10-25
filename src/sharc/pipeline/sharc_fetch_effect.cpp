// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_immediate_indexed.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/peripherals/sharc_control.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/pipeline/sharc_fetch_internal.hpp"
#include "sharc/pipeline/sharc_fetch_mode.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

bool width_independent(const std::optional<SharcFetchWord> &word) {
  if (!word || word->opcode == 0) { return true; }
  const auto instruction = sharc_immediate_indexed_instruction(word->opcode);
  constexpr unsigned int combined_px = 0x6b;
  return instruction && !instruction->long_word && instruction->reg == combined_px && word->dag &&
         word->dag->width == SharcWidth::wide48;
}

}

void sharc_fetch_quiet_effects(SharcState &cpu,
                               const SharcFetchState &transport,
                               const SharcControl *control) {
  if (transport.execute) { throw std::logic_error("SHARC quiet clock has an Execute instruction"); }
  const bool decoded = transport.decode.has_value() || transport.address.has_value();
  if (cpu.pending[sharc_mode1] && (transport.fetch1 || transport.fetch2 || decoded)) {
    if (sharc_fetch_interrupt_mode(cpu)) {
      sharc_fetch_interrupt_mode_guard(cpu, transport);
    } else {
      sharc_fetch_unbanked_mode_guard(cpu, transport);
    }
  }
  if (control != nullptr && control->state().delayed &&
      (!width_independent(transport.decode) || !width_independent(transport.address))) {
    throw std::runtime_error("SHARC quiet SYSCTL Decode/Address width interlock not qualified");
  }
  if (cpu.cache.pending_flush && !cpu.cache.flush_effect_applied) {
    if (transport.lookup || transport.fill || transport.pending_fetch2) {
      throw std::runtime_error("SHARC quiet FLUSH CACHE effect/follower ordering not qualified");
    }
    constexpr std::uint32_t full32 = 0xffffffff;
    cpu.cache.valid = {.known = full32};
    cpu.cache.flush_effect_applied = true;
  }
  // A-32/A-33: these effects mature at the end of the granted extra processor
  // clock, even without Execute. Do not run branch/loop/instruction finish code.
  for (unsigned int index = 0; index < cpu.pending.size(); ++index) {
    const auto pending = cpu.pending[index];
    if (pending) { cpu.effective[index] = *pending; }
  }
  cpu.pending.fill({});
}

}
