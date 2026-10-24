// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_internal.hpp"

#include <cstdint>
#include <exception>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int class_shift = 40;
constexpr std::uint64_t compute_mask = 0x7fffff;
constexpr std::uint64_t register_mask = 0xf00000000000;
constexpr std::uint64_t register_move = 0x700000000000;
constexpr unsigned int unconditional = 31;

struct DagUse {
  std::uint32_t indices{};
  std::uint32_t modifiers{};
  bool qualified{};
};

std::uint32_t bit(std::uint64_t opcode, unsigned int shift, unsigned int group = 0) {
  const auto reg = static_cast<unsigned int>((opcode >> shift) & 7) + group;
  return std::uint32_t{1} << reg;
}

std::optional<DagUse> consumer(std::uint64_t opcode) {
  constexpr std::uint64_t immediate_move = 0x600000000000;
  constexpr std::uint64_t indexed_mask = 0xe00000000000;
  constexpr std::uint64_t indexed_move = 0x400000000000;
  constexpr std::uint64_t dual_move = 0x200000000000;
  constexpr unsigned int index_shift = 41;
  constexpr unsigned int modifier_shift = 38;
  constexpr unsigned int condition_shift = 33;
  const bool plain = (opcode & compute_mask) == 0 &&
                     ((opcode >> condition_shift) & unconditional) == unconditional;
  if ((opcode & register_mask) == immediate_move) {
    constexpr std::uint64_t program = std::uint64_t{1} << 40;
    return DagUse{.indices = bit(opcode, index_shift, (opcode & program) != 0 ? 8U : 0U),
                  .qualified = plain};
  }
  if ((opcode & indexed_mask) == indexed_move) {
    constexpr std::uint64_t group_bit = std::uint64_t{1} << 32;
    constexpr std::uint64_t long_word = std::uint64_t{1} << 30;
    const auto group = (opcode & group_bit) != 0 ? 8U : 0U;
    return DagUse{.indices = bit(opcode, index_shift, group),
                  .modifiers = bit(opcode, modifier_shift, group),
                  .qualified = plain && (opcode & long_word) == 0};
  }
  if ((opcode & indexed_mask) == dual_move) {
    constexpr unsigned int pm_index_shift = 30;
    constexpr unsigned int pm_modifier_shift = 27;
    return DagUse{.indices = bit(opcode, index_shift) | bit(opcode, pm_index_shift, 8),
                  .modifiers = bit(opcode, modifier_shift) | bit(opcode, pm_modifier_shift, 8)};
  }
  return {};
}

bool dependency(const std::optional<SharcFetchWord> &producer, DagUse use) {
  if (!producer) { return false; }
  const auto destination = sharc_fetch_destination(producer->opcode);
  constexpr unsigned int dag_begin = 0x10;
  constexpr unsigned int dag_end = 0x50;
  if (!destination || *destination < dag_begin || *destination >= dag_end) { return false; }
  const auto reg = *destination & 15;
  constexpr unsigned int modifier_group = 2;
  const auto used = *destination / 16 == modifier_group ? use.modifiers : use.indices;
  if ((used & (std::uint32_t{1} << reg)) == 0) { return false; }
  constexpr unsigned int condition_shift = 33;
  if ((producer->opcode & register_mask) == register_move &&
      ((producer->opcode >> condition_shift) & unconditional) != unconditional) {
    throw std::runtime_error("SHARC conditional DAG producer stall not qualified");
  }
  return true;
}

void guard(const SharcState &cpu, const SharcFetchState &state, SharcFetchBus bus) {
  if (!state.flow || state.flow->loop || state.flow->discard_decode != 0 || cpu.branch ||
      state.lookup || state.fill || bus != SharcFetchBus::available) {
    throw std::runtime_error("SHARC DAG stall with loop/branch/PM/cache phases not qualified");
  }
  constexpr unsigned int mode1 = 0x72;
  if (cpu.pending[sharc_mode1]) {
    throw std::runtime_error("SHARC DAG stall with pending MODE1 not qualified");
  }
  for (const auto &word : {state.decode, state.address}) {
    if (!word) { continue; }
    const auto type = word->opcode >> class_shift;
    if ((type >= 6 && type <= 14) || (type == 0 && word->opcode != 0) ||
        sharc_cache_pm_encoding(word->opcode) || sharc_fetch_destination(word->opcode) == mode1) {
      throw std::runtime_error("SHARC DAG stall with control/PM instruction not qualified");
    }
  }
}

}

bool sharc_fetch_dag_stall(const SharcState &cpu,
                           const SharcFetchState &state,
                           SharcFetchBus bus) try {
  if (!state.fetch2 || !state.flow || (state.flow->discard_decode & 1U) != 0) { return false; }
  const auto use = consumer(state.fetch2->opcode);
  if (!use || (!dependency(state.decode, *use) && !dependency(state.address, *use))) {
    return false;
  }
  if (!use->qualified) {
    throw std::runtime_error("SHARC conditional/parallel/long-word DAG stall not qualified");
  }
  guard(cpu, state, bus);
  return true;
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC DAG Decode-input=";
  if (state.fetch2) {
    message << "0x" << std::hex << state.fetch2->address << " opcode=0x" << state.fetch2->opcode;
  } else {
    message << "bubble";
  }
  message << " producer phases=";
  for (const auto &word : {state.decode, state.address}) {
    if (word) {
      message << "0x" << std::hex << word->address << "/0x" << word->opcode << ' ';
    } else {
      message << "bubble ";
    }
  }
  message << ": " << error.what();
  throw std::runtime_error(message.str());
}

}
