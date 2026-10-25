// SPDX-License-Identifier: GPL-3.0-only
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_internal.hpp"
#include "sharc/pipeline/sharc_fetch.hpp"
#include "sharc/pipeline/sharc_fetch_internal.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

constexpr unsigned int class_shift = 40;
constexpr unsigned int relative = 7;
constexpr unsigned int rts = 10;
constexpr unsigned int rti = 11;
constexpr unsigned int always = 31;

void word_branch(const SharcFetchWord &word, bool sequenced) {
  const auto branch = word.branch;
  if (!branch) {
    if (sequenced && sharc_fetch_branch_encoding(word.opcode)) {
      throw std::invalid_argument("Missing SHARC Address-stage branch decision");
    }
    return;
  }
  if (!sequenced || !sharc_fetch_branch_encoding(word.opcode) ||
      branch->taken != branch->target.has_value()) {
    throw std::invalid_argument("Invalid SHARC inherited branch decision");
  }
  const auto condition = static_cast<unsigned int>((word.opcode >> 33) & 31);
  static_cast<void>(sharc_fetch_condition_mask(condition));
  if (!branch->taken) {
    if (condition == always) { throw std::invalid_argument("False SHARC unconditional branch"); }
    return;
  }
  const auto target = *branch->target;
  sharc_fetch_program_address(target);
  const auto type = word.opcode >> class_shift;
  if (type != rts && type != rti) {
    auto direct = static_cast<std::uint32_t>(word.opcode) & sharc_pc_mask;
    if (type == relative) { direct = (word.address + direct) & sharc_pc_mask; }
    if (target != direct) { throw std::invalid_argument("Invalid SHARC inherited direct target"); }
  }
}

void multiplier_wait(const SharcFetchState &state) {
  if (!state.flow || !state.flow->multiplier_branch_wait) { return; }
  const auto flow = *state.flow;
  if (state.address || state.lookup || state.fill || flow.loop || flow.discard_decode != 0 ||
      !state.decode || !sharc_fetch_branch_encoding(state.decode->opcode)) {
    throw std::invalid_argument("Invalid SHARC multiplier branch wait phases");
  }
  const auto condition = static_cast<unsigned int>((state.decode->opcode >> 33) & 31);
  constexpr std::uint32_t flags = 0x3c0;
  if ((sharc_fetch_condition_mask(condition) & flags) == 0 ||
      (state.execute && !sharc_float_multiply_encoding(state.execute->opcode))) {
    throw std::invalid_argument("Invalid SHARC multiplier branch wait dependency");
  }
}

}

bool sharc_fetch_branch_encoding(std::uint64_t opcode) {
  constexpr unsigned int absolute = 6;
  const auto type = opcode >> class_shift;
  return type == absolute || type == relative || type == rts || type == rti;
}

std::uint32_t sharc_fetch_condition_mask(unsigned int code) {
  if (code == always) { return 0; }
  constexpr std::array<std::uint32_t, 15> masks{
      1, 0, 0, 8, 2, 0x80, 0x40, 0x800, 0x1000, 0, 0, 0, 0, sharc_btf, 0x4000};
  const auto base = code & 15;
  constexpr std::uint32_t relational = 0x407; // AF/AN/AV/AZ; MODE1 has a separate guard.
  if (base == 1 || base == 2) { return relational; }
  if (code > always || base >= masks.size() || masks[base] == 0) {
    throw std::runtime_error("SHARC sequenced FLAG/loop condition not qualified");
  }
  return masks[base];
}

void sharc_fetch_branch_validate(const SharcFetchState &state) {
  multiplier_wait(state);
  if ((state.fetch2 && state.fetch2->branch) || (state.decode && state.decode->branch)) {
    throw std::invalid_argument("Invalid SHARC branch decision before Address");
  }
  for (const auto &word : {state.address, state.execute}) {
    if (word) { word_branch(*word, state.flow.has_value()); }
  }
}

std::optional<bool> sharc_fetch_branch_retirement(const SharcState &cpu,
                                                  const SharcFetchWord &word) {
  const auto branch = word.branch;
  if (!branch) { return {}; }
  if (branch->taken && (word.opcode >> class_shift) == rts) {
    auto stack = cpu;
    if (sharc_pop_call(stack) != branch->target) {
      throw std::runtime_error("SHARC Address-stage return frame changed before Execute");
    }
  }
  if (branch->taken && (word.opcode >> class_shift) == rti) {
    auto stack = cpu;
    if (sharc_return_interrupt(stack, word.opcode, true) != branch->target) {
      throw std::runtime_error("SHARC Address-stage interrupt frame changed before Execute");
    }
  }
  return branch->taken;
}

}
