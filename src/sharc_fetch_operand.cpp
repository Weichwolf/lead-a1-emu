// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc.hpp"
#include "lead_a1/sharc_control.hpp"
#include "lead_a1/sharc_dag.hpp"
#include "lead_a1/sharc_fetch.hpp"
#include "sharc_dag_internal.hpp"
#include "sharc_fetch_internal.hpp"
#include "sharc_immediate_indexed.hpp"
#include "sharc_internal.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <ios>
#include <sstream>
#include <stdexcept>

namespace lead_a1::detail {
namespace {

unsigned int bank(const SharcState &cpu, unsigned int index) {
  constexpr std::array<unsigned int, 4> selections{4, 3, 6, 5};
  const auto mode = cpu.effective[sharc_mode1];
  const auto bit = std::uint32_t{1} << selections[index / 4];
  if ((mode.known & bit) == 0) { throw std::runtime_error("Unknown SHARC DAG Decode bank"); }
  return (mode.value & bit) != 0 ? 1U : 0U;
}

void older_guard(const SharcFetchWord &word, SharcDagInstruction consumer) {
  constexpr unsigned int class_shift = 40;
  constexpr unsigned int mode1 = 0x72;
  constexpr std::uint32_t sysctl = 0x30024;
  constexpr unsigned int dm_write = 17;
  constexpr unsigned int pm_write = 19;
  constexpr unsigned int combined_px = 0x6b;
  const auto type = word.opcode >> class_shift;
  if (sharc_fetch_destination(word.opcode) == mode1 ||
      ((type == dm_write || type == pm_write) &&
       static_cast<std::uint32_t>(word.opcode) == sysctl && consumer.reg != combined_px)) {
    throw std::runtime_error("SHARC DAG Decode with in-flight mode/width write not qualified");
  }
  constexpr std::uint64_t dual_mask = 0xe00000000000;
  constexpr std::uint64_t dual = 0x200000000000;
  if ((word.opcode & dual_mask) == dual) {
    throw std::runtime_error("SHARC DAG Decode dual forwarding not qualified");
  }
  const auto producer = sharc_dag_instruction(word.opcode);
  if (producer && producer->post && producer->index == consumer.index && !word.dag) {
    throw std::runtime_error("SHARC conditional/parallel DAG forwarding not qualified");
  }
}

SharcDagAddress capture(const SharcState &cpu,
                        const SharcFetchState &next,
                        const SharcControl *control,
                        SharcDagInstruction instruction) {
  if (cpu.pending[sharc_mode1]) {
    throw std::runtime_error("SHARC DAG Decode with pending MODE1 not qualified");
  }
  constexpr unsigned int combined_px = 0x6b;
  if (control != nullptr && control->snapshot().delayed && instruction.reg != combined_px) {
    throw std::runtime_error("SHARC DAG Decode with pending SYSCTL width not qualified");
  }
  auto forwarded = cpu;
  // Both older post-modifies may be in flight; newest Decode-to-Address wins.
  for (const auto &word : {next.execute, next.address}) {
    if (!word) { continue; }
    older_guard(*word, instruction);
    if (word->dag && word->dag->post) {
      const auto address = *word->dag;
      constexpr std::uint32_t all_bits = 0xffffffff;
      forwarded.dag[address.bank][0][address.index] = {.value = address.after, .known = all_bits};
    }
  }
  sharc_addressing_guard(forwarded, instruction.index);
  sharc_dag_register_guard(instruction.reg, instruction.program ? 1U : 0U);
  const auto before = sharc_value(sharc_register_read(forwarded, 16 + instruction.index));
  const auto modifier =
      instruction.modifier_register
          ? sharc_value(sharc_register_read(forwarded, 32 + *instruction.modifier_register))
          : instruction.immediate;
  const auto sum = static_cast<std::uint32_t>(std::uint64_t{before} + modifier);
  const auto address = instruction.post ? before : sum;
  return {.before = before,
          .modifier = modifier,
          .address = address,
          .after = instruction.post ? sum : before,
          .index = instruction.index,
          .bank = bank(forwarded, instruction.index),
          .width = sharc_dag_width(control, address, instruction),
          .post = instruction.post};
}

}

void sharc_fetch_dag_prepare(const SharcState &cpu,
                             SharcFetchState &next,
                             const SharcControl *control) try {
  if (!next.decode) { return; }
  const auto instruction = sharc_dag_instruction(next.decode->opcode);
  if (!instruction || !instruction->qualified) { return; }
  if (next.decode->dag) {
    throw std::runtime_error("SHARC Decode DAG address was already captured");
  }
  next.decode->dag = capture(cpu, next, control, *instruction);
} catch (const std::exception &error) {

  std::ostringstream message;
  message << "SHARC DAG Decode capture";
  if (next.decode) { message << " pc=0x" << std::hex << next.decode->address; }
  message << ": " << error.what();
  throw std::runtime_error(message.str());
}

void sharc_fetch_dag_validate(const SharcFetchState &state) {
  if (state.fetch2 && state.fetch2->dag) {
    throw std::invalid_argument("Invalid SHARC DAG address before Decode");
  }
  for (const auto &word : {state.decode, state.address, state.execute}) {
    if (!word) { continue; }
    const auto instruction = sharc_dag_instruction(word->opcode);
    const bool required = state.flow && instruction && instruction->qualified;
    if (required != word->dag.has_value()) {
      throw std::invalid_argument("Missing or unexpected SHARC captured DAG address");
    }
    if (word->dag) { sharc_validate_dag_address(word->opcode, *word->dag); }
  }
}

}
