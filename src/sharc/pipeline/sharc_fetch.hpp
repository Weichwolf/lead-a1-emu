// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"
#include "sharc/core/sharc.hpp"
#include "sharc/core/sharc_dag.hpp"
#include "sharc/memory/sharc_memory.hpp"
#include "sharc/visa/sharc_visa.hpp"

#include <cstdint>
#include <optional>

namespace lead_a1 {

class SharcIopBridge;

struct SharcFetchIopAccess {
  std::uint32_t pc{};
  std::uint32_t address{};
  AccessKind operation{AccessKind::read};
  bool operator==(const SharcFetchIopAccess &) const = default;
};

struct SharcFetchBranch {
  bool taken{};
  std::optional<std::uint32_t> target{std::nullopt};
  bool operator==(const SharcFetchBranch &) const = default;
};

struct SharcFetchWord {
  std::uint32_t address{};
  std::uint64_t opcode{};
  // Explicit PM request observed in Address; absent on the standalone path.
  std::optional<bool> pm_data{std::nullopt};
  // Branch condition/target selected in Address, before native Execute effects.
  std::optional<SharcFetchBranch> branch{std::nullopt};
  bool loop_body{};
  bool loop_pop{};
  // Last retained real end instruction after an arithmetic Fetch1 abort.
  bool arithmetic_exit{};
  std::optional<SharcDagAddress> dag{std::nullopt};
  // Actual captured encoding; opcode above is its decoded native operation.
  std::optional<SharcVisaInstruction> visa{std::nullopt};
  bool operator==(const SharcFetchWord &) const = default;
};

struct SharcFetchLookup {
  SharcKnown32 mode;
  std::optional<unsigned int> entry{std::nullopt};
  bool hit{};
  bool wait{};
  bool operator==(const SharcFetchLookup &) const = default;
};

struct SharcFetchFill {
  SharcKnown32 mode;
  unsigned int entry{};
  bool operator==(const SharcFetchFill &) const = default;
};

enum class SharcFetchLoopPhase : std::uint8_t { setup, hold, seed, stream };
enum class SharcFetchLoopPop : std::uint8_t { live, due, execute, done };

struct SharcFetchLoop {
  std::uint32_t setup{};
  std::uint32_t body{};
  std::uint64_t total{};
  std::uint64_t fetch_remaining{};
  std::uint64_t issued{};
  std::uint64_t retired{};
  SharcFetchLoopPhase phase{SharcFetchLoopPhase::setup};
  SharcFetchLoopPop pop{SharcFetchLoopPop::live};
  std::optional<SharcFetchWord> held{std::nullopt};
  std::optional<unsigned int> saved_depth{std::nullopt};
  bool terminal{};
  unsigned int units{1}; // Native NW or uncompressed VISA SW instruction span.
  bool operator==(const SharcFetchLoop &) const = default;
};

struct SharcFetchLongCounter {
  std::uint64_t total{};
  std::uint64_t remaining{};
  std::uint64_t retired{};
  unsigned int saved_depth{};
  bool terminal{};
  bool operator==(const SharcFetchLongCounter &) const = default;
};

enum class SharcFetchFourPhase : std::uint8_t { none, setup, hold, stream };

// Multi-instruction counter loop; the native DO resolves its count.
struct SharcFetchLongLoop {
  std::uint32_t setup{};
  std::uint32_t end{};
  std::uint64_t opcode{};
  std::optional<SharcFetchLongCounter> counter;
  SharcFetchFourPhase four{SharcFetchFourPhase::none};
  std::optional<SharcFetchWord> held{std::nullopt};
  bool refetch_end{};
  bool operator==(const SharcFetchLongLoop &) const = default;
};

struct SharcFetchFlow {
  std::uint32_t next{};
  // Low bit suppresses this clock's Decode input; shifts on an advancing clock.
  unsigned int discard_decode{};
  std::optional<SharcFetchLoop> loop{std::nullopt};
  std::optional<SharcFetchLongLoop> long_loop{std::nullopt};
  // PGR4-116: one further Decode hold after a multiplier flag producer issues.
  bool multiplier_branch_wait{};
  bool operator==(const SharcFetchFlow &) const = default;
};

struct SharcFetchState {
  std::optional<std::uint32_t> fetch1;
  std::optional<SharcFetchWord> fetch2;
  std::optional<SharcFetchWord> decode;
  std::optional<SharcFetchWord> address;
  std::optional<SharcFetchWord> execute;
  std::optional<SharcFetchLookup> lookup;
  std::optional<SharcFetchFill> fill;
  std::optional<SharcFetchFlow> flow{std::nullopt};
  // An issued, uncached RAM fetch whose data response has not arrived in F2.
  // No opcode exists for this address until the owner completes the read.
  std::optional<std::uint32_t> pending_fetch2{std::nullopt};
  std::optional<SharcVisaState> visa{std::nullopt};
  // A redirected SW request must never supply bytes to the new IAB epoch.
  bool visa_fetch_aborted{};
  bool operator==(const SharcFetchState &) const = default;
};

enum class SharcFetchBus : std::uint8_t { available, pm_data, block_conflict };
enum class SharcFetchRead : std::uint8_t { immediate, deferred };

struct SharcFetchEvent {
  bool address_accepted{};
  bool stalled{};
  bool cache_hit{};
  bool forwarded{};
  bool control_stall{};
  bool loop_stall{};
  bool dag_stall{};
  bool compute_stall{};
  // Only the fetch engine waits; Decode/Address/Execute continue advancing.
  bool fetch_stall{};
  bool operator==(const SharcFetchEvent &) const = default;
};

// Internal native/VISA instruction transport, PGR4-5/9. F1 addresses come from
// the owner or the optional bounded sequencer. The owner grants clocks;
// standalone CADIS=1 and bound cache clocks have separate inputs. Complete
// DAG/control interlocks and loop/IRQ/IDLE redirects remain guarded or open.
class SharcFetch {
public:
  explicit SharcFetch(SharcMemory &memory, SharcFetchState initial = {});

  [[nodiscard]] const SharcFetchState &state() const { return state_; }

  // Read-only peripheral DM/DREG/USTAT access in the actual Execute latch. No
  // peripheral data is read here; the board supplies its latency/clock phase.
  [[nodiscard]] std::optional<SharcFetchIopAccess> peripheral_access(const Sharc &cpu) const;
  // One owner-granted core hold for the exact unready bound bridge request.
  // Retains every transport latch/PC and advances supported delayed effects.
  // The owner separately advances peripheral time. Read retirement shares
  // its final response clock; a spaced write still needs its acceptance clock.
  void wait_iop(Sharc &cpu, const SharcIopBridge &bridge);

  // Read-only eligible Decode-to-Address PM request, before the owner supplies
  // its bus event. Qualified interlocks report no new request; this grants no
  // RAM port and does not resolve general conditional-transfer hazards.
  [[nodiscard]] bool address_pm_request(const Sharc &cpu) const;

  // Complete an explicitly deferred uncached F2 read from actual instruction
  // RAM. This is an owner memory-response event, not a processor clock. A ready
  // fetched word cannot be refreshed, and failure retains the pending read.
  void complete_fetch2(const Sharc &cpu);

  // One granted transport clock. nullopt injects a bubble, never a guest NOP.
  // Captures the previous F1 word in F2; later phases retain its actual data.
  // An occupied Execute phase must retire before another advancing clock.
  void advance(SharcKnown32 effective_mode2, std::optional<std::uint32_t> next_address);
  // Bound cache clock, with the actual Address-stage PM bus request supplied
  // explicitly. A miss accepts its F1 address, then one partial stall clock
  // holds F1/F2/Decode while Address progresses to Execute. Retry an unaccepted
  // next_address; no guest NOP is inserted. Block arbitration stays guarded.
  [[nodiscard]] SharcFetchEvent advance(Sharc &cpu,
                                        std::optional<std::uint32_t> next_address,
                                        SharcFetchBus bus = SharcFetchBus::available,
                                        SharcFetchRead read = SharcFetchRead::immediate);
  // Optional sequencer: initialize state.flow.next to the entry address.
  // Generates sequential F1 and qualified native/VISA JUMP/CALL/RTS targets, with
  // real delayed slots and suppressed speculative Decode inputs (PGR4-20/21).
  // The owner still grants clocks, supplies PM requests and retires Execute.
  [[nodiscard]] SharcFetchEvent advance_sequenced(Sharc &cpu,
                                                  SharcFetchBus bus = SharcFetchBus::available,
                                                  SharcFetchRead read = SharcFetchRead::immediate);

private:
  friend class Sharc;
  [[nodiscard]] SharcFetchEvent advance_clock(Sharc &cpu,
                                              std::optional<std::uint32_t> next_address,
                                              SharcFetchBus bus,
                                              bool sequenced,
                                              SharcFetchRead read);
  [[nodiscard]] std::uint64_t for_execution(const SharcMemory &memory, std::uint32_t pc) const;
  SharcMemory &memory_;
  SharcFetchState state_;
};

}
