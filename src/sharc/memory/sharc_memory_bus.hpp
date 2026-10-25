// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "common/memory.hpp"
#include "sharc/memory/sharc_memory.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

enum class SharcMemoryMaster : std::uint8_t { io0, io1, dm, pm };
enum class SharcPeripheralPhase : std::uint8_t { active, gap };

struct SharcMemoryRequest {
  SharcAccess access;
  AccessKind kind{AccessKind::read};
  std::uint64_t value{};

  bool operator==(const SharcMemoryRequest &other) const {
    return access.address == other.access.address && access.width == other.access.width &&
           kind == other.kind && value == other.value;
  }
};

struct SharcMemoryBusState {
  std::array<std::optional<SharcMemoryRequest>, 4> pending{};
  std::array<std::optional<SharcMemoryRequest>, 4> granted{};
  // A losing IO1 request retains eligibility on the following core clock.
  std::array<bool, 2> carried{};
  std::uint64_t clocks{};
  SharcPeripheralPhase next_phase{SharcPeripheralPhase::active};
  bool operator==(const SharcMemoryBusState &) const = default;
};

// Internal RAM block arbitration, PGR7-10/11,7-20/22. Requesters are already
// selected buses, not peripheral-channel arbitration. Explicit core clock
// grants alternate new I/O launch eligibility from the supplied initial phase.
// A previously launched losing IO1 request can finish on the intervening clock.
class SharcMemoryBus {
public:
  SharcMemoryBus(SharcMemory &memory, SharcPeripheralPhase initial_phase);
  SharcMemoryBus(SharcMemory &memory, SharcMemoryBusState initial);

  [[nodiscard]] const SharcMemoryBusState &state() const { return state_; }

  [[nodiscard]] SharcMemory &memory() const { return memory_; }

  void request(SharcMemoryMaster master, SharcMemoryRequest request);
  // Grants at most one access per physical block, IO0>IO1>DM>PM. All grants
  // must finish before advancing or submitting another cycle's requests.
  void advance();
  [[nodiscard]] std::optional<SharcMemoryRequest> grant(SharcMemoryMaster master) const;
  // Actual RAM operation; only success consumes this grant. No other grant,
  // transaction or pending request is consumed on a failed access.
  [[nodiscard]] std::uint64_t execute(SharcMemoryMaster master);

private:
  SharcMemory &memory_;
  SharcMemoryBusState state_;
};

}
