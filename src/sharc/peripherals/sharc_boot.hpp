// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "sharc/memory/sharc_memory.hpp"

#include <cstdint>
#include <optional>

namespace lead_a1 {

struct SharcBootLines {
  bool selected{}; // Active-low physical SPI_DS_I, expressed as logical selection.
  bool clock{};
  std::optional<bool> mosi;
};

struct SharcBootState {
  std::optional<bool> clock;
  bool selected{};
  std::uint32_t shift{};
  unsigned int bits{};
  std::uint32_t next_word{};
  unsigned int remaining{};
  bool operator==(const SharcBootState &) const = default;
};

// Hardcoded initial SPI-slave boot DMA, not the programmable runtime SPI port.
// One owning emulation thread; external clock transitions are explicit inputs.
class SharcSpiBoot {
public:
  [[nodiscard]] static SharcSpiBoot slave_reference_214xx();

  [[nodiscard]] const SharcBootState &state() const { return state_; }

  // DMISO=1 in this boot profile: no driven MISO, including after kernel load.
  [[nodiscard]] static std::optional<bool> miso() { return {}; }

  // Completion metadata: base address of the fully loaded kernel/IVT.
  // The board must separately deliver the initial boot IRQ and execute its RTI
  // with an established return frame; see doc/sharc.txt. No readiness is implied.
  [[nodiscard]] std::optional<std::uint32_t> entry() const;
  void observe(SharcMemory &memory, SharcBootLines lines);

private:
  explicit SharcSpiBoot(SharcBootState state) : state_(state) {}

  SharcBootState state_;
};

}
