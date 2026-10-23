// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/memory.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace lead_a1 {

inline constexpr unsigned int aintc_sources = 101;

struct AintcPriority {
  std::uint32_t index{};
  std::uint32_t vector{};
  bool operator==(const AintcPriority &) const = default;
};

struct AintcState {
  std::optional<std::uint32_t> revision;
  std::uint32_t control{};
  bool global_enabled{};
  std::uint32_t host_enabled{};
  std::uint32_t vector_base{};
  unsigned int vector_size{};
  bool vector_size_programmed{};
  std::uint32_t null_vector{};
  std::uint32_t global_nesting{0x100};
  std::array<std::uint32_t, 2> host_nesting{0x100, 0x100};
  bool global_nesting_override{};
  std::array<bool, 2> host_nesting_override{};
  std::array<std::uint32_t, 4> pending{};
  std::array<std::uint32_t, 4> enabled{};
  std::array<unsigned int, aintc_sources> channels{};
  std::array<std::optional<AintcPriority>, 2> held{};
  bool operator==(const AintcState &) const = default;
};

// Functional AM18xx-family reference; construction requires explicit input state.
class Aintc final : public MemoryDevice {
public:
  explicit Aintc(const AintcState &initial);
  [[nodiscard]] static AintcState reset_reference();
  [[nodiscard]] AintcState state() const;
  // FIQ, IRQ. No host clocks, ARM sampling or input-source wiring is implicit.
  [[nodiscard]] std::array<bool, 2> outputs() const;
  void pulse(unsigned int source);
  [[nodiscard]] std::uint64_t read(MemoryAccess access) override;
  void write(MemoryAccess access, std::uint64_t value) override;
  void begin_instruction() override;
  void commit_instruction() noexcept override;
  void abort_instruction() noexcept override;

private:
  [[nodiscard]] AintcState &working();
  [[nodiscard]] const AintcState &working() const;
  AintcState current_;
  std::optional<AintcState> pending_;
};

}
