// SPDX-License-Identifier: GPL-3.0-only
#pragma once

#include "lead_a1/arm_mmu.hpp"
#include "lead_a1/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace lead_a1::detail {

struct MmuRegister {
  unsigned int crn{};
  unsigned int crm{};
  unsigned int selector{};
};

struct MmuRequest {
  MemoryAccess access;
  AccessKind kind{AccessKind::read};
  std::uint32_t control{};
  bool user{};
  bool retain{true};
};

struct MmuTranslation {
  MemoryAccess physical;
  std::uint32_t modified{};
  bool cacheable{};
  bool bufferable{};
};

class MmuFault final : public std::runtime_error {
public:
  MmuFault(const MmuRequest &request,
           std::uint32_t modified,
           std::uint8_t status,
           std::optional<std::uint8_t> domain);

  [[nodiscard]] std::uint8_t status() const { return status_; }

  [[nodiscard]] std::optional<std::uint8_t> domain() const { return domain_; }

private:
  std::uint8_t status_;
  std::optional<std::uint8_t> domain_;
};

class ArmMmu {
public:
  ArmMmu(Memory &memory, const std::optional<ArmMmuInitial> &initial);

  [[nodiscard]] ArmMmuState state() const { return state_; }

  void begin_instruction();
  void commit_instruction();
  void abort_instruction();
  [[nodiscard]] std::uint32_t read(MmuRegister reg) const;
  void write(MmuRegister reg, std::uint32_t value);
  void require_ready() const;
  [[nodiscard]] MmuTranslation translate(const MmuRequest &request);

private:
  struct Entry {
    std::uint32_t virtual_base{};
    std::uint32_t physical_base{};
    std::uint32_t mask{};
    std::uint8_t domain{};
    std::uint8_t permission{};
    bool section{};
    bool cacheable{};
    bool bufferable{};
    bool valid{};
  };

  static constexpr std::size_t temporary_entries = 64;
  static constexpr std::size_t locked_entries = 8;
  using Cache = std::array<Entry, temporary_entries + locked_entries>;
  [[nodiscard]] const Cache &cache() const;
  [[nodiscard]] Cache &mutable_cache();
  [[nodiscard]] std::uint32_t descriptor(std::uint32_t address) const;
  [[nodiscard]] Entry walk(const MmuRequest &request, std::uint32_t modified) const;
  [[nodiscard]] Entry
  page(const MmuRequest &request, std::uint32_t modified, std::uint32_t first) const;
  [[nodiscard]] static bool matches(const Entry &entry, std::uint32_t modified);
  void permission(const Entry &entry, const MmuRequest &request, std::uint32_t modified) const;
  void retain(const Entry &entry);
  void invalidate(MmuRegister reg, std::uint32_t value);
  Memory &memory_;
  ArmMmuState state_;
  ArmMmuState pending_;
  Cache entries_{};
  std::optional<Cache> pending_entries_;
  bool active_{};
};

}
