// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm_mmu.hpp"

#include "arm/core/arm_data_internal.hpp"
#include "arm/core/arm_mmu_internal.hpp"
#include "common/memory.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <ios>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace lead_a1::detail {
namespace {

constexpr std::uint32_t mmu_enable = 1;
constexpr std::uint32_t pid_low_mask = 0x01ffffff;
constexpr std::uint32_t table_low_mask = 0x3fff;
constexpr std::uint32_t table_index_mask = 0x3ffc;
constexpr std::uint32_t lockdown_mask = 0x1c000001;
constexpr std::uint32_t section_mask = 0xfff00000;
constexpr std::uint32_t large_mask = 0xffff0000;
constexpr std::uint32_t small_mask = 0xfffff000;
constexpr std::uint32_t tiny_mask = 0xfffffc00;
constexpr std::uint32_t large_subpage_mask = 0xffffc000;
constexpr unsigned int domain_shift = 5;
constexpr unsigned int victim_shift = 26;
constexpr std::uint8_t translation_section = 5;
constexpr std::uint8_t translation_page = 7;
constexpr std::uint8_t domain_section = 9;
constexpr std::uint8_t domain_page = 11;
constexpr std::uint8_t permission_section = 13;
constexpr std::uint8_t permission_page = 15;

void require_zero(std::uint32_t value, std::uint32_t mask, const char *name) {
  if ((value & mask) != 0) { throw std::runtime_error(std::string("Invalid ARM MMU ") + name); }
}

std::uint32_t known(const std::optional<std::uint32_t> &value, const char *name) {
  if (!value) { throw std::runtime_error(std::string("Unknown ARM MMU ") + name); }
  return *value;
}

const char *access_name(AccessKind kind) {
  switch (kind) {
    case AccessKind::read: return "read";
    case AccessKind::write: return "write";
    case AccessKind::fetch: return "fetch";
    default: throw std::runtime_error("Invalid ARM MMU access kind");
  }
}

std::uint32_t page_mask(std::uint32_t type) {
  if (type == 1) { return large_mask; }
  if (type == 2) { return small_mask; }
  return tiny_mask;
}

std::string fault_text(const MmuRequest &request,
                       std::uint32_t modified,
                       std::uint8_t status,
                       std::optional<std::uint8_t> domain) {
  std::ostringstream text;
  text << "ARM MMU " << access_name(request.kind) << " va=0x" << std::hex << request.access.address
       << " mva=0x" << modified << " status=0x" << static_cast<unsigned int>(status);
  if (domain) { text << " domain=" << static_cast<unsigned int>(*domain); }
  text << std::dec << " width=" << request.access.size;
  return text.str();
}

bool allowed(std::uint8_t ap, const MmuRequest &request) {
  const bool write = request.kind == AccessKind::write;
  if (ap == 3) { return true; }
  if (ap == 2) { return !request.user || !write; }
  if (ap == 1) { return !request.user; }
  constexpr unsigned int protection_shift = 8;
  const auto protection = (request.control >> protection_shift) & 3;
  if (protection == 3) { throw std::runtime_error("Unpredictable ARM MMU AP=0 with S=R=1"); }
  return !write && (protection == 2 || (protection == 1 && !request.user));
}

}

MmuFault::MmuFault(const MmuRequest &request,
                   std::uint32_t modified,
                   std::uint8_t status,
                   std::optional<std::uint8_t> domain)
    : std::runtime_error(fault_text(request, modified, status, domain)),
      status_(status),
      domain_(domain),
      modified_(modified),
      kind_(request.kind),
      architectural_(request.architectural) {}

ArmMmu::ArmMmu(Memory &memory, const std::optional<ArmMmuInitial> &initial, ArmDataPath *data)
    : memory_(memory), data_(data) {
  if (!initial) { return; }
  state_.translation_table = initial->translation_table;
  state_.domain_access = initial->domain_access;
  state_.fcse_pid = initial->fcse_pid;
  state_.context_id = initial->context_id;
  state_.tlb_lockdown = initial->tlb_lockdown;
  state_.tlb_known = initial->tlb_invalid;
  state_.locked_tlb_known = initial->locked_tlb_invalid;
  if (initial->translation_table) {
    require_zero(*initial->translation_table, table_low_mask, "TTBR alignment");
  }
  if (initial->fcse_pid) { require_zero(*initial->fcse_pid, pid_low_mask, "FCSE reserved bits"); }
  if (initial->tlb_lockdown) {
    require_zero(*initial->tlb_lockdown, ~lockdown_mask, "TLB lockdown reserved bits");
  }
}

void ArmMmu::begin_instruction() {
  if (active_) { throw std::logic_error("Nested ARM MMU transaction"); }
  pending_ = state_;
  pending_entries_.reset();
  active_ = true;
}

void ArmMmu::commit_instruction() {
  if (!active_) { throw std::logic_error("No ARM MMU transaction"); }
  if (pending_entries_) { entries_ = *pending_entries_; }
  state_ = pending_;
  pending_entries_.reset();
  active_ = false;
}

void ArmMmu::abort_instruction() {
  pending_entries_.reset();
  active_ = false;
}

const ArmMmu::Cache &ArmMmu::cache() const {
  return pending_entries_ ? *pending_entries_ : entries_;
}

ArmMmu::Cache &ArmMmu::mutable_cache() {
  if (!pending_entries_) { pending_entries_ = entries_; }
  return *pending_entries_;
}

void ArmMmu::require_ready() const {
  static_cast<void>(known(pending_.translation_table, "translation table"));
  static_cast<void>(known(pending_.domain_access, "domain access control"));
  static_cast<void>(known(pending_.fcse_pid, "FCSE PID"));
  static_cast<void>(known(pending_.tlb_lockdown, "TLB lockdown"));
  if (!pending_.tlb_known || !pending_.locked_tlb_known) {
    throw std::runtime_error("Unknown ARM MMU initial TLB contents");
  }
}

std::uint32_t ArmMmu::read(MmuRegister reg) const {
  if (!active_) { throw std::logic_error("ARM MMU register read outside transaction"); }
  if (reg.crm != 0) { throw std::runtime_error("Unsupported ARM MMU register selector"); }
  if (reg.crn == 2 && reg.selector == 0) { return known(pending_.translation_table, "TTBR"); }
  if (reg.crn == 3 && reg.selector == 0) { return known(pending_.domain_access, "DACR"); }
  if (reg.crn == 10 && reg.selector == 0) { return known(pending_.tlb_lockdown, "TLB lockdown"); }
  if (reg.crn == 13 && reg.selector == 0) { return known(pending_.fcse_pid, "FCSE PID"); }
  if (reg.crn == 13 && reg.selector == 1) { return known(pending_.context_id, "context ID"); }
  throw std::runtime_error("Unsupported ARM MMU register read");
}

void ArmMmu::write(MmuRegister reg, std::uint32_t value) {
  if (!active_) { throw std::logic_error("ARM MMU register write outside transaction"); }
  if (reg.crn == 8) {
    invalidate(reg, value);
    return;
  }
  if (reg.crm != 0) { throw std::runtime_error("Unsupported ARM MMU register selector"); }
  if (reg.crn == 2 && reg.selector == 0) {
    require_zero(value, table_low_mask, "TTBR alignment");
    pending_.translation_table = value;
  } else if (reg.crn == 3 && reg.selector == 0) {
    pending_.domain_access = value;
  } else if (reg.crn == 10 && reg.selector == 0) {
    require_zero(value, ~lockdown_mask, "TLB lockdown reserved bits");
    pending_.tlb_lockdown = value;
  } else if (reg.crn == 13 && reg.selector == 0) {
    require_zero(value, pid_low_mask, "FCSE reserved bits");
    pending_.fcse_pid = value;
  } else if (reg.crn == 13 && reg.selector == 1) {
    pending_.context_id = value;
  } else {
    throw std::runtime_error("Unsupported ARM MMU register write");
  }
}

bool ArmMmu::matches(const Entry &entry, std::uint32_t modified) {
  return entry.valid && (modified & entry.mask) == entry.virtual_base;
}

void ArmMmu::invalidate(MmuRegister reg, std::uint32_t value) {
  if ((reg.crm != 5 && reg.crm != 6 && reg.crm != 7) || reg.selector > 1) {
    throw std::runtime_error("Unsupported ARM TLB maintenance operation");
  }
  if (reg.selector == 0) {
    if (value != 0) { throw std::runtime_error("ARM TLB invalidate-all requires zero operand"); }
    auto &entries = mutable_cache();
    std::fill_n(entries.begin(), temporary_entries, Entry{});
    pending_.tlb_known = true;
    pending_.entries = 0;
  } else {
    constexpr std::uint32_t low_bits = 0x3ff;
    require_zero(value, low_bits, "TLB invalidate MVA alignment");
    auto &entries = mutable_cache();
    for (std::size_t index = 0; index < entries.size(); ++index) {
      if (!matches(entries[index], value)) { continue; }
      entries[index].valid = false;
      if (index < temporary_entries) {
        --pending_.entries;
      } else {
        --pending_.locked_entries;
      }
    }
  }
  ++pending_.invalidations;
}

std::uint32_t ArmMmu::descriptor(std::uint32_t address) const {
  const MemoryAccess access{.address = address, .size = 4};
  return static_cast<std::uint32_t>(data_ != nullptr ? data_->physical_read(access)
                                                     : memory_.read(access));
}

ArmMmu::Entry
ArmMmu::page(const MmuRequest &request, std::uint32_t modified, std::uint32_t first) const {
  const bool fine = (first & 3) == 3;
  const auto table_mask = fine ? small_mask : tiny_mask;
  const auto table_index = fine ? ((modified >> 8) & 0xffc) : ((modified >> 10) & 0x3fc);
  const auto second = descriptor((first & table_mask) | table_index);
  const auto type = second & 3;
  const auto domain = static_cast<std::uint8_t>((first >> domain_shift) & 15);
  if (type == 0 || (type == 3 && !fine)) {
    throw MmuFault(request, modified, translation_page, domain);
  }
  auto mask = page_mask(type);
  constexpr unsigned int ap_shift = 4;
  auto ap = static_cast<std::uint8_t>((second >> ap_shift) & 3);
  if (type == 3) {
    constexpr std::uint32_t tiny_reserved = 0x3c0;
    require_zero(second, tiny_reserved, "tiny-page reserved bits");
  } else {
    if (type == 1) {
      constexpr std::uint32_t large_reserved = 0xf000;
      require_zero(second, large_reserved, "large-page reserved bits");
    }
    constexpr unsigned int ap_byte_mask = 0xff;
    const auto permissions = (second >> ap_shift) & ap_byte_mask;
    const auto replicated = static_cast<unsigned int>(ap) * 0x55;
    if (permissions != replicated) {
      const auto subpage_shift = type == 1 ? 14U : 10U;
      const auto subpage = (modified >> subpage_shift) & 3;
      ap = static_cast<std::uint8_t>((second >> (ap_shift + (subpage * 2))) & 3);
      mask = type == 1 ? large_subpage_mask : tiny_mask;
    }
  }
  const auto full_mask = page_mask(type);
  const auto physical = (second & full_mask) | (modified & ~full_mask);
  return {.virtual_base = modified & mask,
          .physical_base = physical & mask,
          .mask = mask,
          .domain = domain,
          .permission = ap,
          .section = false,
          .cacheable = (second & 8) != 0,
          .bufferable = (second & 4) != 0,
          .valid = true};
}

ArmMmu::Entry ArmMmu::walk(const MmuRequest &request, std::uint32_t modified) const {
  const auto table = known(pending_.translation_table, "TTBR");
  constexpr unsigned int first_level_index_shift = 18;
  const auto first = descriptor(table | ((modified >> first_level_index_shift) & table_index_mask));
  const auto type = first & 3;
  if (type == 0) { throw MmuFault(request, modified, translation_section, std::nullopt); }
  constexpr std::uint32_t descriptor_sbo = 16;
  if ((first & descriptor_sbo) == 0) { throw std::runtime_error("Invalid ARM MMU descriptor SBO"); }
  if (type != 2) {
    const auto reserved = type == 1 ? 0x20cU : 0xe0cU;
    require_zero(first, reserved, "page-table reserved bits");
    return page(request, modified, first);
  }
  constexpr std::uint32_t section_reserved = 0x000ff200;
  require_zero(first, section_reserved, "section reserved bits");
  constexpr unsigned int section_ap_shift = 10;
  return {.virtual_base = modified & section_mask,
          .physical_base = first & section_mask,
          .mask = section_mask,
          .domain = static_cast<std::uint8_t>((first >> domain_shift) & 15),
          .permission = static_cast<std::uint8_t>((first >> section_ap_shift) & 3),
          .section = true,
          .cacheable = (first & 8) != 0,
          .bufferable = (first & 4) != 0,
          .valid = true};
}

void ArmMmu::permission(const Entry &entry,
                        const MmuRequest &request,
                        std::uint32_t modified) const {
  const auto domains = known(pending_.domain_access, "DACR");
  const auto domain = (domains >> (entry.domain * 2)) & 3;
  if (domain == 0 || domain == 2) {
    throw MmuFault(request, modified, entry.section ? domain_section : domain_page, entry.domain);
  }
  if (domain == 1 && !allowed(entry.permission, request)) {
    throw MmuFault(
        request, modified, entry.section ? permission_section : permission_page, entry.domain);
  }
}

void ArmMmu::retain(const Entry &entry) {
  auto &entries = mutable_cache();
  const auto lockdown = known(pending_.tlb_lockdown, "TLB lockdown");
  if ((lockdown & 1) != 0) {
    const auto victim = (lockdown >> victim_shift) & 7;
    auto &target = entries[temporary_entries + victim];
    if (!target.valid) { ++pending_.locked_entries; }
    target = entry;
    pending_.tlb_lockdown = (((victim + 1) & 7U) << victim_shift) | 1U;
    return;
  }
  auto *const end = entries.begin() + static_cast<std::ptrdiff_t>(temporary_entries);
  auto *const slot =
      std::find_if(entries.begin(), end, [](const Entry &value) { return !value.valid; });
  if (slot == end) { throw std::runtime_error("Unqualified ARM MMU TLB replacement state"); }
  *slot = entry;
  ++pending_.entries;
}

MmuTranslation ArmMmu::translate(const MmuRequest &request) {
  if (!active_) { throw std::logic_error("ARM MMU access outside transaction"); }
  const auto access = request.access;
  constexpr auto address_bytes = std::uint64_t{1} << 32;
  if ((access.size != 1 && access.size != 2 && access.size != 4 && access.size != 8) ||
      access.size > address_bytes - access.address) {
    throw std::runtime_error("Invalid ARM MMU access width or address wrap");
  }
  if (request.kind != AccessKind::read && request.kind != AccessKind::write &&
      request.kind != AccessKind::fetch) {
    throw std::runtime_error("Invalid ARM MMU access kind");
  }
  const auto modified = (request.control & mmu_enable) != 0 && access.address <= pid_low_mask
                            ? access.address | known(pending_.fcse_pid, "FCSE PID")
                            : access.address;
  if (access.address % access.size != 0) {
    if (request.kind != AccessKind::fetch && (request.control & 2) != 0) {
      throw MmuFault(request, modified, 1, std::nullopt);
    }
    throw std::runtime_error("Unsupported unaligned ARM MMU access");
  }
  if ((request.control & mmu_enable) == 0) {
    return {.physical = access,
            .modified = access.address,
            .cacheable = request.kind == AccessKind::fetch,
            .bufferable = false};
  }
  require_ready();
  const auto &entries = cache();
  const auto *const found = std::ranges::find_if(
      entries, [modified](const Entry &entry) { return matches(entry, modified); });
  const auto entry = found == entries.end() ? walk(request, modified) : *found;
  permission(entry, request, modified);
  if (request.retain) {
    if (found == entries.end()) {
      ++pending_.walks;
      retain(entry);
    } else {
      ++pending_.hits;
    }
  }
  return {
      .physical = {.address = entry.physical_base | (modified & ~entry.mask), .size = access.size},
      .modified = modified,
      .cacheable = entry.cacheable,
      .bufferable = entry.bufferable};
}

}
