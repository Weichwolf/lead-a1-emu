// SPDX-License-Identifier: GPL-3.0-only
#include "arm/core/arm_mmu.hpp"
#include "arm/core/arm_mmu_internal.hpp"
#include "common/memory.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <optional>
#include <source_location>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using lead_a1::detail::ArmMmu;
using lead_a1::detail::MmuFault;
using lead_a1::detail::MmuRegister;
using lead_a1::detail::MmuRequest;
using lead_a1::detail::MmuTranslation;
constexpr std::uint32_t table_base = 0x10000;
constexpr std::uint32_t alternate_table = 0x14000;
constexpr std::uint32_t coarse_base = 0x18000;
constexpr std::uint32_t fine_base = 0x19000;
constexpr std::uint32_t control = 0x50079;
constexpr std::uint32_t section_entry = 0x12000;
constexpr std::uint32_t section_descriptor = 0x20000c12;
constexpr std::uint32_t coarse_descriptor_address = 0x12040;
constexpr std::uint32_t fine_descriptor_address = 0x12080;
constexpr std::uint32_t coarse_large_address = 0x18004;
constexpr std::uint32_t coarse_small_address = 0x18010;
constexpr std::uint32_t subpage_descriptor_address = 0x18020;
constexpr std::uint32_t fine_tiny_address = 0x19008;
constexpr std::uint32_t fine_small_address = 0x19014;
constexpr std::uint32_t fine_large_address = 0x19120;
constexpr std::uint32_t section_virtual = 0x80001234;
constexpr std::uint32_t section_physical = 0x20001234;
constexpr std::uint32_t remapped_physical = 0x30001234;
constexpr std::uint32_t subpage_zero = 0x81008004;
constexpr std::uint32_t subpage_one = 0x81008404;
constexpr std::uint32_t subpage_two = 0x81008804;
constexpr std::uint32_t subpage_three = 0x81008c04;
constexpr std::uint32_t physical_subpage_one = 0x70008404;
constexpr std::uint32_t physical_subpage_two = 0x70008804;
constexpr std::uint32_t physical_subpage_three = 0x70008c04;
constexpr std::uint32_t invalid_section = 0x83000000;
constexpr std::uint32_t invalid_page = 0x81004000;
constexpr std::uint32_t unaligned_address = 0x83000001;
constexpr std::uint32_t locked_virtual = 0x81001234;
constexpr std::uint32_t locked_physical = 0x40001234;
constexpr std::uint32_t locked_remapped = 0x50001234;
constexpr std::uint32_t alternate_physical = 0x60001234;
constexpr std::uint32_t locked_region = 0x81000000;
constexpr std::uint32_t alternate_descriptor_address = 0x16040;
constexpr std::uint32_t pid_descriptor_address = 0x10100;
constexpr std::uint32_t low_virtual = 0x1234;
constexpr std::uint32_t pid_physical = 0x70001234;
constexpr std::uint32_t modified_virtual = 0x04001234;
constexpr std::uint32_t modified_page = 0x04001000;
constexpr std::uint32_t overflow_region = 0x84000000;

void require(bool condition,
             const std::source_location &location = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Independent MMU expectation failed at line " +
                             std::to_string(location.line()));
  }
}

void map_tables(lead_a1::Memory &memory) {
  constexpr std::size_t table_bytes = 0x4000;
  const std::vector<std::byte> zero(table_bytes);
  memory.map({.address = table_base, .size = table_bytes, .name = "owned first-level table"}, zero);
  memory.map({.address = alternate_table, .size = table_bytes, .name = "owned alternate table"},
             zero);
  memory.map({.address = coarse_base, .size = 0x400, .name = "owned coarse table"},
             std::span(zero).first(0x400));
  memory.map({.address = fine_base, .size = 0x1000, .name = "owned fine table"},
             std::span(zero).first(0x1000));
}

lead_a1::ArmMmuInitial initial() {
  return {.translation_table = table_base,
          .domain_access = 3,
          .fcse_pid = 0,
          .context_id = {},
          .tlb_lockdown = 0,
          .tlb_invalid = true,
          .locked_tlb_invalid = true};
}

MmuRequest access(std::uint32_t address) {
  return {.access = {.address = address, .size = 4},
          .kind = lead_a1::AccessKind::read,
          .control = control,
          .user = false,
          .retain = true};
}

MmuTranslation translate(ArmMmu &mmu, MmuRequest request) {
  mmu.begin_instruction();
  try {
    const auto result = mmu.translate(request);
    mmu.commit_instruction();
    return result;
  } catch (...) {
    mmu.abort_instruction();
    throw;
  }
}

void write(ArmMmu &mmu, MmuRegister reg, std::uint32_t value) {
  mmu.begin_instruction();
  try {
    mmu.write(reg, value);
    mmu.commit_instruction();
  } catch (...) {
    mmu.abort_instruction();
    throw;
  }
}

template <typename Function> void rejected(Function function, const std::string &text) {
  try {
    function();
  } catch (const std::runtime_error &error) {
    require(std::string(error.what()).find(text) != std::string::npos);
    return;
  }
  throw std::runtime_error("Expected MMU rejection did not occur");
}

void fault(ArmMmu &mmu,
           MmuRequest request,
           std::uint8_t status,
           std::optional<std::uint8_t> domain) {
  const auto before = mmu.state();
  try {
    static_cast<void>(translate(mmu, request));
  } catch (const MmuFault &error) {
    require(error.status() == status && error.domain() == domain);
    require(mmu.state() == before);
    require(std::string(error.what()).find("va=0x") != std::string::npos);
    return;
  }
  throw std::runtime_error("Expected architectural MMU fault did not occur");
}

void page_sizes() {
  lead_a1::Memory memory;
  map_tables(memory);
  memory.write({.address = section_entry, .size = 4}, section_descriptor);
  constexpr std::uint32_t coarse_descriptor = 0x18011;
  constexpr std::uint32_t fine_descriptor = 0x19013;
  memory.write({.address = coarse_descriptor_address, .size = 4}, coarse_descriptor);
  memory.write({.address = fine_descriptor_address, .size = 4}, fine_descriptor);
  constexpr std::uint32_t coarse_large = 0x30000ff1;
  constexpr std::uint32_t coarse_small = 0x30004ff2;
  constexpr std::uint32_t fine_tiny = 0x40002033;
  constexpr std::uint32_t fine_small = 0x50008ff2;
  constexpr std::uint32_t fine_large = 0x60000ff1;
  memory.write({.address = coarse_large_address, .size = 4}, coarse_large);
  memory.write({.address = coarse_small_address, .size = 4}, coarse_small);
  memory.write({.address = fine_tiny_address, .size = 4}, fine_tiny);
  memory.write({.address = fine_small_address, .size = 4}, fine_small);
  memory.write({.address = fine_large_address, .size = 4}, fine_large);
  constexpr std::array<std::array<std::uint32_t, 2>, 6> cases{{{section_virtual, section_physical},
                                                               {locked_virtual, remapped_physical},
                                                               {0x81004560, 0x30004560},
                                                               {0x82000abc, 0x400022bc},
                                                               {0x82001560, 0x50008560},
                                                               {0x82012340, 0x60002340}}};
  for (const auto &item : cases) {
    ArmMmu mmu(memory, initial());
    const auto result = translate(mmu, access(item[0]));
    require(result.physical.address == item[1]);
    require(result.modified == item[0] && !result.cacheable && !result.bufferable);
  }
}

struct PermissionExpectation {
  unsigned int privileged;
  unsigned int user;
  bool unpredictable{};
};

void check_permissions(lead_a1::Memory &memory,
                       const PermissionExpectation &expected,
                       MmuRequest request,
                       unsigned int domain) {
  auto state = initial();
  state.domain_access = domain;
  ArmMmu mmu(memory, state);
  if (domain == 0 || domain == 2) {
    fault(mmu, request, 9, 0);
    return;
  }
  if (domain == 1 && expected.unpredictable) {
    rejected([&] { static_cast<void>(translate(mmu, request)); }, "Unpredictable");
    return;
  }
  constexpr unsigned int read_bit = 1;
  constexpr unsigned int write_bit = 2;
  const auto bits = request.user ? expected.user : expected.privileged;
  const auto selected = request.kind == lead_a1::AccessKind::write ? write_bit : read_bit;
  if (domain == 1 && (bits & selected) == 0) {
    fault(mmu, request, 13, 0);
  } else {
    require(translate(mmu, request).physical.address == section_physical);
  }
}

void permission_matrix() {
  constexpr unsigned int read_bit = 1;
  constexpr std::array<PermissionExpectation, 16> permissions{
      {{.privileged = 0, .user = 0},
       {.privileged = read_bit, .user = 0},
       {.privileged = read_bit, .user = read_bit},
       {.privileged = 0, .user = 0, .unpredictable = true},
       {.privileged = 3, .user = 0},
       {.privileged = 3, .user = 0},
       {.privileged = 3, .user = 0},
       {.privileged = 3, .user = 0},
       {.privileged = 3, .user = 1},
       {.privileged = 3, .user = 1},
       {.privileged = 3, .user = 1},
       {.privileged = 3, .user = 1},
       {.privileged = 3, .user = 3},
       {.privileged = 3, .user = 3},
       {.privileged = 3, .user = 3},
       {.privileged = 3, .user = 3}}};
  lead_a1::Memory memory;
  map_tables(memory);
  for (unsigned int ap = 0; ap < 4; ++ap) {
    for (unsigned int sr = 0; sr < 4; ++sr) {
      const auto expected = permissions[(ap * 4) + sr];
      constexpr std::uint32_t descriptor_base = 0x20000012;
      memory.write({.address = section_entry, .size = 4}, descriptor_base | (ap << 10));
      for (unsigned int domain = 0; domain < 4; ++domain) {
        for (const bool user : {false, true}) {
          for (const auto kind : {lead_a1::AccessKind::read, lead_a1::AccessKind::write}) {
            auto request = access(section_virtual);
            request.control |= sr << 8;
            request.user = user;
            request.kind = kind;
            check_permissions(memory, expected, request, domain);
          }
        }
      }
    }
  }
}

void subpages_and_faults() {
  lead_a1::Memory memory;
  map_tables(memory);
  constexpr std::uint32_t coarse_descriptor = 0x18011;
  constexpr std::uint32_t subpage_descriptor = 0x70008e42;
  memory.write({.address = coarse_descriptor_address, .size = 4}, coarse_descriptor);
  memory.write({.address = subpage_descriptor_address, .size = 4}, subpage_descriptor);
  auto state = initial();
  state.domain_access = 1;
  ArmMmu mmu(memory, state);
  fault(mmu, access(subpage_zero), 15, 0);
  require(translate(mmu, access(subpage_one)).physical.address == physical_subpage_one);
  auto user = access(subpage_two);
  user.user = true;
  require(translate(mmu, user).physical.address == physical_subpage_two);
  user.kind = lead_a1::AccessKind::write;
  fault(mmu, user, 15, 0);
  user.access.address = subpage_three;
  require(translate(mmu, user).physical.address == physical_subpage_three);
  require(mmu.state().entries == 3);
  fault(mmu, access(invalid_section), 5, std::nullopt);
  fault(mmu, access(invalid_page), 7, 0);
  constexpr std::uint32_t coarse_tiny = 0x40002033;
  memory.write({.address = coarse_small_address, .size = 4}, coarse_tiny);
  fault(mmu, access(invalid_page), 7, 0);
  constexpr std::uint32_t malformed_section = 0x20000c02;
  memory.write({.address = section_entry, .size = 4}, malformed_section);
  rejected([&] { static_cast<void>(translate(mmu, access(0x80000000))); }, "descriptor SBO");
  auto unaligned = access(unaligned_address);
  unaligned.control |= 2;
  fault(mmu, unaligned, 1, std::nullopt);
}

void tlb_retention() {
  lead_a1::Memory memory;
  map_tables(memory);
  memory.write({.address = section_entry, .size = 4}, section_descriptor);
  ArmMmu mmu(memory, initial());
  auto request = access(section_virtual);
  require(translate(mmu, request).physical.address == section_physical);
  constexpr std::uint32_t remapped = 0x30000c12;
  memory.write({.address = section_entry, .size = 4}, remapped);
  require(translate(mmu, request).physical.address == section_physical);
  write(mmu, {.crn = 8, .crm = 6, .selector = 0}, 0);
  require(translate(mmu, request).physical.address == remapped_physical);
  constexpr std::uint32_t locked_descriptor = 0x40000c12;
  memory.write({.address = coarse_descriptor_address, .size = 4}, locked_descriptor);
  write(mmu, {.crn = 10, .crm = 0, .selector = 0}, 1);
  request = access(locked_virtual);
  require(translate(mmu, request).physical.address == locked_physical);
  constexpr std::uint32_t victim_one = 0x04000001;
  require(mmu.state().tlb_lockdown == victim_one);
  constexpr std::uint32_t changed_locked = 0x50000c12;
  memory.write({.address = coarse_descriptor_address, .size = 4}, changed_locked);
  write(mmu, {.crn = 8, .crm = 5, .selector = 0}, 0);
  require(mmu.state().entries == 0 && mmu.state().locked_entries == 1);
  require(translate(mmu, request).physical.address == locked_physical);
  write(mmu, {.crn = 8, .crm = 6, .selector = 1}, locked_region);
  require(translate(mmu, request).physical.address == locked_remapped);
  constexpr std::uint32_t alternate_descriptor = 0x60000c12;
  memory.write({.address = alternate_descriptor_address, .size = 4}, alternate_descriptor);
  write(mmu, {.crn = 2, .crm = 0, .selector = 0}, alternate_table);
  require(translate(mmu, request).physical.address == locked_remapped);
  write(mmu, {.crn = 8, .crm = 7, .selector = 1}, locked_region);
  require(translate(mmu, request).physical.address == alternate_physical);
}

void large_subpages_and_unknown_tlb() {
  lead_a1::Memory memory;
  map_tables(memory);
  constexpr std::uint32_t domain_fifteen_coarse = 0x181f1;
  constexpr std::uint32_t large_subpages = 0x30000e41;
  memory.write({.address = coarse_descriptor_address, .size = 4}, domain_fifteen_coarse);
  memory.write({.address = coarse_base, .size = 4}, large_subpages);
  constexpr std::uint32_t second_subpage_descriptor = 0x18010;
  memory.write({.address = second_subpage_descriptor, .size = 4}, large_subpages);
  auto state = initial();
  constexpr std::uint32_t domain_fifteen_client = 0x40000000;
  state.domain_access = domain_fifteen_client;
  ArmMmu mmu(memory, state);
  constexpr std::uint32_t first_subpage = 0x81000004;
  constexpr std::uint32_t second_subpage = 0x81004004;
  constexpr std::uint32_t second_physical = 0x30004004;
  fault(mmu, access(first_subpage), 15, 15);
  require(translate(mmu, access(second_subpage)).physical.address == second_physical);
  auto user = access(second_subpage);
  user.user = true;
  fault(mmu, user, 15, 15);
  require(mmu.state().entries == 1);
  state.domain_access = 0;
  ArmMmu denied(memory, state);
  fault(denied, access(second_subpage), 11, 15);
  state.tlb_invalid = false;
  ArmMmu unknown_normal(memory, state);
  rejected([&] { static_cast<void>(translate(unknown_normal, access(second_subpage))); },
           "initial TLB contents");
  write(unknown_normal, {.crn = 8, .crm = 7, .selector = 0}, 0);
  fault(unknown_normal, access(second_subpage), 11, 15);
  state.locked_tlb_invalid = false;
  ArmMmu unknown_locked(memory, state);
  write(unknown_locked, {.crn = 8, .crm = 7, .selector = 0}, 0);
  rejected([&] { static_cast<void>(translate(unknown_locked, access(second_subpage))); },
           "initial TLB contents");
}

void fcse_and_registers() {
  lead_a1::Memory memory;
  map_tables(memory);
  constexpr std::uint32_t pid_two = 0x04000000;
  constexpr std::uint32_t descriptor_pid = 0x70000c12;
  memory.write({.address = pid_descriptor_address, .size = 4}, descriptor_pid);
  auto state = initial();
  state.fcse_pid = pid_two;
  ArmMmu mmu(memory, state);
  auto request = access(low_virtual);
  const auto first = translate(mmu, request);
  require(first.physical.address == pid_physical && first.modified == modified_virtual);
  constexpr std::uint32_t changed = 0x80000c12;
  memory.write({.address = pid_descriptor_address, .size = 4}, changed);
  write(mmu, {.crn = 8, .crm = 7, .selector = 1}, 0x1000);
  require(translate(mmu, request).physical.address == pid_physical);
  write(mmu, {.crn = 8, .crm = 7, .selector = 1}, modified_page);
  require(translate(mmu, request).physical.address == section_virtual);
  request.control = 0;
  require(translate(mmu, request).physical.address == low_virtual);
  constexpr std::uint32_t trace_context = 0x12345678;
  write(mmu, {.crn = 13, .crm = 0, .selector = 1}, trace_context);
  require(mmu.state().context_id == trace_context);
  const auto before = mmu.state();
  rejected([&] { write(mmu, {.crn = 2, .crm = 0, .selector = 0}, table_base + 4); }, "alignment");
  require(mmu.state() == before);
  rejected([&] { write(mmu, {.crn = 13, .crm = 0, .selector = 0}, 1); }, "FCSE reserved");
  require(mmu.state() == before);
  rejected([&] { write(mmu, {.crn = 10, .crm = 0, .selector = 0}, 2); }, "lockdown reserved");
  require(mmu.state() == before);
  ArmMmu unknown(memory, std::nullopt);
  rejected([&] { static_cast<void>(translate(unknown, access(0x80000000))); }, "Unknown ARM MMU");
}

void bounded_tlb() {
  lead_a1::Memory memory;
  map_tables(memory);
  constexpr unsigned int entry_count = 65;
  constexpr std::uint32_t section_bytes = 0x100000;
  constexpr std::uint32_t virtual_base = 0x80000000;
  for (unsigned int index = 0; index < entry_count; ++index) {
    memory.write({.address = section_entry + (index * 4), .size = 4},
                 section_descriptor + (index * section_bytes));
  }
  ArmMmu mmu(memory, initial());
  for (unsigned int index = 0; index < entry_count - 1; ++index) {
    static_cast<void>(translate(mmu, access(virtual_base + (index * section_bytes))));
  }
  const auto before = mmu.state();
  rejected([&] { static_cast<void>(translate(mmu, access(overflow_region))); }, "TLB replacement");
  require(mmu.state() == before);
  auto state = initial();
  constexpr std::uint32_t last_victim = 0x1c000001;
  state.tlb_lockdown = last_victim;
  ArmMmu locked(memory, state);
  for (unsigned int index = 0; index < 9; ++index) {
    static_cast<void>(translate(locked, access(virtual_base + (index * section_bytes))));
  }
  require(locked.state().locked_entries == 8);
  require(locked.state().tlb_lockdown == 1);
}

}

int main() {
  try {
    page_sizes();
    permission_matrix();
    subpages_and_faults();
    tlb_retention();
    large_subpages_and_unknown_tlb();
    fcse_and_registers();
    bounded_tlb();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
