// SPDX-License-Identifier: GPL-3.0-only
#include "arm/peripherals/aintc.hpp"

#include <array>
#include <cstdint>
#include <exception>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>

namespace {

constexpr std::uint32_t none = 0x80000000;
constexpr std::uint32_t cr = 4;
constexpr std::uint32_t gnlr = 0x1c;
constexpr std::uint32_t irq_level = 0x1104;
constexpr std::uint32_t fiq_level = 0x1100;
constexpr std::uint32_t irq_index = 0x904;
constexpr std::uint32_t fiq_index = 0x900;
constexpr std::uint32_t fiq_vector = 0x1600;
constexpr std::uint32_t irq_vector = 0x1604;
constexpr std::uint32_t status_clear = 0x24;
constexpr std::uint32_t source_set = 0x20;
constexpr std::uint32_t initial_level = 0x100;
constexpr unsigned int channels = 32;

void require(bool condition, const std::source_location &where = std::source_location::current()) {
  if (!condition) {
    throw std::runtime_error("Independent nesting expectation at line " +
                             std::to_string(where.line()));
  }
}

std::uint32_t read(lead_a1::Aintc &device, std::uint32_t offset) {
  return static_cast<std::uint32_t>(device.read({.address = offset, .size = 4}));
}

void write(lead_a1::Aintc &device, std::uint32_t offset, std::uint32_t value) {
  device.write({.address = offset, .size = 4}, value);
}

lead_a1::AintcState initial() {
  auto state = lead_a1::Aintc::reset_reference();
  state.global_enabled = true;
  state.host_enabled = 3;
  state.vector_base = 0x4000;
  state.vector_size_programmed = true;
  return state;
}

void cutoff_matrix() {
  for (const std::uint32_t mode : {4U, 8U, 0U, 12U}) {
    for (unsigned int channel = 0; channel < channels; ++channel) {
      for (unsigned int cutoff = 0; cutoff <= channels; ++cutoff) {
        auto state = initial();
        state.control = mode;
        state.channels[3] = channel;
        state.pending[0] = 8;
        state.enabled[0] = 8;
        lead_a1::Aintc device(state);
        write(device, gnlr, none | cutoff);
        write(device, irq_level, none | cutoff);
        write(device, fiq_level, none | cutoff);
        const bool allowed = mode == 0 || mode == 12 || channel < cutoff;
        require(device.outputs() == (std::array{allowed && channel < 2, allowed && channel >= 2}));
        const auto host_register = channel < 2 ? fiq_index : irq_index;
        require(read(device, host_register) == (allowed ? 3 : none));
        require(read(device, irq_level) == cutoff && read(device, fiq_level) == cutoff);
        require(read(device, gnlr) == cutoff);
      }
    }
  }
}

void per_host_protocol() {
  auto state = initial();
  state.control = 4;
  state.channels[7] = 5;
  state.channels[8] = 5;
  state.channels[9] = 2;
  constexpr std::uint32_t sources_seven_to_nine = 0x380;
  state.enabled[0] = sources_seven_to_nine;
  state.pending[0] = 0x80;
  lead_a1::Aintc device(state);
  require(read(device, irq_level) == 5 && read(device, fiq_level) == initial_level);
  write(device, irq_level, none | 5);
  require(read(device, irq_level) == 5 && read(device, irq_index) == none);
  write(device, status_clear, 7);
  write(device, source_set, 8);
  require(!device.outputs()[1] && read(device, irq_index) == none);
  write(device, source_set, 9);
  require(device.outputs()[1] && read(device, irq_index) == 9);
  write(device, irq_level, 0);
  require(read(device, irq_level) == 2);
  write(device, irq_level, none | 2);
  require(!device.outputs()[1] && read(device, irq_index) == none);
  write(device, status_clear, 9);
  write(device, irq_level, none | 5);
  require(!device.outputs()[1]);
  constexpr std::uint32_t observed_board_limit = 100;
  write(device, irq_level, none | observed_board_limit);
  require(device.outputs()[1] && read(device, irq_index) == 8);
  require(read(device, irq_level) == observed_board_limit);
  write(device, irq_level, 0);
  require(read(device, irq_level) == 5);
  write(device, status_clear, 8);
  require(read(device, irq_level) == initial_level);
}

void independent_hosts_and_global() {
  auto state = initial();
  state.control = 4;
  state.channels[0] = 0;
  state.channels[1] = 1;
  state.channels[2] = 2;
  state.channels[3] = 5;
  state.enabled[0] = 15;
  state.pending[0] = 15;
  lead_a1::Aintc device(state);
  write(device, irq_level, none | 2);
  require(device.outputs() == (std::array{true, false}));
  write(device, fiq_level, none | 1);
  require(read(device, fiq_index) == 0 && read(device, irq_index) == none);
  write(device, status_clear, 0);
  require(device.outputs() == (std::array{false, false}));
  write(device, cr, 8);
  require(read(device, gnlr) == 1);
  write(device, gnlr, none | 2);
  require(device.outputs() == (std::array{true, false}));
  write(device, gnlr, none | 1);
  require(device.outputs() == (std::array{false, false}));
  write(device, gnlr, 0);
  require(device.outputs() == (std::array{true, true}));
  require(read(device, gnlr) == 1);
}

void held_priority_and_rollback() {
  auto state = initial();
  constexpr std::uint32_t individual_hold = 0x14;
  state.control = individual_hold;
  state.channels[3] = 5;
  state.enabled[0] = 8;
  state.pending[0] = 8;
  lead_a1::Aintc device(state);
  constexpr std::uint32_t vector_three = 0x400c;
  require(read(device, irq_vector) == vector_three);
  const auto before = device.state();
  device.begin_instruction();
  write(device, irq_level, none | 5);
  require(read(device, irq_index) == 3 && !device.outputs()[1]);
  require(read(device, irq_level) == 5);
  write(device, irq_index, 3);
  require(read(device, irq_index) == none);
  device.abort_instruction();
  require(device.state() == before && device.outputs()[1]);
  require(read(device, irq_index) == 3 && read(device, irq_vector) == vector_three);
  device.begin_instruction();
  write(device, irq_level, none | 5);
  device.commit_instruction();
  require(device.state().host_nesting_override[1] && !device.outputs()[1]);
  write(device, irq_vector, 0);
  require(read(device, irq_index) == none);
  write(device, irq_level, 0);
  require(read(device, irq_index) == none);
  write(device, irq_index, 0);
  require(read(device, irq_index) == 3);
  require(read(device, fiq_vector) == 0);
}

}

int main() {
  try {
    cutoff_matrix();
    per_host_protocol();
    independent_hosts_and_global();
    held_priority_and_rollback();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
