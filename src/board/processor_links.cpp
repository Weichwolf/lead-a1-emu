// SPDX-License-Identifier: GPL-3.0-only
#include "board/processor_links.hpp"

#include "arm/peripherals/gpio.hpp"
#include "sharc/core/sharc.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lead_a1 {
namespace {

constexpr unsigned int pins_per_bank = 16;
constexpr unsigned int flag_count = 16;
constexpr std::uint8_t gpio_driver = 1;
constexpr std::uint8_t dsp_driver = 2;
constexpr std::uint8_t external_driver = 4;
constexpr std::uint8_t all_drivers = 7;

void validate_drive(WireDrive drive) {
  switch (drive) {
    case WireDrive::floating:
    case WireDrive::low:
    case WireDrive::high:
    case WireDrive::unresolved: return;
    case WireDrive::invalid:
    default: throw std::invalid_argument("Invalid processor wire external drive");
  }
}

std::optional<bool> resolved(const ProcessorWireLevel &wire, std::optional<bool> pull) {
  if (wire.unresolved != 0 || (wire.high != 0 && wire.low != 0)) { return {}; }
  if (wire.high != 0) { return true; }
  if (wire.low != 0) { return false; }
  return pull;
}

void validate_level(const ProcessorWireLevel &wire, ProcessorWire topology, bool sampled) {
  const auto used = wire.high | wire.low | wire.unresolved;
  const auto pull = sampled ? topology.pull : std::nullopt;
  if ((used & ~all_drivers) != 0 || (wire.high & wire.low) != 0 ||
      ((wire.high | wire.low) & wire.unresolved) != 0 || wire.level != resolved(wire, pull)) {
    throw std::invalid_argument("Invalid processor wire resolution snapshot");
  }
  if (!sampled && wire != ProcessorWireLevel{}) {
    throw std::invalid_argument("Unsampled processor link has published levels");
  }
}

ProcessorLinkState initial(std::vector<ProcessorWire> wires) {
  const auto count = wires.size();
  return {.wires = std::move(wires),
          .external = std::vector<WireDrive>(count, WireDrive::floating),
          .levels = std::vector<ProcessorWireLevel>(count)};
}

ProcessorWireLevel
resolve(GpioDrive gpio, SharcFlagDrive dsp, ProcessorWire wire, WireDrive drive) {
  const auto pin = std::uint16_t{1} << wire.pin;
  const auto flag = std::uint16_t{1} << wire.flag;
  ProcessorWireLevel result;
  if ((gpio.enabled & pin) != 0) {
    if ((gpio.high & pin) != 0) {
      result.high |= gpio_driver;
    } else {
      result.low |= gpio_driver;
    }
  }
  if ((dsp.high & flag) != 0) { result.high |= dsp_driver; }
  if ((dsp.low & flag) != 0) { result.low |= dsp_driver; }
  if ((dsp.unresolved & flag) != 0) { result.unresolved |= dsp_driver; }
  if (drive == WireDrive::high) { result.high |= external_driver; }
  if (drive == WireDrive::low) { result.low |= external_driver; }
  if (drive == WireDrive::unresolved) { result.unresolved |= external_driver; }
  result.level = resolved(result, wire.pull);
  return result;
}

void replace(GpioSample &sample, unsigned int pin, std::optional<bool> level) {
  const auto bit = static_cast<std::uint16_t>(std::uint32_t{1} << pin);
  sample.value &= static_cast<std::uint16_t>(~bit);
  sample.known &= static_cast<std::uint16_t>(~bit);
  if (level) {
    sample.known |= bit;
    if (*level) { sample.value |= bit; }
  }
}

void replace(SharcKnown32 &sample, unsigned int flag, std::optional<bool> level) {
  const auto bit = std::uint32_t{1} << flag;
  sample.value &= ~bit;
  sample.known &= ~bit;
  if (level) {
    sample.known |= bit;
    if (*level) { sample.value |= bit; }
  }
}

}

ProcessorLinks::ProcessorLinks(Gpio &gpio,
                               std::span<Sharc *const> dsps,
                               std::vector<ProcessorWire> wires)
    : ProcessorLinks(gpio, dsps, initial(std::move(wires))) {}

ProcessorLinks::ProcessorLinks(Gpio &gpio,
                               std::span<Sharc *const> dsps,
                               ProcessorLinkState restored)
    : gpio_(gpio),
      dsps_(dsps.begin(), dsps.end()),
      state_(std::move(restored)),
      flag_samples_(dsps.size()),
      next_levels_(state_.wires.size()) {
  validate();
  for (const auto wire : state_.wires) { used_banks_[wire.bank] = true; }
}

void ProcessorLinks::validate() const {
  if (dsps_.empty() || state_.wires.empty() || state_.external.size() != state_.wires.size() ||
      state_.levels.size() != state_.wires.size()) {
    throw std::invalid_argument("Processor links require explicit DSPs, wires and matching state");
  }
  for (std::size_t index = 0; index < dsps_.size(); ++index) {
    if (dsps_[index] == nullptr) { throw std::invalid_argument("Null processor link DSP"); }
    for (std::size_t previous = 0; previous < index; ++previous) {
      if (dsps_[index] == dsps_[previous]) {
        throw std::invalid_argument("Aliased processor link DSP");
      }
    }
  }
  const auto pins = gpio_.state().pins;
  for (std::size_t index = 0; index < state_.wires.size(); ++index) {
    const auto wire = state_.wires[index];
    if (wire.bank >= gpio_banks || wire.pin >= pins_per_bank ||
        (wire.bank * pins_per_bank) + wire.pin >= pins || wire.dsp >= dsps_.size() ||
        wire.flag >= flag_count) {
      throw std::invalid_argument("Invalid processor link endpoint");
    }
    validate_drive(state_.external[index]);
    validate_level(state_.levels[index], wire, state_.sampled);
    for (std::size_t previous = 0; previous < index; ++previous) {
      const auto other = state_.wires[previous];
      if ((wire.bank == other.bank && wire.pin == other.pin) ||
          (wire.dsp == other.dsp && wire.flag == other.flag)) {
        throw std::invalid_argument("Duplicate processor link endpoint");
      }
    }
  }
}

ProcessorLinkState ProcessorLinks::snapshot() const {
  return state_;
}

void ProcessorLinks::external(unsigned int wire, WireDrive drive) {
  if (wire >= state_.wires.size()) { throw std::invalid_argument("Invalid processor wire index"); }
  validate_drive(drive);
  state_.external[wire] = drive;
}

void ProcessorLinks::prepare() {
  const auto gpio = gpio_.state();
  for (unsigned int bank = 0; bank < gpio_banks; ++bank) {
    gpio_samples_[bank] = gpio.banks[bank].input;
    if (used_banks_[bank]) { gpio_.check_sample(bank, gpio_samples_[bank]); }
  }
  for (std::size_t index = 0; index < dsps_.size(); ++index) {
    flag_samples_[index] = dsps_[index]->state().flag_inputs;
    dsps_[index]->check_flag_inputs(flag_samples_[index]);
  }
  const auto drives = gpio_.drives();
  for (std::size_t index = 0; index < state_.wires.size(); ++index) {
    const auto wire = state_.wires[index];
    const auto level =
        resolve(drives[wire.bank], dsps_[wire.dsp]->flag_drives(), wire, state_.external[index]);
    next_levels_[index] = level;
    replace(gpio_samples_[wire.bank], wire.pin, level.level);
    replace(flag_samples_[wire.dsp], wire.flag, level.level);
  }
}

void ProcessorLinks::sample() {
  prepare();
  for (unsigned int bank = 0; bank < gpio_banks; ++bank) {
    if (used_banks_[bank]) { gpio_.check_sample(bank, gpio_samples_[bank]); }
  }
  for (std::size_t index = 0; index < dsps_.size(); ++index) {
    dsps_[index]->check_flag_inputs(flag_samples_[index]);
  }
  // No callbacks, allocations or intervening guest execution after validation.
  for (unsigned int bank = 0; bank < gpio_banks; ++bank) {
    if (used_banks_[bank]) { gpio_.sample(bank, gpio_samples_[bank]); }
  }
  for (std::size_t index = 0; index < dsps_.size(); ++index) {
    dsps_[index]->flag_inputs(flag_samples_[index]);
  }
  state_.levels.swap(next_levels_);
  state_.sampled = true;
}

}
