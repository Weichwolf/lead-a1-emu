// SPDX-License-Identifier: GPL-3.0-only
#include "arm/peripherals/gpio.hpp"

#include "common/memory.hpp"

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr unsigned int pins_per_bank = 16;
constexpr unsigned int maximum_pins = gpio_banks * pins_per_bank;
constexpr std::uint32_t pair_base = 0x10;
constexpr std::uint32_t pair_stride = 0x28;
constexpr std::uint32_t register_end = pair_base + (5 * pair_stride);
constexpr unsigned int byte_bits = 8;
constexpr std::uint32_t half_mask = 0xffff;
constexpr std::uint32_t data_offset = 4;
constexpr std::uint32_t set_offset = 8;
constexpr std::uint32_t clear_offset = 12;
constexpr std::uint32_t input_offset = 16;
constexpr std::uint32_t rise_set = 20;
constexpr std::uint32_t rise_clear = 24;
constexpr std::uint32_t fall_set = 28;
constexpr std::uint32_t fall_clear = 32;
constexpr std::uint32_t status_offset = 36;

std::uint16_t pin_mask(const GpioState &state, unsigned int bank) {
  const auto start = bank * pins_per_bank;
  if (start >= state.pins) { return 0; }
  const auto remaining = state.pins - start;
  return static_cast<std::uint16_t>(remaining >= pins_per_bank ? half_mask : (1U << remaining) - 1);
}

std::uint32_t bank_mask(const GpioState &state) {
  const auto count = (state.pins + pins_per_bank - 1) / pins_per_bank;
  return (1U << count) - 1;
}

void validate_sample(GpioSample levels, std::uint16_t allowed) {
  if ((levels.known & ~allowed) != 0 || (levels.value & ~levels.known) != 0) {
    throw std::invalid_argument("Invalid GPIO sample knowledge or unavailable pins");
  }
}

void validate(const GpioState &state) {
  if (state.pins == 0 || state.pins > maximum_pins) {
    throw std::invalid_argument("Invalid GPIO reference pin count");
  }
  if ((state.bank_enabled & ~bank_mask(state)) != 0) {
    throw std::invalid_argument("Invalid GPIO bank enable input");
  }
  for (unsigned int index = 0; index < gpio_banks; ++index) {
    const auto &bank = state.banks[index];
    const auto allowed = pin_mask(state, index);
    const auto used = bank.direction | bank.output | bank.rising | bank.falling | bank.pending;
    if ((used & ~allowed) != 0) { throw std::invalid_argument("Invalid GPIO register input"); }
    validate_sample(bank.input, allowed);
  }
}

struct Register {
  unsigned int first_bank;
  std::uint32_t offset;
  unsigned int shift;
  std::uint32_t mask;
};

Register decode(MemoryAccess access) {
  if ((access.size != 1 && access.size != 2 && access.size != 4) ||
      access.address % access.size != 0 || access.address >= register_end ||
      (access.address % 4) + access.size > 4) {
    throw std::runtime_error("Invalid GPIO register width or alignment");
  }
  const auto word = access.address & ~3U;
  if (word < pair_base && word != 0 && word != 8) {
    throw std::runtime_error("Unmapped GPIO register");
  }
  const auto relative = word >= pair_base ? word - pair_base : 0;
  return {.first_bank = (relative / pair_stride) * 2,
          .offset = word >= pair_base ? relative % pair_stride : word,
          .shift = (access.address % 4) * byte_bits,
          .mask = access.size == 4 ? std::numeric_limits<std::uint32_t>::max()
                                   : (1U << (access.size * byte_bits)) - 1};
}

struct BankRead {
  std::uint32_t offset;
  std::uint16_t requested;
};

std::uint32_t read_bank(const GpioBank &bank, BankRead operation) {
  switch (operation.offset) {
    case 0: return bank.direction;
    case data_offset:
    case set_offset:
    case clear_offset: return bank.output;
    case input_offset:
      if ((operation.requested & ~bank.input.known) != 0) {
        throw std::runtime_error("Unknown GPIO synchronized input pins");
      }
      return bank.input.value;
    case rise_set:
    case rise_clear: return bank.rising;
    case fall_set:
    case fall_clear: return bank.falling;
    case status_offset: return bank.pending;
    default: throw std::runtime_error("Unsupported GPIO register read");
  }
}

struct BankWrite {
  std::uint32_t offset;
  std::uint16_t mask;
  std::uint16_t value;
};

void write_bank(GpioBank &bank, BankWrite operation) {
  const auto value = operation.value;
  const auto replace = [operation](std::uint16_t prior) {
    return static_cast<std::uint16_t>((prior & ~operation.mask) | operation.value);
  };
  switch (operation.offset) {
    case 0: bank.direction = replace(bank.direction); break;
    case data_offset: bank.output = replace(bank.output); break;
    case set_offset: bank.output |= value; break;
    case clear_offset: bank.output &= static_cast<std::uint16_t>(~value); break;
    case rise_set: bank.rising |= value; break;
    case rise_clear: bank.rising &= static_cast<std::uint16_t>(~value); break;
    case fall_set: bank.falling |= value; break;
    case fall_clear: bank.falling &= static_cast<std::uint16_t>(~value); break;
    case status_offset: bank.pending &= static_cast<std::uint16_t>(~value); break;
    default: throw std::runtime_error("Read-only or unsupported GPIO register write");
  }
}

std::uint32_t read_word(const GpioState &state, MemoryAccess access, const Register &reg) {
  if (access.address < pair_base) {
    if (reg.offset == 8) { return state.bank_enabled; }
    if (!state.revision) { throw std::runtime_error("Unknown GPIO revision input"); }
    return *state.revision;
  }
  std::uint32_t result{};
  const auto requested = reg.mask << reg.shift;
  for (unsigned int lane = 0; lane < 2; ++lane) {
    const auto index = reg.first_bank + lane;
    if (index >= gpio_banks) { continue; }
    const auto needed =
        static_cast<std::uint16_t>((requested >> (lane * pins_per_bank)) & pin_mask(state, index));
    if (needed == 0) { continue; }
    result |= read_bank(state.banks[index], {.offset = reg.offset, .requested = needed})
              << (lane * pins_per_bank);
  }
  return result;
}

void write_word(GpioState &state, MemoryAccess access, const Register &reg, std::uint32_t value) {
  const auto shifted = value << reg.shift;
  const auto mask = reg.mask << reg.shift;
  if (access.address < pair_base) {
    if (reg.offset != 8) { throw std::runtime_error("Read-only GPIO revision register"); }
    if ((shifted & ~bank_mask(state)) != 0) {
      throw std::runtime_error("Reserved GPIO bank enable bits");
    }
    state.bank_enabled = static_cast<std::uint16_t>((state.bank_enabled & ~mask) | shifted);
    return;
  }
  for (unsigned int lane = 0; lane < 2; ++lane) {
    const auto index = reg.first_bank + lane;
    const auto selected = static_cast<std::uint16_t>(mask >> (lane * pins_per_bank));
    const auto bits = static_cast<std::uint16_t>(shifted >> (lane * pins_per_bank));
    const auto allowed = pin_mask(state, index);
    if ((bits & ~allowed) != 0) { throw std::runtime_error("Reserved GPIO pin bits"); }
    if (index < gpio_banks && selected != 0) {
      write_bank(state.banks[index], {.offset = reg.offset, .mask = selected, .value = bits});
    }
  }
}

}

Gpio::Gpio(const GpioState &initial) : current_(initial) {
  validate(initial);
}

GpioState Gpio::reset_reference(unsigned int pins) {
  constexpr std::uint32_t revision = 0x44830105;
  GpioState result{.pins = pins, .revision = revision, .bank_enabled = 0, .banks = {}};
  validate(result);
  for (unsigned int bank = 0; bank < gpio_banks; ++bank) {
    result.banks[bank].direction = pin_mask(result, bank);
  }
  return result;
}

GpioState Gpio::state() const {
  return working();
}

std::array<GpioDrive, gpio_banks> Gpio::drives() const {
  const auto &state = working();
  std::array<GpioDrive, gpio_banks> result{};
  for (unsigned int bank = 0; bank < gpio_banks; ++bank) {
    const auto &source = state.banks[bank];
    result[bank].enabled = static_cast<std::uint16_t>(pin_mask(state, bank) & ~source.direction);
    result[bank].high = source.output & result[bank].enabled;
  }
  return result;
}

std::array<bool, gpio_banks> Gpio::interrupts() const {
  const auto &state = working();
  std::array<bool, gpio_banks> result{};
  for (unsigned int bank = 0; bank < gpio_banks; ++bank) {
    result[bank] = (state.bank_enabled & (1U << bank)) != 0 && state.banks[bank].pending != 0;
  }
  return result;
}

void Gpio::check_external_event() const {
  if (pending_) { throw std::logic_error("GPIO sample during active instruction"); }
}

void Gpio::check_sample(unsigned int bank, GpioSample levels) const {
  check_external_event();
  if (bank >= gpio_banks || pin_mask(current_, bank) == 0) {
    throw std::invalid_argument("Invalid GPIO sample bank");
  }
  validate_sample(levels, pin_mask(current_, bank));
  const auto &target = current_.banks[bank];
  const auto armed = target.rising | target.falling;
  if ((armed & ~(target.input.known & levels.known)) != 0) {
    throw std::runtime_error("Unknown GPIO edge baseline or new sample");
  }
}

void Gpio::sample(unsigned int bank, GpioSample levels) {
  check_sample(bank, levels);
  auto &target = current_.banks[bank];
  const auto rising = ~target.input.value & levels.value & target.rising;
  const auto falling = target.input.value & ~levels.value & target.falling;
  target.pending |= static_cast<std::uint16_t>(rising | falling);
  target.input = levels;
}

std::uint64_t Gpio::read(MemoryAccess access) {
  const auto reg = decode(access);
  return (read_word(working(), access, reg) >> reg.shift) & reg.mask;
}

void Gpio::write(MemoryAccess access, std::uint64_t value) {
  const auto reg = decode(access);
  if (value > reg.mask) { throw std::runtime_error("Oversized GPIO write value"); }
  auto next = working();
  write_word(next, access, reg, static_cast<std::uint32_t>(value));
  working() = next;
}

void Gpio::begin_instruction() {
  if (pending_) { throw std::logic_error("Nested GPIO instruction transaction"); }
  pending_ = current_;
}

void Gpio::commit_instruction() noexcept {
  if (pending_) { current_ = *pending_; }
  pending_.reset();
}

void Gpio::abort_instruction() noexcept {
  pending_.reset();
}

GpioState &Gpio::working() {
  return pending_ ? *pending_ : current_;
}

const GpioState &Gpio::working() const {
  return pending_ ? *pending_ : current_;
}

}
