// SPDX-License-Identifier: GPL-3.0-only
#include "lead_a1/sharc_float.hpp"

#include <cstdint>
#include <stdexcept>

namespace lead_a1 {
namespace {

constexpr std::uint64_t full = 0xffffffffff;
constexpr std::uint64_t sign = 0x8000000000;
constexpr std::uint64_t fraction = 0x7fffffff;
constexpr std::uint64_t low_byte = 0xff;
constexpr std::uint64_t hidden = 0x80000000;
constexpr std::uint64_t infinity = 0x7f80000000;
constexpr unsigned int exponent_shift = 31;
constexpr unsigned int exponent_mask = 255;
constexpr unsigned int guard_bits = 3;
constexpr std::uint64_t normalized = hidden << guard_bits;

struct Operand {
  std::uint64_t significand{};
  int exponent{};
  bool negative{};
};

void validate_controls(SharcFloatFormat format, SharcFloatRounding rounding) {
  if (format != SharcFloatFormat::binary32 && format != SharcFloatFormat::extended40) {
    throw std::invalid_argument("Invalid SHARC floating precision");
  }
  if (rounding != SharcFloatRounding::nearest_even && rounding != SharcFloatRounding::toward_zero) {
    throw std::invalid_argument("Invalid SHARC floating rounding");
  }
}

Operand unpack(std::uint64_t value, SharcFloatFormat format) {
  if ((value & ~full) != 0) { throw std::invalid_argument("Invalid SHARC40 floating input bits"); }
  if (format == SharcFloatFormat::binary32) { value &= ~low_byte; }
  const auto exponent = static_cast<unsigned int>((value >> exponent_shift) & exponent_mask);
  if (exponent == exponent_mask) {
    throw std::runtime_error("SHARC floating NAN/infinity input not qualified");
  }
  return {.significand = exponent == 0 ? 0 : (value & fraction) | hidden,
          .exponent = static_cast<int>(exponent),
          .negative = (value & sign) != 0};
}

std::uint64_t shift_jam(std::uint64_t value, unsigned int count) {
  if (count == 0) { return value; }
  constexpr unsigned int bits = 64;
  if (count >= bits) { return value == 0 ? 0 : 1; }
  const auto discarded = value & ((std::uint64_t{1} << count) - 1);
  return (value >> count) | (discarded == 0 ? 0U : 1U);
}

SharcFloatResult rounded(Operand sum, SharcFloatFormat format, SharcFloatRounding rounding) {
  auto magnitude = sum.significand;
  auto exponent = sum.exponent;
  const auto signed_bit = sum.negative ? sign : 0;
  if (magnitude == 0) { return {.value = signed_bit}; }
  if (magnitude >= (normalized << 1)) {
    magnitude = shift_jam(magnitude, 1);
    ++exponent;
  }
  while (magnitude < normalized) {
    magnitude <<= 1;
    --exponent;
  }
  constexpr unsigned int extension = 8;
  const auto padding = format == SharcFloatFormat::binary32 ? extension : 0;
  const auto drop = guard_bits + padding;
  const auto half = std::uint64_t{1} << (drop - 1);
  const auto residue = magnitude & ((half << 1) - 1);
  auto kept = magnitude >> drop;
  if (rounding == SharcFloatRounding::nearest_even &&
      (residue > half || (residue == half && (kept & 1) != 0))) {
    ++kept;
  }
  const auto boundary = (hidden << 1) >> padding;
  if (kept >= boundary) {
    kept >>= 1;
    ++exponent;
  }
  if (exponent >= static_cast<int>(exponent_mask)) {
    const auto maximum = (infinity - 1) & ~((std::uint64_t{1} << padding) - 1);
    return {.value =
                signed_bit | (rounding == SharcFloatRounding::toward_zero ? maximum : infinity),
            .overflow = true};
  }
  if (exponent <= 0) { return {.value = signed_bit, .underflow = true}; }
  return {.value = signed_bit | (static_cast<std::uint64_t>(exponent) << exponent_shift) |
                   ((kept << padding) & fraction)};
}

}

SharcFloatResult sharc_float_add(std::uint64_t left,
                                 std::uint64_t right,
                                 SharcFloatFormat format,
                                 SharcFloatRounding rounding,
                                 bool subtract) {
  validate_controls(format, rounding);
  auto first = unpack(left, format);
  auto second = unpack(right, format);
  second.negative = second.negative != subtract;
  if (first.exponent < second.exponent ||
      (first.exponent == second.exponent && first.significand < second.significand)) {
    const auto saved = first;
    first = second;
    second = saved;
  }
  const auto distance = static_cast<unsigned int>(first.exponent - second.exponent);
  const auto larger = first.significand << guard_bits;
  const auto smaller = shift_jam(second.significand << guard_bits, distance);
  const bool same_sign = first.negative == second.negative;
  const auto result = same_sign ? larger + smaller : larger - smaller;
  // Under both supported rounding modes, exact opposite-sign cancellation
  // returns +zero. Two negative zeros add to -zero.
  const bool negative = first.negative && (result != 0 || same_sign);
  return rounded(
      {.significand = result, .exponent = first.exponent, .negative = negative}, format, rounding);
}

SharcFloatResult sharc_float_multiply(std::uint64_t left,
                                      std::uint64_t right,
                                      SharcFloatFormat format,
                                      SharcFloatRounding rounding) {
  validate_controls(format, rounding);
  const auto first = unpack(left, format);
  const auto second = unpack(right, format);
  // Each significand has at most32 bits, so its exact product fits unsigned64.
  // Product bit62 represents1.0; bit34 is the common rounding input's1.0.
  constexpr unsigned int product_shift = 28;
  constexpr int bias = 127;
  const auto product = first.significand * second.significand;
  return rounded({.significand = shift_jam(product, product_shift),
                  .exponent = first.exponent + second.exponent - bias,
                  .negative = first.negative != second.negative},
                 format,
                 rounding);
}

}
