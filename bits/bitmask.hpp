#pragma once

#include <bit>
#include <limits>

#include "enum_common.hpp"
#include "equality.hpp"

template <std::size_t bit>
struct StartBit : public std::integral_constant<decltype(bit), bit> {};

template <std::size_t num_bits>
    requires GreaterThan<num_bits, std::size_t{0u}>
struct NumBits : public std::integral_constant<decltype(num_bits), num_bits> {};

namespace _detail {
template <NumBits num, StartBit begin>
    requires LessThanEqual<
        num.value + begin.value,
        std::bit_width(std::numeric_limits<std::size_t>::max())>
constexpr std::size_t bitmask() {
	return ((std::size_t{1u} << num.value) - std::size_t{1u}) << begin.value;
}

template <>
constexpr std::size_t bitmask<NumBits<64u>{}, StartBit<0u>{}>() {
	return std::numeric_limits<std::size_t>::max();
}

}  // namespace _detail

template <std::size_t mask>
struct BitMask : public std::integral_constant<decltype(mask), mask> {
	consteval auto operator~() const noexcept {
		constexpr auto val = this->value_type();
		return BitMask<~val>{};
	}

	consteval auto operator^(const BitMask &b) const noexcept {
		constexpr auto val = this->value_type();
		return BitMask<val ^ b.value>{};
	}

	consteval auto operator&(const BitMask &b) const noexcept {
		constexpr auto val = this->value_type();
		return BitMask<val & b.value>{};
	}

	consteval auto operator|(const BitMask &b) const noexcept {
		constexpr auto val = this->value_type();
		return BitMask<val | b.value>{};
	}

	template <std::size_t num_bits>
	consteval auto operator<<(const NumBits<num_bits> &num) const noexcept {
		constexpr auto val = this->value_type();
		return BitMask<val << num.value>{};
	}

	template <std::size_t num_bits>
	consteval auto operator>>(const NumBits<num_bits> &num) const noexcept {
		constexpr auto this_val = this->value_type();
		constexpr auto val = this_val >> num.value;
		return BitMask<val>{};
	}

	consteval std::size_t bits_in_mask() const noexcept {
		return std::bit_width(this->value);
	}
};

template <NumBits num, StartBit begin = StartBit<std::size_t{0u}>{}>
    requires GreaterThan<_detail::bitmask<num, begin>(), std::size_t{0u}>
struct ConsecutiveBitMask : public BitMask<_detail::bitmask<num, begin>()> {
	constexpr ConsecutiveBitMask() = default;
};

template <StartBit begin>
struct SingleBitMask
    : public ConsecutiveBitMask<NumBits<std::size_t{1u}>{}, begin> {};
