#pragma once

#include <bitset>
#include <iostream>
#include <type_traits>
#include <utility>

#include "enum_common.hpp"

namespace enum_utils {
template <EnumHasLast EnumT>
class EnumFlag {
   public:
	using AsBits = std::bitset<underlying(EnumT::LAST)>;

	/**
	 * @brief Set the specified bits.
	 *
	 * @tparam Es - The bits to set.
	 *
	 * @return a reference to this flag.
	 */
	template <EnumT... Es>
	constexpr EnumFlag& set() noexcept {
		((bits_.set(UEnumValue<Es>())), ...);
		return *this;
	}

	/**
	 * @brief Set all the bits in this flag.
	 *
	 * @return a reference to this flag.
	 */
	constexpr EnumFlag& set() noexcept {
		bits_.set();
		return *this;
	}

	/**
	 * @brief Reset the specified bits.
	 *
	 * @tparam Es - The bits to reset.
	 *
	 * @return a reference to this flag.
	 */
	template <EnumT... Es>
	constexpr EnumFlag& reset() noexcept {
		(bits_.reset(EnumValue(Es)), ...);
		return *this;
	}

	/**
	 * @brief Reset all the bits.
	 *
	 * @return a reference to this flag.
	 */
	constexpr EnumFlag& reset() noexcept {
		bits_.reset();
		return *this;
	}

	/**
	 * @brief Flip the specified bits.
	 *
	 * @tparam Es - The bits to flip.
	 *
	 * @return a reference to this flag.
	 */
	template <EnumT... Es>
	constexpr EnumFlag& flip() noexcept {
		(bits_.flip(EnumValue(Es)), ...);
		return *this;
	}

	/**
	 * @brief Flip all the bits.
	 *
	 * @return a reference to this flag.
	 */
	constexpr EnumFlag& flip() noexcept {
		bits_.flip();
		return *this;
	}

	/**
	 * @brief Check if all the bits are set.
	 *
	 * @return true if all bits are set.
	 * @return false if at least one bit is not set.
	 */
	[[nodiscard]] constexpr bool all() const noexcept { return bits_.all(); }

	/**
	 * @brief Check if any of the bits are set.
	 *
	 * @return true if at least one bit is set.
	 * @return false if none of the bits are set.
	 */
	[[nodiscard]] constexpr bool any() const noexcept { return bits_.any(); }

	/**
	 * @brief Check if no bits are set.
	 *
	 * @return true if no bits are set.
	 * @return false if any bit is set.
	 */
	[[nodiscard]] constexpr bool none() const noexcept { return bits_.none(); }

	/**
	 * @brief Get the number of bits in this flag.
	 *
	 * @return the number of bits in this flag.
	 */
	[[nodiscard]] constexpr std::size_t size() const noexcept {
		return bits_.size();
	}

	/**
	 * @brief Get the number of bits set.
	 *
	 * @return the number of bits set.
	 */
	[[nodiscard]] constexpr std::size_t count() const noexcept {
		return bits_.count();
	}

	/**
	 * @brief Get a modifyable reference to the specified bit.
	 *
	 * @param e - The chosen bit.
	 * @return a modifyable reference to the underlying bit.
	 */
	template <EnumT e>
	constexpr AsBits::reference operator[]() noexcept {
		return bits_[EnumValue<e>()];
	}

	/**
	 * @brief Get a modifyable reference to the specified bit.
	 *
	 * @param e - The chosen bit.
	 * @return a modifyable reference to the underlying bit.
	 */
	template <EnumT e>
	constexpr bool operator[]() const noexcept {
		return bits_[EnumValue<e>()];
	}

	/**
	 * @brief Perform the &= operation on this flag.
	 *
	 * @param other - Another flag.
	 * @return a reference to this flag.
	 */
	constexpr EnumFlag& operator&=(const EnumFlag<EnumT>& other) noexcept {
		bits_ &= other.bits_;
		return *this;
	}

	constexpr EnumFlag& operator&=(auto e) noexcept {
		constexpr auto is_set = (*this)[e];
		reset();
		(*this)[e] = is_set;
		return *this;
	}

	constexpr EnumFlag& operator|=(const EnumFlag<EnumT>& other) noexcept {
		bits_ |= other.bits_;
		return *this;
	}

	constexpr EnumFlag& operator|=(EnumT e) noexcept {
		auto curr_val = as_bits();
		curr_val |= underlying(e);
		bits_ = curr_val;
		return *this;
	}

	constexpr EnumFlag& operator^=(const EnumFlag<EnumT>& other) noexcept {
		bits_ ^= other.bits_;
		return *this;
	}

	constexpr EnumFlag& operator^=(EnumT other) noexcept {
		auto curr_val = as_bits();
		curr_val ^= underlying(other);
		bits_ = curr_val;

		return *this;
	}

	constexpr EnumFlag operator~() const noexcept {
		return EnumFlag<EnumT>(~bits_);
	}

	constexpr EnumFlag operator<<(std::size_t pos) const noexcept {
		return EnumFlag<EnumT>(bits_ << pos);
	}

	constexpr EnumFlag operator>>(std::size_t pos) const noexcept {
		return EnumFlag<EnumT>(bits_ >> pos);
	}

	constexpr EnumFlag& operator<<=(std::size_t pos) noexcept {
		bits_ <<= pos;
		return *this;
	}

	constexpr EnumFlag& operator>>=(std::size_t pos) noexcept {
		bits_ >>= pos;
		return *this;
	}

	constexpr auto to_ulong() const { return bits_.to_ulong(); }
	constexpr auto to_ullong() const { return bits_.to_ullong(); }

	constexpr AsBits as_bits() const { return AsBits{bits_}; }

	template <typename E>
	friend std::ostream& operator<<(std::ostream& os, const EnumFlag<E>& f);

   private:
	AsBits bits_;
};

template <typename E>
constexpr EnumFlag<E> operator&(const EnumFlag<E>& lhs,
                                const EnumFlag<E>& rhs) noexcept {
	EnumFlag<E> res(lhs);
	res &= rhs;
	return res;
}

template <typename E>
constexpr EnumFlag<E> operator&(const EnumFlag<E>& lhs, auto rhs) noexcept {
	EnumFlag<E> res{};
	res[rhs] = true;
	res &= lhs;
	return res;
}

template <typename E>
constexpr EnumFlag<E> operator&(auto lhs, const EnumFlag<E>& rhs) noexcept {
	EnumFlag<E> res{};
	res[lhs] = true;
	res &= rhs;
	return res;
}

template <typename E>
constexpr EnumFlag<E> operator|(const EnumFlag<E>& lhs,
                                const EnumFlag<E>& rhs) noexcept {
	EnumFlag<E> res(lhs);
	res |= rhs;
	return res;
}

template <typename E>
constexpr EnumFlag<E> operator|(const EnumFlag<E>& lhs, auto rhs) noexcept {
	EnumFlag<E> res{lhs};
	res[rhs] = true;
	return res;
}

template <typename E>
constexpr EnumFlag<E> operator|(auto lhs, const EnumFlag<E>& rhs) noexcept {
	EnumFlag<E> res{rhs};
	res[lhs] = true;
	return res;
}

template <typename E>
constexpr EnumFlag<E> operator^(const EnumFlag<E>& lhs,
                                const EnumFlag<E>& rhs) noexcept {
	EnumFlag<E> res(lhs);
	res ^= rhs;
	return res;
}

template <typename E>
constexpr EnumFlag<E> operator^(const EnumFlag<E>& lhs, auto rhs) noexcept {
	EnumFlag<E> res(lhs);
	res[rhs] ^= true;
	return res;
}

template <typename E>
constexpr EnumFlag<E> operator^(auto lhs, const EnumFlag<E>& rhs) noexcept {
	EnumFlag<E> res(rhs);
	res[lhs] ^= true;
	return res;
}

template <typename E>
std::ostream& operator<<(std::ostream& os, const EnumFlag<E>& f) {
	os << f.bits_;
	return os;
}
};  // namespace enum_utils