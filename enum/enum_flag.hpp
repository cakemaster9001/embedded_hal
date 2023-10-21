#pragma once

#include <bitset>
#include <iostream>
#include <type_traits>
#include <utility>

#include "enum_common.hpp"

namespace enum_utils {
template <Enum EnumT>
    requires EnumHasLast<EnumT>
class EnumFlag {
   public:
	using AsBits = std::bitset<underlying(EnumT::LAST)>;

	template <EnumT... Es>
	    requires([](auto... rest) {
		    return ((rest != EnumT::LAST) && ...);
	    }(Es...))
	constexpr EnumFlag& set() noexcept {
		(bits_.set(underlying(Es), true), ...);
		return *this;
	}

	constexpr EnumFlag& set() noexcept {
		bits_.set();
		return *this;
	}

	template <EnumT... Es>
	    requires([](auto... rest) {
		    return ((rest != EnumT::LAST) && ...);
	    }(Es...))
	constexpr EnumFlag& reset() noexcept {
		(bits_.reset(underlying(Es)), ...);
		return *this;
	}

	constexpr EnumFlag& reset() noexcept {
		bits_.reset();
		return *this;
	}

	template <EnumT... Es>
	    requires([](auto... rest) {
		    return ((rest != EnumT::LAST) && ...);
	    }(Es...))
	constexpr EnumFlag& flip() noexcept {
		(bits_.flip(underlying(Es)), ...);
		return *this;
	}

	constexpr EnumFlag& flip() noexcept {
		bits_.flip();
		return *this;
	}

	[[nodiscard]] constexpr bool all() const noexcept { return bits_.all(); }

	[[nodiscard]] constexpr bool any() const noexcept { return bits_.any(); }

	[[nodiscard]] constexpr bool none() const noexcept { return bits_.none(); }

	[[nodiscard]] constexpr std::size_t size() const noexcept {
		return bits_.size();
	}

	[[nodiscard]] constexpr std::size_t count() const noexcept {
		return bits_.count();
	}

	constexpr std::bitset<underlying(EnumT::LAST)>::reference operator[](
	    EnumT e) {
		return bits_[underlying(e)];
	}

	constexpr bool operator[](EnumT e) const { return bits_[(underlying(e))]; }

	constexpr EnumFlag& operator&=(const EnumFlag<EnumT>& other) noexcept {
		bits_ &= other.bits_;
		return *this;
	}

	// Does this make sense?????....
	// Can be called with EnumT::LAST - not so good
	constexpr EnumFlag& operator&=(EnumT other) noexcept {
		auto is_set = (*this)[other];
		reset();
		(*this)[other] = is_set;
		return *this;
	}

	constexpr EnumFlag& operator|=(const EnumFlag<EnumT>& other) noexcept {
		bits_ |= other.bits_;
		return *this;
	}

	// Does this make sense?????....
	// Can be called with EnumT::LAST - not so good
	constexpr EnumFlag& operator|=(EnumT other) noexcept {
		(*this)[other] = true;
		return *this;
	}

	constexpr EnumFlag& operator^=(const EnumFlag<EnumT>& other) noexcept {
		bits_ ^= other.bits_;
		return *this;
	}

	// Does this make sense?????....
	// Can be called with EnumT::LAST - not so good
	constexpr EnumFlag& operator^=(EnumT other) noexcept {
		(*this)[other] ^= (*this)[other];
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

// Can be called with EnumT::LAST - not so good
template <typename E>
constexpr EnumFlag<E> operator&(const EnumFlag<E>& lhs, E rhs) noexcept {
	EnumFlag<E> res{};
	res[rhs] = true;
	res &= lhs;
	return res;
}

// Can be called with EnumT::LAST - not so good
template <typename E>
constexpr EnumFlag<E> operator&(E lhs, const EnumFlag<E>& rhs) noexcept {
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

// Can be called with EnumT::LAST - not so good
template <typename E>
constexpr EnumFlag<E> operator|(const EnumFlag<E>& lhs, E rhs) noexcept {
	EnumFlag<E> res{lhs};
	res[rhs] = true;
	return res;
}

// Can be called with EnumT::LAST - not so good
template <typename E>
constexpr EnumFlag<E> operator|(E lhs, const EnumFlag<E>& rhs) noexcept {
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

// Can be called with EnumT::LAST - not so good
template <typename E>
constexpr EnumFlag<E> operator^(const EnumFlag<E>& lhs, E rhs) noexcept {
	EnumFlag<E> res(lhs);
	res[rhs] ^= true;
	return res;
}

// Can be called with EnumT::LAST - not so good
template <typename E>
constexpr EnumFlag<E> operator^(E lhs, const EnumFlag<E>& rhs) noexcept {
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