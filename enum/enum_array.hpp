#include <array>
#include <bitset>
#include <cstdint>
#include <iostream>

#include "enum_common.hpp"

namespace enum_utils {
template <EnumHasLast EnumT, std::size_t N>
struct EnumArray : public std::array<EnumT, N> {
	static const constexpr auto LAST_VALUE = underlying(EnumT::LAST);
	static const constexpr auto BIT_WIDTH = std::bit_width(LAST_VALUE - 1u);

   public:
	using bit_type = std::bitset<BIT_WIDTH * N>;

	/**
	 * @brief Converts the enum array into a bitset.
	 *
	 * @return the array as a bitset.
	 */
	constexpr bit_type to_bits() const noexcept {
		bit_type ret{};

		for (auto i = 0; i < N; ++i) {
			ret |= underlying((*this)[i]) << BIT_WIDTH * i;
		}

		return ret;
	}

	using std::array<EnumT, N>::array;
	constexpr explicit EnumArray(const bit_type& bits) {
		static constexpr auto bit_mask = bit_type{(1u << BIT_WIDTH) - 1};

		for (auto i = 0; i < N; ++i) {
			auto curr_offset = i * BIT_WIDTH;
			auto curr_bitmask = bit_mask << curr_offset;
			auto curr_value_as_bits = (bits & curr_bitmask) >> curr_offset;
			auto curr_value = curr_value_as_bits.to_ullong();
			if (curr_value >= LAST_VALUE) {
				// Better error handling??
				// If better handling - change to noexcept
				throw std::overflow_error("Curr value greater than LAST");
			}

			(*this)[i] = static_cast<EnumT>(curr_value);
		}
	}

	template <typename E, std::size_t Ns>
	friend std::ostream& operator<<(std::ostream& os,
	                                const EnumArray<E, Ns>& arr);
};

template <typename EnumT, std::size_t N>
std::ostream& operator<<(std::ostream& os, const EnumArray<EnumT, N>& arr) {
	os << "[";
	for (auto i = 0; i < arr.size(); ++i) {
		os << underlying(arr[i]);
		if (i != arr.size() - 1) {
			os << ", ";
		}
	}
	os << "]";
	return os;
}
};  // namespace enum_utils