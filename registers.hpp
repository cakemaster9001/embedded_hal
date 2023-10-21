
#pragma once

#include <bitset>
#include <concepts>
#include <cstdint>

#include "enum_utils.hpp"

namespace registers {
enum class Policy {
	Readable = 1,
	Writable = 2,
	ReadWritable = Readable | Writable
};

template <Policy P>
concept Readable = (P == Policy::Readable) || (P == Policy::ReadWritable);

template <Policy P>
concept Writable = (P == Policy::Writable) || (P == Policy::ReadWritable);

using BitMaskType = unsigned long long;

template <BitMaskType v>
struct NumBits : public std::integral_constant<BitMaskType, v> {
	static_assert(v > 0ull, "No bits selected");
};

template <BitMaskType v>
struct StartingBit : public std::integral_constant<BitMaskType, v> {
	static_assert(v < sizeof(v) * 8u, "Starting bit too great");
};

/**
 * @brief This should be some kind of constructor i feel.
 */
template <enum_utils::Enum E, E e>
static constexpr auto StartingBitFromEnum() {
	return StartingBit<enum_utils::underlying(e)>{};
}

template <BitMaskType v>
struct BitMask : std::integral_constant<BitMaskType, v> {
	static_assert(v > BitMaskType{0ull}, "Empty bitmask");

	BitMask() = default;

	template <std::size_t N>
	constexpr explicit BitMask(std::bitset<N> bitset) noexcept
	    : BitMask{bitset.to_ullong()} {}

	constexpr BitMask operator~() const noexcept { return BitMask{~v}; }
};

/*
    This should be a constructor of some form i feel.
*/
template <NumBits num_bits, StartingBit begin_bit>
constexpr auto BitMaskFromStartEnd() noexcept {
	static_assert(num_bits.value >= BitMaskType{1ull}, "No bits selected");
	static_assert(begin_bit.value < sizeof(BitMaskType) * 8u,
	              "Mask type too small");

	constexpr BitMaskType mask_value{((1u << num_bits.value) - 1u)
	                                 << begin_bit.value};
	return BitMask<mask_value>();
}

template <typename RegType, Policy P>
class Reg {
	using reg_type = volatile RegType*;
	using const_reg_type = const volatile RegType*;

   public:
	static consteval std::size_t num_bits() noexcept {
		return sizeof(RegType) * 8u;
	}
	using RegAsBits = std::bitset<Reg<RegType, P>::num_bits()>;

	consteval std::size_t size() const noexcept {
		return Reg<RegType, P>::num_bits();
	}

	/**
	 * @brief Construct a Reg from an address.
	 *
	 * @param addr - The address of the register as a uintptr_t.
	 */
	constexpr explicit Reg(uintptr_t addr) noexcept : addr_{addr} {}

	/**
	 * @brief Construct a Reg from an address.
	 *
	 * @param addr - The address of the register as a volatile void*.
	 */
	constexpr explicit Reg(volatile void* addr) noexcept
	    : addr_{reinterpret_cast<uintptr_t>(addr)} {}

	/**
	 * @brief Read the value of the register.
	 *
	 * @tparam AsBits - Specify the return type as a bitset.
	 * @return the value of the register as a bitset if AsBits is true, RegType
	 * otherwise.
	 */
	template <bool AsBits = false>
	[[nodiscard]] constexpr auto read() const noexcept
	    requires Readable<P>
	{
		if constexpr (AsBits == true) {
			return RegAsBits{read<false>()};
		} else {
			return *reinterpret_cast<const_reg_type>(addr_);
		}
	}

	/**
	 * @brief Read the value of the register and apply the specified bitmask to
	 * the read value.
	 *
	 * @tparam mask - The bitmask applied to the read value of the register.
	 * @tparam AsBits - Specify the return type as a bitset.
	 * @return the value of the register as a bitset if AsBits is true, RegType
	 * otherwise.
	 */
	template <BitMask mask, bool AsBits = false>
	[[nodiscard]] constexpr auto read() const noexcept
	    requires Readable<P>
	{
		static_assert(RegAsBits{}.size() >= std::bit_width(mask.value),
		              "Length of mask too great");

		auto value = read<AsBits>();
		value &= mask.value;
		return value;
	}

	/**
	 * @brief Construct a consecutive bitmask of num_bits length starting at
	 * begin_bit and apply it to the value read from the register.
	 *
	 * @tparam num_bits - The number of bits in the bitmask.
	 * @tparam begin_bit - The beginning bit of the bitmask.
	 * @tparam AsBits - Specify the return type as a bitset.
	 * @return the value of the register as a bitset if AsBits is true, RegType
	 * otherwise.
	 */
	template <NumBits num_bits, StartingBit begin_bit, bool AsBits = false>
	[[nodiscard]] constexpr auto read() const noexcept
	    requires Readable<P>
	{
		static_assert(RegAsBits{}.size() > begin_bit.value,
		              "Begin bit not in bit range");
		constexpr auto bitmask = BitMaskFromStartEnd<num_bits, begin_bit>();
		return read<bitmask, AsBits>();
	}

	/**
	 * @brief Read a single bit.
	 *
	 * @tparam begin_bit - The bit to read.
	 * @tparam AsBits - Specify the return type as a bitset.
	 * @return the value of the bit as a bitset if AsBits is true, RegType
	 * otherwise.
	 */
	template <StartingBit begin_bit, bool AsBits = false>
	[[nodiscard]] constexpr auto read() const noexcept
	    requires Readable<P>
	{
		constexpr NumBits<1u> num_bits;
		return read<num_bits, begin_bit, AsBits>();
	}

	/**
	 * @brief Write a value to the register.
	 *
	 * @param val - The value to write into the register.
	 * @return a reference to this register.
	 */
	constexpr auto& write(auto val) const noexcept
	    requires Writable<P>
	{
		*reinterpret_cast<reg_type>(addr_) = val;
		return *this;
	}

	/**
	 * @brief Write a value to the register.
	 *
	 * @param val - The value to write into the register.
	 * @return a reference to this register.
	 */
	constexpr auto& write(const RegAsBits& val) const noexcept
	    requires Writable<P>
	{
		*reinterpret_cast<reg_type>(addr_) = val.to_ullong();
		return *this;
	}

	/**
	 * @brief Write a value anded with a bitmask to the register.
	 *
	 * @tparam mask - The bitmask to apply to the specified value.
	 * @param val - The value to write into the register.
	 * @return a reference to this register.
	 */
	template <BitMask mask>
	constexpr auto& write(auto val) const noexcept
	    requires Writable<P>
	{
		static_assert(RegAsBits{}.size() >= std::bit_width(mask.mask),
		              "Length of mask too great");

		return write(val & mask.mask);
	}

	/**
	 * @brief Construct a consecutive bitmask of num_bits length starting at
	 * begin_bit, apply the & operator to the specified value and write it to
	 * the register.
	 *
	 * @tparam num_bits - The number of bits in the bitmask.
	 * @tparam begin_bit - The starting bit of the bitmask.
	 * @param val - The value to write into the register.
	 * @return a reference to this register.
	 */
	template <NumBits num_bits, StartingBit begin_bit>
	constexpr auto& write(auto val) const noexcept
	    requires Writable<P>
	{
		static_assert(RegAsBits{}.size() > begin_bit.bit,
		              "Begin bit not in bit range");

		return write<createBitMask(num_bits, begin_bit)>(val);
	}

	/**
	 * @brief Construct a bitmask with a single bit set at starting_bit, apply
	 * the & operator to the specified value and write it to the register.
	 *
	 * @tparam begin_bit - The starting bit to write.
	 * @param val - The value to write into the register.
	 * @return a reference to this register.
	 */
	template <StartingBit begin_bit>
	constexpr auto& write(auto val) const noexcept
	    requires Writable<P>
	{
		return write<NumBits<1u>{}, begin_bit>(val);
	}

	/**
	 * @brief Read the current value of the register and then overwrite it.
	 *
	 * @tparam AsBits - Specify the return type as a bitset.
	 * @param val - The new value of the register.
	 * @return the value read from the register as a bitset if AsBits is true,
	 * RegType otherwise.
	 */
	template <bool AsBits = false>
	constexpr auto readWrite(auto val) const noexcept
	    requires Writable<P> && Readable<P>
	{
		auto read_val = read<AsBits>();
		write(val);
		return read_val;
	}

	/**
	 * @brief Modify the bits in the register specified by the bitmask to the
	 * specified value.
	 *
	 * @tparam mask - The bits to modify.
	 * @tparam AsBits - Specify the return type as a bitset.
	 * @param val - The value to write into the register. This value is anded
	 * with the generated bitmask.
	 * @return the new value of the register as a bitset if AsBits is true,
	 * RegType otherwise.
	 */
	template <BitMask mask, bool AsBits = false>
	constexpr auto readModifyWrite(auto val) const noexcept
	    requires Writable<P> && Readable<P>
	{
		static_assert(RegAsBits{}.size() >= std::bit_width(mask.mask),
		              "Length of mask too great");

		auto read_val = read<~mask, AsBits>();
		auto masked_val = val & mask.mask;
		auto write_val = masked_val | read_val;
		write(write_val);

		return write_val;
	}

	/**
	 * @brief Construct a consecutive bitmask of length num_bits starting at
	 * begin_bit and modify the bits specified by that bitmask.
	 *
	 * @tparam num_bits - The number of bits to modify.
	 * @tparam begin_bit - The stargin bit of the bitmask.
	 * @tparam AsBits - Specify the return type as a bitset.
	 * @param val - The value to write into the register. This value is anded
	 * with the generated bitmask.
	 * @return the new value of the register as a bitset if AsBits is true,
	 * RegType otherwise.
	 */
	template <NumBits num_bits, StartingBit begin_bit, bool AsBits = false>
	constexpr auto readModifyWrite(auto val) const noexcept
	    requires Writable<P> && Readable<P>
	{
		static_assert(RegAsBits{}.size() > begin_bit.bit,
		              "Begin bit not in bit range");

		return readModifyWrite<createBitMask(num_bits, begin_bit), AsBits>(val);
	}

	/**
	 * @brief Set all bits in the register.
	 *
	 * @return a reference to this register.
	 */
	constexpr auto& set_bits() const noexcept
	    requires Writable<P>
	{
		auto all_set = RegAsBits{}.set();
		write(all_set);
		return *this;
	}

	/**
	 * @brief Set the specified bits in the register.
	 *
	 * @tparam mask - The bits to set.
	 * @return a reference to this register.
	 */
	template <BitMask mask>
	constexpr auto& set_bits() const noexcept
	    requires Writable<P>
	{
		static_assert(RegAsBits{}.size() >= std::bit_width(mask.value),
		              "Length of mask too great");

		/*
		    This might not be the correct contract... See flip_bits() for
		   comment.
		*/
		*reinterpret_cast<reg_type>(addr_) |= mask.value;
		return *this;
	}

	/**
	 * @brief Create a consecutive bitmask of num_bits length starting from
	 * begin_bit and set those bits.
	 *
	 * @tparam num_bits - The number of bits to set.
	 * @tparam begin_bit - The starting bit of the bitmask.
	 * @return a reference to this register.
	 */
	template <NumBits num_bits, StartingBit begin_bit>
	constexpr auto& set_bits() const noexcept
	    requires Writable<P>
	{
		static_assert(RegAsBits{}.size() > begin_bit.value,
		              "Begin bit not in bit range");
		return set_bits<BitMaskFromStartEnd<num_bits, begin_bit>()>();
	}

	/**
	 * @brief Set the bits specified by a bitset.
	 *
	 * @param mask - The bits to set.
	 * @return a reference to this register.
	 */
	constexpr auto& set_bits(const RegAsBits& mask) const noexcept
	    requires Writable<P>
	{
		*reinterpret_cast<reg_type>(addr_) |= mask.to_ullong();
		return *this;
	}

	/**
	 * @brief Set a single bit in the register.
	 *
	 * @tparam begin_bit - The bit to set.
	 * @return a reference to this register.
	 */
	template <StartingBit begin_bit>
	constexpr auto& set_bit() const noexcept
	    requires Writable<P>
	{
		return set_bits<NumBits<1u>{}, begin_bit>();
	}

	/**
	 * @brief Clear all bits in the register.
	 *
	 * @return a reference to this register.
	 */
	constexpr auto& clear_bits() const noexcept
	    requires Writable<P>
	{
		write(0u);
		return *this;
	}

	/**
	 * @brief Clear the specified bits in the register.
	 *
	 * @tparam mask - The bits to clear.
	 * @return a reference to this register.
	 */
	template <BitMask mask>
	constexpr auto& clear_bits() const noexcept
	    requires Writable<P>
	{
		/*
		    This might not be the correct contract... See flip_bits() for
		   comment.
		*/
		*reinterpret_cast<reg_type>(addr_) &= ~(mask.value);
		return *this;
	}

	/**
	 * @brief Create a consecutive bitmask of num_bits length starting from
	 * begin_bit and clear those bits.
	 *
	 * @tparam num_bits - The number of bits to clear.
	 * @tparam begin_bit - The starting bit of the bitmask.
	 * @return a reference to this register.
	 */
	template <NumBits num_bits, StartingBit begin_bit>
	constexpr auto& clear_bits() const noexcept
	    requires Writable<P>
	{
		static_assert(RegAsBits{}.size() > begin_bit.value,
		              "Begin bit not in bit range");
		constexpr auto bitmask = BitMaskFromStartEnd<num_bits, begin_bit>();
		return clear_bits<bitmask>();
	}

	/**
	 * @brief Clear the bits specified by a bitset.
	 *
	 * @param mask - The bits to clear.
	 * @return a reference to this register.
	 */
	constexpr auto& clear_bits(const RegAsBits& mask) const noexcept
	    requires Writable<P>
	{
		*reinterpret_cast<reg_type>(addr_) &= mask.to_ullong();
		return *this;
	}

	/**
	 * @brief Clear a single bit in the register.
	 *
	 * @tparam begin_bit - The bit to clear.
	 * @return a reference to this register.
	 */
	template <StartingBit begin_bit>
	constexpr auto& clear_bit() const noexcept
	    requires Writable<P>
	{
		return clear_bits<NumBits<1u>{}, begin_bit>();
	}

	/**
	 * @brief Flip all bits in the register.
	 *
	 * @return a reference to this register.
	 */
	constexpr auto& flip_bits() const noexcept
	    requires Readable<P> && Writable<P>
	{
		constexpr auto start = StartingBit<0ull>();
		return flip_bits<NumBits{size()}, start>();
	}

	/**
	 * @brief Flip the specified bits in the register.
	 *
	 * @tparam mask - The bits to flip.
	 * @return a reference to this register.
	 */
	template <BitMask mask, bool AsBits>
	constexpr auto& flip_bits() const noexcept
	    requires Readable<P> && Writable<P>
	{
		// this could also have been implemented as an xor on the register
		// itself... But that might not be desired - i think? Consider changing
		// this later.
		auto read_val = read<AsBits>();
		read_val ^= mask.mask;
		write(read_val);
		return read_val;
	}

	/**
	 * @brief Create a consecutive bitmask of num_bits length starting from
	 * begin_bit and flip those bits.
	 *
	 * @tparam num_bits - The number of bits to flip.
	 * @tparam begin_bit - The starting bit of the bitmask.
	 * @return a reference to this register.
	 */
	template <NumBits num_bits, StartingBit begin_bit>
	constexpr auto& flip_bits() const noexcept
	    requires Readable<P> && Writable<P>
	{
		static_assert(RegAsBits{}.size() > begin_bit.bit,
		              "Begin bit not in bit range");
		return flip_bits<createBitMask(num_bits, begin_bit)>();
	}

	/**
	 * @brief Flip a single bit in the register.
	 *
	 * @tparam begin_bit - The bit to flip.
	 * @return a reference to this register.
	 */
	template <StartingBit begin_bit>
	constexpr auto& flip_bit() const noexcept
	    requires Readable<P> && Writable<P>
	{
		return flip_bits<NumBits<1u>{}, begin_bit>();
	}

   private:
	const uintptr_t addr_;
};
};  // namespace registers