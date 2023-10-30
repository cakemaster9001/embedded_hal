
#pragma once

#include <bitset>

#include "bitmask.hpp"
#include "enum_utils.hpp"
#include "equality.hpp"

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

template <typename RegType, Policy P>
class Reg {
	using reg_type = volatile RegType*;
	using const_reg_type = const volatile RegType*;
	using type = Reg<RegType, P>;

   public:
	static consteval std::size_t num_bits() noexcept {
		constexpr auto max_val = std::numeric_limits<RegType>::max();
		return std::bit_width(max_val);
	}

	using RegAsBits = std::bitset<type::num_bits()>;

	consteval std::size_t size() const noexcept { return type::num_bits(); }

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
	    requires Readable<P> &&
	             GreaterThanEqual<type::num_bits(), mask.bits_in_mask()>
	[[nodiscard]] constexpr auto read() const noexcept {
		auto value = read<AsBits>();
		value &= mask.value;
		return value;
	}

	/**
	 * @brief Read a single bit.
	 *
	 * @tparam begin_bit - The bit to read.
	 * @tparam AsBits - Specify the return type as a bitset.
	 * @return the value of the bit as a bitset if AsBits is true, RegType
	 * otherwise.
	 */
	template <StartBit bit, bool AsBits = false>
	[[nodiscard]] constexpr auto read() const noexcept
	    requires Readable<P>
	{
		constexpr BitMask bitmask = SingleBitMask<bit>{};
		return read<bitmask, AsBits>();
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
		return write(val.to_ullong());
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
	    requires Writable<P> &&
	             GreaterThanEqual<type::num_bits(), mask.bits_in_mask()>
	{
		return write(val & mask.mask);
	}

	/**
	 * @brief Construct a bitmask with a single bit set at starting_bit, apply
	 * the & operator to the specified value and write it to the register.
	 *
	 * @tparam begin_bit - The starting bit to write.
	 * @param val - The value to write into the register.
	 * @return a reference to this register.
	 */
	template <StartBit bit>
	constexpr auto& write(auto val) const noexcept
	    requires Writable<P>
	{
		constexpr BitMask bitmask = SingleBitMask<bit>{};
		return write<bitmask>(val);
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
	    requires Writable<P> && Readable<P> &&
	             GreaterThanEqual<type::num_bits(), mask.bits_in_mask()>
	{
		auto read_val = read<~mask, AsBits>();
		auto masked_val = val & mask.mask;
		auto write_val = masked_val | read_val;
		write(write_val);

		return write_val;
	}

	/**
	 * @brief Modify the specified bit in the register to the specified value.
	 *
	 * @tparam bit - The bit to modify.
	 * @tparam AsBits - Specify the return type as a bitset.
	 * @param val - Set or clear the bit.
	 *
	 * @return the new value of the register as a bitset if AsBits is true,
	 * RegType otherwise.
	 */
	template <StartBit bit, bool AsBits = false>
	constexpr auto readModifyWrite(bool val) const noexcept
	    requires Writable<P> && Readable<P>
	{
		constexpr BitMask bitmask = SingleBitMask<bit>{};
		return readModifyWrite<bitmask, AsBits>(bitmask.value);
	}

	/**
	 * @brief Set the specified bits in the register.
	 *
	 * @tparam mask - The bits to set.
	 * @return a reference to this register.
	 */
	template <BitMask mask>
	constexpr auto& set_bits() const noexcept
	    requires Writable<P> &&
	             GreaterThanEqual<type::num_bits(), mask.bits_in_mask()>
	{
		/*
		    This might not be the correct contract... See flip_bits() for
		   comment.
		*/
		*reinterpret_cast<reg_type>(addr_) |= mask.value;
		return *this;
	}

	/**
	 * @brief Set all bits in the register.
	 *
	 * @return a reference to this register.
	 */
	constexpr auto& set_bits() const noexcept
	    requires Writable<P>
	{
		constexpr auto num_bits = NumBits<type::num_bits()>{};
		constexpr BitMask bitmask = ConsecutiveBitMask<num_bits>{};
		return set_bits<bitmask>();
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
	template <StartBit bit>
	constexpr auto& set_bit() const noexcept
	    requires Writable<P>
	{
		constexpr BitMask bitmask = SingleBitMask<bit>{};
		return set_bits<bitmask>();
	}

	/**
	 * @brief Clear all bits in the register.
	 *
	 * @return a reference to this register.
	 */
	constexpr auto& clear_bits() const noexcept
	    requires Writable<P>
	{
		return write(0u);
	}

	/**
	 * @brief Clear the specified bits in the register.
	 *
	 * @tparam mask - The bits to clear.
	 * @return a reference to this register.
	 */
	template <BitMask mask>
	constexpr auto& clear_bits() const noexcept
	    requires Writable<P> &&
	             GreaterThanEqual<type::num_bits(), mask.bits_in_mask()>
	{
		/*
		    This might not be the correct contract... See flip_bits() for
		   comment.
		*/
		*reinterpret_cast<reg_type>(addr_) &= ~(mask.value);
		return *this;
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
	template <StartBit bit>
	constexpr auto& clear_bit() const noexcept
	    requires Writable<P>
	{
		constexpr BitMask bitmask = SingleBitMask<bit>{};
		return clear_bits<bitmask>();
	}

	/**
	 * @brief Flip all bits in the register.
	 *
	 * @return a reference to this register.
	 */
	constexpr auto& flip_bits() const noexcept
	    requires Readable<P> && Writable<P>
	{
		constexpr auto num_bits = NumBits<type::num_bits()>{};
		constexpr BitMask bitmask = ConsecutiveBitMask<num_bits>{};
		return flip_bits<bitmask>();
	}

	/**
	 * @brief Flip the specified bits in the register.
	 *
	 * @tparam mask - The bits to flip.
	 * @return a reference to this register.
	 */
	template <BitMask mask, bool AsBits>
	constexpr auto& flip_bits() const noexcept
	    requires Readable<P> && Writable<P> &&
	             GreaterThan<type::num_bits(), mask.bits_in_mask()>
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
	 * @brief Flip a single bit in the register.
	 *
	 * @tparam begin_bit - The bit to flip.
	 * @return a reference to this register.
	 */
	template <StartBit begin_bit>
	constexpr auto& flip_bit() const noexcept
	    requires Readable<P> && Writable<P>
	{
		constexpr BitMask bitmask = SingleBitMask<begin_bit>{};
		return flip_bits<bitmask>();
	}

   private:
	const uintptr_t addr_;
};
};  // namespace registers