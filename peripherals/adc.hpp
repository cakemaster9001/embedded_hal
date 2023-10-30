#pragma once

#include <utility>

#include "enum_utils.hpp"
#include "registers.hpp"

// ADC from
// https://ww1.microchip.com/downloads/en/devicedoc/atmel-2549-8-bit-avr-microcontroller-atmega640-1280-1281-2560-2561_datasheet.pdf
// page 282
class ADC {
	const Reg<uint8_t, Policy::Readable> ADCL{0x78};
	const Reg<uint8_t, Policy::Readable> ADCH{0x79};
	const Reg<uint8_t, Policy::ReadWritable> ADCSRA{0x7A};
	const Reg<uint8_t, Policy::ReadWritable> ADCSRB{0x7B};
	const Reg<uint8_t, Policy::ReadWritable> ADMUX{0x7C};
	const Reg<uint8_t, Policy::ReadWritable> DIDR2{0x7D};
	const Reg<uint8_t, Policy::ReadWritable> DIDR0{0x7E};

   public:
	enum class ADC_num {
		ADC0,
		ADC1,
		ADC2,
		ADC3,
		ADC4,
		ADC5,
		ADC6,
		ADC7,
		ADC8,
		ADC9,
		ADC10,
		ADC11,
		ADC12,
		ADC13,
		ADC14,
		ADC15,
		LAST
	};
	using ADC_num_flag = EnumFlag<ADC_num>;

	/**
	 * @brief Enable/Disable digital inputs for the ADC.
	 *
	 * @tparam enable - Enable/Disable control.
	 * @param flags - The digital inputs to change.
	 * @return a reference to this ADC.
	 */
	template <bool enable>
	constexpr auto& digitalInput(const ADC_num_flag& flags) noexcept {
		auto first_byte = flags << 8u;
		auto bitmask1 = BitMask::createBitMask(first_byte >> 8u);

		auto second_byte = flags >> 8u;
		auto bitmask2 = BitMask::createBitMask(second_byte);

		if (first_byte.any()) {
			if constexpr (enable) {
				DIDR0.clear_bits<bitmask>();
			} else {
				DIDR0.set_bits<bitmask>();
			}
		}
		if (second_byte.any()) {
			if constexpr (enable) {
				DIDR2.clear_bits<bitmask>();
			} else {
				DIDR2.set_bits<bitmask>();
			}
		}
		return *this;
	}

	/**
	 * @brief Enable/Disable all the digital inputs.
	 *
	 * @tparam enable - if true the enable the digital input, otherwise disable
	 * the digital input.
	 * @return a reference to this ADC.
	 */
	template <bool enable>
	constexpr auto& digitalInput() noexcept {
		return digitalInput<enable>(ADC_num_flag{}.set());
	}

	enum class Prescaler {
		DIV2,
		DIV2_,
		DIV4,
		DIV8,
		DIV16,
		DIV32,
		DIV64,
		DIV128,
	};

	constexpr auto& setPrescaler(Prescaler pre) {
		constexpr auto pre_start_bit = 0u;
		constexpr auto pre_num_bits = 3u;
		auto val = ADCSRA.read<createBitMask(
		    NumBits{ADCSRA.size() - pre_num_bits},
		    StartingBit{pre_start_bit + pre_num_bits})>();
		auto pre_as_bits = std::to_underlying(pre) << pre_start_bit;
		ADCSRA.write(val | pre_as_bits);
		return *this;
	}

	enum class Reference { AREF, AVCC, Internal1V1, Internal2V56 };

	constexpr auto& setReference(Reference ref) {
		constexpr auto ref_start_bit = 6u;
		constexpr auto reg_start_bit = 0u;
		auto val = ADMUX.read<createBitMask(NumBits{ref_start_bit},
		                                    StartingBit{reg_start_bit})>();
		auto ref_as_bit_flag = std::to_underlying(ref) << ref_start_bit;

		ADMUX.write(val | ref_as_bit_flag);
		return *this;
	}

	enum class MUX {
		ADC0,
		ADC1,
		ADC2,
		ADC3,
		ADC4,
		ADC5,
		ADC6,
		ADC7,
		PDI0_NDI0_G10,
		PDI1_NDI0_G10,
		PDI0_NDI0_G200,
		PDI1_NDI0_G200,
		PDI2_NDI2_G10,
		PDI3_NDI2_G10,
		PDI2_NDI2_G200,
		PDI3_NDI2_G200,
		PDI0_NDI1_G1,
		PDI1_NDI1_G1,
		PDI2_NDI1_G1,
		PDI3_NDI1_G1,
		PDI4_NDI1_G1,
		PDI5_NDI1_G1,
		PDI6_NDI1_G1,
		PDI7_NDI1_G1,
		PDI0_NDI2_G1,
		PDI1_NDI2_G1,
		PDI2_NDI2_G1,
		PDI3_NDI2_G1,
		PDI4_NDI2_G1,
		PDI5_NDI2_G1,
		VBG1V1,
		GND,
		ADC8,
		ADC9,
		ADC10,
		ADC11,
		ADC12,
		ADC13,
		ADC14,
		ADC15,
		PDI8_NDI8_G10,
		PDI9_NDI8_G10,
		PDI8_NDI8_G200,
		PDI9_NDI8_G200,
		PDI10_NDI10_G10,
		PDI11_NDI10_G10,
		PDI10_NDI10_G200,
		PDI11_NDI10_G200,
		PDI8_NDI9_G1,
		PDI9_NDI9_G1,
		PDI10_NDI9_G1,
		PDI11_NDI9_G1,
		PDI12_NDI9_G1,
		PDI13_NDI9_G1,
		PDI14_NDI9_G1,
		PDI15_NDI9_G1,
		PDI8_NDI10_G1,
		PDI9_NDI10_G1,
		PDI10_NDI10_G1,
		PDI11_NDI10_G1,
		PDI12_NDI10_G1,
		PDI13_NDI10_G1
	};
};

int main() {
	ADC adc;

	adc.digitalInput<ADC::ADC_num::ADC0, ADC::ADC_num::ADC1>();

	return sizeof(uintptr_t);
}