#pragma once

#include "enum_utils.hpp"
#include "registers.hpp"

using _eutils = enum_utils;
using _regs = registers;

struct Spi {
   private:
	// SPI control register
	const _regs::Reg<uint8_t, _regs::Policy::ReadWriteable> SPCR{0x4C};
	// SPI status register
	const _regs::Reg<uint8_t, _regs::Policy::ReadWriteable> SPSR{0x4D};
	// SPI data register
	const _regs::Reg<uint8_t, _regs::Policy::ReadWriteable> SPDR(0x4E);

   public:
	enum class SPI_status { SPI2X, WCOL = 6u, SPIF, LAST };
	using SPI_status_flag = _eutils::EnumFlag<SPI_status>;

	constexpr SPI_status_flag status() const noexcept {
		return SPI_status_flag{SPSR.read()};
	}

	template <SPI_status status>
	constexpr bool status() const noexcept {
		auto read_val = SPSR.read<_regs::StartingBitFromEnum<status>()>();
		return read_val > 0u;
	}

	template <bool value>
	constexpr auto& setSPI2X(bool val) const noexcept {
		constexpr auto bit = _regs::StartingBitFromEnum<SPI_status::SPI2X>();
		if constexpr (value) {
			SPSR.set_bit<bit>();
		} else {
			SPSR.clear_bit<bit>();
		}
		return *this;
	}

	template <bool AsBytes>
	constexpr auto data() const noexcept {
		return SPDR.read<AsBytes>();
	}

	constexpr auto& data(auto value) const noexcept {
		SPDR.write(value);
		return *this;
	}

	enum class SPI_ctrl { SPR0, SPR1, CPHA, CPOL, MSTR, DORD, SPE, SPIE, LAST };
	using SPI_ctrl_flag = _eutils::EnumFlag<SPI_ctrl>;

	constexpr SPI_ctrl_flag ctrl() const noexcept {
		return SPI_ctrl_flag{SPCR.read()};
	}

	template <SPI_ctrl ctrl>
	constexpr bool ctrl() const noexcept {
		auto read_val = SPCR.read<_regs::StartingBitFromEnum<ctrl>()>();
		return read_val > 0u;
	}

	template <SPI_ctrl ctrl>
	constexpr auto& setCtrl() const noexcept {
		constexpr auto bit = StartingBitFromEnum<ctrl>();
		SPCR.write<bit>();
		return *this;
	}

	constexpr auto& setCtrl(const SPI_ctrl_flag& flags) const noexcept {
		SPCR.write(flags.as_bits());
		return *this;
	}

	enum class SPI_datamode {
		SPI0,  // Sample rising, setup falling
		SPI1,  // Setup rising, sample falling
		SPI2,  // Sample falling, Setup rising
		SPI3,  // Setup falling, Sample rising
		LAST
	};

	constexpr SPI_datamode dataMode() const noexcept {
		return ctrl_to_mode(ctrl());
	}

	template <SPI_datamode mode>
	    requires(mode != SPI_datamode::LAST)
	constexpr auto& setDataMode() const noexcept {
		using SPI_datamode_arr = _eutils::EnumArray<SPI_datamode, 1u>;
		constexpr auto num_bits = _regs::NumBits<SPI_datamode_arr::BIT_WIDTH>{};
		constexpr auto starting_bit =
		    _eutils::StartingBitFromEnum(SPI_ctrl::CPHA);
		constexpr auto new_mode = mode_to_ctrl<mode>();
		SPCR.write<num_bits, starting_bit>(new_mode);
		return *this;
	}

	enum class SPI_mode { SLAVE, MASTER };

	constexpr SPI_mode mode() const noexcept {
		auto read_val = ctrl();
		if (read_val[SPI_ctrl::MSTR]) {
			return SPI_mode::MASTER;
		}
		return SPI_mode::SLAVE;
	}

	template <SPI_mode mode>
	constexpr auto& setMode() const noexcept {
		constexpr auto bit = _regs::StartingBitFromEnum<SPI_ctrl::MSTR>();
		if constexpr (mode == SPI_mode::SLAVE) {
			SPCR.clear_bit<bit>();
		} else {
			SPCR.set_bit<bit>();
		}
		return *this;
	}

	constexpr bool irq_enabled() const noexcept {
		constexpr auto bit = _eutils::StartingBitFromEnum<SPI_ctrl::SPIE>();

		auto read_val = SPCR.read<bit>();
		return read_val != 0;
	}

	template <bool value>
	constexpr auto& set_spi_irq() const noexcept {
		constexpr auto bit = _eutils::StartingBitFromEnum<SPI_ctrl::SPIE>();

		if constexpr (value) {
			SPCR.set_bit<bit>();
		} else {
			SPCR.clear_bit<bit>();
		}
		return *this;
	}

	enum class DataTXOrder { MSB_first, LSB_first };

	constexpr DataTXOrder dataOrder() const noexcept {
		constexpr auto bit = _eutils::StartingBitFromEnum<SPI_ctrl::SPIE>();
		auto read_val = SPCR.read<bit>();
		if (read_val == 0u) {
			return DataTXOrder::MSB_first;
		}
		return DataTXOrder::LSB_first;
	}

	template <DataTXOrder order>
	constexpr auto& setDataOrder() const noexcept {
		constexpr auto bit = _eutils::StartingBitFromEnum<SPI_ctrl::SPIE>();
		if constexpr (order == MSB_first) {
			SPCR.set_bit<bit>();
		} else {
			SPCR.clear_bit<bit>();
		}
		return *this;
	}

	enum class BaseFrequency { FOSC_4, FOSC_16, FOSC_64, FOSC_128, LAST };
	using BaseFreqs = _eutils::EnumArray<BaseFrequency,
	                                     SPCR.size() * 2 / BaseFrequency::LAST>;

	constexpr BaseFrequency base_frequency() const noexcept {
		return BaseFreqs{ctrl().as_bits()}[0];
	}

	template <BaseFrequency freq>
	constexpr set_base_frequency() const noexcept {
		constexpr auto BaseFreqs val
	}

   private:
	template <SPI_datamode mode>
	    requires(mode != SPI_datamode::LAST)
	static consteval SPI_ctrl_flag dataMode_to_ctrl() noexcept {
		auto ret = SPI_ctrl_flag{}.reset();
		if constexpr (mode == SPI_datamode::SPI1 ||
		              mode == SPI_datamode::SPI3) {
			ret[SPI_ctrl::CPHA] = true;
		}
		if constexpr (mode == SPI_datamode::SPI2 ||
		              mode == SPI_datamode::SPI3) {
			ret[SPI_ctrl::CPOL] = true;
		}
		return ret;
	}

	static constexpr SPI_datamode ctrl_to_dataMode(
	    const SPI_ctrl_flag& flags) noexcept {
		auto cpha_set = flags[SPI_ctrl::CPHA];
		auto cpol_set = flags[SPI_ctrl::CPOL];

		if (!cpol_set) {
			if (!cpha_set) {
				return SPI_datamode::SPI0;
			}

			return SPI_datamode::SPI1;
		} else {
			if (!chpa_set) {
				return SPI_datamode::SPI2;
			}

			return SPI_datamode::SPI3;
		}
	}
};