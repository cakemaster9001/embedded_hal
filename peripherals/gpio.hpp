
#pragma once

#include "bitmask.hpp"
#include "enum_utils.hpp"
#include "peripheral.hpp"
#include "registers.hpp"

namespace gpio {

namespace _eutils = enum_utils;
namespace _regs = registers;
struct Gpio {
   private:
	/* Register for toggling the PORT bits */
	const _regs::Reg<uint8_t, _regs::Policy::ReadWritable> PIN;
	/* Data direction register */
	const _regs::Reg<uint8_t, _regs::Policy::ReadWritable> DDR;
	/* Data register */
	const _regs::Reg<uint8_t, _regs::Policy::ReadWritable> PORT;

   public:
	static consteval std::size_t num_regs() noexcept { return 3u; }

	constexpr explicit Gpio(uintptr_t base_addr) noexcept
	    : PIN{base_addr}, DDR{base_addr + 1u}, PORT{base_addr + 2u} {}

	enum class Pin { PIN0, PIN1, PIN2, PIN3, PIN4, PIN5, PIN6, PIN7, LAST };

	using PinFlag = _eutils::EnumFlag<Pin>;

	enum class PinMode { Input, Output, LAST };

	using PinModeArray = _eutils::EnumArray<PinMode, decltype(DDR)::num_bits()>;

	/**
	 * @brief Set the mode of a pin.
	 *
	 * @tparam mode - The selected pin mode.
	 * @tparam pin - The pin to modify.
	 * @return a reference to this peripheral.
	 */
	template <PinMode mode, Pin pin>
	constexpr auto& setMode() const noexcept {
		constexpr auto bit = StartBit<enum_utils::EnumValue<pin>>();
		constexpr auto mode_value = enum_utils::EnumValue<mode>();
		if constexpr (mode_value == PinMode::Input) {
			DDR.clear_bit<bit>();
		} else {
			DDR.set_bit<bit>();
		}
		return *this;
	}

	/**
	 * @brief Set the mode of the pins.
	 *
	 * @tparam mode - The selected pin mode.
	 * @param pins - The pins to modify.
	 * @return a reference to this peripheral.
	 */
	template <PinMode mode>
	constexpr auto& setMode(const PinFlag& pins) const noexcept {
		constexpr auto mode_val = enum_utils::EnumValue<PinMode, mode>();
		if constexpr (mode_val == PinMode::Input) {
			DDR.clear_bits(pins.as_bits());
		} else {
			DDR.set_bits(pins.as_bits());
		}
		return *this;
	}

	/**
	 * @brief Set the specific mode of all pins.
	 *
	 * @param modes - The specific mode for each pin.
	 * @return a reference to this peripheral.
	 */
	constexpr auto& setMode(const PinModeArray& modes) const noexcept {
		DDR.write(modes.to_bits());
		return *this;
	}

	/**
	 * @brief Set the mode of all pins.
	 *
	 * @tparam mode - The new mode.
	 * @return a reference to this peripheral.
	 */
	template <PinMode mode>
	constexpr auto& setMode() const noexcept {
		PinModeArray modes;
		modes.fill(enum_utils::EnumValue<PinMode, mode>());
		return setMode(modes);
	}

	/**
	 * @brief Get the mode of the pins.
	 *
	 * @return PinModes
	 */
	constexpr PinModeArray mode() const noexcept {
		return PinModeArray{DDR.read<false>()};
	}

	/**
	 * @brief Get the mode of the specified pin.
	 *
	 * @tparam pin - The pin to read.
	 * @return constexpr PinMode
	 */
	template <Pin pin>
	constexpr auto mode() const noexcept {
		constexpr auto bit = StartBit<enum_utils::EnumValue<pin>()>{};
		auto pinVal = DDR.read<bit>();
		if (pinVal == 0u) {
			return PinMode::Input;
		}
		return PinMode::Output;
	}

	enum class PinOutput { LOW, HIGH, LAST };

	using PinOutputArray =
	    _eutils::EnumArray<PinOutput, decltype(PORT)::num_bits()>;

	/**
	 * @brief Set the output of a specific pin.
	 *
	 * @tparam output - The new output.
	 * @tparam pin - The pin to modify.
	 *
	 * @return a reference to this peripheral.
	 */
	template <PinOutput output, Pin pin>
	constexpr auto& setOutput() const noexcept {
		constexpr auto bit = StartingBitFromEnum<pin>();
		if constexpr (output == PinOutput::LOW) {
			PORT.clear_bit<bit>();
		} else {
			PORT.set_bit<bit>();
		}
		return *this;
	}

	/**
	 * @brief Set the output of multiple pins.
	 *
	 * @tparam output - The output of the pins.
	 * @param pins - The pins to modify.
	 *
	 * @return a reference to this peripheral.
	 */
	template <PinOutput output>
	constexpr auto& setOutput(const PinFlag& pins) const noexcept {
		if constexpr (output == PinOutput::LOW) {
			PORT.clear_bits(pins.as_bits());
		} else {
			PORT.set_bits(pins.as_bits());
		}
		return *this;
	}

	/**
	 * @brief Set the specific output of all the pins.
	 *
	 * @param pins - The output of each specific pin.
	 * @return a reference to this peripheral.
	 */
	constexpr auto& setOutput(const PinOutputArray& pins) const noexcept {
		PORT.write(pins.to_bits());
		return *this;
	}

	/**
	 * @brief Set the output of all the pins.
	 *
	 * @tparam output - The output the pins.
	 * @return a reference to this peripheral.
	 */
	template <PinOutput output>
	constexpr auto& setOutput() const noexcept {
		PinOutputArray outputs;
		outputs.fill(enum_utils::EnumValue<output>());
		return setOutput(outputs);
	}

	/**
	 * @brief Read the output of the pins.
	 *
	 * @return the outputs of the pins.
	 */
	constexpr PinOutputArray output() const noexcept {
		return PinOutputArray{PORT.read()};
	}

	/**
	 * @brief Read the output of a specific pin.
	 *
	 * @tparam pin - The pin to read.
	 *
	 * @return The output of the selected pin.
	 */
	template <Pin pin>
	constexpr auto output() const noexcept {
		constexpr auto bit = StartBit<enum_utils::EnumValue<pin>()>{};
		auto res = PORT.read<bit>();
		if (res == 0u) {
			return PinOutput::LOW;
		}
		return PinOutput::HIGH;
	}

	using PinValue = PinOutput;
	using PinValueArray = PinOutputArray;
	/**
	 * @brief Read the value of the pins.
	 *
	 * @return the values of the pins.
	 */
	constexpr PinValueArray read() const noexcept { return output(); }

	/**
	 * @brief Read the value of a specific pin.
	 *
	 * @tparam pin - The pin to read.
	 *
	 * @return The value of the selected pin.
	 */
	template <Pin pin>
	constexpr auto read() const noexcept {
		return output<pin>();
	}

	/**
	 * @brief Toggle the value of the specified pin.
	 *
	 * @tparam pin - The pin to toggle.
	 *
	 * @return a reference to this peripheral.
	 */
	template <Pin pin>
	constexpr auto& toggle() const noexcept {
		constexpr auto bit = StartBit<enum_utils::EnumValue<pin>()>{};
		PIN.set_bit<bit>();
		return *this;
	}

	/**
	 * @brief Toggle the specified pins.
	 *
	 * @param pins - The pins to toggle.
	 * @return a reference to this peripheral.
	 */
	constexpr auto& toggle(const PinFlag& pins) const noexcept {
		PIN.set_bits(pins.as_bits());
		return *this;
	}

	/**
	 * @brief Toggle all the pins.
	 *
	 * @return a reference to this peripheral.
	 */
	constexpr auto& toggle() const noexcept {
		PIN.set_bits();
		return *this;
	}

	enum class PinConfig { Input, InputPU, OutputLow, OutputHigh };

	enum class PinChangeStrategy { ModeFirst, OutputFirst };

   private:
	template <PinConfig cfg>
	static constexpr auto modeValFromCFG() noexcept {
		if constexpr (cfg == PinConfig::Input || cfg == PinConfig::InputPU) {
			return PinMode::Input;
		} else {
			return PinMode::Output;
		}
	}

	template <PinConfig cfg>
	static constexpr auto outputValFromCFG() noexcept {
		if constexpr (cfg == PinConfig::Input || cfg == PinConfig::OutputLow) {
			return PinOutput::LOW;
		} else {
			return PinOutput::HIGH;
		}
	}

   public:
	/**
	 * @brief Configure the value of all the pins.
	 *
	 * @tparam cfg - The configuration of the pins.
	 * @tparam strategy - The switching strategy.
	 * @return a reference to this peripheral.
	 */
	template <PinConfig cfg,
	          PinChangeStrategy strategy = PinChangeStrategy::ModeFirst>
	constexpr auto& configure() const noexcept {
		constexpr auto mode = modeValFromCFG<cfg>();
		constexpr auto output = outputValFromCFG<cfg>();
		if constexpr (strategy == PinChangeStrategy::ModeFirst) {
			setMode<mode>();
		}

		setOutput<output>();

		if constexpr (strategy != PinChangeStrategy::ModeFirst) {
			setMode<mode>();
		}
		return *this;
	}

	/**
	 * @brief Change the configuration of a single pin.
	 *
	 * @tparam cfg - The new configuration.
	 * @tparam pin - The pin to change.
	 * @tparam strategy - The switching strategy.
	 * @return a reference to this.
	 */
	template <PinConfig cfg, Pin pin,
	          PinChangeStrategy strategy = PinChangeStrategy::ModeFirst>
	constexpr auto& configure() const noexcept {
		const constexpr auto mode = modeValFromCFG<cfg>();
		const constexpr auto output = outputValFromCFG<cfg>();
		if constexpr (strategy == PinChangeStrategy::ModeFirst) {
			setMode<mode, pin>();
		}

		setOutput<output, pin>();

		if constexpr (strategy != PinChangeStrategy::ModeFirst) {
			setMode<mode, pin>();
		}
		return *this;
	}

	/**
	 * @brief Change the configuration of a multiple pins.
	 *
	 * @tparam cfg - The new configuration.
	 * @tparam strategy - The switching strategy.
	 * @param pins - The pins to change.
	 * @return a reference to this.
	 */
	template <PinConfig cfg,
	          PinChangeStrategy strategy = PinChangeStrategy::ModeFirst>
	constexpr auto& configure(const PinFlag& pins) const noexcept {
		const constexpr auto mode = modeValFromCFG<cfg>();
		const constexpr auto output = outputValFromCFG<cfg>();
		if constexpr (strategy == PinChangeStrategy::ModeFirst) {
			setMode<mode>(pins);
		}

		setOutput<output>(pins);

		if constexpr (strategy != PinChangeStrategy::ModeFirst) {
			setMode<mode>(pins);
		}
		return *this;
	}
};

struct GpioController {
	enum class Port : uintptr_t {
		A = 0x20,
		B = A + Gpio::num_regs(),
		C = B + Gpio::num_regs(),
		D = C + Gpio::num_regs(),
		E = D + Gpio::num_regs(),
		F = E + Gpio::num_regs(),
		G = F + Gpio::num_regs(),
		H = 0x100,
		J = H + Gpio::num_regs(),
		K = J + Gpio::num_regs(),
		L = K + Gpio::num_regs(),
	};

	static constexpr const Gpio PORTA{_eutils::underlying(Port::A)};
	static constexpr const Gpio PORTB{_eutils::underlying(Port::B)};
	static constexpr const Gpio PORTC{_eutils::underlying(Port::C)};
	static constexpr const Gpio PORTD{_eutils::underlying(Port::D)};
	static constexpr const Gpio PORTE{_eutils::underlying(Port::E)};
	static constexpr const Gpio PORTF{_eutils::underlying(Port::F)};
	static constexpr const Gpio PORTG{_eutils::underlying(Port::G)};
	static constexpr const Gpio PORTH{_eutils::underlying(Port::H)};
	static constexpr const Gpio PORTJ{_eutils::underlying(Port::J)};
	static constexpr const Gpio PORTK{_eutils::underlying(Port::K)};
	static constexpr const Gpio PORTL{_eutils::underlying(Port::L)};

	enum class GlobalPullupControl {
		EnablePullups,
		DisablePullups,
	};

	/**
	 * @brief Set the global pullup control.
	 *
	 * @tparam ctrl - The new value of the global pullup control.
	 * @return a reference to this controller.
	 */
	template <GlobalPullupControl ctrl>
	static constexpr GlobalPullupControl setPullUpCTRL() noexcept {
		if constexpr (ctrl == GlobalPullupControl::EnablePullups) {
			MCURCR.clear_bit<PUD_bit_pos>();
		} else {
			MCURCR.set_bit<PUD_bit_pos>();
		}
		return ctrl;
	}

	/**
	 * @brief Read the global pullup control register value.
	 *
	 * @return the global pullup control value.
	 */
	static constexpr GlobalPullupControl pullUpCTRL() noexcept {
		auto read_val = MCURCR.read<PUD_bit_pos, false>();
		if (read_val == 0u) {
			return GlobalPullupControl::EnablePullups;
		}
		return GlobalPullupControl::DisablePullups;
	}

   private:
	static const constexpr uintptr_t MCUCR_reg_addr{0x55};
	static const constexpr _regs::Reg<uint8_t, _regs::Policy::ReadWritable>
	    MCURCR{MCUCR_reg_addr};
	static const constexpr auto PUD_bit_pos = StartBit<4u>{};
};
}  // namespace gpio