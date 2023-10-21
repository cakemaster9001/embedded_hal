
#include "gpio.hpp"
#include "gpio_test.hpp"

using namespace gpio;

void testGPIO() {
	constexpr auto port = GpioController::PORTA;

	// Will cause seg fauld :)
	port.configure<Gpio::PinConfig::Input,
	               Gpio::PinChangeStrategy::ModeFirst>();
}