/*
 * lwIoT application.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include "app.h"

#include <lwiot.h>
#include <sam.h>

#include <lwiot/samd51/gpio.h>
#include <lwiot/samd51/gpiochip.h>
#include <lwiot/samd51/hardwarei2calgorithm.h>

volatile int irq_triggered;

static void irq_handler()
{
	irq_triggered += 1;
}

extern "C" void app_main()
{
	_Bool value = false;
	lwiot::GpioPin sda(GPIO(GPIO_PORTB, 2));
	lwiot::GpioPin scl(GPIO(GPIO_PORTB, 3));
	lwiot::GpioPin irq_pin12(GPIO(GPIO_PORTA, 17));
	auto chip = lwiot::samd51::GpioChip::Instance();

	lwiot::samd51::gpio_mux_pin(sda, PINMUX_PB02D_SERCOM5_PAD0);
	lwiot::samd51::gpio_mux_pin(scl, PINMUX_PB03D_SERCOM5_PAD1);
	lwiot::samd51::HardwareI2CAlgorithm algo(scl, sda, 100000, SERCOM5);

	irq_pin12.input();
	chip->attachIrqHandler(irq_pin12.pin(), irq_handler, lwiot::IrqEdge::IrqRising);

	chip->printPinMappings();

	while (1) {
		gpio_set_pin_level(LED_PIN, value);
		lwiot_sleep(500);
		gpio_set_pin_level(NEO_PIN,	value);
		print_dbg("IRQ triggered: %i\n", irq_triggered);

		value = !value;
	}
}
