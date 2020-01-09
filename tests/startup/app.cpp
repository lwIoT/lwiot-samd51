/*
 * lwIoT application.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include "app.h"

#include <lwiot.h>
#include <sam.h>
#include <hal_gpio.h>

#include <lwiot/samd51/gpio.h>
#include <lwiot/samd51/gpiochip.h>
#include <lwiot/samd51/hardwarei2calgorithm.h>
#include <lwiot/samd51/adcchip.h>
#include <lwiot/samd51/dacchip.h>
#include <lwiot/samd51/pwm.h>

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
	lwiot::GpioPin led(GPIO(GPIO_PORTA, 16));
	lwiot::GpioPin irq_pin12(GPIO(GPIO_PORTA, 17));
	lwiot::GpioPin pwm_1(GPIO(GPIO_PORTB, 15));

	auto chip = lwiot::samd51::GpioChip::Instance();

	lwiot::samd51::gpio_mux_pin(sda, PINMUX_PB02D_SERCOM5_PAD0);
	lwiot::samd51::gpio_mux_pin(scl, PINMUX_PB03D_SERCOM5_PAD1);

	lwiot::samd51::HardwareI2CAlgorithm algo(scl, sda, 100000, SERCOM5);
	lwiot::samd51::AdcChip adcdev;
	lwiot::samd51::DacChip dacdev;

	lwiot::samd51::PwmTimer pwm(lwiot::samd51::PWM::PWM_0, 2000, lwiot::samd51::PWM0_CHANNELS);
	pin_set_peripheral(pwm_1.pin(), EPioType::PIO_PWM_G);
	pwm[3].setDutyCycle(2.0/3.0);
	pwm[3].setGpioPin(pwm_1);
	pwm[3].enable();

	adcdev.begin();
	dacdev.begin();

	dacdev.enable(0);

	led.output();
	irq_pin12.input();
	chip->attachIrqHandler(irq_pin12.pin(), irq_handler, lwiot::IrqEdge::IrqRising);

	chip->printPinMappings();

	while (1) {
		led.write(value);
		print_dbg("IRQ triggered: %i\n", irq_triggered);
		dacdev.write(0, 1400);
		auto adcvalue = adc.read(2);
		adcvalue = adc.toVoltage(adcvalue);
		print_dbg("ADC value: %u\n", adcvalue);

		value = !value;
		lwiot_sleep(500);
	}
}
