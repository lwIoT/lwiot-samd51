/*
 * GPIO utility functions.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <lwiot/io/gpiopin.h>
#include <hal_gpio.h>

namespace lwiot
{
	namespace samd51
	{
		void gpio_mux_pin(const GpioPin& pin, uint32_t mux)
		{
			uint32_t p = pin.pin();
			gpio_set_pin_function(p, mux);
		}
	}
}
