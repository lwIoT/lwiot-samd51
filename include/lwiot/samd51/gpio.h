/*
 * GPIO utility functions.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <lwiot/io/gpiopin.h>

namespace lwiot
{
	namespace samd51
	{
		extern void gpio_mux_pin(const GpioPin& pin, uint32_t mux);
	}
}
