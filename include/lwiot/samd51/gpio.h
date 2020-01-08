/*
 * GPIO utility functions.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <lwiot.h>
#include <lwiot/samd51/gpiochip.h>

CDECL
extern void pin_set_peripheral(uint32_t pin, EPioType iotype);
CDECL_END

#ifdef CXX
#include <lwiot/io/gpiopin.h>
namespace lwiot
{
	namespace samd51
	{
		extern void gpio_mux_pin(const GpioPin& pin, uint32_t mux);
	}
}
#endif
