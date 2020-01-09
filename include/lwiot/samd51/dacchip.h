/*
 * DAC chip definition.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#pragma once

#include <hpl_gpio.h>

#include <lwiot/io/dacchip.h>
#include <lwiot/io/gpiopin.h>

namespace lwiot
{
	namespace samd51
	{
		class DacChip : public lwiot::DacChip {
		public:
			explicit DacChip();
			void enable(int pin) override ;
			void disable(int pin) override ;

			void write(int pin, const size_t &voltage) override;
		};
	}
}
