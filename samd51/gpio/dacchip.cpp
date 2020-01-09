/*
 * DAC chip implementation.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <lwiot.h>

#include <lwiot/samd51/dacchip.h>
#include <lwiot/samd51/adcchip.h>
#include <lwiot/samd51/gpio.h>
#include <component/dac.h>

namespace lwiot
{
	namespace samd51
	{
		DacChip::DacChip() : lwiot::DacChip(2, 12, 3300)
		{
		}

		void DacChip::enable(int pin)
		{
			int channel = pin;

			pin = adc0_pin_map[channel];
			pin_set_peripheral(pin, EPioType::PIO_ANALOG);

			while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
			DAC->CTRLA.bit.ENABLE = 0;

			while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
			DAC->DACCTRL[channel].bit.ENABLE = 1;

			while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
			DAC->CTRLA.bit.ENABLE = 1;

			if(channel == 0) {
				while ( !DAC->STATUS.bit.READY0 );

				while (DAC->SYNCBUSY.bit.DATA0);
				DAC->DATA[0].reg = 0;
			} else if(channel == 1){
				while ( !DAC->STATUS.bit.READY1 );

				while (DAC->SYNCBUSY.bit.DATA1);
				DAC->DATA[1].reg = 0;
			}

		}

		void DacChip::write(int pin, const size_t &voltage)
		{
			int channel = pin;

			while(!DAC->DACCTRL[channel].bit.ENABLE);

			if(channel == 0){

				while ( !DAC->STATUS.bit.READY0 );

				while (DAC->SYNCBUSY.bit.DATA0);
				DAC->DATA[0].reg = this->map(voltage);
			}
			else if(channel == 1){
				while ( !DAC->STATUS.bit.READY1 );

				while (DAC->SYNCBUSY.bit.DATA1);
				DAC->DATA[1].reg = this->map(voltage);
			}
		}
		
		void DacChip::disable(int pin)
		{
			int channel = pin;

			while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
			DAC->CTRLA.bit.ENABLE = 0;

			while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
			DAC->DACCTRL[channel].bit.ENABLE = 0;

			while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
			DAC->CTRLA.bit.ENABLE = 1;

			while(DAC->SYNCBUSY.bit.ENABLE);
		}
	}
}

static lwiot::samd51::DacChip samd51dac;
lwiot::DacChip& dac = samd51dac;
