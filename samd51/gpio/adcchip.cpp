/*
 * Analog chip implementation.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <lwiot.h>
#include <sam.h>

#include <lwiot/samd51/adcchip.h>
#include <lwiot/samd51/gpio.h>

namespace lwiot
{
	namespace samd51
	{
		AdcChip::AdcChip(lwiot::samd51::ADC adc, ADCResolution res) :
			lwiot::AdcChip(16, 3300, res), adc_(nullptr)
		{
			if(adc == ADC::ADC_0) {
				this->adc_ = ADC0;
				this->pin_map_ = adc0_pin_map;
			} else {
				this->adc_ = ADC1;
				this->pin_map_ = adc1_pin_map;
			}
		}

		void AdcChip::begin()
		{
			auto res = static_cast<ADCResolution>(this->width());
			this->setResolution(res);
		}

		size_t AdcChip::read(int pin) const
		{
			/* Pin is the analog channel number on the ADC */
			auto channel = static_cast<uint32_t>(pin);
			uint32_t value;

			pin = this->pin_map_[channel];
			pin_set_peripheral(pin, EPioType::PIO_ANALOG);

			while(this->adc_->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL);
			this->adc_->INPUTCTRL.bit.MUXPOS = channel;

			while( this->adc_->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE );
			this->adc_->CTRLA.bit.ENABLE = 0x01;

			while( this->adc_->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE );
			this->adc_->SWTRIG.bit.START = 1;
			this->adc_->INTFLAG.reg = ADC_INTFLAG_RESRDY;
			this->adc_->SWTRIG.bit.START = 1;

			while (this->adc_->INTFLAG.bit.RESRDY == 0);
			value = this->adc_->RESULT.reg;

			while( this->adc_->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE );
			this->adc_->CTRLA.bit.ENABLE = 0x00;
			while( this->adc_->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE );

			return value;
		}

		void AdcChip::setResolution(ADCResolution res)
		{
			switch(res) {
			case ADCResolution::ADC_8bit:
				this->adc_->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
				break;

			case ADCResolution::ADC_10bit:
				this->adc_->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
				break;

			case ADCResolution::ADC_12bit:
				this->adc_->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
				break;
			}

			while(this->adc_->SYNCBUSY.reg & ADC_SYNCBUSY_CTRLB);
		}
	}
}

static lwiot::samd51::AdcChip samd51adc;
lwiot::AdcChip& adc = samd51adc;
