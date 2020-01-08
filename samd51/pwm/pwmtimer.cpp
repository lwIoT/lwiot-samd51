/*
 * PWM timer implementation.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <sam.h>

#include <lwiot/io/pwm.h>
#include <lwiot/samd51/pwm.h>

#define TCC_GCLK_IDS { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID }

namespace lwiot
{
	namespace samd51
	{
		PwmTimer::PwmTimer(lwiot::samd51::PWM pwm, int freq, uint8_t channels) : lwiot::PwmTimer(freq), tcc_(nullptr), pwm_(pwm), top_()
		{
			Tcc* insts[] = TCC_INSTS;

			this->tcc_ = insts[pwm];

			for(uint8_t idx = 0; idx < channels; idx++) {
				auto channel = new PwmChannel(*this, idx);
				this->addChannel(*channel);
			}

			this->updateFrequency(freq);
		}

		PwmTimer::~PwmTimer()
		{
			for(auto channel : this->_channels) {
				delete channel;
			}
		}

		void PwmTimer::setFrequency(const int &freq)
		{
			lwiot::PwmTimer::setFrequency(freq);

			this->top_ = this->top(freq);
			this->tcc_->PER.reg = this->top_;

			while(this->tcc_->SYNCBUSY.bit.PER);
		}

		void PwmTimer::updateFrequency(const uint32_t &freq)
		{
			uint32_t gclk_ids[] = TCC_GCLK_IDS;

			GCLK->PCHCTRL[gclk_ids[this->pwm_]].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);

			this->tcc_->CTRLA.bit.SWRST = 1;
			while(this->tcc_->SYNCBUSY.bit.SWRST);

			this->disable();

			/* Calculate and set the prescaler */
			if(freq < 100)
				this->tcc_->CTRLA.reg = TCC_CTRLA_PRESCSYNC_GCLK | TCC_CTRLA_PRESCALER_DIV256;
			else
				this->tcc_->CTRLA.reg = TCC_CTRLA_PRESCSYNC_GCLK | TCC_CTRLA_PRESCALER_DIV64;

			this->tcc_->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
			while(this->tcc_->SYNCBUSY.bit.WAVE);

			this->setFrequency(freq);

			this->enable();
		}

		void PwmTimer::enable()
		{
			this->tcc_->CTRLA.bit.ENABLE = 1;
			while(this->tcc_->SYNCBUSY.bit.ENABLE);
		}

		void PwmTimer::disable()
		{
			this->tcc_->CTRLA.bit.ENABLE = 0;
			while(this->tcc_->SYNCBUSY.bit.ENABLE);
		}

		uint32_t PwmTimer::top(uint32_t freq) const
		{
			uint32_t top;

			if(freq < 100) {
				top = static_cast<uint32_t>((F_CPU / PRESCALER_256) / freq);
			} else  {
				top = static_cast<uint32_t>((F_CPU / PRESCALER_64) / freq);
				top -= 10; /* Calibration */
			}

			return top;
		}
	}
}
