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
		PwmTimer::PwmTimer(lwiot::samd51::PWM pwm, int freq) : lwiot::PwmTimer(freq), tcc_(nullptr), pwm_(pwm), top_()
		{
			Tcc* insts[] = TCC_INSTS;
			
			this->tcc_ = insts[pwm];
			auto channel_1 = new PwmChannel(*this, 0);
			auto channel_2 = new PwmChannel(*this, 1);

			this->addChannel(*channel_1);
			this->addChannel(*channel_2);

			this->updateFrequency(freq);
		}

		PwmTimer::~PwmTimer()
		{
			auto a = this->_channels.at(0);
			auto b = this->_channels.at(0);

			delete a;
			delete b;
		}

		void PwmTimer::setFrequency(const int &freq)
		{
			lwiot::PwmTimer::setFrequency(freq);
		}

		void PwmTimer::updateFrequency(const uint32_t &freq)
		{
			uint32_t gclk_ids[] = TCC_GCLK_IDS;
			uint32_t top;

			GCLK->PCHCTRL[gclk_ids[this->pwm_]].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);

			this->tcc_->CTRLA.bit.SWRST = 1;
			while(this->tcc_->SYNCBUSY.bit.SWRST);

			this->tcc_->CTRLA.bit.ENABLE = 0;
			while(this->tcc_->SYNCBUSY.bit.ENABLE);

			/* Calculate and set the prescaler */
			if(freq < 100)
				this->tcc_->CTRLA.reg = TCC_CTRLA_PRESCSYNC_GCLK | TCC_CTRLA_PRESCALER_DIV256;
			else
				this->tcc_->CTRLA.reg = TCC_CTRLA_PRESCSYNC_GCLK | TCC_CTRLA_PRESCALER_DIV64;

			this->tcc_->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
			while(this->tcc_->SYNCBUSY.bit.WAVE);

			if(freq < 100) {
				top = static_cast<uint32_t>((F_CPU / PRESCALER_256) / freq);
			} else  {
				top = static_cast<uint32_t>((F_CPU / PRESCALER_64) / freq);
				top -= 10;
			}

			this->top_ = top;

//			while(this->tcc_->SYNCBUSY.bit.CC0 || this->tcc_->SYNCBUSY.bit.CC1);
//			this->tcc_->CC[0].reg = top / 2;
//			this->tcc_->CC[1].reg = top / 2;
//			while(this->tcc_->SYNCBUSY.bit.CC0 || this->tcc_->SYNCBUSY.bit.CC1);

			this->tcc_->PER.reg =  top;
			while(this->tcc_->SYNCBUSY.bit.PER);

			this->tcc_->CTRLA.bit.ENABLE = 1;
			while(this->tcc_->SYNCBUSY.bit.ENABLE);
		}
	}
}
