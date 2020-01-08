/*
 * PWM channel implementation.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <lwiot/io/pwm.h>
#include <lwiot/samd51/pwm.h>

namespace lwiot
{
	namespace samd51
	{
		PwmChannel::PwmChannel(PwmTimer &timer, uint8_t id) : lwiot::PwmChannel(), timer_(timer), id_(id)
		{
		}

		void PwmChannel::enable()
		{
			this->reload();
		}

		void PwmChannel::disable()
		{
			this->timer_->tcc_->CC[this->id_].reg = 0;
		}

		void PwmChannel::reload()
		{
			if(this->freq_cache > 0) {
				this->timer_->setFrequency(static_cast<int>(this->freq_cache));
				this->freq_cache = 0;
			}

			uint32_t msk = 1 << this->id_;

			msk <<= TCC_SYNCBUSY_CC0_Pos;

			while(this->timer_->tcc_->SYNCBUSY.reg & msk) ;

			auto fraction = static_cast<uint16_t>(this->timer_->top_ * this->duty());
			this->timer_->tcc_->CC[this->id_].reg = fraction;

			while(this->timer_->tcc_->SYNCBUSY.reg & msk) ;
		}

		void PwmChannel::update(int freq)
		{
		}
	}
}
