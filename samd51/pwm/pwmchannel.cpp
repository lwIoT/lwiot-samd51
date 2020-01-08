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
		PwmChannel::PwmChannel(PwmTimer &timer, uint8_t id) : timer_(timer), id_(id)
		{
		}

		void PwmChannel::enable()
		{
			while(this->timer_->tcc_->SYNCBUSY.bit.CC0 || this->timer_->tcc_->SYNCBUSY.bit.CC1)
				;

			auto fraction = static_cast<uint16_t>(this->timer_->top_ * this->duty());
			this->timer_->tcc_->CC[this->id_].reg = fraction;

			while(this->timer_->tcc_->SYNCBUSY.bit.CC0 || this->timer_->tcc_->SYNCBUSY.bit.CC1)
				;
		}

		void PwmChannel::disable()
		{

		}

		void PwmChannel::reload()
		{

		}

		void PwmChannel::update(int freq)
		{

		}
	}
}
