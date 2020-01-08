/*
 * PWM timer definition.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#pragma once

#include <lwiot.h>
#include <sam.h>

#include <lwiot/io/pwm.h>
#include <lwiot/stl/referencewrapper.h>

namespace lwiot
{
	namespace samd51
	{
		enum PWM {
			PWM_0,
			PWM_1,
			PWM_2,
			PWM_3,
			PWM_4,
			PWM_TIMER_COUNT
		};

		class PwmChannel;

		class PwmTimer : public lwiot::PwmTimer {
		public:
			explicit PwmTimer(PWM pwm, int freq = 100);
			~PwmTimer() override;

			void setFrequency(const int &freq) override ;

		private:
			Tcc* tcc_;
			PWM pwm_;
			uint32_t top_;

			friend class PwmChannel;

			void updateFrequency(const uint32_t& freq);
			static constexpr int PRESCALER_64 = 64;
			static constexpr int PRESCALER_256 = 256;
			static constexpr uint32_t BASE_FREQ = 48000000;
		};

		class PwmChannel : public lwiot::PwmChannel {
		public:
			explicit PwmChannel(PwmTimer& timer, uint8_t id);

			void enable() override;
			void disable() override;
			void reload() override;

		protected:
			void update(int freq) override;

		private:
			stl::ReferenceWrapper<PwmTimer> timer_;
			uint8_t id_;
		};
	}
}