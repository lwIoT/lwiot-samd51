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

		enum PwmChannelNum {
			PWM0_CHANNELS = 6,
			PWM1_CHANNELS = 4,
			PWM2_CHANNELS = 3,
			PWM3_CHANNELS = 2,
			PWM4_CHANNELS = 2
		};

		class PwmChannel;

		class PwmTimer : public lwiot::PwmTimer {
		public:
			explicit PwmTimer(PWM pwm, int freq = 100, uint8_t channels = 2);
			~PwmTimer() override;

			PwmTimer(const PwmTimer&) = delete;
			PwmTimer& operator=(const PwmTimer&) = delete;

			void setFrequency(const int &freq) override ;

		private:
			Tcc* tcc_;
			PWM pwm_;
			uint32_t top_;

			friend class PwmChannel;

			void updateFrequency(const uint32_t& freq);
			void enable();
			void disable();
			uint32_t top(uint32_t freq) const;

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