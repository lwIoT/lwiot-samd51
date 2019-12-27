/*
 * I2C algorithm.
 *
 * @author Michel Megens
 * @email  dev@bietje.net
 */

#pragma once

#include <stdlib.h>
#include <lwiot.h>

#include <lwiot/types.h>
#include <lwiot/log.h>
#include <lwiot/sharedpointer.h>

#include <lwiot/io/hardwarei2calgorithm.h>
#include <lwiot/io/i2calgorithm.h>
#include <lwiot/io/gpiopin.h>
#include <lwiot/kernel/lock.h>

#include <lwiot/samd51/sercom.h>

#include <component/sercom.h>

namespace lwiot
{
	namespace samd51
	{
		class HardwareI2CAlgorithm : public lwiot::HardwareI2CAlgorithm {
		public:
			explicit HardwareI2CAlgorithm(const GpioPin& scl, const GpioPin& sda, uint32_t frequency, Sercom* coms);
			HardwareI2CAlgorithm(HardwareI2CAlgorithm&& other) noexcept;
			virtual ~HardwareI2CAlgorithm();

			HardwareI2CAlgorithm& operator=(HardwareI2CAlgorithm&& rhs) noexcept;
			void setFrequency(const uint32_t &freq) override;

		protected:
			void start(uint16_t sla, bool repeated) override;
			void stop() override;
			void write(const uint8_t *byte, bool ack) override;
			void write(const uint8_t *bytes, size_t length, bool ack) override;
			void read(uint8_t *byte, bool ack) override;
			void read(uint8_t *bytes, size_t length, bool ack) override;
			int flush() const override;
			void reset() override;

		private:
			UniquePointer<SERCOM> sercom_;
		};
	}
}
