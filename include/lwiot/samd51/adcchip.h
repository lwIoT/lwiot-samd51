/*
 * Analog chip header.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#pragma once

#include <lwiot.h>
#include <sam.h>

typedef enum
{
	AR_DEFAULT,
	AR_INTERNAL1V0,
	AR_INTERNAL1V1,
	AR_INTERNAL1V2,
	AR_INTERNAL1V25,
	AR_INTERNAL2V0,
	AR_INTERNAL2V2,
	AR_INTERNAL2V23,
	AR_INTERNAL2V4,
	AR_INTERNAL2V5,
	AR_INTERNAL1V65,
	AR_EXTERNAL
}  AnalogReference;

#define ADC_CHANNELS_NUM 15

extern uint8_t adc0_pin_map[];
extern uint8_t adc1_pin_map[];

CDECL
extern void analog_set_reference(AnalogReference mode);
CDECL_END

#include <component/adc.h>

#ifdef __cplusplus
#include <lwiot/io/adcchip.h>

namespace lwiot
{
	namespace samd51
	{
		enum class ADC {
			ADC_0,
			ADC_1
		};

		enum ADCResolution {
			ADC_8bit = 256,
			ADC_10bit = 1024,
			ADC_12bit = 4096
		};

		class AdcChip : public lwiot::AdcChip {
		public:
			explicit AdcChip(ADC = ADC::ADC_0, ADCResolution res = ADCResolution::ADC_10bit);

			~AdcChip() override = default;

			void begin() override;
			size_t read(int pin) const override;


		private:
			Adc* adc_;
			uint8_t* pin_map_;

			void setResolution(ADCResolution res);
		};
	}
}
#endif

