/*
 * GPIO chip for the SAMD51 chip.
 *
 * @author Michel Megens
 * @email  dev@bietje.net
 */

#pragma once

#include <lwiot/lwiot.h>

typedef enum {
	EXTERNAL_INT_0 = 0,
	EXTERNAL_INT_1,
	EXTERNAL_INT_2,
	EXTERNAL_INT_3,
	EXTERNAL_INT_4,
	EXTERNAL_INT_5,
	EXTERNAL_INT_6,
	EXTERNAL_INT_7,
	EXTERNAL_INT_8,
	EXTERNAL_INT_9,
	EXTERNAL_INT_10,
	EXTERNAL_INT_11,
	EXTERNAL_INT_12,
	EXTERNAL_INT_13,
	EXTERNAL_INT_14,
	EXTERNAL_INT_15,
	EXTERNAL_INT_NMI,
	EXTERNAL_NUM_INTERRUPTS,
	NOT_AN_INTERRUPT = -1,
	EXTERNAL_INT_NONE = NOT_AN_INTERRUPT,
} ExtIrqs;

#ifdef __cplusplus
extern "C" void invokeIrq(ExtIrqs num);
#else
extern void invokeIrq(ExtIrqs num);
#endif

#ifdef __cplusplus
#include <lwiot/io/gpiochip.h>
#include <lwiot/stl/referencewrapper.h>

namespace lwiot
{
	namespace samd51
	{
		static constexpr int PINS = 64;
		static constexpr bool HIGH = true;
		static constexpr bool LOW = false;

		namespace detail_gpio
		{
			typedef ::ExtIrqs ExtIrqs;

			typedef enum {
				PIO_NOT_A_PIN=-1,
				PIO_EXTINT=0,
				PIO_ANALOG,
				PIO_SERCOM,
				PIO_SERCOM_ALT,
				PIO_TIMER,
				PIO_TIMER_ALT,
				PIO_TCC_PDEC,
				PIO_COM,
				PIO_SDHC,
				PIO_I2S,
				PIO_PCC,
				PIO_GMAC,
				PIO_AC_CLK,
				PIO_CCL,
				PIO_DIGITAL,
				PIO_INPUT,
				PIO_INPUT_PULLUP,
				PIO_OUTPUT,

				PIO_PWM     = PIO_TIMER,
				PIO_PWM_ALT = PIO_TIMER_ALT,
			} EPioType ;

			struct IrqMap {
				irq_handler_t handler;
				uint32_t pin;
			};

			static constexpr int NMI_IRQ_PIN = GPIO(GPIO_PORTA, 8);
		}

		class GpioChip : public lwiot::GpioChip {
		public:
			static stl::ReferenceWrapper<GpioChip> Instance()
			{
				static GpioChip chip;
				return stl::MakeRef(chip);
			}

			virtual ~GpioChip() = default;

			void mode(int pin, const PinMode &mode) override;
			void write(int pin, bool value) override;
			bool read(int pin) const override;
			void setOpenDrain(int pin) override;
			void odWrite(int pin, bool value) override;
			void attachIrqHandler(int pin, irq_handler_t handler, IrqEdge edge) override;
			void detachIrqHandler(int pin) override;
			void invoke(ExtIrqs num);

#ifdef HAVE_DEBUG
			void printPinMappings();
#endif

		private:
			explicit GpioChip();
			int mapIrqType(const IrqEdge &edge) const;
			void init();
			void initIrqs();

			detail_gpio::IrqMap irq_map_[EXTERNAL_NUM_INTERRUPTS];
			bool initialized_;
			static constexpr uint32_t NOT_INITIALIZED = 0xFFFF;
		};
	}
}

#endif
