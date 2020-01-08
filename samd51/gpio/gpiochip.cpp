/*
 * GPIO implementation for the ESP32.
 *
 * @author Michel Megens
 * @email  dev@bietje.net
 */

#include <lwiot.h>
#include <hal_gpio.h>

#include <lwiot/error.h>
#include <lwiot/log.h>

#include <lwiot/io/gpiochip.h>

#include <lwiot/samd51/gpio.h>
#include <lwiot/samd51/gpiochip.h>

namespace lwiot
{
	namespace samd51
	{
		GpioChip::GpioChip() : lwiot::GpioChip(PINS), irq_map_(), initialized_(false)
		{
			this->init();
		}

		void GpioChip::init()
		{
			for(auto& pin : this->irq_map_) {
				pin.pin = NOT_INITIALIZED;
			}
		}

		void GpioChip::initIrqs()
		{
			for(auto idx = 0U; idx < EXTERNAL_NUM_INTERRUPTS; idx++) {
				uint8_t irqn = EIC_0_IRQn + idx;

				NVIC_DisableIRQ(static_cast<IRQn_Type>(irqn));
				NVIC_ClearPendingIRQ(static_cast<IRQn_Type>(irqn));
				NVIC_SetPriority(static_cast<IRQn_Type>(irqn), 0);
				NVIC_EnableIRQ(static_cast<IRQn_Type>(irqn));
			}

			GCLK->PCHCTRL[EIC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);

			EIC->CTRLA.bit.ENABLE = 1;
			while (EIC->SYNCBUSY.bit.ENABLE == 1) { }
		}

		void GpioChip::mode(int pin, const PinMode &mode)
		{
			switch(mode) {
			case INPUT_NOPULLUP:
			case INPUT_PULLUP:
				if(mode == INPUT_NOPULLUP) {
					gpio_set_pin_pull_mode(static_cast<uint8_t>(pin), GPIO_PULL_OFF);
				} else {
					gpio_set_pin_pull_mode(static_cast<uint8_t>(pin), GPIO_PULL_UP);
				}

			case INPUT:
				gpio_set_pin_direction(static_cast<uint8_t>(pin), GPIO_DIRECTION_IN);
				break;

			case OUTPUT_OPEN_DRAIN:
				gpio_set_pin_pull_mode(static_cast<uint8_t>(pin), GPIO_PULL_OFF);

			default:
			case OUTPUT:
				gpio_set_pin_direction(static_cast<uint8_t>(pin), GPIO_DIRECTION_OUT);
				break;
			}
		}

		void GpioChip::write(int pin, bool value)
		{
			gpio_set_pin_level(static_cast<uint8_t>(pin), value);
		}

		bool GpioChip::read(int pin) const
		{
			return gpio_get_pin_level(static_cast<uint8_t >(pin));
		}

		void GpioChip::setOpenDrain(int pin)
		{
			this->mode(pin, OUTPUT_OPEN_DRAIN);
		}

		void GpioChip::odWrite(int pin, bool value)
		{
			gpio_set_pin_level(static_cast<uint8_t>(pin), value);
		}

		void GpioChip::attachIrqHandler(int pin, irq_handler_t handler, IrqEdge edge)
		{
			uint32_t config;
			uint32_t pos;

			if(!this->initialized_) {
				this->initIrqs();
				this->initialized_ = true;
			}

			if(pin == detail_gpio::NMI_IRQ_PIN) {
				EIC->NMIFLAG.bit.NMI = 1;

				switch(edge) {
				default:
				case IrqEdge::IrqRisingFalling:
					EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_BOTH;
					break;

				case IrqEdge::IrqFalling:
					EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_FALL;
					break;

				case IrqEdge::IrqRising:
					EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_RISE;
					break;
				}

				this->irq_map_[EXTERNAL_INT_NMI].handler = handler;
				this->irq_map_[EXTERNAL_INT_NMI].pin = pin;

				EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << EXTERNAL_INT_NMI);
			} else {
				pin_set_peripheral(pin, PIO_EXTINT);

				auto irqno = GPIO_PIN(pin) % (EXTERNAL_NUM_INTERRUPTS - 1);

				this->irq_map_[irqno].handler = handler;
				this->irq_map_[irqno].pin = pin;

				if(irqno > EXTERNAL_INT_7) {
					config = 1;
					pos = (irqno - 8) << 2;
				} else {
					config = 0;
					pos = irqno << 2;
				}

				EIC->CTRLA.bit.ENABLE = 0;
				while(EIC->SYNCBUSY.bit.ENABLE == 1) {
				}

				EIC->CONFIG[config].reg &= ~(EIC_CONFIG_SENSE0_Msk << pos);

				switch(edge) {
				default:
				case IrqEdge::IrqRisingFalling:
					EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_BOTH_Val << pos;
					break;

				case IrqEdge::IrqFalling:
					EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_FALL_Val << pos;
					break;

				case IrqEdge::IrqRising:
					EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_RISE_Val << pos;
					break;
				}

				EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << irqno);
			}

			EIC->CTRLA.bit.ENABLE = 1;
			while(EIC->SYNCBUSY.bit.ENABLE == 1) { }

			print_dbg("IRQ enabled!\n");
		}

#ifdef HAVE_DEBUG
		void GpioChip::printPinMappings()
		{
			auto pb16 = GPIO(GPIO_PORTB, 16);
			auto pb23 = GPIO(GPIO_PORTB, 23);
			auto pa17 = GPIO(GPIO_PORTA, 17);

			auto irq_pb16 = GPIO_PIN(pb16) % (EXTERNAL_NUM_INTERRUPTS - 1);
			auto irq_pb23 = GPIO_PIN(pb23) % (EXTERNAL_NUM_INTERRUPTS - 1);
			auto irq_pa17 = GPIO_PIN(pa17) % (EXTERNAL_NUM_INTERRUPTS - 1);

			print_dbg("IRQn for PB16: %u - Should be 0\n", irq_pb16);
			print_dbg("IRQn for PB23: %u - Should be 7\n", irq_pb23);
			print_dbg("IRQn for PA17: %u - Should be 1\n", irq_pa17);
		}
#endif

		void GpioChip::detachIrqHandler(int pin)
		{
		}

		int GpioChip::mapIrqType(const IrqEdge &edge) const
		{
			return -EINVALID;
		}

		void GpioChip::invoke(ExtIrqs num)
		{
			auto irqnum = static_cast<uint8_t>(num);
			uint32_t msk = 1UL << irqnum;

			if(this->irq_map_[irqnum].pin != NOT_INITIALIZED)
				this->irq_map_[irqnum].handler();

			EIC->INTFLAG.reg = msk;
		}

	}

	extern "C" void pinMode(int pin, int mode)
	{
	}
}

void invokeIrq(ExtIrqs num)
{
	auto instance = lwiot::samd51::GpioChip::Instance();
	instance->invoke(num);
}


lwiot::GpioChip& gpio = lwiot::samd51::GpioChip::Instance().get();
