/*
 * GPIO implementation for the ESP32.
 *
 * @author Michel Megens
 * @email  dev@bietje.net
 */

#include <lwiot.h>
#include <hal_gpio.h>

#include <lwiot/io/gpiochip.h>
#include <lwiot/samd51/gpiochip.h>
#include <lwiot/error.h>
#include <lwiot/log.h>

namespace lwiot
{
	namespace samd51
	{
		GpioChip::GpioChip() : lwiot::GpioChip(PINS), irq_map_()
		{
			this->init();
		}

		void GpioChip::init()
		{
			for(auto& pin : this->irq_map_) {
				pin.pin = 0xFFFF;
			}

			this->initIrqs();
		}

		void GpioChip::initIrqs()
		{
			for(auto idx = 0U; idx < detail_gpio::EXTERNAL_NUM_INTERRUPTS; idx++) {
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
			uint32_t temp;
			auto raw_pin = GPIO_PIN(pin);
			auto port = GPIO_PORT(pin);
			uint32_t config;
			uint32_t pos;

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

				this->irq_map_[detail_gpio::EXTERNAL_INT_NMI].handler = handler;
				this->irq_map_[detail_gpio::EXTERNAL_INT_NMI].pin = pin;

				EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << detail_gpio::EXTERNAL_INT_NMI);
			} else {
				if(raw_pin & 1) {
					temp = (PORT->Group[port].PMUX[raw_pin >> 1].reg) & PORT_PMUX_PMUXE(0xF);
					PORT->Group[port].PMUX[raw_pin >> 1].reg = temp | PORT_PMUX_PMUXO(detail_gpio::PIO_EXTINT);
					PORT->Group[port].PINCFG[raw_pin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;
				} else {
					temp = (PORT->Group[port].PMUX[raw_pin >> 1].reg) & PORT_PMUX_PMUXO(0xF);
					PORT->Group[port].PMUX[raw_pin >> 1].reg = temp | PORT_PMUX_PMUXE(detail_gpio::PIO_EXTINT);
					PORT->Group[port].PINCFG[raw_pin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;
				}

				auto irqno = raw_pin % (detail_gpio::EXTERNAL_NUM_INTERRUPTS - 1);

				this->irq_map_[irqno].handler = handler;
				this->irq_map_[irqno].pin = pin;

				if(irqno > detail_gpio::EXTERNAL_INT_7) {
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
			while(EIC->SYNCBUSY.bit.ENABLE == 1) {
			}
		}

#ifdef HAVE_DEBUG
		void GpioChip::printPinMappings()
		{
			auto pb16 = GPIO(GPIO_PORTB, 16);
			auto pb23 = GPIO(GPIO_PORTB, 23);
			auto pa14 = GPIO(GPIO_PORTA, 14);

			auto irq_pb16 = GPIO_PIN(pb16) % (detail_gpio::EXTERNAL_NUM_INTERRUPTS - 1);
			auto irq_pb23 = GPIO_PIN(pb23) % (detail_gpio::EXTERNAL_NUM_INTERRUPTS - 1);
			auto irq_pa14 = GPIO_PIN(pa14) % (detail_gpio::EXTERNAL_NUM_INTERRUPTS - 1);

			print_dbg("IRQn for PB16: %u - Should be 0\n", irq_pb16);
			print_dbg("IRQn for PB23: %u - Should be 7\n", irq_pb23);
			print_dbg("IRQn for PA14: %u - Should be 14\n", irq_pa14);
		}
#endif

		void GpioChip::detachIrqHandler(int pin)
		{
		}

		int GpioChip::mapIrqType(const IrqEdge &edge) const
		{
			return -EINVALID;
		}
	}

	extern "C" void pinMode(int pin, int mode)
	{
	}
}

lwiot::GpioChip& gpio = lwiot::samd51::GpioChip::Instance().get();
