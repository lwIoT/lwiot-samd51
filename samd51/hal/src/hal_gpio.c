/**
 * \file
 *
 * \brief Port
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include <lwiot.h>
#include <hal_gpio.h>

#include <lwiot/samd51/gpiochip.h>

/**
 * \brief Driver version
 */
#define DRIVER_VERSION 0x00000001u

uint32_t gpio_get_version(void)
{
	return DRIVER_VERSION;
}

void pin_set_peripheral(uint32_t pin, EPioType iotype)
{
	uint32_t temp;
	uint8_t raw_pin = GPIO_PIN(pin);
	uint8_t port = GPIO_PORT(pin);

	switch(iotype) {
	default:
	case PIO_DIGITAL:
		break;

	case PIO_INPUT:
	case PIO_INPUT_PULLUP:
		gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);

		if(iotype == PIO_INPUT_PULLUP)
			gpio_set_pin_pull_mode(pin, GPIO_PULL_UP);
		else
			gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
		break;

	case PIO_OUTPUT:
		gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
		break;

	case PIO_ANALOG:
	case PIO_SERCOM:
	case PIO_SERCOM_ALT:
	case PIO_TIMER:
	case PIO_TIMER_ALT:
	case PIO_EXTINT:
	case PIO_TCC_PDEC:
	case PIO_COM:
	case PIO_SDHC:
	case PIO_I2S:
	case PIO_PCC:
	case PIO_GMAC:
	case PIO_AC_CLK:
	case PIO_CCL:
		if(raw_pin & 1) {
			temp = (PORT->Group[port].PMUX[raw_pin >> 1].reg) & PORT_PMUX_PMUXE(0xF);
			PORT->Group[port].PMUX[raw_pin >> 1].reg = temp | PORT_PMUX_PMUXO(iotype);
			PORT->Group[port].PINCFG[raw_pin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;
		} else {
			temp = (PORT->Group[port].PMUX[raw_pin >> 1].reg) & PORT_PMUX_PMUXO(0xF);
			PORT->Group[port].PMUX[raw_pin >> 1].reg = temp | PORT_PMUX_PMUXE(iotype);
			PORT->Group[port].PINCFG[raw_pin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;
		}

		break;
	}
}
