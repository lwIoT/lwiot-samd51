/*
 * SAMD51 example application. In order compile define:
 *
 * F_CPU = CONF_CPU_FREQUENCY = 120000000 (120MHz)
 */

#include <stdio.h>
#include <lwiot.h>

#include <lwiot/log.h>
#include <lwiot/samd51/usb/usb_dev.h>

#include "app.h"

extern void app_main();
extern void chip_init();

int main(void)
{
	/* Initializes MCU, drivers and middleware */
//	chip_init();
//	system_init();
//	usb_init();
//	usb_init();

//	setbuf(stdout, NULL);
//	setbuf(stdin, NULL);
//	setbuf(stderr, NULL);
//	cdcd_acm_start();
//	cdcd_acm_example();

	lwiot_init();
	app_main();
}
