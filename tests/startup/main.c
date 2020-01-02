/*
 * SAMD51 example application. In order compile define:
 *
 * F_CPU = CONF_CPU_FREQUENCY = 120000000 (120MHz)
 */

#include <stdio.h>
#include <lwiot.h>

#include <lwiot/log.h>

#include "app.h"
#include "usb_start.h"

extern void app_main();

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	system_init();
	usb_init();

	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
	setbuf(stderr, NULL);
	cdcd_acm_example();

	lwiot_init();
	app_main();
}
