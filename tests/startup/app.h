/*
 * Application header.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */
 
#pragma once

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

extern void system_init();

#define LED_PIN GPIO(GPIO_PORTA, 16)
#define NEO_PIN GPIO(GPIO_PORTB, 22)

#define PA24 GPIO(GPIO_PORTA, 24)
#define PA25 GPIO(GPIO_PORTA, 25)
