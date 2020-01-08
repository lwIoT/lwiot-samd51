/*
 * SAMD51 standalone environment.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sam.h>
#include <core_cm4.h>
#include <lwiot/samd51/usb/usb_dev.h>

volatile uint32_t __tick;

void SysTick_Handler(void)
{
	__tick++;
}

void lwiot_sleep(int ms)
{
	uint32_t next = __tick + (unsigned int)ms;
	while(__tick < next) { }
}

void lwiot_udelay(uint32_t us)
{
	uint32_t start, elapsed;
	uint32_t count;

	if (us == 0)
		return;

	count = us * (F_CPU / 1000000) - 20;  // convert us to cycles.
	start = DWT->CYCCNT;  //CYCCNT is 32bits, takes 37s or so to wrap.
	while (1) {
		elapsed = DWT->CYCCNT - start;
		if (elapsed >= count)
			return;
	}
}

void enter_critical()
{
	__disable_irq();
}

void exit_critical()
{
	__enable_irq();
}

void lwiot_mem_free(void *ptr)
{
	free(ptr);
}

void *lwiot_mem_alloc(size_t size)
{
	return malloc(size);
}

void *lwiot_mem_zalloc(size_t size)
{
	void *ptr = malloc(size);

	memset(ptr, 0, size);
	return ptr;
}

void *lwiot_mem_realloc(void *ptr, size_t size)
{
	return realloc(ptr, size);
}

extern void samd51_init();
extern void chip_init();

void no_os_init()
{
	__tick = 0;

	chip_init();
	usb_init();

	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
	setbuf(stderr, NULL);
	cdcd_acm_start();

	SysTick->CTRL = 0;
	SysTick-> LOAD = (F_CPU / 1000) - 1;
	NVIC_SetPriority(SysTick_IRQn, 3);
	SysTick->VAL = 0;
	SysTick->CTRL = 7;
	NVIC_EnableIRQ(SysTick_IRQn);

	while(__tick < 500);
}

time_t lwiot_tick_ms()
{
	return __tick;
}
