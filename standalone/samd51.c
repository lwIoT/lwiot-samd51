/*
 * SAMD51 standalone environment.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <sam.h>
#include <core_cm4.h>

#include <lwiot/types.h>

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

void no_os_init()
{
	__tick = 0;

	SysTick->CTRL = 0;
	SysTick-> LOAD = (F_CPU / 1000) - 1;
	NVIC_SetPriority(SysTick_IRQn, 3);
	SysTick->VAL = 0;
	SysTick->CTRL = 7;
	NVIC_EnableIRQ(SysTick_IRQn);

	while(__tick < 100);
}

time_t lwiot_tick_ms()
{
	return __tick;
}
