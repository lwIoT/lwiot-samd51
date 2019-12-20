/*
 * HAL time wrapper.
 */

#include <lwiot.h>

#include <lwiot/error.h>
#include <sys/time.h>

int _gettimeofday(struct timeval* tv, void* tzvp)
{
	time_t tick = lwiot_tick_ms();

	tick *= 1000 * 1000; // Perform calculations in nanoseconds.
	tv->tv_sec = tick / 1000000000UL;
	tv->tv_usec = (tick / 1000000000UL) / 1000;

	return -EOK;
}