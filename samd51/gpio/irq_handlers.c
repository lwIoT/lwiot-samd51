/*
 * GPIO external IRQ handlers.
 *
 * @author Michel Megens
 * @email  michel@michelmegens.net
 */

#include <sam.h>
#include <lwiot/samd51/gpiochip.h>

void EIC_0_Handler(void)
{
	invokeIrq(EXTERNAL_INT_0);
}

void EIC_1_Handler(void)
{
	invokeIrq(EXTERNAL_INT_1);
	EIC->INTFLAG.reg = 1 << 1;
}

void EIC_2_Handler(void)
{
	invokeIrq(EXTERNAL_INT_2);
}

void EIC_3_Handler(void)
{
	invokeIrq(EXTERNAL_INT_3);
}

void EIC_4_Handler(void)
{
	invokeIrq(EXTERNAL_INT_4);
}

void EIC_5_Handler(void)
{
	invokeIrq(EXTERNAL_INT_5);
}

void EIC_6_Handler(void)
{
	invokeIrq(EXTERNAL_INT_6);
}

void EIC_7_Handler(void)
{
	invokeIrq(EXTERNAL_INT_7);
}

void EIC_8_Handler(void)
{
	invokeIrq(EXTERNAL_INT_8);
}

void EIC_9_Handler(void)
{
	invokeIrq(EXTERNAL_INT_9);
}

void EIC_10_Handler(void)
{
	invokeIrq(EXTERNAL_INT_10);
}

void EIC_11_Handler(void)
{
	invokeIrq(EXTERNAL_INT_11);
}

void EIC_12_Handler(void)
{
	invokeIrq(EXTERNAL_INT_12);
}

void EIC_13_Handler(void)
{
	invokeIrq(EXTERNAL_INT_13);
}

void EIC_14_Handler(void)
{
	invokeIrq(EXTERNAL_INT_14);
}

void EIC_15_Handler(void)
{
	invokeIrq(EXTERNAL_INT_15);
}

void NonMaskableInt_Handler(void)
{
	invokeIrq(EXTERNAL_INT_NMI);
}
