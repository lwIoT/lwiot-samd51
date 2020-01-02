/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */

#pragma  once

#include <lwiot.h>

CDECL

#include <lwiot/samd51/usb/cdcdf_acm.h>
#include <lwiot/samd51/usb/cdcdf_acm_desc.h>

extern void cdcd_acm_start(void);
extern void cdc_device_acm_init(void);

void usb_init(void);

CDECL_END
