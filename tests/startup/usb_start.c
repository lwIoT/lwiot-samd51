/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */

#include <stdlib.h>

#include "app.h"
#include "usb_start.h"

extern void free(void*);

#if CONF_USBD_HS_SP
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_LS_FS};
static uint8_t single_desc_bytes_hs[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_HS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS
#else
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_DESCES_LS_FS};
#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#endif

static struct usbd_descriptors single_desc[]
    = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
       ,
       {single_desc_bytes_hs, single_desc_bytes_hs + sizeof(single_desc_bytes_hs)}
#endif
};

/** Buffers to receive and echo the communication bytes. */
static uint32_t usbd_cdc_buffer[CDCD_ECHO_BUF_SIZ / 4];

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

volatile bool cdcTransferRead = false;
volatile uint32_t cdcTransferReadLen;
volatile bool cdcTransferWrite = false;
volatile bool listener = false;

/**
 * \brief Callback invoked when bulk OUT data received
 */
static bool usb_device_cb_bulk_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	cdcTransferReadLen = count;
    cdcTransferRead = false;
    return false;
}

/**
 * \brief Callback invoked when bulk IN data received
 */
static bool usb_device_cb_bulk_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	cdcTransferWrite = false;
    return false;
}

int __attribute__((weak)) _write(int file, const char *ptr, int len); /* Remove GCC compiler warning */

static uint8_t outBuf[80];
static uint32_t outLen = 0;
int __attribute__((weak)) _write(int file, const char *buf, int length)
{
	if(!listener)
		return length;

	if(*buf == '\n') {
		const char creturn = '\r';
		_write(file, &creturn, 1);
	}

	const uint8_t* end = ((void*)buf + (unsigned int)length);
	for (const uint8_t* p = (const uint8_t*)buf; p < end; ++p) {
		outBuf[outLen++] = *p;

		if (*p == '\n' || outLen==sizeof(outBuf)) {
			cdcTransferWrite = true;
			cdcdf_acm_write(outBuf, outLen);
			while(cdcTransferWrite);
			outLen = 0;
		}
	}
	return length;
}

/**
 * \brief Callback invoked when Line State Change
 */
static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
	if (state.rs232.DTR) {
		/* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_bulk_out);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_bulk_in);
		listener = true;
		/* Start Rx */
		cdcdf_acm_read((uint8_t *)usbd_cdc_buffer, sizeof(usbd_cdc_buffer));
	} else if(!state.rs232.DTR) {
		listener = false;
	}
	

	/* No error. */
	return false;
}

/**
 * \brief CDC ACM Init
 */
void cdc_device_acm_init(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	cdcdf_acm_init();

	usbdc_start(single_desc);
	usbdc_attach();
}

void cdcd_acm_example(void)
{
	while (!cdcdf_acm_is_enabled()) {
		// wait cdc acm to be installed
	};

	cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);
}

void usb_init(void)
{
	cdc_device_acm_init();
}
