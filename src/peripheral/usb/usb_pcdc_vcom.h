/*
 * usb_pcdc_vcom.c
 *
 *  Created on: 2023年4月10日
 *      Author: Nick
 */
#ifndef USB_PCDC_VCOM_H_
#define USB_PCDC_VCOM_H_

#include "FreeRTOS.h"
#include "hal_data.h"

FSP_CPP_HEADER

/* Function declaration */
fsp_err_t comms_open(uint8_t wait);
fsp_err_t comms_send(uint8_t * p_src, uint32_t len, uint32_t period);
fsp_err_t comms_read(uint8_t * p_dest, uint32_t * len, uint32_t timeout_milliseconds);
fsp_err_t comms_close(void);
fsp_err_t comms_suspend(void);
fsp_err_t comms_resume(void);
bool comms_get_is_open(void);

FSP_CPP_FOOTER

#endif /* USB_PCDC_VCOM_H_ */
