/*
 * console.h
 *
 *  Created on: 2022年11月24日
 *      Author: Wenxue
 */

#ifndef BLE_UART_H_
#define BLE_UART_H_

#include <stdio.h>
#include <hal_data.h>

extern volatile bool            g_uart3_txComplete;  /* Tx complete flags */

FSP_CPP_HEADER

/* Function declaration */
extern fsp_err_t ble_uart_init(void);
extern fsp_err_t ble_uart_deinit(void);
extern void ble_uart_callback(uart_callback_args_t *p_args);
extern int ble_send(char *pBuffer, int size);

FSP_CPP_FOOTER

#endif /* BLE_UART_H_ */
