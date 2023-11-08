/*
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef CONSOLE_H_
#define CONSOLE_H_

#include <stdio.h>
#include "ndp/fat_load.h"
#include <hal_data.h>

FSP_CPP_HEADER

/* Macro definition */
#define CARRIAGE_ASCII            (13u)     /* Carriage return */
#define ZERO_ASCII                (48u)     /* ASCII value of zero */
#define NINE_ASCII                (57u)     /* ASCII value for nine */
#define DATA_LENGTH               (4u)      /* Expected Input Data length */

/* UART micro */
#define g_console(x)    &g_uart4_##x
#define g_console_ctrl  g_console(ctrl)
#define g_console_cfg   g_console(cfg)

/* Function declaration */
extern fsp_err_t console_init(void);
extern fsp_err_t console_deinit(void);
extern void console_callback(uart_callback_args_t *p_args);
extern int _write(int fd, char *pBuffer, int size);
extern char uart_get_rx_data(uint8_t is_inference_running);
extern char ei_get_serial_byte(uint8_t is_inference_running);
extern fsp_err_t uart_set_baud(bool is_max_baud);

FSP_CPP_FOOTER

#endif /* CONSOLE_H_ */
