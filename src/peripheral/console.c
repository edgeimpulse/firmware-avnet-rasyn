/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "FreeRTOS.h"
#include "event_groups.h"

#include "console.h"
#include "peripheral/usb/usb_pcdc_vcom.h"
#include "usb_thread_interface.h"
#include "ei_main_event.h"

#define UART_RX_BUFFER_SIZE         512
#define RESET_VALUE                 0

/*
 * Private function declarations
 */
const baud_setting_t g_console_baud_max_setting =
    {
    /* Baud rate calculated with 4.334% error. */.semr_baudrate_bits_b.abcse = 0,
      .semr_baudrate_bits_b.abcs = 0, .semr_baudrate_bits_b.bgdm = 1, .cks = 0, .brr = 12, .mddr = (uint8_t) 256, .semr_baudrate_bits_b.brme =
              false };

const baud_setting_t g_console_baud_default_setting =
        {
         /* Baud rate calculated with 0.469% error. */.semr_baudrate_bits_b.abcse = 0,
           .semr_baudrate_bits_b.abcs = 0, .semr_baudrate_bits_b.bgdm = 1, .cks = 0, .brr = 53, .mddr = (uint8_t) 256, .semr_baudrate_bits_b.brme =
                   false };

static uint8_t g_temp_buffer[UART_RX_BUFFER_SIZE] = {0};
/* Flag for user callback */
static volatile uint8_t g_uart_event = RESET_VALUE;

/* Flag RX completed */
static volatile uint8_t g_uart_rx_completed = false;

/* Flag TX completed */
static volatile uint8_t g_uart_tx_completed = false;

/* Flag error occurred */
static volatile uint8_t g_uart_error = false;

/* Counter to update g_temp_buffer index */
static volatile uint16_t g_rx_index = RESET_VALUE;

/* Index of data sent to at hanlder */
static volatile uint16_t g_uart_rx_read_index = RESET_VALUE;

/**
 *
 * @return
 */
fsp_err_t console_init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = R_SCI_UART_Open (g_console_ctrl, g_console_cfg);
    FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

    return FSP_SUCCESS;
}

/**
 *
 * @return
 */
fsp_err_t console_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err =  R_SCI_UART_Close (g_console_ctrl);
    FSP_ERROR_RETURN(FSP_SUCCESS == err, err);

    return FSP_SUCCESS;
}

/**
 *
 * @param p_args
 */
void console_callback(uart_callback_args_t *p_args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            g_temp_buffer[g_rx_index++] = (uint8_t ) p_args->data;
            if (p_args->data == CARRIAGE_ASCII) {

                g_uart_rx_completed = true;
                xEventGroupSetBitsFromISR(g_ei_main_event_group, EI_EVENT_RX, &xHigherPriorityTaskWoken);
            }
        }
        break;
        case UART_EVENT_RX_COMPLETE:
        {
            g_uart_rx_completed = true;
        }
        break;
        case UART_EVENT_TX_COMPLETE:
        {
            g_uart_tx_completed = true;
        }
        break;
        default:
        {

        }
        break;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Print by UART
 * @param pBuffer
 * @param size
 * @return
 */
int console_print(char *pBuffer, int size)
{
    fsp_err_t err = FSP_SUCCESS;

    g_uart_tx_completed = false;
    err = R_SCI_UART_Write(g_console_ctrl, (uint8_t *)pBuffer, (uint32_t)size);

    if(FSP_SUCCESS != err) {
        __BKPT();
    }


    while(g_uart_tx_completed == false) {
        //
    }
    return size;
}

/* redirecting output */
int _write(int fd, char *pBuffer, int size)
{
    (void)fd;

    if (get_print_console_type() == CONSOLE_USB_CDC) {
        //usb_pcdc_print(pBuffer, size);
        comms_send(pBuffer, (uint32_t)size, 100);
    }
    else {
        console_print(pBuffer, size);
    }

    return size;
}

/**
 *
 * @param is_inference_running
 * @return
 */
char ei_get_serial_byte(uint8_t is_inference_running)
{
    FSP_PARAMETER_NOT_USED(is_inference_running);
    char to_send = -1;

    if (get_print_console_type() == CONSOLE_USB_CDC) {
        to_send = get_usb_byte(is_inference_running);
    }
    else {
        if (g_rx_index != 0) {
            to_send = g_temp_buffer[g_uart_rx_read_index++];    // get one
        }

        if ((g_uart_rx_read_index == g_rx_index) && (g_uart_rx_read_index != 0)) {  // when equal and different from zero
            g_uart_rx_read_index = 0;   // reset
            g_rx_index = 0;
            memset(g_temp_buffer, 0, sizeof(g_temp_buffer));
        }
    }

    return to_send;
}

/**
 *
 * @param is_max_baud
 * @return
 */
fsp_err_t uart_set_baud(bool is_max_baud)
{
    fsp_err_t err = FSP_SUCCESS;

    if (is_max_baud == true) {
        err = R_SCI_UART_BaudSet(g_console_ctrl, &g_console_baud_max_setting);
    }
    else{
        err = R_SCI_UART_BaudSet(g_console_ctrl, &g_console_baud_default_setting);
    }

    return err;
}
