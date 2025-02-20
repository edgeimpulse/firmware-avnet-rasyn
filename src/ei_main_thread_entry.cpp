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

/* Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "ei_main_thread.h"
#include "ndp_thread_interface.h"
#include "ei_main_event.h"

#include "peripheral/console.h"
#include "peripheral/spi_drv.h"
#include "peripheral/button.h"
#include "peripheral/ble_uart.h"

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "inference/ei_run_impulse.h"
#include "ingestion-sdk-platform/rasyn/ei_device_rasyn.h"
#include "ingestion-sdk-platform/rasyn/ei_at_handlers.h"
#include "ingestion-sdk-platform/sensor/ei_inertial.h"

#include "led_thread_interface.h"

/* Private variables -------------------------------------------------------------------- */
static ATServer *at;
static EiRASyn* pdev;

/* EiMainThread entry function */
/* pvParameters contains TaskHandle_t */
void ei_main_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED (pvParameters);
    pdev =  static_cast<EiRASyn*>(EiDeviceInfo::get_device());
    bool in_rx_loop = false;
    char data = 0xFF;
    EventBits_t   evbits;

    R_BSP_PinAccessEnable(); /* Enable access to the PFS registers. */
    R_BSP_PinWrite(LED_USER, BSP_IO_LEVEL_HIGH); /* Turn off User Led */
    /* WIFI reset */
    R_BSP_PinWrite(DA16600_RstPin, BSP_IO_LEVEL_LOW);
    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
    R_BSP_PinWrite(DA16600_RstPin, BSP_IO_LEVEL_HIGH);

    /* will create a task if using USB CDC */
    console_init();

    button_init();
    ble_uart_init();

    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);

    led_thread_start();
    ei_inertial_init();

    ndp_thread_start();
    xSemaphoreTake(g_binary_semaphore, portMAX_DELAY);

    at = ei_at_init(pdev);

    ei_printf("Hello from Edge Impulse\n");
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    at->print_prompt();

    while (1) {
        evbits = xEventGroupWaitBits(
                g_ei_main_event_group,   /* The event group being tested. */
                EI_EVENT_RX, /* The bits within the event group to wait for. */
                  pdTRUE,        /* should be cleared before returning. */
                  pdFALSE,       /* Don't wait for both bits, either bit will do. */
                  portMAX_DELAY );/* Wait a maximum of 100ms for either bit to be set. */

        if (evbits & EI_EVENT_RX) {
            /* handle command comming from uart */
            data = ei_get_serial_byte(1);
            in_rx_loop = false;

            while ((uint8_t)data != 0xFF) {
                if ((ei_run_impulse_is_active() == true) && (data == 'b') && (in_rx_loop == false)) {
                    ei_start_stop_run_impulse(false);
                    at->print_prompt();
                }

                in_rx_loop = true;
                at->handle(data);
                data = ei_get_serial_byte(1);
            }
        }

    }

    while (1) {
        /* we should never end here */
        vTaskDelay (portMAX_DELAY);
    }
}
