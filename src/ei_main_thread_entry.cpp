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

/* Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "ei_main_thread.h"

#include "ndp_thread_interface.h"

#include "peripheral/console.h"
#include "peripheral/spi_drv.h"
#include "peripheral/button.h"
#include "peripheral/ble_uart.h"

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "inference/ei_run_impulse.h"
#include "ingestion-sdk-platform/rasyn/ei_device_rasyn.h"
#include "ingestion-sdk-platform/rasyn/ei_at_handlers.h"
#include "ingestion-sdk-platform/sensor/ei_inertial.h"
#include "usb_thread_interface.h"

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

    R_BSP_PinAccessEnable(); /* Enable access to the PFS registers. */
    R_BSP_PinWrite(LED_USER, BSP_IO_LEVEL_HIGH); /* Turn off User Led */
    /* WIFI reset */
    R_BSP_PinWrite(DA16600_RstPin, BSP_IO_LEVEL_LOW);
    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
    R_BSP_PinWrite(DA16600_RstPin, BSP_IO_LEVEL_HIGH);

    /* will create a task if using USB CDC */
    console_init();
    spi_init();

    init_fatfs();
    button_init();
    ble_uart_init();

    R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);

    /* read config info of ndp firmwares */
    get_synpkg_config_info();
    
    /* Choose the appropriate debug print console */
    if (get_print_console_type() == CONSOLE_USB_CDC) {
        start_usb_pcdc_thread();
        console_deinit();
    }
    ei_inertial_init();

    ndp_thread_start();
    xSemaphoreTake(g_binary_semaphore, portMAX_DELAY);

    at = ei_at_init(pdev);

    ei_printf("Hello from Edge Impulse\n");
    ei_printf("Type AT+HELP to see a list of commands.\r\n");
    ei_printf("Starting main loop\r\n");

    at->print_prompt();

    while (1) {
        /* handle command comming from uart */
        char data = ei_get_serial_byte(1);
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

        vTaskDelay (10);
    }

    while (1) {
        /* we should never end here */
        vTaskDelay (portMAX_DELAY);
    }
}
