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

/* Include ----------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "common_data.h"
#include "led_thread_interface.h"
#include "ingestion-sdk-platform/rasyn/ei_device_rasyn.h"
#include "peripheral/led.h"
#include <stdio.h>

#define LED_TASK_STACK_SIZE_BYTE        (512u)
#define LED_TASK_PRIORITY               (configMAX_PRIORITIES - 1)

/* FreeRTOS module */
static TaskHandle_t led_pcdc_thread;

static void led_thread_entry(void *pvParameters);

/* Public functions -------------------------------------------------------- */
/**
 * @brief Start ndp thread
 */
void led_thread_start(void)
{
    BaseType_t retval;
    /* create a task to send data via usb */
    retval = xTaskCreate(led_thread_entry,
        (const char*) "NDP Thread",
        LED_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        LED_TASK_PRIORITY, //uxPriority
        &led_pcdc_thread);

    if (retval != pdTRUE) {
        // error !!
        while(1){};
    }

}

/**
 * @brief LED IDLE Thread entry function
 * @param pvParameters contains TaskHandle_t
 */
static void led_thread_entry(void *pvParameters)
{
    EiRASyn         *dev = static_cast<EiRASyn*>(EiDeviceInfo::get_device());
    EiRASynState    old_status = EiRASynStateIdle;
    TickType_t      delay = 250;
    BaseType_t      fr_err;
    uint32_t        len = 0;
    bool            toogle_state = false;
    uint32_t        counter = 0;

    FSP_PARAMETER_NOT_USED (pvParameters);

    while (1)
    {
        fr_err = xQueueReceive(g_new_state_queue, &len, delay); // check if new state received ?

        if (pdPASS == fr_err) {
            old_status = dev->get_state();
            toogle_state = false;
            counter = 0;
        }

        switch(old_status)
        {
            case EiRASynStateIdle:  // green
            {
                turn_led(BSP_LEDGREEN, BSP_LEDON);
                turn_led(BSP_LEDBLUE, BSP_LEDOFF);
                turn_led(BSP_LEDRED, BSP_LEDOFF);

                delay = 500;
            }
            break;
            case EiRASynStateErasingFlash:  // yellow
            {
                turn_led(BSP_LEDGREEN, BSP_LEDON );
                turn_led(BSP_LEDBLUE, BSP_LEDOFF );
                turn_led(BSP_LEDRED, BSP_LEDON );

                delay = 500;
            }
            break;
            case EiRASynStateSampling:  // blink blue
            {
                if (toogle_state) {
                    turn_led(BSP_LEDGREEN, BSP_LEDOFF );
                    turn_led(BSP_LEDBLUE, BSP_LEDON );
                    turn_led(BSP_LEDRED, BSP_LEDOFF );
                    toogle_state = false;
                }
                else {
                    turn_led(BSP_LEDGREEN, BSP_LEDOFF);
                    turn_led(BSP_LEDBLUE, BSP_LEDOFF);
                    turn_led(BSP_LEDRED, BSP_LEDOFF);
                    toogle_state = true;
                }

                delay = 500;
            }
            break;
            case EiRASynStateUploading: // blink cyan
            {
                if (toogle_state) {
                    turn_led(BSP_LEDGREEN, BSP_LEDON );
                    turn_led(BSP_LEDBLUE, BSP_LEDON );
                    turn_led(BSP_LEDRED, BSP_LEDOFF );
                    toogle_state = false;
                }
                else {
                    turn_led(BSP_LEDGREEN, BSP_LEDOFF);
                    turn_led(BSP_LEDBLUE, BSP_LEDOFF);
                    turn_led(BSP_LEDRED, BSP_LEDOFF);
                    toogle_state = true;
                }

                delay = 250;
            }
            break;
            case EiRASynStateFinished:  // gren blue
            {
                if (toogle_state) {
                    turn_led(BSP_LEDGREEN, BSP_LEDON);
                    turn_led(BSP_LEDBLUE, BSP_LEDOFF);
                    turn_led(BSP_LEDRED, BSP_LEDOFF);
                    toogle_state = false;
                }
                else {
                    turn_led(BSP_LEDGREEN, BSP_LEDOFF);
                    turn_led(BSP_LEDBLUE, BSP_LEDON);
                    turn_led(BSP_LEDRED, BSP_LEDOFF);
                    toogle_state = true;
                }

                delay = 250;
            }
            break;
            case EiRASynStateMatch: // magenta
            {
                if (toogle_state) {
                    turn_led(BSP_LEDGREEN, BSP_LEDOFF);
                    turn_led(BSP_LEDBLUE, BSP_LEDON);
                    turn_led(BSP_LEDRED, BSP_LEDON);
                    toogle_state = false;
                }
                else {
                    turn_led(BSP_LEDGREEN, BSP_LEDOFF);
                    turn_led(BSP_LEDBLUE, BSP_LEDOFF);
                    turn_led(BSP_LEDRED, BSP_LEDOFF);
                    toogle_state = true;
                }

                if (++counter >= 5) {
                    dev->set_state(EiRASynStateIdle);
                }

                delay = 250;
            }
            break;
            default:
            {

            }
            break;
        }
    }
}
