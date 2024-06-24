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
