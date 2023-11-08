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
#include "semphr.h"
#include "ndp_thread_interface.h"
#include <stdio.h>
#include "peripheral/console.h"
#include "peripheral/spi_drv.h"
#include "peripheral/sdcard.h"
#include "peripheral/led.h"
#include "peripheral/button.h"
#include "peripheral/DA9231.h"
#include "ndp/fat_load.h"
#include "ndp/ndp_flash.h"
#include "ndp/ndp_irq_service.h"
#include "syntiant_platform.h"

#include "ingestion-sdk-platform/rasyn/ei_device_rasyn.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "ingestion-sdk-platform/sensor/ei_inertial.h"
#include "inference/ei_run_impulse.h"
#include "peripheral/usb/usb_pcdc_vcom.h"

#define NDP_TASK_STACK_SIZE_BYTE        32768

/* Private variables -------------------------------------------------------------------- */
static EiRASyn* pdev;

/* FreeRTOS module */
TaskHandle_t ndp_thread_handle;

typedef struct blink_msg
{
    int led;
	TickType_t  timestamp;
} blink_msg_t;

static void ndp_thread_entry(void *pvParameters);

/* Public functions -------------------------------------------------------- */
/**
 * @brief Start ndp thread
 */
void ndp_thread_start(void)
{
    BaseType_t retval;
    /* create a task to send data via usb */
    retval = xTaskCreate(ndp_thread_entry,
        (const char*) "NDP Thread",
        NDP_TASK_STACK_SIZE_BYTE / 4, // in words
        NULL, //pvParameters
        configMAX_PRIORITIES - 1, //uxPriority
        &ndp_thread_handle);

    if (retval != pdTRUE) {
        // error !!
        while(1){};
    }

}

/* NDP Thread entry function */
/* pvParameters contains TaskHandle_t */
static void ndp_thread_entry(void *pvParameters)
{
    FSP_PARAMETER_NOT_USED (pvParameters);
    int ret;
    uint8_t ndp_class_idx;
    uint8_t ndp_nn_idx;
    uint8_t sec_val;
    EventBits_t   evbits;
    uint32_t notifications;
	blink_msg_t  last_stat, current_stat;
	int retries = 10;
	int ndp_boot_mode = NDP_CORE2_BOOT_MODE_BOOT_FLASH;

	pdev =  static_cast<EiRASyn*>(EiDeviceInfo::get_device());

    /* Set BUCK and LDO for NDP120 */
    DA9231_open();
    DA9231_disable();
    DA9231_enable();
    //R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MILLISECONDS);
   // DA9231_dump_regs();
    DA9231_close();

    ndp_irq_init();

    if (get_print_console_type() == CONSOLE_USB_CDC) {
        comms_close();
    }

    /* Delay 100 ms */
    ei_sleep(100);

    if (get_synpkg_boot_mode() == BOOT_MODE_SD)
    {
        ndp_boot_mode = NDP_CORE2_BOOT_MODE_HOST_FILE;
    }

    /* Start NDP120 program */
    ret = ndp_core2_platform_tiny_start(1, 1, ndp_boot_mode);
    if(ret == 0) {
#if NDP_DEBUG == 1
        ei_printf("ndp_core2_platform_tiny_start done\r\n");
#endif
        xSemaphoreGive(g_binary_semaphore);
    } else {
        ei_printf("ndp_core2_platform_tiny_start failed %d\r\n", ret);
    }

    do {
        ret = ndp_core2_platform_tiny_feature_set(NDP_CORE2_FEATURE_PDM);
        if (ret) {
            ei_printf("ndp_core2_platform_tiny_feature_set set 0x%x failed %d\r\n", NDP_CORE2_FEATURE_PDM, ret);
            ei_sleep(50);
        }

    }while((!ret) && (--retries));

    if (get_synpkg_boot_mode() != BOOT_MODE_SD) {
        // read back info from FLASH
        config_data_in_flash_t flash_data = {0};

        if (0 == ndp_flash_read_infos(&flash_data)){
            mode_circular_motion = flash_data.ndp_mode_motion;
            memcpy(&config_items, &flash_data.cfg, sizeof(struct config_ini_items));
#if NDP_DEBUG == 1
            ei_printf("read back from FLASH got mode_circular_motion: %s, button_switch: %s\n",
                    (mode_circular_motion?"disable":"enable"), button_switch);
#endif
        }
    }

    read_ndp_model();
    //print_ndp_model();

	if (motion_to_disable() == CIRCULAR_MOTION_DISABLE) {
	    pdev->set_used_sensor(e_enabled_mic);

        ret = ndp_core2_platform_tiny_sensor_ctl(0, 0); // disable IMU
        if (!ret){
            ei_printf("disable sensor[0] functionality\n");
            pdev->set_imu_ok(true);
        }
        else {
            ei_printf("error in disable sensor[0] functionality %d\n", ret);
            ei_sleep(100);
        }

	}
	else {
	    pdev->set_used_sensor(e_enabled_imu);
	}

    /* Enable NDP IRQ */
    ei_run_nn_normal();

    if (get_print_console_type() == CONSOLE_USB_CDC) {
        comms_open(1);
    }

    memset(&last_stat, 0, sizeof(blink_msg_t));
    memset(&current_stat, 0, sizeof(blink_msg_t));

    pdev->init_sd_card();

    while (1) {
        evbits = xEventGroupWaitBits(
                g_ndp_event_group,   /* The event group being tested. */
                EVENT_BIT_VOICE | EVENT_BIT_FLASH, /* The bits within the event group to wait for. */
                  pdTRUE,        /* should be cleared before returning. */
                  pdFALSE,       /* Don't wait for both bits, either bit will do. */
                  portMAX_DELAY );/* Wait a maximum of 100ms for either bit to be set. */

        if (evbits & EVENT_BIT_VOICE) {
            // do something
            xSemaphoreTake(g_ndp_mutex, portMAX_DELAY);
            pdev->set_state(EiRASynStateMatch);
            ndp_core2_platform_tiny_poll(&notifications, 1);
            if (NDP_CORE2_ERROR_NONE == ndp_core2_platform_tiny_match_process(&ndp_nn_idx, &ndp_class_idx, &sec_val, NULL)) {
                ei_classification_output(ndp_nn_idx, ndp_class_idx, sec_val);

            }
            else {
                ei_printf("Error in ndp_core2_platform_tiny_match_process\r\n");
            }

            xSemaphoreGive(g_ndp_mutex);

            xSemaphoreGive(g_binary_semaphore);
            /* Store the led state */
            memcpy(&last_stat, &current_stat, sizeof(blink_msg_t));
        }
        else if (evbits & EVENT_BIT_FLASH) {
            //
        }
    }
}
