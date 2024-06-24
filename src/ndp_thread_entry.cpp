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
#include "ingestion-sdk-platform/sensor/ei_inertial.h"

#include "usb_thread_interface.h"

#define NDP_TASK_STACK_SIZE_BYTE        (32768)
#define NDP_TASK_PRIORITY               (configMAX_PRIORITIES - 4)

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
static void set_decimation_inshift(void);
static int bff_reinit_imu(void);

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
        NDP_TASK_PRIORITY, //uxPriority
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
    int fatal_error;
    uint8_t ndp_class_idx;
    uint8_t ndp_nn_idx;
    uint8_t sec_val;
    EventBits_t   evbits;
    uint32_t notifications;
	blink_msg_t  last_stat, current_stat;
	int retries = 10;
	int ndp_boot_mode = NDP_CORE2_BOOT_MODE_BOOT_FLASH;

	pdev =  static_cast<EiRASyn*>(EiDeviceInfo::get_device());

    spi_init();
    init_fatfs();

    /* Set BUCK and LDO for NDP120 */
    DA9231_open();
    DA9231_disable();
    DA9231_enable();
    //R_BSP_SoftwareDelay(1000, BSP_DELAY_UNITS_MILLISECONDS);
   // DA9231_dump_regs();
    DA9231_close();

    ndp_irq_init();

    /* Delay 100 ms */
    ei_sleep(100);
    /* read config info of ndp firmwares */
    get_synpkg_config_info();

    if (get_synpkg_boot_mode() == BOOT_MODE_SD) {
        ndp_boot_mode = NDP_CORE2_BOOT_MODE_HOST_FILE;
    }

    /* Start NDP120 program */
    ret = ndp_core2_platform_tiny_start(4, 1, ndp_boot_mode);
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

    if (ndp_boot_mode == NDP_CORE2_BOOT_MODE_BOOT_FLASH) {
        // read back info from FLASH
        config_data_in_flash_t flash_data = {0};

        if (0 == ndp_flash_read_infos(&flash_data)){
            set_event_watch_mode (flash_data.watch_mode);
            memcpy(&config_items, &flash_data.cfg, sizeof(struct config_ini_items));
#if NDP_DEBUG == 1
            ei_printf("read back from FLASH got mode_circular_motion: %s, button_switch: %s\n",
                    (mode_circular_motion?"disable":"enable"), button_switch);
#endif
        }
    }

    read_ndp_model();
    start_usb_pcdc_thread();

    if (get_event_watch_mode() & WATCH_TYPE_AUDIO) {
        set_decimation_inshift();

	    pdev->set_used_sensor(e_enabled_mic);
	    ret = ndp_core2_platform_tiny_feature_set(NDP_CORE2_FEATURE_PDM);

        if (ret){
            ei_printf("ndp_core2_platform_tiny_feature_set set 0x%x failed %d\r\n",
            NDP_CORE2_FEATURE_PDM, ret);
        }
            
    }
    else {

        if (ndp_boot_mode == NDP_CORE2_BOOT_MODE_BOOT_FLASH) {
            ret = bff_reinit_imu();
            if (ret) {
                ei_printf("bff reinit IMU failed: %d\n", ret);
            }
        }
        else {
            ret = ndp_core2_platform_tiny_dsp_restart();
            if (ret) {
                ei_printf("restart DSP failed: %d\n", ret);
            }
            vTaskDelay (pdMS_TO_TICKS(1000UL));
        }

        ret = ndp_core2_platform_tiny_sensor_ctl(EI_FUSION_IMU_SENSOR_INDEX, 1);
        if (ret) {
            ei_printf("Enable sensor[%d] icm-42670 failed: %d\n", EI_FUSION_IMU_SENSOR_INDEX, ret);
        }
        else {
            ei_printf("Enable sensor[%d] icm-42670 done\n", EI_FUSION_IMU_SENSOR_INDEX);
            pdev->set_imu_ok(true);
            pdev->set_used_sensor(e_enabled_imu);
        }
    }

    /* Enable NDP IRQ */
    ei_run_nn_normal();

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
            ndp_core2_platform_tiny_poll(&notifications, 1, &fatal_error);
            if (fatal_error) {
                ei_printf("\nNDP Fatal Error!!!\n\n");
            }
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

/**
 * @brief Set the decimation inshift object
 * 
 */
static void set_decimation_inshift(void)
{

#define INSHIFT_AUDIO_ID 0
#define INSHIFT_SINGLE_MIC_ID 0
#define INSHIFT_DUAL_MIC_ID 1

    uint8_t decimation_inshift_mic0_read_value = 0;
    uint8_t decimation_inshift_mic1_read_value = 0;
    uint8_t decimation_inshift_calculated_value = 0;

    uint8_t decimation_inshift_value_mic0 = (uint8_t)get_dec_inshift_value();
    uint8_t decimation_inshift_offset = (uint8_t)get_dec_inshift_offset();

    // Catch the case where we don't make any changes; just bail out.
    if((decimation_inshift_value_mic0 == DEC_INSHIFT_VALUE_DEFAULT) &&
       (decimation_inshift_offset == DEC_INSHIFT_OFFSET_DEFAULT )){
        return;
    }

    // Read the decimation_inshift values for both mics
    ndp_core2_platform_tiny_audio_config_get(INSHIFT_AUDIO_ID, INSHIFT_SINGLE_MIC_ID, 0, &decimation_inshift_mic0_read_value);
    ndp_core2_platform_tiny_audio_config_get(INSHIFT_AUDIO_ID, INSHIFT_DUAL_MIC_ID, 0, &decimation_inshift_mic1_read_value);

    ei_printf("    Decimation Inshift Details\n");
    ei_printf("    Model Decimation Inshift Mic0: %d\n", decimation_inshift_mic0_read_value);
    ei_printf("    Model Decimation Inshift Mic1: %d\n", decimation_inshift_mic1_read_value);

    // Check the configuration
    if(decimation_inshift_value_mic0 != DEC_INSHIFT_VALUE_DEFAULT){

        ei_printf("DECIMATION_INSHIFT_VALUE     : %d\n", decimation_inshift_value_mic0);
        decimation_inshift_calculated_value = decimation_inshift_value_mic0;

    }
    else { // User did not define a custom decimation_inshift value, apply the offset.  Note the default
           // value for the offset is zero.  So we can safely apply this offset without any validation.  We'll
           // verify the final value below before applying them to the NDP120.

        ei_printf("DECIMATION_INSHIFT_OFFSET    : %d\n", decimation_inshift_offset);
        decimation_inshift_calculated_value = decimation_inshift_mic0_read_value + decimation_inshift_offset;

    }

    // Check for invalid low value
    if(decimation_inshift_calculated_value < DEC_INSHIFT_VALUE_MIN ){

        ei_printf("Warning calculated value %d is below the min allowed value of %d\n", decimation_inshift_calculated_value, DEC_INSHIFT_VALUE_MIN);
        decimation_inshift_calculated_value = DEC_INSHIFT_VALUE_MIN;
    }

    // Check for invalid low value
    else if(decimation_inshift_calculated_value > DEC_INSHIFT_VALUE_MAX){

        ei_printf("Warning calculated value %d is above max allowed value of %d\n", decimation_inshift_calculated_value, DEC_INSHIFT_VALUE_MAX);
        decimation_inshift_calculated_value = DEC_INSHIFT_VALUE_MAX;
    }

    ei_printf("Setting new value to %d\n", decimation_inshift_calculated_value);

    // Write the new value(s) into the NDP120
    ndp_core2_platform_tiny_audio_config_set(INSHIFT_AUDIO_ID, INSHIFT_SINGLE_MIC_ID, &decimation_inshift_calculated_value);
    ei_printf("Final Decimation Inshift Mic0: %d\n", decimation_inshift_calculated_value);


    // Only update the mic1 value if one was set in the model
    if(0 != decimation_inshift_mic1_read_value){
        ndp_core2_platform_tiny_audio_config_set(INSHIFT_AUDIO_ID, INSHIFT_DUAL_MIC_ID, &decimation_inshift_calculated_value);
        ei_printf("Final Decimation Inshift Mic1: %d\n", decimation_inshift_calculated_value);
    }

    ei_printf("\n");
}

/**
 *
 * @return
 */
static int bff_reinit_imu(void)
{
    int s;

    s = ndp_core2_platfom_tiny_gpio_release(MSPI_IMU_SSB);
    if (s) {
        ei_printf("release gpio%d failed: %d\n", MSPI_IMU_SSB, s);
        return s;
    }

    s = ndp_core2_platform_tiny_dsp_restart();
    if (s) {
        ei_printf("restart DSP failed: %d\n", s);
        return s;
    }
    vTaskDelay (pdMS_TO_TICKS(1000UL));

    return s;
}
