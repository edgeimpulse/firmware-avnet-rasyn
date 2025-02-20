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
#include "semphr.h"
#include "ei_inertial.h"
#include "ingestion-sdk-platform/rasyn/ei_device_rasyn.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "syntiant_platform.h"
#include "syntiant_common.h"
#include "ndp/ndp_irq_service.h"
#include "peripheral/usb/usb_pcdc_vcom.h"
#include "inference/ei_run_impulse.h"
#include "ndp/fat_load.h"

/* Constant ----------------------------------------------------------------- */
#define IMU_SCALE_DATA              (1)

#define CONVERT_G_TO_MS2            (9.80665f)
#define ACC_RAW_SCALING             (32767.5f)
#define ACC_SCALE_FACTOR            (2.0f*CONVERT_G_TO_MS2)/ACC_RAW_SCALING

#define CONVERT_ADC_GYR             (float)(250.0f/32768.0f)
#define NORMALIZE_GYR               (250.0f)

typedef struct {
    float* pdata;
    uint16_t sample_nr;
    uint16_t max_sample_nr;
}t_imu;

static void icm42670_extraction_cb(uint32_t sample_size, uint8_t *sensor_data, void *sensor_arg);

static void ndp_print_imu(void);
/* Public functions -------------------------------------------------------- */
/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_inertial_init(void)
{

    return true;
}

/**
 *
 */
int ei_fusion_inertial_setup_recording(bool start)
{
    int s = 0;

    if (start == true) {
        if (ei_run_impulse_is_active() == true) {
            ndp_irq_disable();
        }

        // disable pdm clk for confusion if audio enabled
        if (get_event_watch_mode() & WATCH_TYPE_AUDIO) {
            s = ndp_core2_platform_tiny_feature_set(NDP_CORE2_FEATURE_NONE);
            if (s){
                ei_printf("feature set 0x%x failed %d\r\n", NDP_CORE2_FEATURE_NONE, s);
            }
        }


        // enable sensor if sensor disable
        if (!(get_event_watch_mode() & WATCH_TYPE_MOTION)) {
            s = ndp_core2_platform_tiny_sensor_ctl(EI_FUSION_IMU_SENSOR_INDEX, 1);
            if (s) {
                ei_printf("enable sensor[%d] failed: %d\n", EI_FUSION_IMU_SENSOR_INDEX, s);
            }
        }

        s = ndp_core2_platform_tiny_config_interrupts(
                    NDP_CORE2_INTERRUPT_EXTRACT_READY, 1);
        if (s) {
            ei_printf("enable extract interrupt failed: %d\n", s);
            return s;
        }
    }
    else {
        s = ndp_core2_platform_tiny_config_interrupts(
                    NDP_CORE2_INTERRUPT_EXTRACT_READY, 0);
        if (s) {
            ei_printf("disable extract interrupt failed: %d\n", s);
            return s;
        }

        // enable pdm clk if audio enabled
        if (get_event_watch_mode() & WATCH_TYPE_AUDIO) {
            s = ndp_core2_platform_tiny_feature_set(NDP_CORE2_FEATURE_PDM);
            if (s){
                ei_printf("feature set 0x%x failed %d\r\n", NDP_CORE2_FEATURE_PDM, s);
            }
        }

        // disable sensor if sensor disable
        if (!(get_event_watch_mode() & WATCH_TYPE_MOTION)) {
            s = ndp_core2_platform_tiny_sensor_ctl(EI_FUSION_IMU_SENSOR_INDEX, 0);
            if (s) {
                ei_printf("disable sensor[%d] failed: %d\n", EI_FUSION_IMU_SENSOR_INDEX, s);
            }
        }

        if (ei_run_impulse_is_active() == true) {
            ndp_irq_enable();
        }
    }

    return s;
}

/**
 *
 * @param buffer
 * @param max_sample
 * @return
 */
uint32_t ei_inertial_read_data(float* buffer, uint32_t max_sample, uint32_t current_sample)
{
    t_imu my_imu = {.pdata = buffer, .sample_nr = 0, .max_sample_nr = max_sample};
    uint32_t save_sample_size;
    int max_num_frames;
    uint8_t data_ptr[1024] = {0};
    int s;

    s = ndp_core2_platform_tiny_get_sensor_sample_size(&save_sample_size);
    if (s) {
        return s;
    }

    while(max_sample > (my_imu.sample_nr/INERTIAL_AXIS_SAMPLED)) {

        s = ndp_core2_platform_tiny_sensor_extract_data(data_ptr,
                EI_FUSION_IMU_SENSOR_INDEX, save_sample_size, max_sample,
                (!current_sample)?1:0,
                icm42670_extraction_cb, &my_imu);

        if ((s) && (s != NDP_CORE2_ERROR_DATA_REREAD)) {
            printf("sensor extract data failed: %d\n", s);
            break;
        }
    }

#if defined(IMU_SCALE_DATA) && (IMU_SCALE_DATA == 1)
    uint16_t i;
    uint16_t j;

    for (j = 0; j < (my_imu.sample_nr/INERTIAL_AXIS_SAMPLED); j++) {
        for (i = 0; i < 3; i++) {
            buffer[(j * INERTIAL_AXIS_SAMPLED) + i] *= ACC_SCALE_FACTOR;
        }

        for (i = 3; i < INERTIAL_AXIS_SAMPLED; i++) {
            buffer[(j * INERTIAL_AXIS_SAMPLED) + i] *= CONVERT_ADC_GYR;
        }
    }
#endif

    return (my_imu.sample_nr);
}

/**
 *
 */
static void ndp_print_imu(void)
{
    uint8_t imu_val = 0;
    uint8_t reg = 0x75 | 0x80 ; /*WHO_AM_I*/

    /* set MSSB0/GPIO0 pin */
    //ndp_core2_platform_gpio_config(0, NDP_CORE2_CONFIG_VALUE_GPIO_DIR_OUT, 1);
    /* reset MSSB1/GPIO1 pin */
    //ndp_core2_platform_gpio_config(1, NDP_CORE2_CONFIG_VALUE_GPIO_DIR_OUT, 0);

    //ndp_core2_platform_tiny_mspi_config();
    ndp_core2_platform_tiny_mspi_write(1, 1, &reg, 0);
    ndp_core2_platform_tiny_mspi_read(1, 1, &imu_val, 1);
    ei_printf("Attched IMU ID = 0x%02x\n", imu_val); /*id = 0x67*/

}

/**
 *
 * @param sample_size BYTES received
 * @param sensor_data pointer to buffer where have been stores samples
 * @param sensor_arg
 */
static void icm42670_extraction_cb(uint32_t sample_size, uint8_t *sensor_data, void *sensor_arg)
{
    int16_t i;
    t_imu *cb_sensor_arg = (t_imu*)sensor_arg;
    int16_t *acc_samples = (int16_t *)(sensor_data);

    if (cb_sensor_arg->max_sample_nr > (cb_sensor_arg->sample_nr/INERTIAL_AXIS_SAMPLED)) {
        for (i = 0; i < (sample_size >> 1); i++) {
            cb_sensor_arg->pdata[cb_sensor_arg->sample_nr + i] = acc_samples[i];
        }

        cb_sensor_arg->sample_nr += (sample_size >> 1);
    }

}
