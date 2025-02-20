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

#ifndef EI_DEVICE_RASYN_H_
#define EI_DEVICE_RASYN_H_

#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"

/** Sensors */
typedef enum
{
    MICROPHONE      = 0,
    IMU             = 1,
    E_MAX_SENSOR    = 2
}used_sensors_t;

typedef enum
{
    e_enabled_mic,
    e_enabled_imu,
    e_enabled_none
}enabled_sensor_t;

typedef enum
{
    EiRASynStateIdle            = eiStateIdle,
    EiRASynStateErasingFlash    = eiStateErasingFlash,
    EiRASynStateSampling        = eiStateSampling,
    EiRASynStateUploading       = eiStateUploading,
    EiRASynStateFinished        = eiStateFinished,
    EiRASynStateMatch           = eiStateFinished + 1,
} EiRASynState;

/** Baud rates */

class EiRASyn : public EiDeviceInfo {
private:
    EiRASyn() = delete;
    ei_device_sensor_t sensors[E_MAX_SENSOR];
    EiRASynState state;
    EiDeviceMemory *data_flash;
    EiDeviceMemory *sd_card;

    bool is_sampling;
    void (*sample_read_callback)(void);

    enabled_sensor_t sensor_in_use;
    bool imu_ok;

public:
    EiRASyn(EiDeviceMemory* code_flash, EiDeviceMemory* data_flash_to_set, EiDeviceMemory *sd_card_to_set);
    ~EiRASyn();
    void init_device_id(void);
    void load_config(void) override;
    bool save_config(void) override;
    void clear_config(void);
    uint32_t get_data_output_baudrate(void) override;

    bool start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms) override;
    bool stop_sample_thread(void) override;
    void sample_thread(void);

    void set_state(EiRASynState new_state);
    EiRASynState get_state(void);

    bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size) override;

    bool test_flash(void);

    void set_default_data_output_baudrate(void) override;
    void set_max_data_output_baudrate(void) override;
    EiDeviceMemory* get_sdcard(void) {return sd_card;};

    enabled_sensor_t get_used_sensor(void) {return sensor_in_use;};
    void set_used_sensor(enabled_sensor_t in_use) {sensor_in_use = in_use;};

    void test_sd_card(void);
    void init_sd_card(void);
    bool get_is_sampling(void) {return is_sampling;};
    float get_sample_interval_ms(void) {return sample_interval_ms;};
    bool get_imu_ok(void);
    void set_imu_ok(bool new_set);
};

#endif /* EI_DEVICE_TEMPLATE_H_ */
