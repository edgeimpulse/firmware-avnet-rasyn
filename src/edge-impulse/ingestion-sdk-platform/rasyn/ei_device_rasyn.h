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
