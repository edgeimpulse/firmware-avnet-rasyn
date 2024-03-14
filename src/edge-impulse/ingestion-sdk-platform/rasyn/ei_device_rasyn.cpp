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
#include "queue.h"
#include <edge-impulse/ingestion-sdk-platform/rasyn/ei_device_rasyn.h>
#include <edge-impulse/ingestion-sdk-platform/rasyn/ei_flash_memory.h>
#include <edge-impulse/ingestion-sdk-platform/rasyn/ei_sd_memory.h>
#include <edge-impulse/ingestion-sdk-platform/sensor/ei_microphone.h>
#include <string>

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"
#include "bsp_api.h"
#include "ei_sd_memory.h"
#include "edge-impulse/ingestion-sdk/ei_sampler.h"
#include "common_data.h"
#include "ndp/fat_load.h"
#include "peripheral/console.h"

//#define MAX_UART_BAUD_RATE  (460800u)
#define MAX_UART_BAUD_RATE  (115200u)
#define MAX_USB_BAUD_RATE   (36846400u)

/******
 *
 * @brief EdgeImpulse Device structure and information
 *
 ******/

EiRASyn::EiRASyn(EiDeviceMemory* code_flash, EiDeviceMemory* data_flash_to_set, EiDeviceMemory *sd_card_to_set)
{
    EiDeviceInfo::memory = code_flash;
    EiRASyn::data_flash = data_flash_to_set;
    EiRASyn::sd_card = sd_card_to_set;

    init_device_id();

    load_config();

    device_type = "RASYN_BOARD";
    state = EiRASynStateIdle;
    
    sensors[MICROPHONE].name = "Microphone";
    sensors[MICROPHONE].frequencies[0] = 16000.0f;
    sensors[MICROPHONE].start_sampling_cb = &ei_microphone_start_sampling;
    sensors[MICROPHONE].max_sample_length_s = 10;    // fixed for now

    sensors[IMU].name = "Inertial";
    sensors[IMU].frequencies[0] = 200.0f;
    sensors[IMU].start_sampling_cb = &ei_imu_start_sampling;
    sensors[IMU].max_sample_length_s = 20;    // fixed for now

    sensor_in_use = e_enabled_none;

    imu_ok = false;
}

bool EiRASyn::get_imu_ok(void)
{
    return imu_ok;
}

void EiRASyn::set_imu_ok(bool new_set)
{
    imu_ok = new_set;
}

EiRASyn::~EiRASyn()
{

}

/**
 *
 * @return
 */
EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    /* Initializing EdgeImpulse classes here in order for
     * Flash memory to be initialized before mainloop start
     */
    static EiFlashMemory data_memory(e_flash_data, sizeof(EiConfig));                  /* code flash doesn't store config !*/
    static EiFlashMemory code_memory(e_flash_code, sizeof(EiConfig));
    static EiSDMemory sd_card_to_set;

    static EiRASyn dev(&code_memory, &data_memory, &sd_card_to_set);

    return &dev;
}

void EiRASyn::init_device_id(void)
{
    const bsp_unique_id_t *pdev_id;
    char temp[20];

    pdev_id = R_BSP_UniqueIdGet();

    snprintf(temp, sizeof(temp), "%02x:%02x:%02x:%02x:%02x:%02x",
            pdev_id->unique_id_bytes[5],
            pdev_id->unique_id_bytes[4],
            pdev_id->unique_id_bytes[3],
            pdev_id->unique_id_bytes[2],
            pdev_id->unique_id_bytes[1],
            pdev_id->unique_id_bytes[0]);

    device_id = std::string(temp);
}

void EiRASyn::load_config(void)
{
    EiConfig buf;

    memset(&buf, 0, sizeof(EiConfig));
    data_flash->load_config((uint8_t *)&buf, sizeof(EiConfig)); /* load from data flash */

    if (buf.magic == 0xdeadbeef)
    {
        wifi_ssid = std::string(buf.wifi_ssid, 128);
        wifi_password = std::string(buf.wifi_password, 128);
        wifi_security = buf.wifi_security;
        sample_interval_ms = buf.sample_interval_ms;
        sample_length_ms = buf.sample_length_ms;
        sample_label = std::string(buf.sample_label, 128);
        sample_hmac_key = std::string(buf.sample_hmac_key, 33);
        upload_host = std::string(buf.upload_host, 128);
        upload_path = std::string(buf.upload_path, 128);
        upload_api_key = std::string(buf.upload_api_key, 128);
        management_url = std::string(buf.mgmt_url, 128);
    }
}

bool EiRASyn::save_config(void)
{
    EiConfig buf;

    memset(&buf, 0, sizeof(EiConfig));

    strncpy(buf.wifi_ssid, wifi_ssid.c_str(), 128);
    strncpy(buf.wifi_password, wifi_password.c_str(), 128);
    buf.wifi_security = wifi_security;
    buf.sample_interval_ms = sample_interval_ms;
    buf.sample_length_ms = sample_length_ms;
    strncpy(buf.sample_label, sample_label.c_str(), 128);
    strncpy(buf.sample_hmac_key, sample_hmac_key.c_str(), 33);
    strncpy(buf.upload_host, upload_host.c_str(), 128);
    strncpy(buf.upload_path, upload_path.c_str(), 128);
    strncpy(buf.upload_api_key, upload_api_key.c_str(), 128);
    strncpy(buf.mgmt_url, management_url.c_str(), 128);
    buf.magic = 0xdeadbeef;

    return data_flash->save_config((uint8_t *)&buf, sizeof(EiConfig)); /* save config in data flash memory */
}

/**
 *
 */
void EiRASyn::clear_config(void)
{
    EiDeviceInfo::clear_config();

    init_device_id();
    save_config();
}

/**
 *
 * @return
 */
uint32_t EiRASyn::get_data_output_baudrate(void)
{
    if (get_print_console_type() == CONSOLE_USB_CDC) {
        return MAX_USB_BAUD_RATE;
    }
    return MAX_UART_BAUD_RATE;
}

/**
 *
 * @param sample_read_cb
 * @param sample_interval_ms
 * @return
 */
bool EiRASyn::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    this->sample_read_callback = sample_read_cb;
    this->sample_interval_ms = sample_interval_ms;
    this->is_sampling = true;

    return true;
}

/**
 *
 * @return
 */
bool EiRASyn::stop_sample_thread(void)
{
    this->is_sampling = false;
    return true;
}

/**
 *
 */
void EiRASyn::sample_thread(void)
{
    if ((sample_read_callback != nullptr) && (is_sampling = true)) {
        sample_read_callback();
    }

}

/**
 *
 * @param state
 */
void EiRASyn::set_state(EiRASynState new_state)
{
    static EiRASynState local_state;

    this->state = (EiRASynState)new_state;
    local_state = new_state;
    xQueueSend(g_new_state_queue, &local_state, 0);
}

/**
 *
 * @return
 */
EiRASynState EiRASyn::get_state(void)
{
    return this->state;
}

/**
 *
 * @param sensor_list
 * @param sensor_list_size
 * @return
 */
bool EiRASyn::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    *sensor_list      = sensors;
    *sensor_list_size = (size_t)E_MAX_SENSOR;

    return false;
}

/**
 *
 */
void EiRASyn::set_default_data_output_baudrate(void)
{
    if (get_print_console_type() == CONSOLE_UART) {
        uart_set_baud(false);
    }
    // usb do nothing
}

/**
 *
 */
void EiRASyn::set_max_data_output_baudrate(void)
{
    if (get_print_console_type() == CONSOLE_UART) {
        uart_set_baud(true);
    }
    // usb do nothing
}

/**
 *
 */
void EiRASyn::test_sd_card(void)
{
    EiSDMemory *test_sd_instance = static_cast<EiSDMemory*>(sd_card);

    test_sd_instance->test();
}

/**
 *
 */
void EiRASyn::init_sd_card(void)
{
    EiSDMemory *sd_instance = static_cast<EiSDMemory*>(sd_card);

    sd_instance->init();
}
