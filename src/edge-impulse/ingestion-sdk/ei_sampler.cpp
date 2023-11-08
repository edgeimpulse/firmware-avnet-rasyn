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
#include "ei_sampler.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "firmware-sdk/sensor-aq/sensor_aq.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ingestion-sdk-platform/rasyn/ei_device_rasyn.h"
#include "ingestion-sdk-platform/rasyn/ei_sd_memory.h"
#include "ingestion-sdk-platform/sensor/ei_inertial.h"
#include "bsp_api.h"

/* Private function prototypes --------------------------------------------- */
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght);
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *);
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin);
static bool create_header(sensor_aq_payload_info *payload);
static void ei_write_last_data(void);

/* Private variables ------------------------------------------------------- */
static uint32_t samples_required;
static uint32_t current_sample;
static uint32_t headerOffset = 0;
EI_SENSOR_AQ_STREAM stream;

static uint8_t write_word_buf[4];
static int write_addr = 0;
static unsigned char ei_sensor_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_sensor_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_sensor_hs_ctx;
static sensor_aq_ctx ei_sensor_ctx = {
    { ei_sensor_ctx_buffer, 1024 },
    &ei_sensor_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

bool ei_sampler_start_sampling(void *v_ptr_payload, starter_callback ei_sample_start, uint32_t sample_size)
{
    // no fusion used!
    return false;
}

/**
 * @brief      Setup and start sampling, write CBOR header to flash
 *
 * @param      v_ptr_payload  sensor_aq_payload_info pointer hidden as void
 * @param[in]  sample_size    Number of bytes for 1 sample (include all axis)
 *
 * @return     true if successful
 */
bool ei_imu_start_sampling(void)
{
    EiRASyn* dev = static_cast<EiRASyn*>(EiDeviceInfo::get_device());
    EiSDMemory* mem = static_cast<EiSDMemory*>(dev->get_sdcard());

    sensor_aq_payload_info payload = {
        dev->get_device_id().c_str(),
        dev->get_device_type().c_str(),
        dev->get_sample_interval_ms(),
#if INERTIAL_AXIS_SAMPLED == 6
        { {"accX", "m/s2"}, {"accY", "m/s2"}, {"accZ", "m/s2"}, {"gyrX", "dps"}, {"gyrY", "dps"}, {"gyrZ", "dps"}}
#elif INERTIAL_AXIS_SAMPLED == 3
        { {"accX", "m/s2"}, {"accY", "m/s2"}, {"accZ", "m/s2"}}
#else
#error "INERTIAL_AXIS_SAMPLED should b 3 or 6!"
#endif
    };

    if (dev->get_imu_ok() == false) {
        ei_printf("ERR: error, IMU not avialable for ingestion\r\n");
        return false;
    }

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float((float)dev->get_sample_interval_ms());
    ei_printf(" ms.\n");
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", (dev->get_sample_label().c_str()));
    ei_printf("\tHMAC Key: %s\n", (dev->get_sample_hmac_key().c_str()));
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    samples_required = (uint32_t)(dev->get_sample_length_ms()/dev->get_sample_interval_ms());

    ei_printf("samples_required %d\n", samples_required);

    current_sample = 0;

    ei_printf("Starting in 2000 ms... (or until all flash was erased)\n");

    ei_sleep(2000);
    dev->set_state(EiRASynStateSampling);

    char filename[100];
    int fn_r = snprintf(filename, 100, "imu_%s.cbor", dev->get_sample_label().c_str());
    if (fn_r <= 0) {
        ei_printf("ERR: Failed to allocate file name\n");
        return false;
    }

    if (mem->open_sample_file(filename, true) == false) {
        ei_printf("Error in creating file\n");
        return false;
    }
    if (create_header(&payload) == false) {
        ei_printf("Error in create_header\n");
        return false;
    }

    // not sure if here or in dev->start_thread ?
    if (ei_fusion_inertial_setup_recording(true) != 0) {
        ei_printf("Error in enabling the sensor\r\n");
        return false;
    }

    ei_printf("Sampling...\n");

    const uint16_t max_sample = 10;
    uint32_t sample_read = 0;
    float buffer[INERTIAL_AXIS_SAMPLED * 10];

    while(current_sample < samples_required) {
        sample_read = ei_inertial_read_data(buffer, max_sample);
        sample_data_callback((const void *)buffer, sample_read*sizeof(float));
        //wait on mutex done recording ?
        ei_sleep(10);
    };

    ei_fusion_inertial_setup_recording(false);

    ei_write_last_data();

    write_addr++;
    uint8_t final_byte[] = {0xff};

    int ctx_err = ei_sensor_ctx.signature_ctx->update(ei_sensor_ctx.signature_ctx, final_byte, 1);
    if (ctx_err != 0) {
        ei_printf("Error in update %d\n", ctx_err);
        return ctx_err;
    }

    // finish the signing
    ctx_err = ei_sensor_ctx.signature_ctx->finish(ei_sensor_ctx.signature_ctx, ei_sensor_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        ei_printf("Error in finish %d\n", ctx_err);
        return ctx_err;
    }

    mem->close_sample_file();
    dev->set_state(EiRASynStateIdle);

    ei_printf("Done sampling, total bytes collected: %lu\n", samples_required);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", write_addr + headerOffset);
    ei_printf("OK\n");

    return true;
}

/**
 * @brief      Write samples to FLASH in CBOR format
 *
 * @param[in]  sample_buf  The sample buffer
 * @param[in]  byteLenght  The byte lenght
 *
 * @return     true if all required samples are received. Caller should stop sampling,
 */
static bool sample_data_callback(const void *sample_buf, uint32_t byteLenght)
{
    int retval = 0;
    float* fp = (float*)sample_buf;

    if (byteLenght != 0) {
        uint32_t step = byteLenght/(sizeof(float) * INERTIAL_AXIS_SAMPLED);

        for (int i = 0; i < step ; i++ ) {
            retval = sensor_aq_add_data(&ei_sensor_ctx, (float *)&fp[i * INERTIAL_AXIS_SAMPLED], INERTIAL_AXIS_SAMPLED);
            if (retval) {
                ei_printf("sensor_aq_add_data error %d\r\n", retval);
            }
            current_sample++;
        }
    }

    if(current_sample >= samples_required) {
        return true;
    }
    else {
        return false;
    }
}

/**
 * @brief      Write sample data to FLASH
 * @details    Write size is always 4 bytes to keep alignment
 *
 * @param[in]  buffer     The buffer
 * @param[in]  size       The size
 * @param[in]  count      The count
 * @param      EI_SENSOR_AQ_STREAM file pointer (not used)
 *
 * @return     number of bytes handled
 */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM *)
{
    EiRASyn* dev = static_cast<EiRASyn*>(EiDeviceInfo::get_device());
    EiSDMemory* mem = static_cast<EiSDMemory*>(dev->get_sdcard());
    (void)size;

    for (size_t i = 0; i < count; i++) {
        write_word_buf[write_addr & 0x3] = *((char *)buffer + i);

        if ((++write_addr & 0x03) == 0x00) {
            mem->write_sample_data(write_word_buf, (write_addr - 4) + headerOffset, 4);
        }
    }

    return count;
}

/**
 * @brief      File handle seed function. Not used
 */
static int ei_seek(EI_SENSOR_AQ_STREAM *, long int offset, int origin)
{
    (void)origin;
    (void)offset;

    return 0;
}

/**
 * @brief      Create and write the CBOR header to FLASH
 *
 * @param      payload  The payload
 *
 * @return     True on success
 */
static bool create_header(sensor_aq_payload_info *payload)
{
    EiRASyn* dev = static_cast<EiRASyn*>(EiDeviceInfo::get_device());
    EiSDMemory* mem = static_cast<EiSDMemory*>(dev->get_sdcard());

    sensor_aq_init_mbedtls_hs256_context(&ei_sensor_signing_ctx, &ei_sensor_hs_ctx, dev->get_sample_hmac_key().c_str());

    int tr = sensor_aq_init(&ei_sensor_ctx, payload, NULL, true);

    if (tr != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", tr);
        return false;
    }
    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_sensor_ctx.cbor_buffer.len - 1; ix != 0; ix--) {
        if (((uint8_t *)ei_sensor_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    // Write to blockdevice
    tr = mem->write_sample_data((uint8_t*)ei_sensor_ctx.cbor_buffer.ptr, 0, end_of_header_ix);

    if (tr != (int)end_of_header_ix) {
//        end_of_header_ix = tr;  // we can write block of 128. so could be they are different.
        ei_printf("Failed to write to header blockdevice (%d)\n", tr);
        return false;
    }

    ei_sensor_ctx.stream = &stream;

    headerOffset = end_of_header_ix;
    write_addr = 0;

    return true;
}

/**
 * @brief      Write out remaining data in word buffer to FLASH.
 *             And append CBOR end character.
 */
static void ei_write_last_data(void)
{
    EiRASyn* dev = static_cast<EiRASyn*>(EiDeviceInfo::get_device());
    EiSDMemory* mem = static_cast<EiSDMemory*>(dev->get_sdcard());
    uint8_t fill = ((uint8_t)write_addr & 0x03);
    uint8_t insert_end_address = 0;

    if (fill != 0x00) {
        for (uint8_t i = fill; i < 4; i++) {
            write_word_buf[i] = 0xFF;
        }

        mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset, 4);
        insert_end_address = 4;
    }

    /* Write appending word for end character */
    for (uint8_t i=0; i < 4; i++) {
        write_word_buf[i] = 0xFF;
    }
    mem->write_sample_data(write_word_buf, (write_addr & ~0x03) + headerOffset + insert_end_address, 4);
}
