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
#include "ei_microphone.h"
#include "ingestion-sdk-platform/rasyn/ei_device_rasyn.h"
#include "firmware-sdk/sensor-aq/sensor_aq.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "ingestion-sdk-platform/rasyn/ei_sd_memory.h"
#include "syntiant_platform.h"
#include "peripheral/usb/usb_pcdc_vcom.h"
#include "ndp/fat_load.h"
#include "ndp/ndp_irq_service.h"
#include "inference/ei_run_impulse.h"

/* Constant ---------------------------------------------------------------- */

/* Types ***---------------------------------------------------------------- */

/** Status and control struct for inferencing struct */
typedef struct {
    int16_t     *buffers[2];
    uint8_t     buf_select;
    uint8_t     buf_ready;
    uint32_t    buf_count;
    uint32_t    n_samples;
} inference_t;

struct cb_audio_arg_s {
    uint32_t total_len_bytes;
};

/* Private functions ------------------------------------------------ */
static size_t ei_write(const void *buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM * stream);
static int ei_seek(EI_SENSOR_AQ_STREAM * stream, long int offset, int origin);
static int insert_ref(char *buffer, int hdrLength);
static bool create_header(sensor_aq_payload_info *payload);

static int ei_audio_record_operation(int isstart, uint32_t *sample_size);
void audio_extraction_cb (uint32_t extract_size, uint8_t *audio_data,  void *audio_arg);
static int ei_audio_record(uint32_t len);

/* Private variables ------------------------------------------------------- */
static uint8_t local_mic_stack[10 * 1024];
static uint32_t required_samples_size;
static uint32_t headerOffset = 0;
microphone_sample_t *sample_buffer_processed;

static uint32_t current_sample;
static bool sampling_finished;
static uint32_t required_samples;

/* hash stuff */
static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

/**
 * @brief Initialize the ingestion 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_start_sampling(void)
{
    EiRASyn* dev = static_cast<EiRASyn*>(EiDeviceInfo::get_device());
    EiSDMemory* mem = static_cast<EiSDMemory*>(dev->get_sdcard());
    uint32_t sample_size = 0;

    sensor_aq_payload_info payload = {
        dev->get_device_id().c_str(),
        dev->get_device_type().c_str(),
        dev->get_sample_interval_ms(),
        { { "audio", "wav" } }
    };

    if (motion_running() == CIRCULAR_MOTION_ENABLE) {
        ei_printf("ERR: error, microphone not available for ingestion\r\n");
        return false;
    }


    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float(dev->get_sample_interval_ms());
    ei_printf(" ms.\n");
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    required_samples = (uint32_t)((dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    /* Round to even number of samples for word align flash write */
    if(required_samples & 1) {
        required_samples++;
    }

    required_samples_size = required_samples * sizeof(microphone_sample_t);
    current_sample = 0;

    ei_printf("Starting in 2000 ms... (or until all flash was erased)\n");
    ei_sleep(2000); // no need to erase, we create a new file



    char filename[256];
    int fn_r = snprintf(filename, 256, "mic_%s.cbor", dev->get_sample_label().c_str());
    if (fn_r <= 0) {
        ei_printf("ERR: Failed to allocate file name\n");
        return false;
    }

    if (mem->open_sample_file(filename, true) == false) {
        ei_printf("Error in creating file\n");
        return false;
    }

    if (create_header(&payload) == false) {
        return false;
    }
    
    dev->set_state(EiRASynStateSampling);
    ei_printf("Sampling...\r\n");
    //comms_close();

    int s = ei_audio_record_operation(1, &sample_size);
    if (s) {
        ei_printf("Err: can't start mic recording\r\n");
        return false;
    }

    ei_audio_record(required_samples);  // samples required

    s = ei_audio_record_operation(0, NULL);

    if (get_print_console_type() == CONSOLE_USB_CDC) {
        //comms_open(1);
    }

    sampling_finished = false;

    uint8_t final_byte[] = {0xff};
    int ctx_err = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, final_byte, 1);
    if (ctx_err != 0) {
        ei_printf("Error in update %d\n", ctx_err);
        return ctx_err;
    }

    // finish the signing
    ctx_err = ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        ei_printf("Error in finish %d\n", ctx_err);
        return ctx_err;
    }

    mem->close_sample_file();
    dev->set_state(EiRASynStateIdle);

    ei_printf("Done sampling, total bytes collected: %lu\n", required_samples_size);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", required_samples_size + headerOffset);
    ei_printf("OK\n");

    return true;
}

/**
 * @brief 
 * 
 */
void ei_microphone_stop_stream(void)
{
    //
}

/**
 * @brief 
 * 
 */
static void mic_thread_function(void)
{

    sampling_finished = true;
}

/**
 * @brief Create a header object
 * 
 * @param payload 
 * @return true 
 * @return false 
 */
static bool create_header(sensor_aq_payload_info *payload)
{
    int ret;
    EiRASyn* dev = static_cast<EiRASyn*>(EiDeviceInfo::get_device());
    EiSDMemory* mem = static_cast<EiSDMemory*>(dev->get_sdcard());

    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    ret = sensor_aq_init(&ei_mic_ctx, payload, NULL, true);

    if (ret != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", ret);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix != 0; ix--) {
        if (((uint8_t *)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    int ref_size = insert_ref(((char*)ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), end_of_header_ix);
    // and update the signature
    ret = ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)(ei_mic_ctx.cbor_buffer.ptr + end_of_header_ix), ref_size);
    if (ret != 0) {
        ei_printf("Failed to update signature from header (%d)\n", ret);
        return false;
    }
    end_of_header_ix += ref_size;

    // Write to blockdevice
    ei_printf("Trying to write %ld byte\r\n", end_of_header_ix);
    ret = mem->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    if ((size_t)ret != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", ret);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}

/**
 *
 * @param buffer
 * @param hdrLength
 * @return
 */
static int insert_ref(char *buffer, int hdrLength)
{
    #define EXTRA_BYTES(a)  ((a & 0x3) ? 4 - (a & 0x3) : (a & 0x03))

    const char *ref = "Ref-BINARY-i16";
    int addLength = 0;
    int padding = EXTRA_BYTES(hdrLength);

    buffer[addLength++] = 0x60 + 14 + padding;
    for(unsigned int i = 0; i < strlen(ref); i++) {
        buffer[addLength++] = *(ref + i);
    }
    for(int i = 0; i < padding; i++) {
        buffer[addLength++] = ' ';
    }

    buffer[addLength++] = 0xFF;

    return addLength;
}

/* Dummy functions for sensor_aq_ctx type */
/**
 *
 * @param
 * @param size
 * @param count
 * @param
 * @return
 */
static size_t ei_write(const void* buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM* stream)
{
    (void)buffer;
    (void)size;
    (void)stream;

    return count;
}

/**
 *
 * @param
 * @param offset
 * @param origin
 * @return
 */
static int ei_seek(EI_SENSOR_AQ_STREAM* stream, long int offset, int origin)
{
    (void)stream;
    (void)offset;
    (void)origin;

    return 0;
}

/**
 *
 * @param isstart
 * @param sample_size
 * @return
 */
static int ei_audio_record_operation(int isstart, uint32_t *sample_size)
{
    int s = 0;

    if (isstart) {
        if (ei_run_impulse_is_active() == true) {
            ndp_irq_disable();

            if (motion_running() == CIRCULAR_MOTION_ENABLE) {
                s = ndp_core2_platform_tiny_feature_set(NDP_CORE2_FEATURE_PDM);
                if (s){
                    ei_printf("ndp_core2_platform_tiny_feature_set set 0x%x failed %d\r\n",
                        NDP_CORE2_FEATURE_PDM, s);
                }
            }
        }
    }
    else {
        if (ei_run_impulse_is_active() == true) {
            if (motion_running() == CIRCULAR_MOTION_ENABLE) {
            s = ndp_core2_platform_tiny_feature_set(NDP_CORE2_FEATURE_NONE);
                if (s){
                    ei_printf("ndp_core2_platform_tiny_feature_set set 0x%x failed %d\r\n",
                                NDP_CORE2_FEATURE_NONE, s);
                }
            }

            ndp_irq_enable();
        }
    }

    return s;
}

/**
 *
 * @param len
 * @return
 */
static int ei_audio_record(uint32_t len)
{
    struct cb_audio_arg_s audio_cb_data {.total_len_bytes = 0,};
    int s = 0;
    uint32_t sample_size;
    uint32_t sample_bytes = ndp_core2_platform_tiny_get_samplebytes();
    uint32_t collected = 0;

    /* sample ready interrupt is enabled in MCU firmware */
    s = ndp_core2_platform_tiny_get_recording_metadata(&sample_size, NDP_CORE2_GET_FROM_MCU);
    if (s) {
        ei_printf("audio record get metadata from mcu with notify failed: %d\n", s);
        return s;
    }

    while (len > (audio_cb_data.total_len_bytes >> 1)) {
        s = ndp_core2_platform_tiny_notify_extract_data(local_mic_stack,
                sample_size, audio_extraction_cb, &audio_cb_data);

        if ((s) && (s != NDP_CORE2_ERROR_DATA_REREAD)) {
            ei_printf("audio extract data failed: %d\n", s);
            break;
        }
        else if (s == 0) {
            collected += audio_cb_data.total_len_bytes;
            //ei_printf("Total collected  %d of %d\r\n", collected, len);
        }

        //ei_sleep(2);
    }
    s = ndp_core2_platform_tiny_config_interrupts(NDP_CORE2_INTERRUPT_EXTRACT_READY, 0);
    /* sample ready interrupt is enabled in MCU firmware */
    if (s) {
        ei_printf("ndp_core2_platform_tiny_config_interrupts failed: %d\n", s);
        return s;
    }

    return s;
}

/**
 *
 * @param extract_size
 * @param audio_data
 * @param audio_arg
 */
void audio_extraction_cb (uint32_t extract_size, uint8_t *audio_data,  void *audio_arg)
{
    EiRASyn* dev = static_cast<EiRASyn*>(EiDeviceInfo::get_device());
    EiSDMemory* mem = static_cast<EiSDMemory*>(dev->get_sdcard());

    struct cb_audio_arg_s *cb_audio_arg = (struct cb_audio_arg_s*)audio_arg;
    int16_t *pread_data = (int16_t*)audio_data;

    if (extract_size > 0) {
        xSemaphoreTake(g_ndp_mutex, portMAX_DELAY);
        cb_audio_arg->total_len_bytes += extract_size;
        mem->write_sample_data(audio_data, headerOffset + extract_size, extract_size);

        //update data hash
        ei_mic_ctx.signature_ctx->update(ei_mic_ctx.signature_ctx, (uint8_t*)local_mic_stack, extract_size);
        xSemaphoreGive(g_ndp_mutex);
    }

    // queue ?
}
