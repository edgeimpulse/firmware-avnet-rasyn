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

#include "ei_run_impulse.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "model-parameters/model_variables.h"
#include "ndp/fat_load.h"
#include "ndp/ndp_flash.h"
#include "ndp/ndp_irq_service.h"
#include "syntiant_platform.h"

static bool _run_impulse = false;

#define SYNTIANT_NDP120_MAX_CLASSES     32
#define SYNTIANT_NDP120_MAX_NNETWORKS   4
#define NDP120_MCU_LABELS_MAX_LEN       (0x200)

static char *labels[SYNTIANT_NDP120_MAX_CLASSES];
static char *labels_per_network[SYNTIANT_NDP120_MAX_NNETWORKS]
            [SYNTIANT_NDP120_MAX_CLASSES];
static char label_data[NDP120_MCU_LABELS_MAX_LEN] = "";

static uint16_t num_labels;

/**
 * @brief      Start impulse, print settings
 */
void ei_run_nn_normal(void)
{
    if (_run_impulse == false) {
        ei_start_stop_run_impulse(true);
        // summary of inferencing settings (from model_metadata.h)
        ei_printf("Inferencing settings:\n");
        ei_printf("\tInterval: ");
        ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
        ei_printf(" ms.\n");
        ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
        ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
        print_ndp_model();

        ei_printf("Starting inferencing, press 'b' to break\n");
    }
}

/**
 * @brief      Called from the ndp120 read out. Print classification output
 *             and send matched output string to user callback
 *
 * @param[in]  matched_feature
 */
void ei_classification_output(uint8_t matched_feature_id, uint8_t class_idx, uint8_t sec_val)
{
    if (_run_impulse == true) {

        ei_printf("NDP MATCH!!! -- [%d:%d]:%s %s sec-val\n\n",
                matched_feature_id, class_idx, labels_per_network[matched_feature_id][class_idx],
                            (sec_val>0)?"with":"without");

    }
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool ei_run_impulse_is_active(void)
{
    return _run_impulse;
}

/**
 * @brief
 *
 * @param start
 */
void ei_start_stop_run_impulse(bool start)
{
    if ((_run_impulse != start) && (!start)) {

        // stop
        ndp_irq_disable();

        _run_impulse = start;
        ei_printf("Inferencing stopped by user\r\n");
    }
    else if ((_run_impulse != start) && (start)) {
        _run_impulse = true;

        ndp_core2_platform_tiny_config_interrupts(NDP_CORE2_INTERRUPT_EXTRACT_READY, 0);
        // start
        ndp_irq_enable();
    }
}

void read_ndp_model(void)
{
    int s, total_nn, total_labels;
    int j, class_num, nn_num, prev_nn_num, labels_len;
    char *label_string;

    s = ndp_core2_platform_tiny_get_info(&total_nn, &total_labels,
            label_data, &labels_len);
    if (s) {
        ei_printf("ndp_core2_platform_tiny_get_info error! retval %d\r\n", s);
        return;
    }

#if NDP_DEBUG == 1
    ei_printf("ndp120 has %d network and %d labels loaded\n", total_nn, total_labels);
#endif

    /* get pointers to the labels */
    num_labels = 0;
    j = 0;

    /* labels_len is 4 byte aligned. We continue processing
       labels until the running sum of label characters
       processed is within 3 bytes of labels_len */
    while ((labels_len - j > 3) &&
            (num_labels < SYNTIANT_NDP120_MAX_CLASSES)) {
        labels[num_labels] = &label_data[j];
        (num_labels)++;
        for (; label_data[j]; j++)
            ;
        j++;
    }

    /* build an array that hold all labels based on network number */
    class_num = 0;
    nn_num = 0;
    prev_nn_num = 0;

    //ei_printf("\nFrom model total labels: %d\n", num_labels);
    for (j = 0; j < num_labels; j++) {
        label_string = labels[j];
        nn_num = *(label_string + 2) - '0';
        if (nn_num < 0 || nn_num >= SYNTIANT_NDP120_MAX_NNETWORKS) {
#if NDP_DEBUG == 1
            s = SYNTIANT_NDP_ERROR_INVALID_NETWORK;
            ei_printf("NDP invalid network\r\n");
#endif
            return;
        }
        if (nn_num != prev_nn_num) {
            class_num = 0;
        }

        labels_per_network[nn_num][class_num++] = label_string;
        //numlabels_per_network[nn_num] = class_num;
        prev_nn_num = nn_num;
    }
}

/**
 *
 */
void print_ndp_model(void)
{
    uint16_t i;

    ei_printf("From model total labels: %d\n", num_labels);
    ei_printf("Classes:\n");
    for (i = 0; i < num_labels; i++) {
        ei_printf("\tlabel %d: %s\r\n", i, labels[i]);
    }

}
