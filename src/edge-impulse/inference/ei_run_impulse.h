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

#ifndef EI_RUN_IMPULSE_H
#define EI_RUN_IMPULSE_H

#include "stdint.h"

extern void ei_run_nn_normal(void);
extern void ei_classification_output(uint8_t matched_feature_id, uint8_t class_idx, uint8_t sec_val);
extern bool ei_run_impulse_is_active(void);
extern void ei_start_stop_run_impulse(bool start);
extern void read_ndp_model(void);
extern void print_ndp_model(void);

#endif /* EI_RUN_IMPULSE_H */
