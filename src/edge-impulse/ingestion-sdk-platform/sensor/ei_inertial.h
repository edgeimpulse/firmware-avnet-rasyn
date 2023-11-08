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
#ifndef INGESTION_SDK_PLATFORM_SENSORS_EI_INERTIAL_SENSOR_H_
#define INGESTION_SDK_PLATFORM_SENSORS_EI_INERTIAL_SENSOR_H_

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_fusion.h"

#define   EI_FUSION_IMU_SENSOR_INDEX         0

/** Number of axis used and sample data format */
#define INERTIAL_AXIS_SAMPLED       6

/* Function prototypes ----------------------------------------------------- */
extern bool ei_inertial_init(void);
extern int ei_fusion_inertial_setup_recording(bool start);
extern void ei_fusion_inertial_test(void);
extern uint32_t ei_inertial_read_data(float* buffer, uint32_t max_sample);

#endif /* INGESTION_SDK_PLATFORM_SENSORS_EI_INERTIAL_SENSOR_H_ */
