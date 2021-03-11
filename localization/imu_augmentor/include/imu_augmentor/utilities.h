/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */
#ifndef IMU_AUGMENTOR_UTILITIES_H_
#define IMU_AUGMENTOR_UTILITIES_H_

#include <config_reader/config_reader.h>
#include <imu_augmentor/imu_augmentor_params.h>
#include <imu_augmentor/utilities.h>

namespace imu_augmentor {
void LoadImuAugmentorParams(config_reader::ConfigReader& config, ImuAugmentorParams& params);
}  // namespace imu_augmentor

#endif  // IMU_AUGMENTOR_UTILITIES_H_
