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
#include <imu_augmentor/utilities.h>
#include <imu_integration/utilities.h>
#include <msg_conversions/msg_conversions.h>

namespace imu_augmentor {
namespace ii = imu_integration;
namespace mc = msg_conversions;
void LoadImuAugmentorParams(config_reader::ConfigReader& config, ImuAugmentorParams& params) {
  ii::LoadImuIntegratorParams(config, params);
  params.standstill_enabled = mc::LoadBool(config, "imu_augmentor_standstill");
  params.use_constant_velocity_kalman_filter =
    mc::LoadBool(config, "imu_augmentor_use_constant_velocity_kalman_filter");
}
}  // namespace imu_augmentor
