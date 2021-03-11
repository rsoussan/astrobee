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

#ifndef IMU_AUGMENTOR_CONSTANT_VELOCITY_KALMAN_FILTER_UTILITIES_H_
#define IMU_AUGMENTOR_CONSTANT_VELOCITY_KALMAN_FILTER_UTILITIES_H_

#include <imu_augmentor/kalman_filter_utilities.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_common/time.h>

#include <gtsam/Vector.h>
#include <gtsam/Matrix.h>

#include <utility>

namespace imu_augmentor {
std::pair<gtsam::Vector6, gtsam::Matrix6> ConstantVelocityKalmanFilterStateAndCovariances(
  const lc::CombinedNavState& combined_nav_state,
  const lc::CombinedNavStateCovariances& combined_nav_state_covariances);

std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances> ConstantVelocityKalmanFilterEstimate(
  const lc::CombinedNavState& start_combined_nav_state,
  const lc::CombinedNavStateCovariances& start_combined_nav_state_covariances,
  const lc::CombinedNavState& predicted_combined_nav_state,
  const lc::CombinedNavStateCovariances& predicted_combined_nav_state_covariances);

std::pair<gtsam::Vector6, gtsam::Matrix6> ConstantVelocityKalmanFilterEstimate(const gtsam::Vector6& x,
                                                                               const gtsam::Matrix6& P,
                                                                               const gtsam::Vector6& z,
                                                                               const gtsam::Matrix6& R,
                                                                               const lc::Time dt);
}  // namespace imu_augmentor

#endif  // IMU_AUGMENTOR_CONSTANT_VELOCITY_KALMAN_FILTER_UTILITIES_H_
