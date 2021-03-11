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

#include <imu_augmentor/constant_velocity_kalman_filter_utilities.h>
#include <imu_augmentor/kalman_filter_utilities.h>
#include <imu_integration/utilities.h>
#include <localization_common/logger.h>

namespace imu_augmentor {
namespace lc = localization_common;
std::pair<gtsam::Vector6, gtsam::Matrix6> ConstantVelocityKalmanFilterStateAndCovariances(
  const lc::CombinedNavState& combined_nav_state,
  const lc::CombinedNavStateCovariances& combined_nav_state_covariances) {
  gtsam::Vector6 state;
  state << combined_nav_state.pose().position() << combined_nav_state.velocity();
  // TODO(rsoussan): create cov matrix!!!! (AAAAAA)
  return {state, covariances};
}

lc::CombinedNavState ConstantVelocityKalmanFilterEstimate(
  const lc::CombinedNavState& start_combined_nav_state,
  const lc::CombinedNavStateCovariances& start_combined_nav_state_covariances,
  const lc::CombinedNavState& predicted_combined_nav_state,
  const lc::CombinedNavStateCovariances& predicted_combined_nav_state_covariances) {
  const auto start_state_and_covariances =
    ConstantVelocityKalmanFilterStateAndCovariances(start_combined_nav_state, start_combined_nav_state_covariances);
  const auto predicted_state_and_covariances = ConstantVelocityKalmanFilterStateAndCovariances(
    predicted_combined_nav_state, predicted_combined_nav_state_covariances);
  const lc::Time dt = predicted_combined_nav_state.timestamp() - start_combined_nav_state.timestamp();
  const auto estimated_state_and_covariances = ConstantVelocityKalmanFilterEstimate(
    start_state_and_covariances.first, start_state_and_covariances.second, predicted_state_and_covariances.first,
    predicted_state_and_covariances.second, dt);
  // todo: return result as combined nav state and covs
}

std::pair<gtsam::Vector6, gtsam::Matrix6> ConstantVelocityKalmanFilterEstimate(const gtsam::Vector6& x,
                                                                               const gtsam::Matrix6& P,
                                                                               const gtsam::Vector6& z,
                                                                               const gtsam::Matrix6& R,
                                                                               const lc::Time dt) {
  // Constant Velocity Transition Matrix
  gtsam::Matrix6 F = gtsam::Matrix6::Identity();
  // todo: check this!!
  F.block<3, 3>(3, 0) = dt * gtsam::Matrix3::Identity();
  // TODO(rsoussan): Make these config variables
  constexpr double kPosSigma = 0.01;
  constexpr double kVelSigma = 0.01;
  // TODO(rsoussan): add correlation for velocities and positions
  gtsam::Matrix6 Q = gtsam::Matrix6::Identity();
  Q.block<3, 3>(0, 0) = dt * dt * kPosSigma * kPosSigma * gtsam::Matrix3::Identity();
  Q.block<3, 3>(3, 3) = dt * dt * kVelSigma * kVelSigma * gtsam::Matrix3::Identity();
  // Constant Velocity Observation Matrix
  gtsam::Matrix6 H = gtsam::Matrix6::Identity();
  return KalmanFilterEstimate(x, P, F, Q, z, H, R);
}
}  // namespace imu_augmentor
