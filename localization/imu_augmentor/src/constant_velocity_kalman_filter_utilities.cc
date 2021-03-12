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
  state << combined_nav_state.pose().translation(), combined_nav_state.velocity();
  gtsam::Matrix6 covariances(gtsam::Matrix6::Zero());
  // TODO(rsoussan): Store off diagonal components in combined nav state covs and fill in here
  covariances.block<3, 3>(0, 0) = combined_nav_state_covariances.pose_covariance().block<3, 3>(0, 0);
  covariances.block<3, 3>(3, 3) = combined_nav_state_covariances.velocity_covariance();
  return {state, covariances};
}

std::pair<lc::CombinedNavState, lc::CombinedNavStateCovariances> ConstantVelocityKalmanFilterEstimate(
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
  const gtsam::Point3 translation = estimated_state_and_covariances.first.head<3>();
  const gtsam::Vector3 velocity = estimated_state_and_covariances.first.tail<3>();
  // KF only estimates translation and velocity, get orientation from integrated imu measurements on starting combined
  // nav state
  const gtsam::Rot3 orientation = predicted_combined_nav_state.pose().rotation();
  const gtsam::Pose3 pose(orientation, translation);
  const lc::CombinedNavState estimated_combined_nav_state(pose, velocity, start_combined_nav_state.bias(),
                                                          predicted_combined_nav_state.timestamp());
  gtsam::Matrix6 pose_covariance(gtsam::Matrix6::Zero());
  pose_covariance.block<3, 3>(0, 0) = estimated_state_and_covariances.second.block<3, 3>(0, 0);
  pose_covariance.block<3, 3>(3, 3) = start_combined_nav_state_covariances.pose_covariance().block<3, 3>(3, 3);
  const lc::CombinedNavStateCovariances estimated_combined_nav_state_covariances(
    pose_covariance, estimated_state_and_covariances.second.block<3, 3>(3, 3),
    start_combined_nav_state_covariances.bias_covariance());
  return {estimated_combined_nav_state, estimated_combined_nav_state_covariances};
}

std::pair<gtsam::Vector6, gtsam::Matrix6> ConstantVelocityKalmanFilterEstimate(const gtsam::Vector6& x,
                                                                               const gtsam::Matrix6& P,
                                                                               const gtsam::Vector6& z,
                                                                               const gtsam::Matrix6& R,
                                                                               const lc::Time dt) {
  // Constant Velocity Transition Matrix
  gtsam::Matrix6 F = gtsam::Matrix6::Identity();
  F.block<3, 3>(0, 3) = dt * gtsam::Matrix3::Identity();
  // TODO(rsoussan): Make these config variables
  constexpr double kPosSigma = 0.01;
  constexpr double kVelSigma = 0.01;
  // TODO(rsoussan): add correlation for velocities and positions
  gtsam::Matrix6 Q = gtsam::Matrix6::Zero();
  Q.block<3, 3>(0, 0) = dt * dt * kPosSigma * kPosSigma * gtsam::Matrix3::Identity();
  Q.block<3, 3>(3, 3) = dt * dt * kVelSigma * kVelSigma * gtsam::Matrix3::Identity();
  // Observation Matrix for position and veloctity
  gtsam::Matrix6 H = gtsam::Matrix6::Identity();
  return KalmanFilterEstimate(x, P, F, Q, z, H, R);
}
}  // namespace imu_augmentor
