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

#include <graph_localizer/acceleration_command_factor.h>
#include <graph_localizer/acceleration_command_factor_adder.h>
#include <graph_localizer/utilities.h>
#include <imu_integration/utilities.h>
#include <localization_common/logger.h>
#include <localization_measurements/imu_measurement.h>

#include <gtsam/inference/Symbol.h>

/*namespace {
localization_measurements::ImuMeasurement MakeImuMeasurement(const localization_measurements::AccelerationCommand&
last_acceleration_command, const double elapsed_time){
  // TODO: integrate angular acceleration command!!!! (A)
 return localization_measurements::ImuMeasurement(last_acceleration_command.linear_acceleration, angular_velocity_diff,
last_acceleration_command.timestamp);
}
}*/

namespace graph_localizer {
namespace go = graph_optimizer;
namespace ii = imu_integration;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
AccelerationCommandFactorAdder::AccelerationCommandFactorAdder(
  const AccelerationCommandFactorAdderParams& params,
  std::shared_ptr<imu_integration::LatestImuIntegrator> latest_imu_integrator,
  std::shared_ptr<CombinedNavStateGraphValues> graph_values)
    : AccelerationCommandFactorAdder::Base(params),
      latest_imu_integrator_(latest_imu_integrator),
      graph_values_(graph_values) {}

boost::optional<localization_measurements::ImuMeasurement> AccelerationCommandFactorAdder::ClosestImuMeasurement(
  const lc::Time time) const {
  if (latest_imu_integrator_->Empty()) return boost::none;
  const auto& measurements = latest_imu_integrator_->measurements();
  const auto& upper_bound_it = measurements.lower_bound(time);
  const auto& lower_bound_it = std::prev(upper_bound_it);
  const double upper_bound_time_diff = std::abs(time - upper_bound_it->first);
  const double lower_bound_time_diff = std::abs(time - lower_bound_it->first);
  return upper_bound_time_diff < lower_bound_time_diff ? upper_bound_it->second : lower_bound_it->second;
}

boost::optional<gtsam::Vector3> AccelerationCommandFactorAdder::ClosestGyroBias(const lc::Time time) const {
  const auto closest_timestamp = graph_values_->ClosestPoseTimestamp(time);
  if (!closest_timestamp) {
    LogError("ClosestGyroBias: Failed to get closest timestamp");
    return boost::none;
  }

  const double time_diff = std::abs(*closest_timestamp - time);
  if (time_diff > 1) {
    LogWarning("ClosestGyroBias: Time diff " << time_diff << " > 1 second");
  }

  const auto combined_nav_state = graph_values_->GetCombinedNavState(*closest_timestamp);
  if (!combined_nav_state) {
    LogError("ClosestGyroBias: Failed to get closest combined nav state");
    return boost::none;
  }

  return combined_nav_state->bias().gyroscope();
}

/*boost::optional<gtsam::Vector3> AngularVelocityDiff(const lc::Time time_a, const lc::Time time_b){


}*/

std::vector<go::FactorsToAdd> AccelerationCommandFactorAdder::AddFactors(
  const lm::AccelerationCommand& acceleration_command) {
  acceleration_commands_.emplace(acceleration_command.timestamp, acceleration_command);

  std::vector<go::FactorsToAdd> factors_to_add;

  if (!last_acceleration_command_) {
    last_acceleration_command_ = acceleration_command;
    return factors_to_add;
  }

  if (last_acceleration_command_->timestamp > acceleration_command.timestamp) {
    LogDebug("AddFactors: Out of order acceleration command received.");
    return factors_to_add;
  }

  // pim_.resetIntegrationAndSetBias(gtsam::imuBias::ConstantBias());
  const double dt = acceleration_command.timestamp - last_acceleration_command_->timestamp;
  // const lm::ImuMeasurement acceleration_command_measurement = MakeImuMeasurement(last_acceleration_command_,
  // elapsed_time); ii::AddMeasurement(last_acceleration_command_measurement_, last_acceleration_command_.timestamp,
  // pim_);
  // TODO: get relative velocity diff and orientation diff from pim!!!
  go::FactorsToAdd acceleration_command_factors_to_add;
  const gtsam::Vector3 linear_acceleration_command_noise_sigmas(
    (gtsam::Vector(3) << params().linear_acceleration_stddev, params().linear_acceleration_stddev,
     params().linear_acceleration_stddev)
      .finished());
  const auto linear_acceleration_command_noise = Robust(
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(linear_acceleration_command_noise_sigmas)),
    params().huber_k);
  // CombinedNavState a Keys
  const go::KeyInfo pose_a_key_info(&sym::P, go::NodeUpdaterType::CombinedNavState,
                                    last_acceleration_command_->timestamp);
  const go::KeyInfo velocity_a_key_info(&sym::V, go::NodeUpdaterType::CombinedNavState,
                                        last_acceleration_command_->timestamp);
  const go::KeyInfo imu_bias_a_key_info(&sym::B, go::NodeUpdaterType::CombinedNavState,
                                        last_acceleration_command_->timestamp);
  // CombinedNavState b Keys
  const go::KeyInfo pose_b_key_info(&sym::P, go::NodeUpdaterType::CombinedNavState, acceleration_command.timestamp);
  const go::KeyInfo velocity_b_key_info(&sym::V, go::NodeUpdaterType::CombinedNavState, acceleration_command.timestamp);

  gtsam::AccelerationCommandFactor::shared_ptr acceleration_command_factor(new gtsam::AccelerationCommandFactor(
    *last_acceleration_command_, dt, linear_acceleration_command_noise, pose_a_key_info.UninitializedKey(),
    velocity_a_key_info.UninitializedKey(), imu_bias_a_key_info.UninitializedKey(), pose_b_key_info.UninitializedKey(),
    velocity_b_key_info.UninitializedKey()));
  acceleration_command_factors_to_add.push_back(
    {{pose_a_key_info, velocity_a_key_info, imu_bias_a_key_info, pose_b_key_info, velocity_b_key_info},
     acceleration_command_factor});
  acceleration_command_factors_to_add.SetTimestamp(acceleration_command.timestamp);
  LogDebug("AddFactors: Added " << acceleration_command_factors_to_add.size() << " acceleration command factors.");
  factors_to_add.emplace_back(acceleration_command_factors_to_add);
  last_acceleration_command_ = acceleration_command;
  return factors_to_add;
}
}  // namespace graph_localizer
