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
      graph_values_(graph_values) {
  pim_params_.reset(new gtsam::PreintegratedCombinedMeasurements::Params(params.gravity));
  // Set sensor covariances
  pim_params_->gyroscopeCovariance = params.gyro_sigma * params.gyro_sigma * gtsam::I_3x3;
  pim_params_->accelerometerCovariance = params.accel_sigma * params.accel_sigma * gtsam::I_3x3;
  pim_params_->integrationCovariance = params.integration_variance * gtsam::I_3x3;
  // Set bias random walk covariances only for Gyro
  pim_params_->biasAccCovariance = gtsam::Z_3x3;
  pim_params_->biasOmegaCovariance = params.gyro_bias_sigma * params.gyro_bias_sigma * gtsam::I_3x3;
  // Set bias covariance used for pim integration
  pim_params_->biasAccOmegaInt = params.bias_acc_omega_int * gtsam::I_6x6;

  pim_.reset(new gtsam::PreintegratedCombinedMeasurements(pim_params_));
}

boost::optional<localization_measurements::ImuMeasurement> AccelerationCommandFactorAdder::GetImuMeasurement(
  const lc::Time time) const {
  if (latest_imu_integrator_->Empty()) return boost::none;
  const auto& measurements = latest_imu_integrator_->measurements();
  const auto& upper_bound_it = measurements.lower_bound(time);
  if (upper_bound_it == measurements.begin()) return upper_bound_it->second;
  const auto& lower_bound_it = std::prev(upper_bound_it);
  if (upper_bound_it == measurements.end()) return lower_bound_it->second;
  return ii::Interpolate(lower_bound_it->second, upper_bound_it->second, time);
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

double AccelerationCommandFactorAdder::ElapsedTime() const {
  if (acceleration_commands_.size() <= 1) return 0;
  return acceleration_commands_.crbegin()->first - acceleration_commands_.cbegin()->first;
}

bool AccelerationCommandFactorAdder::AddMeasurements(const gtsam::Vector3& initial_angular_velocity,
                                                     std::map<lc::Time, lm::AccelerationCommand>& acceleration_commands,
                                                     gtsam::PreintegratedCombinedMeasurements& pim) {
  if (acceleration_commands.size() < 2) return false;
  gtsam::Vector3 angular_velocity = initial_angular_velocity;
  for (auto acceleration_command_it = acceleration_commands.begin();
       acceleration_command_it != std::prev(acceleration_commands.end()); ++acceleration_command_it) {
    const auto& acceleration_command = acceleration_command_it->second;
    const auto& next_acceleration_command = std::next(acceleration_command_it)->second;
    const double dt = next_acceleration_command.timestamp - acceleration_command.timestamp;
    angular_velocity += dt * acceleration_command.angular_acceleration;
    pim.integrateMeasurement(acceleration_command.linear_acceleration, angular_velocity, dt);
  }
  return true;
}

std::vector<go::FactorsToAdd> AccelerationCommandFactorAdder::AddFactors(
  const lm::AccelerationCommand& acceleration_command) {
  acceleration_commands_.emplace(acceleration_command.timestamp, acceleration_command);
  const double dt = ElapsedTime();
  if (dt < params().min_dt) return {};
  const auto first_timestamp = acceleration_commands_.cbegin()->first;
  const auto closest_gyro_bias = ClosestGyroBias(first_timestamp);
  if (!closest_gyro_bias) {
    LogWarning("AddFactors: Failed to get closest gyro bias.");
    return {};
  }

  const auto initial_imu_measurement = GetImuMeasurement(first_timestamp);
  if (!initial_imu_measurement) {
    LogWarning("AddFactors: Failed to get initial IMU measurement.");
    return {};
  }
  const auto& initial_angular_velocity = initial_imu_measurement->angular_velocity;

  // No linear acceleration bias since is used directly from the acceleration command, whereas
  // the intregrated angular acceleration bias is added to the closest angular velocity measurement
  // coming from the IMU which needs to have its bias removed.
  const gtsam::imuBias::ConstantBias initial_bias(gtsam::Vector3::Zero(), *closest_gyro_bias);
  pim_->resetIntegrationAndSetBias(initial_bias);

  if (!AddMeasurements(initial_angular_velocity, acceleration_commands_, *pim_)) {
    LogError("AddFactors: Failed to add acceleration commands.");
    return {};
  }

  // CombinedNavState a Keys
  const lc::Time starting_timestamp = acceleration_commands_.cbegin()->first;
  const go::KeyInfo pose_a_key_info(&sym::P, go::NodeUpdaterType::CombinedNavState, starting_timestamp);
  const go::KeyInfo velocity_a_key_info(&sym::V, go::NodeUpdaterType::CombinedNavState, starting_timestamp);
  const go::KeyInfo imu_bias_a_key_info(&sym::B, go::NodeUpdaterType::CombinedNavState, starting_timestamp);
  // CombinedNavState b Keys
  const lc::Time ending_timestamp = acceleration_commands_.crbegin()->first;
  const go::KeyInfo pose_b_key_info(&sym::P, go::NodeUpdaterType::CombinedNavState, ending_timestamp);
  const go::KeyInfo velocity_b_key_info(&sym::V, go::NodeUpdaterType::CombinedNavState, ending_timestamp);

  gtsam::AccelerationCommandFactor::shared_ptr acceleration_command_factor(
    new gtsam::AccelerationCommandFactor(*pim_, params().huber_k, pose_a_key_info.UninitializedKey(),
                                         velocity_a_key_info.UninitializedKey(), imu_bias_a_key_info.UninitializedKey(),
                                         pose_b_key_info.UninitializedKey(), velocity_b_key_info.UninitializedKey()));
  go::FactorsToAdd acceleration_command_factors_to_add;
  acceleration_command_factors_to_add.push_back(
    {{pose_a_key_info, velocity_a_key_info, imu_bias_a_key_info, pose_b_key_info, velocity_b_key_info},
     acceleration_command_factor});
  acceleration_command_factors_to_add.SetTimestamp(ending_timestamp);
  LogDebug("AddFactors: Added " << acceleration_command_factors_to_add.size() << " acceleration command factors.");
  acceleration_commands_.clear();
  return {acceleration_command_factors_to_add};
}
}  // namespace graph_localizer
