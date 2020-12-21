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

#include <imu_integration/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

namespace imu_integration {
namespace lc = localization_common;
namespace lm = localization_measurements;
boost::optional<gtsam::imuBias::ConstantBias> EstimateAndSetImuBiases(
  const lm::ImuMeasurement& imu_measurement, const int num_imu_measurements_per_bias_estimate,
  std::vector<lm::ImuMeasurement>& imu_bias_measurements) {
  imu_bias_measurements.emplace_back(imu_measurement);
  if (imu_bias_measurements.size() < num_imu_measurements_per_bias_estimate) return boost::none;

  Eigen::Vector3d sum_of_acceleration_measurements = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_of_angular_velocity_measurements = Eigen::Vector3d::Zero();
  for (const auto& imu_measurement : imu_bias_measurements) {
    sum_of_acceleration_measurements += imu_measurement.acceleration;
    sum_of_angular_velocity_measurements += imu_measurement.angular_velocity;
  }

  LogDebug("Number of imu measurements per bias estimate: " << num_imu_measurements_per_bias_estimate);
  const Eigen::Vector3d accelerometer_bias = sum_of_acceleration_measurements / imu_bias_measurements.size();
  const Eigen::Vector3d gyro_bias = sum_of_angular_velocity_measurements / imu_bias_measurements.size();
  LogInfo("Accelerometer bias: " << std::endl << accelerometer_bias.matrix());
  LogInfo("Gyro bias: " << std::endl << gyro_bias.matrix());

  imu_bias_measurements.clear();
  return gtsam::imuBias::ConstantBias(accelerometer_bias, gyro_bias);
}

boost::optional<lm::ImuMeasurement> Interpolate(const lm::ImuMeasurement& imu_measurement_a,
                                                const lm::ImuMeasurement& imu_measurement_b, const lc::Time timestamp) {
  if (timestamp < imu_measurement_a.timestamp || timestamp > imu_measurement_b.timestamp) {
    LogError(
      "Interpolate: Interpolation timestamp out of range of imu "
      "measurements.");
    return boost::none;
  }

  const double alpha =
    (timestamp - imu_measurement_a.timestamp) / (imu_measurement_b.timestamp - imu_measurement_a.timestamp);
  const Eigen::Vector3d interpolated_acceleration =
    (1.0 - alpha) * imu_measurement_a.acceleration + alpha * imu_measurement_b.acceleration;
  const Eigen::Vector3d interpolated_angular_velocity =
    (1.0 - alpha) * imu_measurement_a.angular_velocity + alpha * imu_measurement_b.angular_velocity;

  return lm::ImuMeasurement(interpolated_acceleration, interpolated_angular_velocity, timestamp);
}

gtsam::PreintegratedCombinedMeasurements Pim(
  const gtsam::imuBias::ConstantBias& bias,
  const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>& params) {
  gtsam::PreintegratedCombinedMeasurements pim(params);
  pim.resetIntegrationAndSetBias(bias);
  return pim;
}

void AddMeasurement(const lm::ImuMeasurement& imu_measurement, lc::Time& last_added_imu_measurement_time,
                    gtsam::PreintegratedCombinedMeasurements& pim) {
  const double dt = imu_measurement.timestamp - last_added_imu_measurement_time;
  // TODO(rsoussan): check if dt too large? Pass threshold param?
  if (dt == 0) {
    LogError("AddMeasurement: Timestamp difference 0, failed to add measurement.");
    return;
  }
  pim.integrateMeasurement(imu_measurement.acceleration, imu_measurement.angular_velocity, dt);
  last_added_imu_measurement_time = imu_measurement.timestamp;
}

lc::CombinedNavState PimPredict(const lc::CombinedNavState& combined_nav_state,
                                const gtsam::PreintegratedCombinedMeasurements& pim) {
  const gtsam::NavState predicted_nav_state = pim.predict(combined_nav_state.nav_state(), pim.biasHat());
  const lc::Time timestamp = combined_nav_state.timestamp() + pim.deltaTij();
  return lc::CombinedNavState(predicted_nav_state, pim.biasHat(), timestamp);
}

gtsam::CombinedImuFactor::shared_ptr MakeCombinedImuFactor(const int key_index_0, const int key_index_1,
                                                           const gtsam::PreintegratedCombinedMeasurements& pim) {
  return gtsam::CombinedImuFactor::shared_ptr(
    new gtsam::CombinedImuFactor(sym::P(key_index_0), sym::V(key_index_0), sym::P(key_index_1), sym::V(key_index_1),
                                 sym::B(key_index_0), sym::B(key_index_1), pim));
}

void LoadImuIntegratorParams(config_reader::ConfigReader& config, ImuIntegratorParams& params) {
  params.gravity = lc::LoadVector3(config, "world_gravity_vector");
  const bool ignore_gravity = lc::LoadBool(config, "ignore_gravity");
  if (ignore_gravity) params.gravity = gtsam::Vector3::Zero();
  params.body_T_imu = lc::LoadTransform(config, "imu_transform");
  params.filter.type = lc::LoadString(config, "imu_filter");
  params.gyro_sigma = lc::LoadDouble(config, "gyro_sigma");
  params.accel_sigma = lc::LoadDouble(config, "accel_sigma");
  params.accel_bias_sigma = lc::LoadDouble(config, "accel_bias_sigma");
  params.gyro_bias_sigma = lc::LoadDouble(config, "gyro_bias_sigma");
  params.integration_variance = lc::LoadDouble(config, "integration_variance");
  params.bias_acc_omega_int = lc::LoadDouble(config, "bias_acc_omega_int");
}
}  // namespace imu_integration
