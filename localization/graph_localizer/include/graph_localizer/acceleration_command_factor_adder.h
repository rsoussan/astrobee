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

#ifndef GRAPH_LOCALIZER_ACCELERATION_COMMAND_FACTOR_ADDER_H_
#define GRAPH_LOCALIZER_ACCELERATION_COMMAND_FACTOR_ADDER_H_

#include <graph_localizer/acceleration_command_factor_adder_params.h>
#include <graph_localizer/combined_nav_state_graph_values.h>
#include <graph_optimizer/factor_adder.h>
#include <imu_integration/latest_imu_integrator.h>
#include <localization_measurements/acceleration_command.h>

#include <gtsam/navigation/CombinedImuFactor.h>

#include <vector>

namespace graph_localizer {
class AccelerationCommandFactorAdder
    : public graph_optimizer::FactorAdder<localization_measurements::AccelerationCommand,
                                          AccelerationCommandFactorAdderParams> {
  using Base =
    graph_optimizer::FactorAdder<localization_measurements::AccelerationCommand, AccelerationCommandFactorAdderParams>;

 public:
  AccelerationCommandFactorAdder(const AccelerationCommandFactorAdderParams& params,
                                 std::shared_ptr<imu_integration::LatestImuIntegrator> latest_imu_integrator,
                                 std::shared_ptr<CombinedNavStateGraphValues> graph_values);

  std::vector<graph_optimizer::FactorsToAdd> AddFactors(
    const localization_measurements::AccelerationCommand& acceleration_command) final;

 private:
  boost::optional<localization_measurements::ImuMeasurement> GetImuMeasurement(
    const localization_common::Time time) const;
  boost::optional<gtsam::Vector3> ClosestGyroBias(const localization_common::Time time) const;
  double ElapsedTime() const;
  void AddMeasurements(
    std::map<localization_common::Time, localization_measurements::AccelerationCommand>& acceleration_commands,
    gtsam::PreintegratedCombinedMeasurements& pim);

  std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim_;
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> pim_params_;
  std::shared_ptr<const imu_integration::LatestImuIntegrator> latest_imu_integrator_;
  std::shared_ptr<const CombinedNavStateGraphValues> graph_values_;
  std::map<localization_common::Time, localization_measurements::AccelerationCommand> acceleration_commands_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_ACCELERATION_COMMAND_FACTOR_ADDER_H_
