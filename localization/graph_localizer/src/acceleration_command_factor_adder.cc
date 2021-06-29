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

#include <graph_localizer/acceleration_command_factor_adder.h>
#include <localization_common/logger.h>

#include <gtsam/inference/Symbol.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
AccelerationCommandFactorAdder::AccelerationCommandFactorAdder(const AccelerationCommandFactorAdderParams& params)
    : AccelerationCommandFactorAdder::Base(params) {}

std::vector<go::FactorsToAdd> AccelerationCommandFactorAdder::AddFactors(
  const lm::AccelerationCommand& acceleration_command) {
  std::vector<go::FactorsToAdd> factors_to_add;
  /*go::FactorsToAdd standstill_prior_factors_to_add;
  const gtsam::Vector3 velocity_prior_noise_sigmas((gtsam::Vector(3) << params().prior_velocity_stddev,
                                                    params().prior_velocity_stddev, params().prior_velocity_stddev)
                                                     .finished());
  const auto velocity_noise =
    Robust(gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(velocity_prior_noise_sigmas)),
           params().huber_k);

  const go::KeyInfo velocity_key_info(&sym::V, go::NodeUpdaterType::CombinedNavState,
                                      feature_points_measurement.timestamp);
  gtsam::PriorFactor<gtsam::Velocity3>::shared_ptr velocity_prior_factor(new gtsam::PriorFactor<gtsam::Velocity3>(
    velocity_key_info.UninitializedKey(), gtsam::Velocity3::Zero(), velocity_noise));
  standstill_prior_factors_to_add.push_back({{velocity_key_info}, velocity_prior_factor});
  standstill_prior_factors_to_add.SetTimestamp(feature_points_measurement.timestamp);
  LogDebug("AddFactors: Added " << standstill_prior_factors_to_add.size() << " standstill velocity prior factors.");
  factors_to_add.emplace_back(standstill_prior_factors_to_add);*/
  return factors_to_add;
}
}  // namespace graph_localizer
