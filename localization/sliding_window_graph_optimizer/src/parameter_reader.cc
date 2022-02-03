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

#include <graph_optimizer/parameter_reader.h>
#include <sliding_window_graph_optimizer/parameter_reader.h>
#include <msg_conversions/msg_conversions.h>

namespace sliding_window_graph_optimizer {
namespace mc = msg_conversions;

void LoadSlidingWindowGraphOptimizerParams(config_reader::ConfigReader& config,
                                           SlidingWindowGraphOptimizerParams& params) {
  graph_optimizer::LoadGraphOptimizerParams(params);
  const std::string marginals_factorization = mc::LoadString(config, "marginals_factorization");
  if (marginals_factorization == "qr") {
    params.marginals_factorization = gtsam::Marginals::Factorization::QR;
  } else if (params_.marginals_factorization == "cholesky") {
    params.marginals_factorization = gtsam::Marginals::Factorization::CHOLESKY;
  } else {
    LogError("GraphOptimizer: No marginals factorization entered, defaulting to qr.");
    params_.marginals_factorization = gtsam::Marginals::Factorization::QR;
  }

  params.add_marginal_factors = mc::LoadBool(config, "add_marginal_factors");
}
}  // namespace sliding_window_graph_optimizer
