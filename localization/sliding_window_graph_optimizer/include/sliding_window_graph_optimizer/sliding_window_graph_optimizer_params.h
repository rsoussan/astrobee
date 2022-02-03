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
#ifndef SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_PARAMS_H_
#define SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_PARAMS_H_

#include <graph_optimizer/graph_optimizer.params.h>

#include <gtsam/nonlinear/Marginals.h>

#include <string>

namespace sliding_window_graph_optimizer {
struct SlidingWindowGraphOptimizerParams : public GraphOptimizer::GraphOptimizerParams {
  gtsam::Marginals::Factorization marginals_factorization;
  bool add_marginal_factors;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(marginals_factorization);
    ar& BOOST_SERIALIZATION_NVP(add_marginal_factors);
  }
};
}  // namespace sliding_window_graph_optimizer

#endif  // SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_PARAMS_H_
