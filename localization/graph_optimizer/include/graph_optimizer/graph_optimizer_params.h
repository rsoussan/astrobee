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
#ifndef GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_PARAMS_H_
#define GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_PARAMS_H_

#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

namespace graph_optimizer {
struct GraphOptimizerParams {
  gtsam::LevenbergMarquardtParams levenberg_marquardt;
  double huber_k;
  bool fatal_failures;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    // TODO(rsoussan): Put back when serialization is added to lm params in gtsam
    // ar& BOOST_SERIALIZATION_NVP(levenberg_marquardt);
    ar& BOOST_SERIALIZATION_NVP(huber_k);
    ar& BOOST_SERIALIZATION_NVP(fatal_failures);
  }
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_PARAMS_H_
