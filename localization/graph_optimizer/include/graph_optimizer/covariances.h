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
#ifndef GRAPH_OPTIMIZER_COVARIANCES_H_
#define GRAPH_OPTIMIZER_COVARIANCES_H_

#include <graph_optimizer/covariances_params.h>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/serialization/serialization.hpp>

namespace graph_optimizer {
class Covariances {
 public:
  explicit Covariances(const CovariancesParams& params);

  // For serialization only
  Covariances() {}

  bool Update(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values);

  boost::optional<gtsam::noiseModel::Gaussian::shared_ptr> Get(const gtsam::Key key) const;

  bool UpdateMarginals(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values);

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(params_);
    // TODO(rsoussan): Put back when serialization for marginals is added in gtsam
    // ar& BOOST_SERIALIZATION_NVP(marginals_);
  }

  CovariancesParams params_;
  boost::optional<gtsam::Marginals> marginals_;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_COVARIANCES_H_
