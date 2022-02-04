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

#ifndef GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_H_
#define GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_H_

#include <graph_optimizer/covariances.h>
#include <graph_optimizer/graph_optimizer_params.h>
#include <graph_optimizer/nodes.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/serialization/serialization.hpp>

#include <string>
#include <vector>

namespace graph_optimizer {
class GraphOptimizer {
 public:
  explicit GraphOptimizer(const GraphOptimizerParams& params, std::shared_ptr<Nodes> nodes = std::make_shared<Nodes>());

  // Default constructor for serialization only
  GraphOptimizer() {}

  // Default destructor for inheritance
  ~GraphOptimizer() {}

  void AddFactor(boost::shared_ptr<gtsam::NonlinearFactor> factor);

  template <typename FactorType>
  void AddFactor(const FactorType& factor);

  bool Optimize();

  // Removes all factors which contain key
  void RemoveFactors(const gtsam::Key key, boost::optional<gtsam::NonlinearFactorGraph&> removed_factors = boost::none);

  // Removes all factors which contain any key in keys
  void RemoveFactors(const gtsam::KeyVector& keys,
                     boost::optional<gtsam::NonlinearFactorGraph&> removed_factors = boost::none);

  // Remove all factors of give FactorType
  template <typename FactorType>
  void RemoveFactors();

  template <typename FactorType>
  const std::vector<boost::shared_ptr<const FactorType>> Factors() const;

  boost::optional<gtsam::noiseModel::Gaussian::shared_ptr> Covariance(const gtsam::Key key) const;

  template <typename FactorType>
  int NumFactors() const;

  const int TotalNumFactors() const;

  void SaveDotFile(const std::string& output_path = "graph.dot") const;

  const gtsam::NonlinearFactorGraph& factors() const;

  gtsam::NonlinearFactorGraph& factors();

  const gtsam::Values& values() const;

  std::shared_ptr<Nodes> nodes();

 private:
  // Optional validity check for graph before optimizing
  virtual bool Valid() const;

  // virtual void Print() const;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(covariances_);
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(nodes_);
    ar& BOOST_SERIALIZATION_NVP(factors_);
    ar& BOOST_SERIALIZATION_NVP(has_optimized_);
  }

  GraphOptimizerParams params_;
  Covariances covariances_;
  std::shared_ptr<Nodes> nodes_;
  gtsam::NonlinearFactorGraph factors_;
  bool has_optimized_;
};

// Implementation
template <typename FactorType>
void GraphOptimizer::AddFactor(const FactorType& factor) {
  factors_.push_back(factor);
}

template <typename FactorType>
void GraphOptimizer::RemoveFactors() {
  for (auto factor_it = factors_.begin(); factor_it != factors_.end();) {
    if (dynamic_cast<FactorType*>(factor_it->get())) {
      factor_it = factors_.erase(factor_it);
      continue;
    }
    ++factor_it;
  }
}

template <typename FactorType>
const std::vector<boost::shared_ptr<const FactorType>> GraphOptimizer::Factors() const {
  typename std::vector<boost::shared_ptr<const FactorType>> factors;
  for (const auto& factor : factors_) {
    const auto casted_factor = boost::dynamic_pointer_cast<const FactorType>(factor);
    if (casted_factor) factors.emplace_back(casted_factor);
  }
  return factors;
}

template <typename FactorType>
int GraphOptimizer::NumFactors() const {
  int num_factors = 0;
  for (const auto& factor : factors_) {
    if (dynamic_cast<const FactorType*>(factor.get())) {
      ++num_factors;
    }
  }
  return num_factors;
}
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_H_
