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

#include <graph_optimizer/graph_optimizer.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace graph_optimizer {
namespace lc = localization_common;

GraphOptimizer::GraphOptimizer(const GraphOptimizerParams& params, std::shared_ptr<Nodes> nodes)
    : params_(params), nodes_(std::move(nodes)), covariances_(params.covariances), has_optimized_(false) {}

bool GraphOptimizer::Valid() const { return true; }

void GraphOptimizer::AddFactor(boost::shared_ptr<gtsam::NonlinearFactor> factor) {
  factors_.push_back(std::move(factor));
}

bool GraphOptimizer::Optimize() {
  if (!Valid()) {
    LogError("Optimize: Invalid graph, not optimizing.");
    return false;
  }

  gtsam::LevenbergMarquardtOptimizer optimizer(factors_, nodes_->values(), params_.levenberg_marquardt);
  try {
    nodes_->values() = optimizer.optimize();
  } catch (gtsam::IndeterminantLinearSystemException) {
    LogOptionallyFatal("Update: Graph optimization failed, indeterminant linear system, keeping old values.",
                       params_.fatal_failures);
  } catch (gtsam::InvalidNoiseModel) {
    LogOptionallyFatal("Update: Graph optimization failed, invalid noise model, keeping old values.",
                       params_.fatal_failures);
  } catch (gtsam::InvalidMatrixBlock) {
    LogOptionallyFatal("Update: Graph optimization failed, invalid matrix block, keeping old values.",
                       params_.fatal_failures);
  } catch (gtsam::InvalidDenseElimination) {
    LogOptionallyFatal("Update: Graph optimization failed, invalid dense elimination, keeping old values.",
                       params_.fatal_failures);
  } catch (...) {
    LogOptionallyFatal("Update: Graph optimization failed, keeping old values.", params_.fatal_failures);
  }

  if (!covariances_.Update(factors_, values())) return false;
  has_optimized_ = true;
  return true;
}

void GraphOptimizer::RemoveFactors(const gtsam::Key key,
                                   boost::optional<gtsam::NonlinearFactorGraph&> removed_factors) {
  for (auto factor_it = factors_.begin(); factor_it != factors_.end();) {
    if ((*factor_it)->find(key) != (*factor_it)->end()) {
      if (removed_factors) removed_factors->push_back(*factor_it);
      factor_it = factors_.erase(factor_it);
    } else {
      ++factor_it;
    }
  }
}

void GraphOptimizer::RemoveFactors(const gtsam::KeyVector& keys,
                                   boost::optional<gtsam::NonlinearFactorGraph&> removed_factors) {
  for (const auto& key : keys) {
    RemoveFactors(key, removed_factors);
  }
}

boost::optional<gtsam::noiseModel::Gaussian::shared_ptr> GraphOptimizer::Covariance(const gtsam::Key key) const {
  return covariances_.Get(key);
}

const int GraphOptimizer::TotalNumFactors() const { return factors_.size(); }

void GraphOptimizer::SaveDotFile(const std::string& output_path) const {
  std::ofstream of(output_path.c_str());
  factors_.saveGraph(of, nodes_->values());
}

const gtsam::NonlinearFactorGraph& GraphOptimizer::factors() const { return factors_; }

gtsam::NonlinearFactorGraph& GraphOptimizer::factors() { return factors_; }

const gtsam::Values& GraphOptimizer::values() const { return nodes_->values(); }

std::shared_ptr<Nodes> GraphOptimizer::nodes() { return nodes_; }
}  // namespace graph_optimizer
