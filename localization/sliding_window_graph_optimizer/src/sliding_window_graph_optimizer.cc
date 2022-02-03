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

#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <gtsam/nonlinear/LinearContainerFactor.h>

namespace sliding_window_graph_optimizer {
namespace go = graph_optimizer;
namespace lc = localization_common;

SlidingWindowGraphOptimizer::SlidingWindowGraphOptimizer(const SlidingWindowGraphOptimizerParams& params,
                                                         std::shared_ptr<Nodes> nodes)
    : go::GraphOptimizer(params, std::move(nodes)), params_(params) {}

void GraphOptimizer::AddNodeUpdater(std::shared_ptr<SlidingWindowNodeUpdater> node_updater) {
  node_updaters_.emplace_back(std::move(node_updater));
}

// Adapted from gtsam::BatchFixedLagSmoother
gtsam::NonlinearFactorGraph GraphOptimizer::MarginalFactors(
  const gtsam::NonlinearFactorGraph& old_factors, const gtsam::KeyVector& old_keys,
  const gtsam::GaussianFactorGraph::Eliminate& eliminate_function) const {
  // Old keys not present in old factors.  This shouldn't occur.
  if (old_keys.size() == 0) {
    LogDebug("MarginalFactors: No old keys provided.");
    return old_factors;
  }

  // Linearize Graph
  const auto linearized_graph = old_factors.linearize(*values_);
  const auto linear_marginal_factors =
    *(linearized_graph->eliminatePartialMultifrontal(old_keys, eliminate_function).second);
  return gtsam::LinearContainerFactor::ConvertLinearGraph(linear_marginal_factors, *values_);
}

boost::optional<lc::Time> GraphOptimizer::SlideWindowNewOldestTime() const {
  boost::optional<lc::Time> new_oldest_time;
  for (const auto& node_updater : node_updaters_) {
    const auto node_new_oldest_time = node_updater->SlideWindowNewOldestTime();
    if (node_new_oldest_time) {
      new_oldest_time = new_oldest_time ? std::min(*node_new_oldest_time, *new_oldest_time) : *node_new_oldest_time;
    }
  }

  return new_oldest_time;
}

gtsam::KeyVector GraphOptimizer::OldKeys(const localization_common::Time oldest_allowed_time) const {
  gtsam::KeyVector all_old_keys;
  for (const auto& node_updater : node_updaters_) {
    const auto old_keys = node_updater->OldKeys(oldest_allowed_time, graph_);
    all_old_keys.insert(all_old_keys.end(), old_keys.begin(), old_keys.end());
  }
  return all_old_keys;
}

std::pair<gtsam::KeyVector, gtsam::NonlinearFactorGraph> GraphOptimizer::OldKeysAndFactors(
  const lc::Time oldest_allowed_time) {
  const auto old_keys = OldKeys(oldest_allowed_time);
  // Since cumlative factors have many keys and shouldn't be marginalized, need to remove old measurements depending on
  // old keys before marginalizing and sliding window
  RemoveOldMeasurementsFromCumulativeFactors(old_keys);
  const auto old_factors = RemoveFactors(old_keys, graph_);
  return std::make_pair(old_keys, old_factors);
}

void GraphOptimizer::SlideWindow(const boost::optional<gtsam::Marginals>& marginals, const lc::Time last_latest_time) {
  const auto ideal_new_oldest_time = SlideWindowNewOldestTime();
  if (!ideal_new_oldest_time) {
    LogDebug("SlideWindow: No states removed. ");
    return true;
  }
  // Ensure that new oldest time isn't more recent than last latest time
  // since then priors couldn't be added for the new oldest state
  if (last_latest_time < *ideal_new_oldest_time)
    LogError("SlideWindow: Ideal oldest time is more recent than last latest time.");
  const auto new_oldest_time = std::min(last_latest_time, *ideal_new_oldest_time);

  const auto old_keys_and_factors = OldKeysAndFactors(new_oldest_time);
  if (params_.add_marginal_factors) {
    const auto marginal_factors =
      MarginalFactors(old_keys_and_factors.second, old_keys_and_factors.first, gtsam::EliminateQR);
    for (const auto& marginal_factor : marginal_factors) {
      graph_.push_back(marginal_factor);
    }
  }

  for (auto& node_updater : node_updaters_)
    node_updater->SlideWindow(new_oldest_time, marginals, old_keys_and_factors.first, params_.huber_k, graph_);
}

void GraphOptimizer::RemoveOldMeasurementsFromCumulativeFactors(const gtsam::KeyVector& old_keys) {}

boost::optional<lc::Time> GraphOptimizer::OldestTimestamp() const {
  boost::optional<lc::Time> oldest_timestamp;
  for (const auto& node_updater : node_updaters_) {
    const auto node_oldest_timestamp = node_updater->OldestTimestamp();
    if (node_oldest_timestamp) {
      oldest_timestamp =
        oldest_timestamp ? std::min(*oldest_timestamp, *node_oldest_timestamp) : *node_oldest_timestamp;
    }
  }
  return oldest_timestamp;
}

boost::optional<lc::Time> GraphOptimizer::LatestTimestamp() const {
  boost::optional<lc::Time> latest_timestamp;
  for (const auto& node_updater : node_updaters_) {
    const auto node_latest_timestamp = node_updater->LatestTimestamp();
    if (node_latest_timestamp) {
      latest_timestamp =
        latest_timestamp ? std::max(*latest_timestamp, *node_latest_timestamp) : *node_latest_timestamp;
    }
  }
  return latest_timestamp;
}

bool GraphOptimizer::MeasurementRecentEnough(const lc::Time timestamp) const {
  const auto oldest_timestamp = OldestTimestamp();
  if (!oldest_timestamp) {
    LogError("MeasurementRecentEnough: Failed to get oldest timestamp.");
    return false;
  }
  if (timestamp < *oldest_timestamp) return false;
  return true;
}

bool GraphOptimizer::UpdateMarginals() {
  try {
    marginals_ = gtsam::Marginals(graph_, values(), marginals_factorization_);
  } catch (gtsam::IndeterminantLinearSystemException) {
    log(params_.fatal_failures, "Update: Indeterminant linear system error during computation of marginals.");
    marginals_ = boost::none;
    return false;
  } catch (const std::exception& exception) {
    log(params_.fatal_failures, "Update: Computing marginals failed. " + std::string(exception.what()));
    marginals_ = boost::none;
    return false;
  } catch (...) {
    log(params_.fatal_failures, "Update: Computing marginals failed.");
    marginals_ = boost::none;
    return false;
  }
  return true;
}

// const boost::optional<gtsam::Marginals>& GraphOptimizer::marginals() const { return marginals_; }

bool GraphOptimizer::Update() {
  LogDebug("Update: Updating.");
  // graph_stats_->update_timer_.Start();
  // Only get marginals and slide window if optimization has already occured
  // TODO(rsoussan): Make cleaner way to check for this
  if (last_latest_time_) {
    // graph_stats_->marginals_timer_.Start();
    // Calculate marginals for covariances
    UpdateMarginals();
    // graph_stats_->marginals_timer_.Stop();

    // graph_stats_->slide_window_timer_.Start();
    if (!SlideWindow(marginals_, *last_latest_time_)) {
      LogError("Update: Failed to slide window.");
      return false;
    }
    // graph_stats_->slide_window_timer_.Stop();
  }

  // TODO(rsoussan): Is ordering required? if so clean these calls open and unify with marginalization
  // TODO(rsoussan): Remove this now that marginalization occurs before optimization?
  if (params_.add_marginal_factors) {
    // Add graph ordering to place keys that will be marginalized in first group
    const auto new_oldest_time = SlideWindowNewOldestTime();
    if (new_oldest_time) {
      const auto old_keys = OldKeys(*new_oldest_time);
      const auto ordering = gtsam::Ordering::ColamdConstrainedFirst(graph_, old_keys);
      params_.levenberg_marquardt.setOrdering(ordering);
    } else {
      params_.levenberg_marquardt.orderingType = gtsam::Ordering::COLAMD;
    }
  }

  if (!ValidGraph()) {
    LogError("Update: Invalid graph, not optimizing.");
    return false;
  }

  Optimize();
  // graph_stats_->optimization_timer_.Stop();

  // Calculate marginals after the first optimization iteration so covariances
  // can be used for first loc msg
  // TODO(rsoussan): Clean this up
  if (!last_latest_time_) {
    // graph_stats_->marginals_timer_.Start();
    // Calculate marginals for covariances
    UpdateMarginals();
    // graph_stats_->marginals_timer_.Stop();
  }

  last_latest_time_ = LatestTimestamp();

  /*graph_stats_->log_stats_timer_.Start();
  graph_stats_->iterations_averager_.Update(optimizer.iterations());
  graph_stats_->UpdateStats(graph_);
  graph_stats_->log_stats_timer_.Stop();
  graph_stats_->log_error_timer_.Start();
  graph_stats_->UpdateErrors(graph_);
  graph_stats_->log_error_timer_.Stop();*/

  // graph_stats_->update_timer_.Stop();
  return true;
}
}  // namespace sliding_window_graph_optimizer
