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

namespace sliding_window_graph_optimizer {
namespace go = graph_optimizer;
namespace lc = localization_common;

SlidingWindowGraphOptimizer::SlidingWindowGraphOptimizer(const SlidingWindowGraphOptimizerParams& params,
                                                         std::shared_ptr<Nodes> nodes)
    : go::GraphOptimizer(params, std::move(nodes)), params_(params) {}

void GraphOptimizer::AddNodeUpdater(std::shared_ptr<SlidingWindowNodeUpdater> node_updater) {
  node_updaters_.emplace_back(std::move(node_updater));
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

void GraphOptimizer::SlideWindow(const lc::Time last_window_latest_time) {
  const auto ideal_new_oldest_time = SlideWindowNewOldestTime();
  if (!ideal_new_oldest_time) {
    LogDebug("SlideWindow: No states removed. ");
    return true;
  }
  // Ensure that new oldest time isn't more recent than last latest time
  // since then priors couldn't be added for the new oldest state
  if (last_window_latest_time < *ideal_new_oldest_time)
    LogError("SlideWindow: Ideal oldest time is more recent than last latest time.");
  const auto new_oldest_time = std::min(last_window_latest_time, *ideal_new_oldest_time);

  const auto old_keys_and_factors = OldKeysAndFactors(new_oldest_time);
  if (params_.add_marginal_factors) {
    marginalizer_.AddMarginalFactors(old_keys_and_factors.second, old_keys_and_factors.first, factors_);
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

void GraphOptimizer::UpdateOrdering() {
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

bool GraphOptimizer::SlideWindowAndOptimize() {
  LogDebug("Update: Updating.");
  // graph_stats_->update_timer_.Start();
  if (params_.add_marginal_factors) {
    marginalizer_.CacheOriginalValues(values());
    UpdateOrdering();
  }
  // Only slide window if have optimization has already occured since
  // covariances and/or marginals for marginalizer rely on updated values and factors
  if (has_optimized_) {
    // graph_stats_->slide_window_timer_.Start();
    if (!SlideWindow(*last_window_latest_time_)) {
      LogError("Update: Failed to slide window.");
      return false;
    }
    // graph_stats_->slide_window_timer_.Stop();
  }

  if (!ValidGraph()) {
    LogError("Update: Invalid graph, not optimizing.");
    return false;
  }

  Optimize();
  // graph_stats_->optimization_timer_.Stop();
  last_window_latest_time_ = LatestTimestamp();

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
