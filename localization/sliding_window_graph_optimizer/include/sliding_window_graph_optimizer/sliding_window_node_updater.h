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

#ifndef SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_NODE_UPDATER_H_
#define SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_NODE_UPDATER_H_

#include <graph_optimizer/node_updater_with_priors.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace sliding_window_graph_optimizer {
class SlidingWindowNodeUpdater : public graph_optimizer::NodeUpdaterWithPriors {
 public:
  virtual ~SlidingWindowNodeUpdater() {}

  virtual bool Update(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) = 0;

  virtual bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                           const boost::optional<gtsam::Marginals>& marginals, const gtsam::KeyVector& old_keys,
                           const double huber_k, gtsam::NonlinearFactorGraph& factors) = 0;

  // Returns the oldest time that will be in graph values once the window is slid using params
  virtual boost::optional<localization_common::Time> SlideWindowNewOldestTime() const = 0;

  virtual gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                                   const gtsam::NonlinearFactorGraph& graph) const = 0;

  virtual boost::optional<localization_common::Time> OldestTimestamp() const = 0;

  virtual boost::optional<localization_common::Time> LatestTimestamp() const = 0;
};
}  // namespace sliding_window_graph_optimizer

#endif  // SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_NODE_UPDATER_H_
