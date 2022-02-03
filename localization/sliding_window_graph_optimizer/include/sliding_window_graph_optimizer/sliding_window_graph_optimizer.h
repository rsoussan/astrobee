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

#ifndef SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_H_
#define SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_H_

#include <graph_optimizer/graph_optimizer_params.h>
#include <graph_optimizer/graph_stats.h>
#include <sliding_window_graph_optimizer/marginalizer.h>
#include <sliding_window_graph_optimizer/sliding_window_node_updater.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/serialization/serialization.hpp>

#include <utility>
#include <vector>

namespace sliding_window_graph_optimizer {
class SlidingWindowGraphOptimizer : public graph_optimizer::GraphOptimizer {
 public:
  explicit SlidingWindowGraphOptimizer(const SlidingWindowGraphOptimizerParams& params,
                                       std::shared_ptr<Nodes> nodes = std::make_shared<Nodes>());

  // Default constructor for serialization only
  SlidingWindowGraphOptimizer() {}

  ~SlidingWindowGraphOptimizer();

  void AddNodeUpdater(std::shared_ptr<SlidingWindowNodeUpdater> node_updater);

  bool SlideWindowAndOptimize();

  bool MeasurementRecentEnough(const lc::Time timestamp) const;

  const SlidingWindowGraphOptimizerParams& params() const;

 private:
  // Removes Keys and Values outside of sliding window.
  // Removes any factors depending on removed values
  // Optionally adds marginalized factors encapsulating linearized error of removed factors
  // Optionally adds priors using marginalized covariances for new oldest states
  void SlideWindow(const localization_common::Time last_window_latest_time);

  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time) const;

  std::pair<gtsam::KeyVector, gtsam::NonlinearFactorGraph> OldKeysAndFactors(
    const localization_common::Time oldest_allowed_time);

  /*// Called after SlideWindow
  virtual void DoPostSlideWindowActions(const localization_common::Time oldest_allowed_time,
                                        const boost::optional<gtsam::Marginals>& marginals);

  // Removes old measurements from cumulative factors that do not fit in sliding window so cumulative factors are still
  // valid after the SlideWindow call
  virtual void RemoveOldMeasurementsFromCumulativeFactors(const gtsam::KeyVector& old_keys);

  // Calls Update for each registered NodeUpdater to create required nodes while inserting new factors
  bool UpdateNodes(const KeyInfo& key_info);

  boost::optional<localization_common::Time> OldestTimestamp() const;

  boost::optional<localization_common::Time> LatestTimestamp() const;

  // Removes buffered factors that are too old for insertion into graph
  void RemoveOldBufferedFactors(const localization_common::Time oldest_allowed_timestamp);*/

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(graph_optimizer::GraphOptimizer);
    ar& BOOST_SERIALIZATION_NVP(params_);
    ar& BOOST_SERIALIZATION_NVP(node_updaters_);
    ar& BOOST_SERIALIZATION_NVP(marginalizer_);
    ar& BOOST_SERIALIZATION_NVP(last_window_latest_time_);
  }

  SlidingWindowGraphOptimizerParams params_;
  std::vector<SlidingWindowNodeUpdater> node_updaters_;
  Marginalizer marginalizer_;
  boost::optional<localization_common::Time> last_window_latest_time_;
};
}  // namespace sliding_window_graph_optimizer

#endif  // SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_H_
