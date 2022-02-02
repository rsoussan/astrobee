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

#ifndef GRAPH_LOCALIZER_COMBINED_NAV_STATE_NODES_H_
#define GRAPH_LOCALIZER_COMBINED_NAV_STATE_NODES_H_

#include <graph_optimizer/timestamped_nodes.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/logger.h>
#include <localization_common/time.h>

#include <boost/optional.hpp>

#include <utility>
#include <vector>

namespace graph_localizer {
class CombinedNavStateNodes {
 public:
  explicit CombinedNavStateNodes(std::shared_ptr<graph_optimizer::Nodes> nodes);

  boost::optional<localization_common::CombinedNavState> Get(const localization_common::Time timestamp) const;

  bool Add(const localization_common::CombinedNavState& combined_nav_state);

  bool Remove(const localization_common::Time timestamp);

  boost::optional<localization_common::CombinedNavState> LatestNode() const;

  boost::optional<localization_common::CombinedNavState> OldestNode() const;

  int RemoveOldNodes(const localization_common::Time oldest_allowed_time);

  boost::optional<localization_common::Time> OldestTimestamp() const;

  boost::optional<localization_common::Time> LatestTimestamp() const;

  boost::optional<localization_common::Time> ClosestTimestamp(const localization_common::Time timestamp) const;

  // Assumes timestamp is within bounds of graph values timestamps.
  std::pair<boost::optional<localization_common::Time>, boost::optional<localization_common::Time>>
  LowerAndUpperBoundTimestamps(const localization_common::Time timestamp) const;

  boost::optional<localization_common::Time> LowerBoundOrEqualTimestamp(
    const localization_common::Time timestamp) const;

  boost::optional<localization_common::CombinedNavState> LowerBoundOrEqualCombinedNavState(
    const localization_common::Time timestamp) const;

  double Duration() const;

  int size() const;

  std::vector<localization_common::Time> Timestamps() const;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time) const;

  bool empty() const;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(pose_nodes_);
    ar& BOOST_SERIALIZATION_NVP(velocity_nodes_);
    ar& BOOST_SERIALIZATION_NVP(bias_nodes_);
  }

  graph_optimizer::TimestampedNodes<gtsam::Pose3> pose_nodes_;
  graph_optimizer::TimestampedNodes<gtsam::Velocity3> velocity_nodes_;
  graph_optimizer::TimestampedNodes<gtsam::imuBias::ConstantBias> bias_nodes_;
};
}  // namespace graph_localizer

/*
  boost::optional<gtsam::Key> PoseKey(const localization_common::Time timestamp) const;

  boost::optional<gtsam::Key> GetKey(graph_optimizer::KeyCreatorFunction key_creator_function,
                                     const localization_common::Time timestamp) const;


  boost::optional<localization_common::Time> Timestamp(graph_optimizer::KeyCreatorFunction key_creator_function,
                                                       const gtsam::Key key) const;

 bool HasKey(const localization_common::Time timestamp) const;

  template <class FACTOR>
  static bool Contains(const FACTOR& factor, const int key_index) {
    if (factor.find(sym::P(key_index)) != factor.end()) return true;
    if (factor.find(sym::V(key_index)) != factor.end()) return true;
    if (factor.find(sym::B(key_index)) != factor.end()) return true;
    return false;
  }

  // Returns the oldest time that will be in graph values once the window is slid using params
  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const;
*/

#endif  // GRAPH_LOCALIZER_COMBINED_NAV_STATE_NODES_H_
