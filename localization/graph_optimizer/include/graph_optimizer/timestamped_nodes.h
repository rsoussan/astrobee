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

#ifndef GRAPH_OPTIMIZER_TIMESTAMPED_NODES_H_
#define GRAPH_OPTIMIZER_TIMESTAMPED_NODES_H_

#include <graph_optimizer/nodes.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/optional.hpp>

#include <map>

namespace graph_optimizer {
template <typename NodeType>
class TimestampedNodes {
 public:
  explicit TimestampedNodes(std::shared_ptr<Nodes> nodes);

  gtsam::Key Add(const localization_common::Time timestamp, const NodeType& node);

  bool Remove(const localization_common::Time timestamp);

  boost::optional<NodeType> Get(const localization_common::Time timestamp) const;

  /*  // Returns the oldest time that will be in graph values once the window is slid using params
    virtual boost::optional<localization_common::Time> SlideWindowNewOldestTime() const = 0;

    virtual gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                                     const gtsam::NonlinearFactorGraph& graph) const = 0;

    virtual boost::optional<gtsam::Key> GetKey(KeyCreatorFunction key_creator_function,
                                               const localization_common::Time timestamp) const = 0;

    // TODO(rsoussan): Move implementations from CombinedNavSTateGraphValues to here, make generic, store timestamp map
    // here
    virtual boost::optional<localization_common::Time> OldestTimestamp() const = 0;

    virtual boost::optional<localization_common::Time> LatestTimestamp() const = 0;*/

 private:
  bool Contains(const localization_common::Time timestamp) const;

  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/);

  std::shared_ptr<Nodes> nodes_;
  std::map<localization_common::Time, gtsam::Key> timestamp_key_map_;
};

// Implementation
template <typename NodeType>
TimestampedNodes<NodeType>::TimestampedNodes(std::shared_ptr<Nodes> nodes) : nodes_(std::move(nodes)) {}

template <typename NodeType>
bool TimestampedNodes<NodeType>::Add(const localization_common::Time timestamp, const NodeType& node) {
  if (Contains(timestamp)) return false;
  const auto key = nodes_->Add(node);
  timestamp_key_map_.emplace(timestamp, key);
  return true;
}

template <typename NodeType>
bool TimestampedNodes<NodeType>::Remove(const localization_common::Time timestamp) {
  if (!Contains(timestamp)) return false;
  const auto key = timestamp_key_map_[timestamp];
  timestamp_key_map_.erase(timestamp);
  nodes_->Remove(key);
  return true;
}

template <typename NodeType>
boost::optional<NodeType> TimestampedNodes<NodeType>::Get(const localization_common::Time timestamp) const {
  if (!Contains(timestamp)) return boost::none;
  const auto key = timestamp_key_map_[timestamp];
  return nodes_->Get<NodeType>(key);
}

template <typename NodeType>
bool TimestampedNodes<NodeType>::Contains(const localization_common::Time timestamp) const {
  return timestamp_key_map_.count(timestamp) > 0;
}

template <typename NodeType>
template <class ARCHIVE>
void TimestampedNodes<NodeType>::serialize(ARCHIVE& ar, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(nodes_);
  ar& BOOST_SERIALIZATION_NVP(timestamp_key_map_);
}
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_TIMESTAMPED_NODES_H_
