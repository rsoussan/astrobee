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

#include <graph_localizer/combined_nav_state_nodes.h>
#include <localization_common/logger.h>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lc = localization_common;
CombinedNavStateNodes::CombinedNavStateNodes(std::shared_ptr<go::Nodes> nodes)
    : pose_nodes_(nodes), velocity_nodes_(nodes), bias_nodes_(nodes) {}

boost::optional<lc::CombinedNavState> CombinedNavStateNodes::Get(const lc::Time timestamp) const {
  const auto pose = pose_nodes_.Get(timestamp);
  if (!pose) return boost::none;
  const auto velocity = velocity_nodes_.Get(timestamp);
  if (!velocity) return boost::none;
  const auto bias = bias_nodes_.Get(timestamp);
  if (!bias) return boost::none;
  return lc::CombinedNavState{*pose, *velocity, *bias, timestamp};
}

bool CombinedNavStateNodes::Add(const lc::CombinedNavState& combined_nav_state) {
  const auto& timestamp = combined_nav_state.timestamp();
  if (!pose_nodes_.Add(timestamp, combined_nav_state.pose())) return false;
  if (!velocity_nodes_.Add(timestamp, combined_nav_state.velocity())) return false;
  if (!bias_nodes_.Add(timestamp, combined_nav_state.bias())) return false;
  return true;
}

bool CombinedNavStateNodes::Remove(const lc::Time timestamp) {
  if (!pose_nodes_.Remove(timestamp)) return false;
  if (!velocity_nodes_.Remove(timestamp)) return false;
  if (!bias_nodes_.Remove(timestamp)) return false;
  return true;
}

std::vector<lc::Time> CombinedNavStateNodes::Timestamps() const { return pose_nodes_.Timestamps(); }

boost::optional<lc::CombinedNavState> CombinedNavStateNodes::Latest() const {
  const auto latest_timestamp = pose_nodes_.LatestTimestamp();
  if (!latest_timestamp) return boost::none;
  return Get(*latest_timestamp);
}

boost::optional<lc::CombinedNavState> CombinedNavStateNodes::Oldest() const {
  const auto oldest_timestamp = pose_nodes_.OldestTimestamp();
  if (!oldest_timestamp) return boost::none;
  return Get(*oldest_timestamp);
}

boost::optional<lc::Time> CombinedNavStateNodes::OldestTimestamp() const { return pose_nodes_.OldestTimestamp(); }

boost::optional<lc::Time> CombinedNavStateNodes::LatestTimestamp() const { return pose_nodes_.LatestTimestamp(); }

boost::optional<lc::Time> CombinedNavStateNodes::ClosestTimestamp(const lc::Time timestamp) const {
  return pose_nodes_.ClosestTimestamp(timestamp);
}

std::pair<boost::optional<lc::Time>, boost::optional<lc::Time>> CombinedNavStateNodes::LowerAndUpperBoundTimestamps(
  const lc::Time timestamp) const {
  return pose_nodes_.LowerAndUpperBoundTimestamps(timestamp);
}

bool CombinedNavStateNodes::Empty() const { return pose_nodes_.empty(); }

double CombinedNavStateNodes::Duration() const { return pose_nodes_.Duration(); }

int CombinedNavStateNodes::size() const { return pose_nodes_.size(); }

boost::optional<lc::Time> CombinedNavStateNodes::LowerBoundOrEqualTimestamp(const lc::Time timestamp) const {
  return pose_nodes_.LowerBoundOrEqualTimestamp(timestamp);
}

boost::optional<lc::CombinedNavState> CombinedNavStateNodes::LowerBoundOrEqualCombinedNavState(
  const lc::Time timestamp) const {
  const auto lower_bound_or_equal_timestamp = pose_nodes_.LowerBoundOrEqualTimestamp(timestamp);
  if (!lower_bound_or_equal_timestamp) return boost::none;
  return Get(*lower_bound_or_equal_timestamp);
}

int CombinedNavStateNodes::RemoveOldNodes(const lc::Time oldest_allowed_time) {
  const int num_nodes_removed = pose_nodes_.RemoveOldNodes(oldest_allowed_time);
  velocity_nodes_.RemoveOldNodes(oldest_allowed_time);
  bias_nodes_.RemoveOldNodes(oldest_allowed_time);
  return num_nodes_removed;
}

gtsam::KeyVector CombinedNavStateNodes::OldKeys(const lc::Time oldest_allowed_time) const {
  gtsam::KeyVector old_keys;
  const auto old_pose_keys = pose_nodes_.OldKeys(oldest_allowed_time);
  const auto old_velocity_keys = velocity_nodes_.OldKeys(oldest_allowed_time);
  const auto old_bias_keys = bias_nodes_.OldKeys(oldest_allowed_time);
  old_keys.insert(old_keys.end(), old_pose_keys.begin(), old_pose_keys.end());
  old_keys.insert(old_keys.end(), old_velocity_keys.begin(), old_velocity_keys.end());
  old_keys.insert(old_keys.end(), old_bias_keys.begin(), old_bias_keys.end());
  return old_keys;
}

/*boost::optional<gtsam::Key> CombinedNavStateNodes::GetKey(go::KeyCreatorFunction key_creator_function,
                                                                const localization_common::Time timestamp) const {
  if (timestamp_key_index_map_.count(timestamp) == 0) {
    LogError("GetKey: No key index found at timestamp.");
    return boost::none;
  }

  const int key_index = timestamp_key_index_map_.at(timestamp);

  const auto key = key_creator_function(key_index);
  if (!Contains(key)) {
    LogError("GetKey: Key not present in values.");
    return boost::none;
  }

  return key;
}

bool CombinedNavStateNodes::HasKey(const lc::Time timestamp) const {
  return (timestamp_key_index_map_.count(timestamp) != 0);
}*/

/*boost::optional<gtsam::Key> CombinedNavStateNodes::PoseKey(const lc::Time timestamp) const {
  return GetKey(&sym::P, timestamp);
}*/
/*boost::optional<lc::Time> CombinedNavStateNodes::Timestamp(const int key_index) const {
  for (const auto& timestamp_key_index_pair : timestamp_key_index_map_) {
    if (timestamp_key_index_pair.second == key_index) return timestamp_key_index_pair.first;
  }
  return boost::none;
}

boost::optional<lc::Time> CombinedNavStateNodes::Timestamp(
  graph_optimizer::KeyCreatorFunction key_creator_function, const gtsam::Key key) const {
  for (const auto& timestamp_key_index_pair : timestamp_key_index_map_) {
    if (key_creator_function(timestamp_key_index_pair.second) == key) return timestamp_key_index_pair.first;
  }
  return boost::none;
}

boost::optional<int> CombinedNavStateNodes::LatestCombinedNavStateKeyIndex() const {
  if (Empty()) {
    LogError("LatestCombinedNavStateKeyIndex: No combined nav states available.");
    return boost::none;
  }
  return timestamp_key_index_map_.crbegin()->second;
}

boost::optional<int> CombinedNavStateNodes::OldestCombinedNavStateKeyIndex() const {
  if (Empty()) {
    LogError("OldestCombinedNavStateKeyIndex: No combined nav states available.");
    return boost::none;
  }
  return timestamp_key_index_map_.cbegin()->second;
}*/

/*boost::optional<lc::Time> CombinedNavStateNodes::SlideWindowNewOldestTime() const {
  if (Empty()) {
    LogDebug("SlideWindowOldestTime: No states in map.");
    return boost::none;
  }

  if (NumStates() <= params().min_num_states) {
    LogDebug("SlideWindowOldestTime: Not enough states to remove.");
    return boost::none;
  }

  const double total_duration = timestamp_key_index_map_.crbegin()->first - timestamp_key_index_map_.cbegin()->first;
  LogDebug("SlideWindowOldestTime: Starting total num states: " << timestamp_key_index_map_.size());
  LogDebug("SlideWindowOldestTime: Starting total duration is " << total_duration);
  const lc::Time ideal_oldest_allowed_state =
    std::max(0.0, timestamp_key_index_map_.crbegin()->first - params().ideal_duration);

  int num_states_to_be_removed = 0;
  // Ensures that new oldest time is consistent with a number of states <= max_num_states
  // and >= min_num_states.
  // Assumes min_num_states < max_num_states.
  for (const auto& timestamp_key_pair : timestamp_key_index_map_) {
    ++num_states_to_be_removed;
    const int new_num_states = NumStates() - num_states_to_be_removed;
    if (new_num_states > params().max_num_states) continue;
    const auto& time = timestamp_key_pair.first;
    if (new_num_states <= params().min_num_states) return time;
    if (time >= ideal_oldest_allowed_state) return time;
  }

  // Shouldn't occur
  return boost::none;
}*/
}  // namespace graph_localizer
