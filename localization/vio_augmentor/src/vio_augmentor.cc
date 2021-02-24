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

#include <localization_common/logger.h>
#include <vio_augmentor/vio_augmentor.h>

namespace vio_augmentor {
namespace lc = localization_common;
void VIOAugmentor::AddState(const VIONavState& state) { AddState(state.timestamp, state.pose, state.velocity); }

void VIOAugmentor::AddState(const localization_common::Time time, const Eigen::Isometry3d& pose,
                            const Eigen::Vector3d& velocity) {
  latest_velocity_ = velocity;
  timestamp_pose_map_.emplace(time, pose);
}

boost::optional<std::pair<localization_common::Time, Eigen::Isometry3d>> VIOAugmentor::LatestRelativePose(
  const localization_common::Time start_time) {
  const auto lower_bound_it = timestamp_pose_map_.lower_bound(start_time);
  if (lower_bound_it == timestamp_pose_map_.end()) {
    LogError("LatestRelativePose: Failed to get lower bound time.");
    return boost::none;
  }

  if (lower_bound_it == timestamp_pose_map_.begin() && start_time != lower_bound_it->first) {
    LogError("LatestRelativePose: Start time occurs before oldest pose time.");
    return boost::none;
  }

  const Eigen::Isometry3d world_T_start_pose =
    start_time == lower_bound_it->first
      ? lower_bound_it->second
      : InterpolatePose(start_time, std::prev(lower_bound_it)->first, std::prev(lower_bound_it)->second,
                        lower_bound_it->first, lower_bound_it->second);
  const Eigen::Isometry3d& world_T_latest_pose = timestamp_pose_map_.rbegin()->second;
  const Eigen::Isometry3d start_pose_T_latest_pose = world_T_start_pose.inverse() * world_T_latest_pose;
  const auto latest_timestamp = timestamp_pose_map_.rbegin()->first;
  return std::make_pair(latest_timestamp, start_pose_T_latest_pose);
}

Eigen::Isometry3d VIOAugmentor::InterpolatePose(const localization_common::Time desired_time,
                                                const localization_common::Time time_a, const Eigen::Isometry3d& pose_a,
                                                const localization_common::Time time_b,
                                                const Eigen::Isometry3d& pose_b) {
  const double alpha = time_a == time_b ? 0 : (desired_time - time_a) / (time_b - time_a);
  Eigen::Isometry3d interpolated_pose(Eigen::Isometry3d::Identity());
  interpolated_pose.linear() =
    Eigen::Quaterniond(pose_a.linear()).slerp(alpha, Eigen::Quaterniond(pose_b.linear())).toRotationMatrix();
  interpolated_pose.translation() = pose_a.translation() * (1.0 - alpha) + pose_b.translation() * alpha;
  return interpolated_pose;
}

void VIOAugmentor::RemoveOldPoses(const localization_common::Time oldest_desired_time) {
  const auto lower_bound_it = timestamp_pose_map_.lower_bound(oldest_desired_time);
  if (lower_bound_it == timestamp_pose_map_.end()) {
    timestamp_pose_map_.clear();
    return;
  }
  if (lower_bound_it == timestamp_pose_map_.begin()) return;
  // Erase up to previous pose so that first two poses can be used for interpolation if necessary
  timestamp_pose_map_.erase(timestamp_pose_map_.begin(), std::prev(lower_bound_it));
}

boost::optional<std::pair<localization_common::Time, Eigen::Isometry3d>> VIOAugmentor::ExtrapolatePose(
  const localization_common::Time timestamp, const Eigen::Isometry3d& pose) {
  const auto latest_relative_pose = LatestRelativePose(timestamp);
  if (!latest_relative_pose) {
    LogError("ExtrapolatePose: Failed to get latest relative pose.");
    return boost::none;
  }
  const Eigen::Isometry3d extrapolated_pose = pose * (latest_relative_pose->second);
  return std::make_pair(latest_relative_pose->first, extrapolated_pose);
}

boost::optional<Eigen::Vector3d> VIOAugmentor::latest_velocity() { return latest_velocity_; }
}  // end namespace vio_augmentor
