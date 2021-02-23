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

#ifndef VIO_AUGMENTOR_VIO_AUGMENTOR_H_
#define VIO_AUGMENTOR_VIO_AUGMENTOR_H_

#include <localization_common/time.h>
#include <vio_augmentor/vio_nav_state.h>

#include <boost/optional.hpp>
#include <Eigen/Geometry>

#include <map>
#include <utility>

namespace vio_augmentor {
class VIOAugmentor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void AddState(const VIONavState& state);
  void AddState(const localization_common::Time time, const Eigen::Isometry3d& pose, const Eigen::Vector3d& velocity);
  boost::optional<std::pair<localization_common::Time, Eigen::Isometry3d>> LatestRelativePose(
    const localization_common::Time start_time);
  Eigen::Isometry3d InterpolatePose(const localization_common::Time desired_time,
                                    const localization_common::Time time_a, const Eigen::Isometry3d& pose_a,
                                    const localization_common::Time time_b, const Eigen::Isometry3d& pose_b);
  void RemoveOldPoses(const localization_common::Time oldest_desired_time);
  boost::optional<std::pair<localization_common::Time, Eigen::Isometry3d>> ExtrapolatePose(
    const localization_common::Time timestamp, const Eigen::Isometry3d& pose);

 private:
  Eigen::Vector3d latest_velocity_;
  std::map<localization_common::Time, Eigen::Isometry3d> timestamp_pose_map_;
};
}  // end namespace vio_augmentor

#endif  // VIO_AUGMENTOR_VIO_AUGMENTOR_H_
