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

#ifndef SPARSE_MAP_MATCHER_ESTIMATE_POSE_RESULTS_H_
#define SPARSE_MAP_MATCHER_ESTIMATE_POSE_RESULTS_H_

#include <Eigen/Geometry>

#include <vector>

namespace sparse_map_matcher {
struct EstimatePoseResults {
  boost::optional<Eigen::Isometry3d> pose;
  boost::optional<std::vector<Eigen::Vector3d>> inlier_landmarks;
  boost::optional<std::vector<Eigen::Vector2d>> inlier_observations;
};
}  // namespace sparse_map_matcher
#endif  // SPARSE_MAP_MATCHER_ESTIMATE_POSE_RESULTS_H_
