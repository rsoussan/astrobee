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

#ifndef SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_WRAPPER_H_
#define SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_WRAPPER_H_

#include <ff_msgs/VisualLandmarks.h>
#include <localization_common/time.h>
#include <sparse_map_matcher/sparse_map_matcher.h>

#include <vector>

namespace sparse_map_matcher {
class SparseMapMatcherWrapper {
 public:
  SparseMapMatcherWrapper();
  boost::optional<ff_msgs::VisualLandmarks> ImageCallback(const sensor_msgs::ImageConstPtr& image_msg);

 private:
  ff_msgs::VisualLandmarks VlMsg(const Eigen::Isometry3d& world_T_camera, const localization_common::Time& timestamp,
                                 const std::vector<Eigen::Vector2d>& observations,
                                 const std::vector<Eigen::Vector3d>& landmarks) const;

  std::unique_ptr<SparseMapMatcher> matcher_;
  int vl_msg_count_;
};
}  // namespace sparse_map_matcher

#endif  // SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_WRAPPER_H_

