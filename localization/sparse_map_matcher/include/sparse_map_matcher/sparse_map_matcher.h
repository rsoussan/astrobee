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

#ifndef SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_H_
#define SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_H_

#include <config_reader/config_reader.h>
#include <cv_bridge/cv_bridge.h>
#include <ff_msgs/VisualLandmarks.h>
#include <sparse_mapping/sparse_map.h>

namespace sparse_map_matcher {
class SparseMapMatcher {
 public:
  explicit SparseMapMatcher(std::shared_ptr<sparse_mapping::SparseMap> map);
  // TODO(Rsoussan): change this?
  void ReadParams(config_reader::ConfigReader* config);
  // TODO(rsoussan): change this interface?
  bool Match(cv_bridge::CvImageConstPtr image_ptr, ff_msgs::VisualLandmarks* vl,
     Eigen::Matrix2Xd* image_keypoints = NULL);
 private:
  std::shared_ptr<sparse_mapping::SparseMap> map_;
  std::unique_ptr<interest_point::FeatureDetector> detector_;
};
}  // namespace sparse_map_matcher

#endif  // SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_H_
