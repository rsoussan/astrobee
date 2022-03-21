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

#include <sparse_map_matcher/sparse_map_matcher.h>
#include <sparse_mapping/estimate_pose_utilities.h>
#include <sparse_mapping/sparse_map_utilities.h>

namespace sparse_map_matcher {
namespace sm = sparse_mapping;

SparseMapMatcher::SparseMapMatcher(const SparseMapMatcherParams& params)
    : params_(params), detector_(params.detector), map_(params.map_name, true) {
  sm::HistogramEqualizationCheck(map_->GetHistogramEqualization(), params_.histogram_equalization);
}

sm::EstimatePoseResults SparseMapMatcher::Match(const cv::Mat& image) {
  cv::Mat descriptors;
  Eigen::Matrix2Xd keypoints;
  sm::DetectFeatures(image, params_.histogram_equalization, detector_, &descriptors, &keypoints);
  return sm::EstimatePose(descriptors, keypoints, map_, params_.estimate_pose);
}
}  // namespace sparse_map_matcher
