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

#include <sparse_map_matcher/estimate_pose_utilities.h>
#include <sparse_map_matcher/sparse_map_matcher.h>

namespace sparse_map_matcher {
SparseMapMatcher::SparseMapMatcher(const SparseMapMatcherParams& params)
    : params_(params), detector_(params.detector), map_(params.map_name) {}

boost::optional<EstimatePoseResults> SparseMapMatcher::Match(const cv::Mat& image) {
  return EstimatePose(image, params_.estimate_pose, detector_, map_);
}
}  // namespace sparse_map_matcher
