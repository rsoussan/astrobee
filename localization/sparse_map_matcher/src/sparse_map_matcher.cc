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

#include <camera/camera_params.h>
#include <ff_msgs/VisualLandmarks.h>
#include <msg_conversions/msg_conversions.h>
#include <sparse_map_matcher/sparse_map_matcher.h>
#include <sparse_mapping/sparse_map.h>

#include <ros/ros.h>

namespace sparse_map_matcher {

SparseMapMatcher::SparseMapMatcher(const SparseMapMatcherParams& params, std::shared_ptr<sparse_mapping::SparseMap> map)
    : params_(params), map_(std::move(map)) {}
// TODO(rsoussan): Add constructor for detector using params!!! move detector params here!
detector_ = std::make_unqiue<interest_point::FeatureDetector>(map_->params().detector.name, min_features, max_features,
                                                              retries, min_thresh, default_thresh, max_thresh);
}

bool SparseMapMatcher::Match(const cv::Mat& image, cv::Mat& descriptors, Eigen::Matrix2Xd& keypoints) {
  sparse_mapping::DetectFeatures(image, map_->params().histogram_equalization, *detector_,
                                 descriptors, keypoints);
  // TODO(rsoussan): Update this with new estimate pose interface!!! Add params, change results!
  if (!map_->EstimatePose(image_descriptors, *image_keypoints,
                               &camera, &landmarks, &observations)) {
    return false;
  }
}
}  // namespace sparse_map_matcher
