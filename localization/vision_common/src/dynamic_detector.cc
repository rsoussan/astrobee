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

#include <vision_common/dynamic_detector.h>

namespace vision_common {
DynamicDetector::DynamicDetector(const DynamicDetectorParams& params)
    : params_(params), dynamic_threshold_(params.starting_threshold) {}

void DynamicDetector::DetectAndCompute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                                       cv::Mat& descriptors) {
  for (int i = 0; i < params_.max_retries; ++i) {
    keypoints.clear();
    detector_->detect(image, keypoints);
    if (keypoints.size() < params_.min_features)
      DecreaseThreshold();
    else if (keypoints.size() > params_.max_features)
      IncreaseThreshold();
    else
      break;
  }
  detector_->compute(image, keypoints, descriptors);

  if (params_.center_keypoints) {
    for (auto& keypoint : keypoints) {
      keypoint.pt.x -= image.cols / 2.0;
      keypoint.pt.y -= image.rows / 2.0;
    }
  }
}
void IncreaseThreshold() {
  dynamic_threshold_ = std::min(dynamic_threshold_ * params_.increase_threshold_multiplier, params_.max_threshold);
  detector_->setThreshold(dynamic_threshold_);
}

void DecreaseThreshold() {
  dynamic_threshold_ = std::max(dynamic_threshold_ * params_.decrease_threshold_multiplier, params_.min_threshold);
  detector_->setThreshold(dynamic_threshold_);
}

}  // namespace vision_common
