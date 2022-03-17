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
  DynamicDetector::DynamicDetector(const DynamicDetectorParams& params) : params_(params), dynamic_threshold_(0) {}

  void DynamicDetector::DetectAndCompute(const cv::Mat& image,
                               std::vector<cv::KeyPoint>& keypoints,
                               cv::Mat& descriptors) {
    for (int i = 0; i < params_.max_retries; ++i) {
      keypoints.clear();
      Detect(image, keypoints);
      if (keypoints.size() < params_.min_features)
        DecreaseThreshold();
      else if (keypoints.size() > params_.max_features)
        IncreaseThreshold();
      else
        break;
    }
    Compute(image, keypoints, descriptors);

    if (params_.center_keypoints) {
      for (auto& keypoint : keypoints) {
        keypoint.pt.x -= image.cols / 2.0;
        keypoint.pt.y -= image.rows / 2.0;
      }
    }
  }
}  // namespace vision_common
