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
#ifndef VISION_COMMON_DYNAMIC_DETECTOR_H_
#define VISION_COMMON_DYNAMIC_DETECTOR_H_

#include <opencv2/features2d/features2d.hpp>

#include <vector>

namespace vision_common {
  class DynamicDetector {
   public:
    explicit DynamicDetector(const DynamicDetectorParams& params);
    virtual ~DynamicDetector() {}
    virtual void InitializeDetector() = 0;
    virtual void SetThreshold(const double threshold) = 0;
    void DetectAndCompute(const cv::Mat& image,
                        std::vector<cv::KeyPoint>& keypoints,
                        cv::Mat& descriptors);
    void IncreaseThreshold();
    void DecreaseThreshold();

   private:
    DynamicDetectorParams params_;
    cv::Ptr<cv::Feature2D> detector_;
    double dynamic_threshold_;
  };
}  // namespace vision_common

#endif  // VISION_COMMON_DYNAMIC_DETECTOR_H_
