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
#include <vision_common/brisk_dynamic_detector.h>
#include <vision_common/dynamic_brisk.h>

namespace vision_common {
    BriskDynamicDetector::BriskDynamicDetector(const BriskDynamicDetectorParams& params): DynamicDetector(params), params_(params) {}
    void BriskDynamicDetector::InitializeDetector() {
      detector_ = cv::DynamicBRISK::create(params_.threshold, params_.octaves, params_.float_pattern_scale);
    }
    void BriskDynamicDetector::SetThreshold(const double threshold) { detector_->setThreshold(static_cast<int>(threshold)); }
}  // namespace vision_common
