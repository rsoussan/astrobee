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
#ifndef VISION_COMMON_SURF_DYNAMIC_DETECTOR_H_
#define VISION_COMMON_SURF_DYNAMIC_DETECTOR_H_

#include <vision_common/dynamic_detector.h>
#include <vision_common/surf_dynamic_detector_params.h>

namespace vision_common {
  class SurfDynamicDetector: public DynamicDetector {
   public:
    explicit SurfDynamicDetector(const SurfDynamicDetectorParams& params);
    void InitializeDetector() final;
    void SetThreshold(const double threshold) final;

   private:
    SurfDynamicDetectorParams params_;
  };
}  // namespace vision_common

#endif  // VISION_COMMON_SURF_DYNAMIC_DETECTOR_H_
