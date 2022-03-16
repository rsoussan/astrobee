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

#ifndef SPARSE_MAPPING_ESTIMATE_POSE_PARAMS_H_
#define SPARSE_MAPPING_ESTIMATE_POSE_PARAMS_H_

#include <vector>

namespace sparse_mapping {
struct EstimatePoseParams {
      int num_ransac_iterations;
     int ransac_inlier_tolerance;
    int early_break_landmarks;
    int histogram_equalization;
    int num_similar;
    bool inlier_landmarks;
    bool inlier_observations;
};
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_ESTIMATE_POSE_PARAMS_H_
