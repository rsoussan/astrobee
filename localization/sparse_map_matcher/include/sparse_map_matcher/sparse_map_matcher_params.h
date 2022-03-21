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

#ifndef SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_PARAMS_H_
#define SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_PARAMS_H_

#include <sparse_mapping/estimate_pose_params.h>
#include <vision_common/brisk_dynamic_detector_params.h>

#include <string>

namespace sparse_map_matcher {
struct SparseMapMatcherParams {
  int histogram_equalization;
  std::string map_name;
  sparse_mapping::EstimatePoseParams estimate_pose;
  vision_common::BriskDynamicDetectorParams brisk_detector;
  int num_cv_threads;
};
}  // namespace sparse_map_matcher

#endif  // SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_PARAMS_H_
