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

#include <sparse_map_matcher/parameter_reader.h>
#include <sparse_mapping/parameter_reader.h>
#include <vision_common/parameter_reader.h>

namespace sparse_map_matcher {
namespace sm = sparse_mapping;
namespace vc = vision_common;

LoadSparseMapMatcherParams(config_reader::ConfigReader& config, SparseMapMatcherParams& params) {
  params.map_name = mc::LoadString(config, "world_vision_map_filename");
  params.estimate_pose = sm::LoadEstimatePoseParams(config, params);
  params.brisk_detector = vc::LoadBriskDynamicDetectorParams(config, params);
  params.num_cv_threads = mc::LoadInt(config, "num_cv_threads");
}
}  // namespace sparse_map_matcher
