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

#include <localization_common/logger.h>
#include <msg_conversions/msg_conversions.h>
#include <sparse_mapping/parameter_reader.h>

namespace vision_common {
namespace mc = msg_conversions;

void LoadEstimatePoseParams(config_reader::ConfigReader& config, EstimatePoseParams& params) {
  params.num_ransac_iterations = mc::LoadInt(config, "num_ransac_iterations");
  params.ransac_inlier_tolerance = mc::LoadInt(config, "ransac_inlier_tolerance");
  params.histogram_equalization = mc::LoadInt(config, "histogram_equalization");
  params.max_num_total_feature_matches = mc::LoadInt(config, "max_num_total_feature_matches");
  params.check_point_3d_exists = mc::LoadBool(config, "check_point_3d_exists");
  params.max_image_matches = mc::LoadInt(config, "max_image_matches");
  params.inlier_landmarks = mc::LoadBool(config, "inlier_landmarks");
  params.inlier_observations = mc::LoadBool(config, "inlier_observations");
}
}  // namespace vision_common
