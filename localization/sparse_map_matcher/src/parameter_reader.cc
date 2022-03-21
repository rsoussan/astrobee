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
  params.histogram_equalization = mc::LoadInt(config, "histogram_equalization");
  params.num_cv_threads = mc::LoadInt(config, "num_cv_threads");
}

/* int num_similar, ransac_inlier_tolerance, ransac_iterations, early_break_landmarks, histogram_equalization;
 int min_features, max_features, detection_retries;
 double min_brisk_threshold, default_brisk_threshold, max_brisk_threshold;
 camera::CameraParameters cam_params(config, "nav_cam");
 if (!config->GetInt("num_similar", &num_similar)) ROS_FATAL("num_similar not specified in localization.");
 if (!config->GetInt("ransac_inlier_tolerance", &ransac_inlier_tolerance))
   ROS_FATAL("ransac_inlier_tolerance not specified in localization.");
 if (!config->GetInt("ransac_iterations", &ransac_iterations))
   ROS_FATAL("ransac_iterations not specified in localization.");
 if (!config->GetInt("min_features", &min_features)) ROS_FATAL("min_features not specified in localization.");
 if (!config->GetInt("max_features", &max_features)) ROS_FATAL("max_features not specified in localization.");
 if (!config->GetInt("detection_retries", &detection_retries))
   ROS_FATAL("detection_retries not specified in localization.");
 if (!config->GetInt("histogram_equalization", &histogram_equalization))
   ROS_FATAL("histogram_equalization not specified in localization.");

 // For the brisk thresholds and other values, quietly assume some defaults
 if (!config->GetReal("min_brisk_threshold", &min_brisk_threshold)) min_brisk_threshold = 20.0;
 if (!config->GetReal("default_brisk_threshold", &default_brisk_threshold)) default_brisk_threshold = 90.0;
 if (!config->GetReal("max_brisk_threshold", &max_brisk_threshold)) max_brisk_threshold = 110.0;
 if (!config->GetInt("early_break_landmarks", &early_break_landmarks)) early_break_landmarks = 100;
}*/
}  // namespace sparse_map_matcher
