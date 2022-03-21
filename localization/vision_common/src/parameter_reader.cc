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
#include <vision_common/parameter_reader.h>

namespace vision_common {
namespace mc = msg_conversions;

void LoadBriskFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                              BriskFeatureDetectorAndMatcherParams& params) {
  LoadBriskDetectorParams(config, params);
  params.max_match_hamming_distance = mc::LoadInt(config, "brisk_max_match_hamming_distance");
  params.flann_table_number = mc::LoadInt(config, "brisk_flann_table_number");
  params.flann_key_size = mc::LoadInt(config, "brisk_flann_key_size");
  params.flann_multi_probe_level = mc::LoadInt(config, "brisk_flann_multi_probe_level");
}

void LoadGoodFeaturesToTrackDetectorParams(config_reader::ConfigReader& config,
                                           GoodFeaturesToTrackDetectorParams& params) {
  params.max_corners = mc::LoadInt(config, "lk_max_corners");
  params.quality_level = mc::LoadDouble(config, "lk_quality_level");
  params.min_distance = mc::LoadDouble(config, "lk_min_distance");
  params.block_size = mc::LoadInt(config, "lk_block_size");
  params.use_harris_detector = mc::LoadBool(config, "lk_use_harris_detector");
  params.k = mc::LoadDouble(config, "lk_k");
}

void LoadLKOpticalFlowFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                                      LKOpticalFlowFeatureDetectorAndMatcherParams& params) {
  params.max_iterations = mc::LoadInt(config, "lk_max_iterations");
  params.termination_epsilon = mc::LoadDouble(config, "lk_termination_epsilon");
  params.window_length = mc::LoadInt(config, "lk_window_length");
  params.max_level = mc::LoadInt(config, "lk_max_level");
  params.min_eigen_threshold = mc::LoadDouble(config, "lk_min_eigen_threshold");
  params.max_flow_distance = mc::LoadDouble(config, "lk_max_flow_distance");
  params.max_backward_match_distance = mc::LoadDouble(config, "lk_max_backward_match_distance");
  LoadGoodFeaturesToTrackDetectorParams(config, params.good_features_to_track);
}

void LoadSurfFeatureDetectorAndMatcherParams(config_reader::ConfigReader& config,
                                             SurfFeatureDetectorAndMatcherParams& params) {
  LoadSurfDetectorParams(config, params);
  params.max_match_distance = mc::LoadDouble(config, "surf_max_match_distance");
}

void LoadBriskDetectorParams(config_reader::ConfigReader& config,
                                              BriskDetectorParams& params) {
  params.threshold = mc::LoadInt(config, "brisk_threshold");
  params.octaves = mc::LoadInt(config, "brisk_octaves");
  params.float_pattern_scale = mc::LoadFloat(config, "brisk_float_pattern_scale");
}


void LoadSurfDetectorParams(config_reader::ConfigReader& config,
                                             SurfDetectorParams& params) {
  params.threshold = mc::LoadInt(config, "surf_threshold");
}

void LoadDynamicDetectorParams(config_reader::ConfigReader& config, DynamicDetectorParams& params) {
  params.name = mc::LoadString(config, "detector_name");
  params.min_features = mc::LoadInt(config, "min_features");
  params.max_features = mc::LoadInt(config, "max_features");
  params.max_retries = mc::LoadInt(config, "max_retries");
  params.min_threshold = mc::LoadDouble(config, "min_threshold");
  params.starting_threshold = mc::LoadDouble(config, "starting_threshold");
  params.max_threshold = mc::LoadDouble(config, "max_threshold");
  params.center_keypoints = mc::LoadBool(config, "center_keypoints");
  params.increase_threshold_multiplier = mc::LoadDouble(config, "increase_threshold_multiplier");
  params.decrease_threshold_multiplier = mc::LoadDouble(config, "decrease_threshold_multiplier");
}

void LoadBriskDynamicDetectorParams(config_reader::ConfigReader& config, BriskDynamicDetectorParams& params) {
  LoadDynamicDetectorParams(config, params);
  LoadBriskDetectorParams(config, params);
  // Set starting threshold using Brisk threshold
  params.starting_threshold = params.threshold;
}

void LoadSurfDynamicDetectorParams(config_reader::ConfigReader& config, SurfDynamicDetectorParams& params) {
  LoadDynamicDetectorParams(config, params);
  LoadSurfDetectorParams(config, params);
  // Set starting threshold using Surf threshold
  params.starting_threshold = params.threshold;
}
}  // namespace vision_common
