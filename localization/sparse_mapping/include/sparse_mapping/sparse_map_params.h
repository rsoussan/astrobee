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

#ifndef SPARSE_MAPPING_SPARSE_MAP_PARAMS_H_
#define SPARSE_MAPPING_SPARSE_MAP_PARAMS_H_

#include <camera/camera_params.h>
#include <sparse_mapping/image_database_params.h>
#include <vision_common/brisk_dynamic_detector_params.h>
#include <vision_common/surf_dynamic_detector_params.h>

#include <string>

namespace sparse_mapping {
struct SparseMapParams {
  ImageDatabaseParams image_database;
  camera::CameraParameters camera;
  vision_common::BriskDynamicDetectorParams brisk_detector;
  vision_common::SurfDynamicDetectorParams surf_detector;
  std::string detector_name;
  bool histogram_equalization;
  // Image/Feature Matching Params
  int max_db_query_image_match_candidates;
  int max_sequential_image_match_candidates;
  int max_num_image_pair_feature_matches;
  int min_num_inliers_for_valid_match;
  int min_feature_track_length;
  // Incremental BA params
  ceres::Solver::Options incremental_ba_options;
  // TODO(rsoussan): set these when load params
  // TODO(rsoussan): Use same options for full ba?
  /*  ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.max_num_iterations = 500;
    options.logging_type = ceres::SILENT;
    options.num_threads = FLAGS_num_threads;*/
  // TODO(rsoussan): same for incremental and full ba?
  double cauchy_loss;                          // 0.5
  int min_num_cams_to_incrementally_optimize;  // 4
  int max_num_cams_to_incrementally_optimize;  // 128
};
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_SPARSE_MAP_PARAMS_H_
