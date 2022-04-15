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

#ifndef SPARSE_MAPPING_SPARSE_MAP_MERGER_PARAMS_H_
#define SPARSE_MAPPING_SPARSE_MAP_MERGER_PARAMS_H_

namespace sparse_mapping {
struct SparseMapMergerParams {
  int max_db_query_image_match_candidates;
  int min_shared_track_features_for_merged_tracks = 2;
  double min_match_ratio_for_merged_tracks = 0.75;
  int min_num_features_for_new_track = 3;
  int max_num_matching_pids_for_appended_track = 1;
  double inlier_threshold_low_index_percent = 0.25;
  double inlier_threshold_high_index_percent = 0.75;
  double inlier_threshold_scale_factor = 0.2;
};
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_SPARSE_MAP_MERGER_PARAMS_H_
