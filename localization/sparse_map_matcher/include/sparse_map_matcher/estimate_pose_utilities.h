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
#ifndef SPARSE_MAP_MATCHER_ESTIMATE_POSE_UTILITIES_H_
#define SPARSE_MAP_MATCHER_ESTIMATE_POSE_UTILITIES_H_

#include <ff_common/eigen_vectors.h>
#include <sparse_map_matcher/estimate_pose_params.h>
#include <sparse_map_matcher/estimate_pose_results.h>
#include <sparse_mapping/sparse_map.h>

#include <Eigen/Geometry>

#include <vector>
#include <string>

namespace sparse_map_matcher {
using Keypoints = std::vector<Keypoints>;
using Descriptors = std::vector<Descriptor>;

struct ImageMatch {
  std::vector<cv::DMatch> matches;
  int num_valid_matches;
  int cid;
  bool operator<(const ImageMatch& rhs) { return num_valid_matches < rhs.num_valid_matches; }
};

void GetMatchingObservationsAndLandmarks(const std::vector<ImageMatches>& image_matches, const SparseMap& map,
                                         const Keypoints& keypoints, std::vector<Eigen::Vector2d>& observations,
                                         std::vector<Eigen::Vector3d>& landmarks);

std::vector<ImageMatch> FindAndSortMatches(const std::vector<int>& matching_cids, const Descriptors& descriptors,
                                           const SparseMap& map, const int max_num_total_feature_matches = 100,
                                           const bool check_point_3d_exists = true,
                                           const int min_matches_per_image = 5);

EstimatePoseResults EstimatePose(const Descriptors& descriptors, const Keypoints& keypoints, const SparseMap& map,
                                 const EstimatePoseParams& params);

EstimatePoseResults EstimatePose(const cv::Mat& image, const EstimatePoseParams& params, SparseMap& map);

EstimatePoseResults EstimatePose(const std::string& image_filename, const EstimatePoseParams& params, SparseMap& map);
}  // namespace sparse_map_matcher

#endif  // SPARSE_MAP_MATCHER_ESTIMATE_POSE_UTILITIES_H_
