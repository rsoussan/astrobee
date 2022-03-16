/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http:  //  www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <sparse_mapping/utilities.h>

namespace sparse_mapping {
EstimatePoseResults EstimatePose(
  const cv::Mat& descriptors,  // TODO(rsoussan): change this to vector of descriptors
                               // TODO(rsoussan): change this to vector of Eigen::Vector2ds
  const Eigen::Matrix2Xd& keypoints, const SparseMap& map, const EstimatePoseParams& params) {
  const auto indices = params.cid_list ? *params.cid_list : image_database.Query(descriptors, params.num_similar);
  // TODO(rsoussan): Remove this?
  if (indices.empty()) {
    LOG(FATAL) << "No indices found.";
  }

  // Check each image index for feature matches with given descriptors.
  // Keep at most params.num_similar image match candidates, ordered by the number of matches per image candidate.
  // Terminate early if the total number of features checked is larger than params.early_break_landmarks.
  std::vector<int> similarity_rank(indices.size(), 0);
  std::vector<std::vector<cv::DMatch> > all_matches(indices.size());
  int total_feature_matches = 0;
  // TODO(oalexan1): Use multiple threads here?
  for (const auto index : indices) {
    const int cid = indices[index];
    const auto& map_image_descriptors =  map.cid_to_descriptor_map_[cid];
    std::vector<cv::DMatch>& matches = all_matches[index];
    interest_point::FindMatches(descriptors,
                                map_image_descriptors,
                                &matches);
    for (const auto& match : matches) {
      const bool map_point_3d_exists = map.cid_fid_to_pid_[cid].count(match.trainIdx) > 0;
      if (!map_point_3d_exists) continue;
      ++similarity_rank[index];
    }

    LOG(DEBUG) << "Overall matches and validated matches to: "
                << cid_to_filename[cid] << ": "
                << matches.size() << " "
                << similarity_rank[index];
    total_feature_matches += similarity_rank[index];
    if (total_feature_matches >= params.early_break_landmarks)
      break;
  }

  std::vector<Eigen::Vector2d> observations;
  std::vector<Eigen::Vector3d> landmarks;
  const std::vector<int> highly_ranked = ff_common::rv_order(similarity_rank);
  const int end = std::min(static_cast<int>(highly_ranked.size()), num_similar);
  std::set<int> seen_landmarks;
  LOG(DEBUG) << "Similar images: ";
  for (int i = 0; i < end; ++i) {
    const int cid = indices[highly_ranked[i]];
    const std::vector<cv::DMatch>& matches = all_matches[highly_ranked[i]];
    int num_matches = 0;
    for (const auto& match : matches) {
      const bool map_point_3d_exists = map.cid_fid_to_pid_[cid].count(match.trainIdx) > 0;
      if (!map_point_3d_exists) continue;
      const int landmark_id = map.cid_fid_to_pid_.at(cid).at(match.trainIdx);
      if (seen_landmarks.count(landmark_id) > 0) continue;
      const Eigen::Vector2d observation(keypoints.col(match.queryIdx)[0],
                          keypoints.col(match.queryIdx)[1]);
      observations.emplace_back(observation);
      landmarks.push_back(map.pid_to_xyz_[landmark_id]);
      seen_landmarks.insert(landmark_id);
      ++num_matches;
    }
    if (num_matches > 0)
      LOG(DEBUG) << " " << cid_to_filename[cid];
  }

  // TODO(rsoussan): Update this to return estimate pose results or use vision_common function
std::vector<Eigen::Vector2d> inlier_landmarks_vec;
std::vector<Eigen::Vector2d>* inlier_landmarks = params.inlier_landmarks ? &inlier_landmarks_vec : nullptr;
std::vector<Eigen::Vector3d> inlier_observations_vec;
std::vector<Eigen::Vector3d>* inlier_observations = params.inlier_observations ? &inlier_observations_vec : nullptr;
camera::CameraModel camera_estimate;
int ret = RansacEstimateCamera(landmarks, observations, params.num_ransac_iterations, params.ransac_inlier_tolerance,
                               camera_estimate, inlier_landmarks, inlier_observations,
                               // TODO(rsoussan): Change this to use LOG(DEBUG)
                               FLAGS_verbose_localization);
EstimatePoseResults results;
if (ret) {
  results.pose = camera_estimate;
  if (params.inlier_landmarks) results.inlier_landmarks = *inlier_landmarks;
  if (params.inlier_observations) results.inlier_observations = *inlier_observations;
}
  return results;
}

// From pid_to_cid_fid, create cid_fid_to_pid for lookup.
void InitializeCidFidToPid(int num_cid,
                           std::vector<std::map<int, int> > const& pid_to_cid_fid,
                           std::vector<std::map<int, int> > * cid_fid_to_pid) {
  cid_fid_to_pid->clear();
  cid_fid_to_pid->resize(num_cid, std::map<int, int>());

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (std::pair<int, int> const& cid_fid : pid_to_cid_fid[pid]) {
      (*cid_fid_to_pid)[cid_fid.first][cid_fid.second] = pid;
    }
  }
}
}  // namespace sparse_mapping
