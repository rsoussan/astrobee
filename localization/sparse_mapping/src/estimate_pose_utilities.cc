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

#include <sparse_mapping/estimate_pose_utilities.h>
#include <sparse_mapping/utilities.h>

#include <opencv2/core/mat.hpp>
#include <gflags/gflags.h>

namespace sparse_mapping {
std::vector<ImageMatch> FindAndSortMatches(const std::vector<int>& matching_cids, const SparseMap& map,
                                           const int max_num_total_feature_matches, const bool check_point_3d_exists,
                                           const int min_matches_per_image) {
  std::vector<ImageMatch> sorted_image_matches;
  int total_matches = 0;
  // TODO(oalexan1): Use multiple threads here?
  for (const auto cid : matching_cids) {
    const auto& map_image_descriptors = map.cid_to_descriptors_[cid];
    ImageMatch image_match;
    image_match.cid = cid;
    FindMatches(descriptors, map_image_descriptors, &image_match.matches);
    int num_valid_matches = 0;
    if (!check_point_3d_exists) {
      image_match.num_valid_matches = image_match.matches.size();
    } else {
      for (const auto& match : image_match.matches) {
        const bool map_point_3d_exists = map.cid_to_fid_to_pid_[cid].count(match.trainIdx) > 0;
        if (!map_point_3d_exists) continue;
        ++image_match.num_valid_matches;
      }
    }

    sorted_image_matches.emplace_back(image_match);
    total_matches += image_match.num_valid_matches;
    if (total_matches >= max_num_total_feature_matches) break;
  }

  std::sort(image_matches.begin(), image_matches.end(), std::greater<>());
  return image_matches;
}

void GetMatchingObservationsAndLandmarks(const std::vector<ImageMatches>& image_matches, const SparseMap& map,
                                         std::vector<Eigen::Vector2d>& observations,
                                         std::vector<Eigen::Vector3d>& landmarks) {
  std::set<int> seen_landmarks;
  for (const auto& image_match : image_matches) {
    for (const auto& match : image_match.matches) {
      const bool map_point_3d_exists = map.cid_to_fid_to_pid_[cid].count(match.trainIdx) > 0;
      if (!map_point_3d_exists) continue;
      const int landmark_id = map.cid_to_fid_to_pid_.at(cid).at(match.trainIdx);
      if (seen_landmarks.count(landmark_id) > 0) continue;
      const Eigen::Vector2d observation(keypoints.col(match.queryIdx)[0], keypoints.col(match.queryIdx)[1]);
      observations.emplace_back(observation);
      landmarks.push_back(map.pid_to_global_t_point_[landmark_id]);
      seen_landmarks.insert(landmark_id);
      ++num_matches;
    }
  }
}

EstimatePoseResults EstimatePose(const cv::Mat& descriptors, const Eigen::Matrix2Xd& keypoints, const SparseMap& map,
                                 const EstimatePoseParams& params) {
  const auto matching_cids = map.image_database().Query(descriptors, params.max_image_matches);
  if (matching_cids.empty()) {
    LOG(FATAL) << "No matching cids found.";
  }

  const auto image_matches =
    FindAndSortMatches(matching_cids, map, params.max_num_total_feature_matches, params.check_point_3d_exists);
  std::vector<Eigen::Vector2d> observations;
  std::vector<Eigen::Vector3d> landmarks;
  GetMatchingObservationsAndLandmarks(image_matches, map, observations, landmarks);

  const Eigen::Vector2d zero_principal_points(Eigen::Vector2d::Zero());
  const Eigen::VectorXd zero_distortion(1);
  const Eigen::Vector2d focal_lengths = map.params().camera.GetFocalVector();
  const auto pose = vc::ReprojectionPoseEstimate<vc::IdentityDistorter>(observations, landmarks, focal_lengths,
                                                                        zero_principal_points, zero_distortion, params);
  EstimatePoseResults results;
  if (!pose) {
    results.pose = boost::none;
    return results;
  }
  results.pose = pose->pose;
  results.inlier_landmarks = std::vector<Eigen::Vector3d>();
  results.inlier_observations = std::vector<Eigen::Vector2d>();
  for (int i = 0; i < pose->inliers.size(); ++i) {
    results.inlier_landmarks->emplace_back(landmarks[i]);
    results.inlier_observations->emplace_back(observations[i]);
  }
  return results;
}

EstimatePoseResults EstimatePose(const cv::Mat& image, const EstimatePoseParams& params, SparseMap& map) {
  cv::Mat descriptors;
  Eigen::Matrix2Xd keypoints;
  DetectFeatures(image, map.params().histogram_equalization, descriptors, keypoints);
  return EstimatePose(descriptors, keypoints, map, params);
}

EstimatePoseResults EstimatePose(const std::string& image_filename, const EstimatePoseParams& params, SparseMap& map) {
  const auto image = LoadImage(filename);
  return EstimatePose(image, params, map);
}

}  // namespace sparse_mapping
