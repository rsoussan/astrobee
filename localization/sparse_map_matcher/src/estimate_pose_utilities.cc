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

#include <sparse_map_matcher/estimate_pose_utilities.h>
#include <vision_common/mapping_utilities.h>

#include <opencv2/core/mat.hpp>
#include <gflags/gflags.h>

namespace sparse_map_matcher {
namespace vc = vision_common;
std::vector<ImageMatch> FindAndSortMatches(const std::vector<int>& matching_cids, const Descriptors& descriptors,
                                           const SparseMap& map, const int max_num_total_feature_matches,
                                           const bool check_point_3d_exists, const int min_matches_per_image) {
  std::vector<ImageMatch> sorted_matches;
  int total_matches = 0;
  // TODO(oalexan1): Use multiple threads here?
  for (const auto cid : matching_cids) {
    const auto& map_descriptors = map.descriptors(cid);
    ImageMatch match;
    match.cid = cid;
    vc::FindMatches(descriptors, map_descriptors, &match.matches);
    int num_valid_matches = 0;
    if (!check_point_3d_exists) {
      match.num_valid_matches = match.matches.size();
    } else {
      match.num_valid_matches = 0;
      for (const auto& match : match.matches) {
        if (!map.ContainsPid(cid, match.trainIdx) continue;
        ++match.num_valid_matches;
      }
    }

    sorted_matches.emplace_back(match);
    total_matches += match.num_valid_matches;
    if (total_matches >= max_num_total_feature_matches) break;
  }

  std::sort(matches.begin(), matches.end(), std::greater<>());
  return matches;
}

void GetMatchingObservationsAndLandmarks(const std::vector<ImageMatches>& image_matches, const SparseMap& map,
                                         const Keypoints& keypoints, std::vector<Eigen::Vector2d>& observations,
                                         std::vector<Eigen::Vector3d>& landmarks) {
  std::set<int> seen_landmarks;
  for (const auto& image_match : image_matches) {
    for (const auto& match : image_match.matches) {
      if (!map.ContainsPid(match.cid, match.trainIdx) continue;
      const Pid pid = map.Pid(match.cid, match.trainIdx);
      if (seen_landmarks.count(pid) > 0) continue;
      observations.emplace_back(keypoints[match.queryIdx]);
      landmarks.push_back(map.global_t_point(pid));
      seen_landmarks.insert(pid);
      ++num_matches;
    }
  }
}

EstimatePoseResults EstimatePose(const Descriptors& descriptors, const Keypoints& keypoints, const SparseMap& map,
                                 const EstimatePoseParams& params) {
  const auto matching_cids = map.image_database().Query(descriptors, params.max_image_matches);
  if (matching_cids.empty()) {
    LOG(FATAL) << "No matching cids found.";
  }

  const auto image_matches = FindAndSortMatches(matching_cids, descriptors, map, params.max_num_total_feature_matches,
                                                params.check_point_3d_exists);
  std::vector<Eigen::Vector2d> observations;
  std::vector<Eigen::Vector3d> landmarks;
  GetMatchingObservationsAndLandmarks(image_matches, map, keypoints, observations, landmarks);

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
  for (const auto inlier_index : pose->inliers) {
    results.inlier_landmarks.emplace_back(landmarks[inlier_index]);
    results.inlier_observations.emplace_back(observations[inlier_index]);
  }
  return results;
}

EstimatePoseResults EstimatePose(const cv::Mat& image, const EstimatePoseParams& params, SparseMap& map) {
  Descriptors descriptors;
  Keypoints keypoints;
  vc::DetectFeatures(image, map.params().histogram_equalization, descriptors, keypoints);
  return EstimatePose(descriptors, keypoints, map, params);
}

EstimatePoseResults EstimatePose(const std::string& image_filename, const EstimatePoseParams& params, SparseMap& map) {
  const auto image = vc::LoadImage(filename);
  return EstimatePose(image, params, map);
}

}  // namespace sparse_map_matcher
