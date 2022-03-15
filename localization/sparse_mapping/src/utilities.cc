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

DEFINE_int32(num_extra_localization_db_images, 0,
             "Match this many extra images from the Vocab DB, only keep num_similar.");

DEFINE_bool(verbose_localization, false,
            "If true, list the images most similar to the one being localized.");

namespace sparse_mapping {
// A non-member Localize() function that can be invoked for a non-fully
// formed map.
bool Localize(cv::Mat const& test_descriptors,
              Eigen::Matrix2Xd const& test_keypoints,
              camera::CameraParameters const& camera_params,
              camera::CameraModel* pose,
              std::vector<Eigen::Vector3d>* inlier_landmarks,
              std::vector<Eigen::Vector2d>* inlier_observations,
              int num_cid,
              std::string const& detector_name,
              const ImageDatabas& image_database,
              int num_similar,
              std::vector<std::string> const& cid_to_filename,
              std::vector<cv::Mat> const& cid_to_descriptor_map,
              std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
              std::vector<std::map<int, int> > const& cid_fid_to_pid,
              std::vector<Eigen::Vector3d> const& pid_to_xyz,
              int num_ransac_iterations, int ransac_inlier_tolerance,
              int early_break_landmarks, int histogram_equalization,
              std::vector<int> * cid_list) {
  std::vector<int> indices;
  // Notice that we request more similar images than what we need. We'll prune them below.
  // TODO(rsoussan): why?
  const int max_results = num_similar + FLAGS_num_extra_localization_db_images;
  // Query the vocab tree.
  if (cid_list == NULL)
    indices = image_database.Query(test_descriptors, max_results);
  else
    indices = *cid_list;
  if (indices.empty()) {
    LOG(WARNING) << "Localizing against all keyframes as the vocab database is missing.";
    // Use all images, as no tree is available.
    for (int cid = 0; cid < num_cid; cid++)
      indices.push_back(cid);
  }

  // To turn on verbose localization for debugging
  // google::SetCommandLineOption("verbose_localization", "true");

  // Find matches to each image in map. Do this in two passes. First,
  // find matches to all map images, then keep only num_similar
  // best matched images, then localize against those.

  // We will not localize using all images having matches, there are too
  // many false positives that way. Instead, limit ourselves to the images
  // which have most observations in common with the current one.
  std::vector<int> similarity_rank(indices.size(), 0);
  std::vector<std::vector<cv::DMatch> > all_matches(indices.size());
  int total = 0;
  // TODO(oalexan1): Use multiple threads here?
  for (size_t i = 0; i < indices.size(); i++) {
    int cid = indices[i];
    interest_point::FindMatches(test_descriptors,
                                cid_to_descriptor_map[cid],
                                &all_matches[i]);

    for (size_t j = 0; j < all_matches[i].size(); j++) {
      if (cid_fid_to_pid[cid].count(all_matches[i][j].trainIdx) == 0)
        continue;
      similarity_rank[i]++;
    }
    if (FLAGS_verbose_localization)
      std::cout << "Overall matches and validated matches to: "
                << cid_to_filename[cid] << ": "
                << all_matches[i].size() << " "
                << similarity_rank[i] << "\n";
    total += similarity_rank[i];
    if (total >= early_break_landmarks)
      break;
  }

  std::vector<Eigen::Vector2d> observations;
  std::vector<Eigen::Vector3d> landmarks;
  std::vector<int> highly_ranked = ff_common::rv_order(similarity_rank);
  int end = std::min(static_cast<int>(highly_ranked.size()), num_similar);
  std::set<int> seen_landmarks;
  if (FLAGS_verbose_localization)
    std::cout << "Similar images: ";
  for (int i = 0; i < end; i++) {
    int cid = indices[highly_ranked[i]];
    std::vector<cv::DMatch>* matches = &all_matches[highly_ranked[i]];
    int num_matches = 0;
    for (size_t j = 0; j < matches->size(); j++) {
      if (cid_fid_to_pid[cid].count(matches->at(j).trainIdx) == 0)
        continue;
      const int landmark_id = cid_fid_to_pid.at(cid).at(matches->at(j).trainIdx);
      if (seen_landmarks.count(landmark_id) > 0)
        continue;
      Eigen::Vector2d obs(test_keypoints.col(matches->at(j).queryIdx)[0],
                          test_keypoints.col(matches->at(j).queryIdx)[1]);
      observations.push_back(obs);
      landmarks.push_back(pid_to_xyz[landmark_id]);
      seen_landmarks.insert(landmark_id);
      num_matches++;
    }
    if (FLAGS_verbose_localization && num_matches > 0)
      std::cout << " " << cid_to_filename[cid];
  }
  if (FLAGS_verbose_localization) std::cout << std::endl;

  int ret = RansacEstimateCamera(landmarks, observations,
                                 num_ransac_iterations,
                                 ransac_inlier_tolerance, pose,
                                 inlier_landmarks, inlier_observations,
                                 FLAGS_verbose_localization);
  return (ret == 0);
}

// Non-member InitializeCidFidToPid() function, useful
// without a fully-formed map.
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
