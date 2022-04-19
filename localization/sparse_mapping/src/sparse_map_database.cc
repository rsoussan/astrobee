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

#include <sparse_mapping/sparse_map_database.h>

namespace sparse_mapping {
void SparseMapDatabase::ResizeFeatureMaps() {
  const int num_cids = NumCIDs();
  cid_to_keypoint_map_.resize(num_cids);
  cid_to_descriptor_map_.resize(num_cids);
}

void SparseMapDatabase::AddImagesAndFeatures(const SparseMapDatabase& map) {
  const int num_initial_cids = NumCIDs();
  const int num_final_cids = num_initial_cids + map.NumCIDs();
  cid_to_filename_.reserve(num_final_cids);
  cid_to_keypoint_map_.reserve(num_final_cids);
  cid_to_cam_t_global_.reserve(num_final_cids);
  cid_to_descriptor_map_.reserve(num_final_cids);
  for (int cid = 0; cid < map.NumCIDs(); ++cid) {
    cid_to_filename_.emplace_back(map.cid_to_filename_[cid]);
    cid_to_keypoint_map_.emplace_back(map.cid_to_keypoint_map_[cid]);
    cid_to_descriptor_map_.emplace_back(map.cid_to_descriptor_map_[cid]);
  }
}

void SparseMapDatabase::AddPoses(const std::vector<Eigen::Affine3d>& cam_T_global_vec) {
  for (const auto& cam_T_global : cam_T_global_vec) {
    cid_to_cam_t_global_.emplace_back(cam_T_global);
  }
}

void SparseMapDatabase::InitializeCidFidToPid() {
  sparse_mapping::InitializeCidFidToPid(NumCIDs(),
                                        pid_to_cid_fid_,
                                        &cid_fid_to_pid_);
}

  std::vector<cv::Mat> SparseMapDatabase::Features(const int cid) const {
      std::vector<cv::Mat> features;
      const int num_features = Keypoints(cid).outerSize();
      for (int i = 0; i < num_features; ++i) {
        features.emplace_back(GetDescriptor(cid, i));
      }
  }

  std::vector<std::vector<cv::Mat>> SparseMapDatabase::AllFeatures() const {
      std::vector<std::vector<cv::Mat>> all_features;
      const int num_cids = NumCIDs();
      for (int cid = 0; cid < num_cids; ++cid) {
        all_features.emplace_back(Features(cid));
      }
  }

int SparseMapDatabase::NumFeatures() const {
  int num_features = 0;
  for (int cid = 0; cid < NumCIDs(); ++cid) {
    total_features += Keypoints(cid).outerSize();
  }
  return num_features;
}

void SparseMapDatabase::Transform(const Eigen::Affine3d& new_global_T_global) {
  for (auto& point : pid_to_xyz_) {
    point = new_global_T_global * point;
  }

  const Eigen::Affine3d global_T_new_global = new_global_T_global.inverse();
  for (auto& cam_T_global : cid_to_cam_t_global_) {
    cam_T_global = cam_T_global * global_T_new_global;
  }
}

void SparseMapDatabase::AddTrack(const Eigen::Vector3d& global_t_point, const std::map<int, int>& cid_to_fid) {
  // Assumes feature points and descriptors are already in the map
  pid_to_cid_fid_.emplace_back(cid_to_fid);
  pid_to_xyz_.emplace_back(global_t_point);
}

void SparseMapDatabase::ExtendTrack(const int pid, const std::map<int, int>& cid_to_fid) {
  // Assumes feature points and descriptors are already in the map
  auto& feature_track = feature_track(pid);
  feature_track.insert(cid_to_fid.begin(), cid_to_fid.end());
}

void SparseMapDatabase::MergeTrack(const int pid, const Eigen::Vector3d& global_t_point,
                                   const std::map<int, int>& cid_to_fid) {
  // Assumes feature points and descriptors are already in the map
  ExtendTrack(pid, cid_to_fid);
  auto& current_global_t_point = global_t_point(pid);
  const int total_size = feature_track(pid).size();
  // TODO(rsoussan): Allow for user defined weight?
  const double weight = cid_to_fid.size()/(static_cast<double>(total_size));
  current_global_t_point = (1.0 - weight)*current_global_t_point + weight*global_t_point;
}
}  // namespace sparse_mapping
