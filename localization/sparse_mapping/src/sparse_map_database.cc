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
  const int num_cids = NumCids();
  cid_to_keypoints_.resize(num_cids);
  cid_to_descriptors_.resize(num_cids);
}

void SparseMapDatabase::SetPoses(const CidPoseMap& cid_to_cam_T_global) {
  cid_to_cam_T_global_ = cid_to_cam_T_global;
}

void SparseMapDatabase::AddImagesAndFeatures(const SparseMapDatabase& map) {
  const int num_initial_cids = NumCids();
  const int num_final_cids = num_initial_cids + map.NumCids();
  cid_to_filename_.reserve(num_final_cids);
  cid_to_keypoints_.reserve(num_final_cids);
  cid_to_cam_T_global_.reserve(num_final_cids);
  cid_to_descriptors_.reserve(num_final_cids);
  for (int cid = 0; cid < map.NumCids(); ++cid) {
    cid_to_filename_.emplace_back(map.filename(cid));
    cid_to_keypoints_.emplace_back(map.keypoints(cid));
    cid_to_descriptors_.emplace_back(map.descriptors(cid));
  }
}

void SparseMapDatabase::AddPoses(const std::vector<Eigen::Affine3d>& cam_T_global_vec) {
  for (const auto& cam_T_global : cam_T_global_vec) {
    cid_to_cam_T_global_.emplace_back(cam_T_global);
  }
}

void SparseMapDatabase::InitializeCidFidPidMap() {
  InitializeCidFidPidMap(NumCids(),
                                        pid_to_feature_track_,
                                        &cid_to_fid_to_pid_);
}

  std::vector<Descriptors> SparseMapDatabase::AllDescriptors() const {
      std::vector<Descriptors> all_descriptors;
      const int num_cids = NumCids();
      for (int cid = 0; cid < num_cids; ++cid) {
        all_descriptors.emplace_back(Descriptors(cid));
      }
  }

void SparseMapDatabase::Transform(const Eigen::Affine3d& new_global_T_global) {
  for (auto& point : pid_to_global_t_point_) {
    point = new_global_T_global * point;
  }

  const Eigen::Affine3d global_T_new_global = new_global_T_global.inverse();
  for (auto& cam_T_global : cid_to_cam_T_global_) {
    cam_T_global = cam_T_global * global_T_new_global;
  }
}

void SparseMapDatabase::AddTrack(const Eigen::Vector3d& global_t_point, const FeatureTrack& feature_track) {
  // Assumes feature keypoints and descriptors are already in the map
  pid_to_feature_track_.emplace_back(feature_track);
  pid_to_global_t_point_.emplace_back(global_t_point);
}

void SparseMapDatabase::ExtendTrack(const int pid, const FeatureTrack& feature_track_to_add) {
  // Assumes feature keypoints and descriptors are already in the map
  auto& feature_track = feature_track(pid);
  feature_track.insert(feature_track_to_add.begin(), feature_track_to_add.end());
}

void SparseMapDatabase::MergeTrack(const int pid, const Eigen::Vector3d& global_t_point,
                                   const FeatureTrack& feature_track) {
  // Assumes feature keypoints and descriptors are already in the map
  ExtendTrack(pid, feature_track);
  auto& current_global_t_point = global_t_point(pid);
  const int total_size = feature_track(pid).size();
  // TODO(rsoussan): Allow for user defined weight?
  const double weight = feature_track.size()/(static_cast<double>(total_size));
  current_global_t_point = (1.0 - weight)*current_global_t_point + weight*global_t_point;
}
}  // namespace sparse_mapping
