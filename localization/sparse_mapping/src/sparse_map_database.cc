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

void SparseMapDatabase::SetPoses(const CidPoseMap& cid_to_cam_T_global) { cid_to_cam_T_global_ = cid_to_cam_T_global; }

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
    AddPose(cam_T_global);
  }
}

void SparseMapDatabase::AddPose(const Eigen::Affine3d& cam_T_global) {
  cid_to_cam_T_global_.emplace_back(cam_T_global);
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
  const double weight = feature_track.size() / (static_cast<double>(total_size));
  current_global_t_point = (1.0 - weight) * current_global_t_point + weight * global_t_point;
}

void SparseMapDatabase::InitializeCidFidPidMap() {
  cid_to_fid_to_pid_->clear();
  cid_to_fid_to_pid_->resize(NumCids(), std::map<int, int>());

  for (int pid = 0; pid < NumPoints(); ++pid) {
    for (const auto& cid_fid : feature_track(pid)) {
      cid_to_fid_to_pid_[cid_fid.first][cid_fid.second] = pid;
    }
  }
}

void SparseMapDatabase::AddControlPoints(const std::vector<std::string>& image_names,
                                         std::vector<ControlPoint>& control_points) {
  std::unordered_map<std::string, Cid> filename_to_cid;
  for (const auto& cid_filename_pair : cid_to_filename()) {
    filename_to_cid.emplace(cid_filename_pair.second, cid_filename_pair.first);
  }

  // Remove control points which contain images not contained in the map
  for (auto control_point = control_points.begin(); control_point != control_points.end(); ++control_point) {
    if (filename_to_cid.count(image_names[control_point->cid_left]) == 0 ||
        filename_to_cid.count(image_names[control_point->cid_right]) == 0) {
      control_point = control_points.erase(control_point);
      continue;
    } else {
      // Remap cids using map cids
      control_point->cid_left = filename_to_cid[image_names[control_point->cid_left]];
      control_point->cid_right = filename_to_cid[image_names[control_point->cid_right]];
      ++control_point;
    }
  }

  control_point_cid_to_keypoints_.resize(NumCids());
  const int num_control_points = control_points.size();
  control_point_pid_to_feature_track_.resize(num_control_points);
  control_point_pid_to_global_t_point_.resize(num_control_points);
  for (int pid = 0; pid < num_control_points; ++pid) {
    AddControlPoint(control_points[pid], pid);
  }
}

void SparseMapDatabase::AddControlPoint(const ControlPoint& control_point, const Pid pid) {
  control_point_cid_to_keypoints_[control_point.cid_left].emplace_back(control_point.keypoint_left);
  control_point_cid_to_keypoints_[control_point.cid_right].emplace_back(control_point.keypoint_right);
  // Use latest fid
  const Fid left_fid = control_point_cid_to_keypoints_[control_point.cid_left].size() - 1;
  const Fid right_fid = control_point_cid_to_keypoints_[control_point.cid_right].size() - 1;
  control_point_pid_to_feature_track_[pid][control_point.cid_left] = left_fid;
  control_point_pid_to_feature_track_[pid][control_point.cid_right] = right_fid;
  control_point_pid_to_global_t_point_[pid] = control_point.global_t_point;
}

void SparseMapDatabase::RemoveUnusedFeatures() {
  std::vector<bool> remove_fid;
  for (int cid = 0; cid < NumCids(); ++cid) {
    // Fid order is always sequential, so when an unused fid is removed,
    // all larger fids need to shift down by one
    std::unordered_map<Fid, Fid> fid_remapping;
    int new_fid = 0;
    for (int fid = 0; fid < NumFeatures(cid); ++fid) {
      if (!ContainsPid(cid, fid)) {
        remove_fid.emplace_back(true);
      } else {
        remove_fid.emplace_back(false);
        fid_remapping[fid] = new_fid++;
      }
    }
    lc::RemoveElements(removed_fid, desciptors(cid));
    lc::RemoveElements(removed_fid, keypoints(cid));
    RemapFeatureTracks(cid, fid_remapping);
  }
  InitializeCidFidPidMap();
}

void SparseMapDatabase::RemapFeatureTracks(const Cid cid, const std::unordered_map<Fid, Fid>& fid_remapping) {
  auto& fid_to_pid = fid_to_pid(cid);
  // Remap each fid to the new fid for each feature track containing a feature from the given cid
  for (auto& fid_pid : fid_to_pid) {
    const Fid fid = fid_pid.first;
    const Pid pid = fid_pid.second;
    auto& feature_track = feature_track(pid);
    feature_track.at(cid) = fid_remapping.at(fid);
  }
}
}  // namespace sparse_mapping
