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

#ifndef SPARSE_MAPPING_SPARSE_MAP_DATABASE_H_
#define SPARSE_MAPPING_SPARSE_MAP_DATABASE_H_

#include <ff_common/eigen_vectors.h>
#include <localization_common/utilities.h>
#include <sparse_mapping/datatypes.h>

#include <Eigen/Geometry>

#include <string>
#include <utility>
#include <vector>

namespace sparse_mapping {
/**
 * A database of image filenames, detected image features, 3D points for select features, feature tracks, and camera poses.
  *  Terminology used in this code:
  *  Cid = Camera id. A unique sequential id for each camera, where a camera (containing a pose and detected features)
  *        is created for each image.
  *  Pid = Point id. A unique sequential id for each 3d point created for a feature track in a global frame.
  *  Fid = Feature id. A unique id for each detected feature in all the images.
  *  Keypoint = The location of a feature in image space.
  *  Descriptor = The descriptor vector for an image feature.
  *  FeatureTrack = Detected features from different unique images that match with eachother.  **/
class SparseMapDatabase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void ResizeFeatureMaps();

  void SetPoses(const CidPoseMap& cid_to_cam_T_global);

  bool ContainsPid(const Cid cid, const int fid) const { return (cid_to_fid_to_pid[cid].count(fid) > 0);}

  int NumCids() const {return static_cast<int>(cid_to_filename_.size());}

  int NumPoses() const {return static_cast<int>(cid_to_cam_T_global_.size());}

  // Assumes cid filenames, keypoints, and descriptors have already been added using AddImagesAndFeatures
  void AddPoses(const std::vector<Eigen::Affine3d>& cam_T_global_vec);

  // Assumes cid filenames, keypoints, and descriptors have already been added using AddImagesAndFeatures
  void AddPose(const Eigen::Affine3d > &cam_T_global);

  // Counts features that have been included in existing points
  int NumUsedFeatures() const {return std::accumulate(pid_to_feature_track_.begin(),
                                                                pid_to_feature_track_.end(),
                                                                0, [](size_t size, const FeatureTrack& feature_track)
                                                                { return size + feature_track.size(); }); }

  // TODO(rsoussan): rename this??
  void AddImagesAndFeatures(const SparseMapDatabase& map);

  void Transform(const Eigen::Affine3d& new_global_T_global);

  // From pid_to_feature_track_, create cid_to_fid_to_pid for lookup.
  void InitializeCidFidPidMap();

  std::vector<Descriptors> AllDescriptors() const;

  int NumFeatures() const {return std::accumulate(cid_to_keypoints_.begin(),
                                                                cid_to_keypoints_.end(),
                                                                0, [](size_t size, const Keypoints& keypoints)
                                                                { return size + keypoints.size(); }); }



  // Use pid_to_feature_track_ instead of pid_to_global_t_point since this is filled sooner in the mapping pipeline
  int NumPoints() const { return pid_to_feature_track_.size(); }

  int NumFeatures(const Cid cid) const { return cid_to_keypoints_[cid].size(); }

  int FeatureTrackLength(const Pid pid) const { return feature_track(pid).size(); }

  void AddTrack(const Eigen::Vector3d& global_t_point, const FeatureTrack& feature_track);

  void ExtendTrack(const Pid pid, const FeatureTrack& feature_track_to_add);

  void MergeTrack(const Pid pid, const Eigen::Vector3d& global_t_point, const FeatureTrack& feature_track);

void RemovePoints(const std::vector<bool>& indices_to_remove) {
  localization_common::RemoveElements(indices_to_remove, pid_to_feature_track_);
  localization_common::RemoveElements(indices_to_remove, pid_to_global_t_point_);
}

  void RemovePoint(const Pid pid) {
      pid_to_global_t_point_.erase(pid_to_global_t_point_.begin() + pid);
      pid_to_feature_track_.erase(pid_to_feature_track_.begin() + pid);
  }

  // Accessors
  const std::string& filename(const Cid cid) const {return cid_to_filename_[cid];}

  const CidPoseMap& cid_to_cam_T_global() const { return cid_to_cam_T_global_; }

  const Eigen::Affine3d& cam_T_global(const Cid cid) const {return cid_to_cam_T_global_[cid];}

  const Keypoints& keypoints(const Cid cid) const {return cid_to_keypoints_[cid];}

  const Keypoint& keypoint(const Cid cid, const int fid) {return keypoints(cid)[fid];}

  const Keypoint& keypoint(const std::pair<Cid, Fid>& cid_fid) { return keypoint(cid_fid.first, cid_fid.second); }

  const Descriptors& descriptors(const Cid cid) const { return cid_to_descriptors_[cid];}

  const Descriptor& descriptor(const Cid cid, const int fid) const { return desciptors(cid)[fid];}

  const FidPidMap& fid_to_pid(const Cid cid) const {return cid_to_fid_to_pid_[cid];}

  Pid pid(const Cid cid, const int fid) const {return cid_to_fid_to_pid_[cid][fid];}

  const Eigen::Vector3d& global_t_point(const Pid pid) const {return pid_to_global_t_point_[pid];}

  const FeatureTrack& feature_track(const Pid pid) const { return pid_to_feature_track_[pid]; }

  const Eigen::Affine3d& cam_T_global(const Cid cid) const { return cid_to_cam_T_global_[cid]; }

  const PidPoseMap& cid_to_cam_T_global() const { return cid_to_cam_T_global_; }

 protected:
  const PidPointMap& pid_to_global_t_point() const { return pid_to_global_t_point_; }

  const PidFeatureTrackMap& pid_to_feature_track() const { return pid_to_feature_track_; }

  const CidFilenameMap& cid_to_filename() const {return cid_to_filename_; }

  const CidKeypointsMap& cid_to_keypoints() const { return cid_to_keypoints_; }

  const CidDescriptorsMap& cid_to_descriptors() const  { return cid_to_descriptors_; }

  const CidKeypointsMap& fixed_cid_to_keypoints() const { return fixed_cid_to_keypoints_; }

  const PidFeatureTrackMap& fixed_pid_to_feature_track() const { return fixed_pid_to_feature_track_; }

  Keypoints& keypoints(const Cid cid) {return cid_to_keypoints_[cid];}

  Descriptors& descriptors(const Cid cid) { return cid_to_descriptors_[cid];}

  Eigen::Affine3d& cam_T_global(const Cid cid) { return cid_to_cam_T_global_[cid]; }

  FeatureTrack& feature_track(const Pid pid) { return pid_to_feature_track_[pid]; }

  Eigen::Vector3d& global_t_point(const Pid pid) {return pid_to_global_t_point_[pid];}

  PidPoseMap& cid_to_cam_T_global() { return cid_to_cam_T_global_; }

  PidPointMap& pid_to_global_t_point() { return pid_to_global_t_point_; }

  PidFeatureTrackMap& pid_to_feature_track() { return pid_to_feature_track_; }

  PidPointMap& fixed_pid_to_global_t_point() { return fixed_pid_to_global_t_point_; }

 private:
  CidFilenameMap cid_to_filename_;
  CidKeypointsMap cid_to_keypoints_;
  CidDescriptorsMap cid_to_descriptors_;
  CidPoseMap cid_to_cam_T_global_;
  PidPointMap pid_to_global_t_point_;
  PidFeatureTrackMap pid_to_feature_track_;
  CidFidPidMap cid_to_fid_to_pid_;

  // Fix point sets that shouldn't be optimized, allows for manual registration
  // of map points using known point locations.
  CidKeypointsMap fixed_cid_to_keypoints_;
  PidFeatureTrackMap fixed_pid_to_feature_track_;
  PidPointMap fixed_pid_to_global_t_point_;
};
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_SPARSE_MAP_DATABASE_H_
