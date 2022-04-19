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
#include <sparse_mapping/sparse_mapping.h>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include <map>
// #include <numeric>
#include <set>
#include <string>
#include <vector>
// #include <utility>
// #include <limits>

namespace sparse_mapping {
/**
 * A database of image features, 3D points for select features, camera poses in a global frame, and image files.
  * Terminology used in this code:
  *  CID = Camera ID. A unique ID for each camera.  A camera is created for each image.
  *  PID = Point ID. A unique ID for each point. Can be observed by
  *        multiple cameras.
  *  FID = Feature ID. A unique ID for each feature observing a PID. Each CID has a number of FIDs which 
  *  provide both image space locations (keypoints) and feature decriptors.
  *  filename = Image filename.  Each image filename is assigned a unique CID
  *
  * cid_to_filename - Indexed on CID. Provides the filename for each CID.
  * cid_to_keypoint_map - Indexed first on CID and then on each FID. Provides the feature image space location (keypoint) for the FID.
  * cid_to_descriptor_map - Indexed on CID then on each FID. Provides the feature descriptor for the FID.
  * cid_to_cam_t_global - Indexed on CID. Provides the affine camera_T_global transform.
  * cid_fid_to_pid - Indexed on CID and then on each FID.  Provides the PID for each FID.
  * pid_to_cid_fid - Indexed on PID. Provides the CID and its FID that observed the PID.
  * pid_to_xyz - Indexed on PID. Provides the XYZ position of each PID.
 **/
class SparseMapDatabase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int NumCIDs() const {return static_cast<int>(cid_to_filename_.size());}

  const std::string& Filename(int cid) const {return cid_to_filename_[cid];}

  void ResizeFeatureMaps();

  // Assumes cid filenames, keypoints, and descriptors have already been added using AddImagesAndFeatures
  void AddPoses(const std::vector<Eigen::Affine3d>& cam_T_global_vec);

  // TODO(rsoussan): these should all be lower snake case!!!!

  const std::vector<Eigen::Affine3d>& cid_to_cam_T_global() const { return cid_to_cam_t_global_; }

  const Eigen::Affine3d& CamTGlobal(int cid) const {return cid_to_cam_t_global_[cid];}

  const Eigen::Matrix2Xd& Keypoints(int cid) const {return cid_to_keypoint_map_[cid];}

  const Eigen::Matrix2Xd& Keypoint(int cid, int fid) {return Keypoints(cid).col(fid);}

  const cv::Mat& Descriptor(int cid, int fid) const { return cid_to_descriptor_map_[cid].row(fid);}

  const std::map<int, int>& FidToPid(int cid) const {return cid_fid_to_pid_[cid];}

  int Pid(int cid, int fid) const {return cid_fid_to_pid_[cid][fid];}

  bool ContainsPid(int cid, int fid) const { return (cid_fid_to_pid[cid].count(fid) > 0);}

  int NumPoints() const {return pid_to_xyz_.size();}

  // TODO(rsoussan): rename this to global_t_point
  const Eigen::Vector3d& Point(int pid) const {return pid_to_xyz_[pid];}

  const std::map<int, int>& CidToFid(int pid) const {return pid_to_cid_fid_[pid];}

  int NumObservations() const {return std::accumulate(pid_to_cid_fid_.begin(),
                                                                pid_to_cid_fid_.end(),
                                                                0, [](size_t v, const std::map<int, int>& map)
                                                                { return v + map.size(); }); }

  // TODO(rsoussan): rename this??
  void AddImagesAndFeatures(const SparseMapDatabase& map);

  // TODO(rsoussan): What is the framing of T?? Update!
  void Transform(Eigen::Affine3d const& T) {
    sparse_mapping::TransformCamerasAndPoints(T, &cid_to_cam_t_global_, &pid_to_xyz_);
  }

  void InitializeCidFidToPid();

  FeatureSet Features(const int cid) const;

  FetaureSets AllFeatures() const;

  int NumFeatures() const;

  const std::string& filename(const int cid) const { return cid_to_filename_[cid]; }

  const cv::Mat& descriptors(const int cid) const { return cid_to_descriptor_map_[cid]; }

  const Eigen::Matrix2Xd& keypoints(const int cid) const { return cid_to_keypoint_map_[cid]; }

  const std::map<int, int>& feature_track(const int pid) const { return pid_to_cid_fid_[pid]; }

  // Use pid_to_cid_fid_ instead of pid_to_xyz since this is filled sooner in the mapping pipeline
  int NumPoints() const { return pid_to_cid_fid_.size(); }

  int NumFeatures(const int cid) const { return cid_to_keypoint_map_[cid].cols(); }

  const Eigen::Affine3d& cam_T_global(const int cid) const { return cid_to_cam_t_global_[cid]; }

  void AddTrack(const Eigen::Vector3d& global_t_point, const std::map<int, int>& cid_to_fid);

 protected:
  cv::Mat& descriptors(const int cid) { return cid_to_descriptor_map_[cid]; }

  Eigen::Matrix2Xd& keypoints(const int cid) { return cid_to_keypoint_map_[cid]; }

  Eigen::Affine3d& cam_T_global(const int cid) { return cid_to_cam_t_global_[cid]; }

  // TODO(rsoussan): These should be private
  // TODO(rsoussan): Make maps unodered?
  // stored in map file
  std::vector<std::string> cid_to_filename_;
  // TODO(bcoltin) replace Eigen2Xd everywhere with one keypoint class
  std::vector<Eigen::Matrix2Xd > cid_to_keypoint_map_;
  std::vector<cv::Mat> cid_to_descriptor_map_;
  std::vector<Eigen::Affine3d > cid_to_cam_t_global_;
  std::vector<std::map<int, int> > cid_fid_to_pid_;
  std::vector<Eigen::Vector3d> pid_to_xyz_;
  std::vector<std::map<int, int> > pid_to_cid_fid_;

  // If datastructure is available, match only pairs of cids
  // that are present in it (this info can come from example from
  // a map that was built previously with the same images but
  // a different descriptor.
  // TODO(rsoussan): Make new file containing this! store elsewhere!
  std::map< int, std::set<int> > cid_to_cid_;  // TODO(oalexan1): Need not be a member, remove

  // Optional user defined 3D points and image observations.
  // Enables manually registering the sparse map with control points
  // from a 3D model
  // TODO(rsoussan): Remove these??
  std::vector<Eigen::Matrix2Xd> user_cid_to_keypoint_map_;
  std::vector<std::map<int, int> > user_pid_to_cid_fid_;
  std::vector<Eigen::Vector3d> user_pid_to_xyz_;
};
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_SPARSE_MAP_DATABASE_H_
