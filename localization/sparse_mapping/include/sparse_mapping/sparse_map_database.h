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

  // TODO(rsoussan): Rename this to NumCids, return int
  // TODO(rsoussan): deprecate
  size_t GetNumFrames() const {return cid_to_filename_.size();}

  int num_cameras() const {return static_cast<int>(cid_to_filename_.size());}
  /**
   * Get the filename of a keyframe in the map.
   **/
  // TODO(rsoussan): Rename this to CidFilename
  const std::string & GetFrameFilename(int frame) const {return cid_to_filename_[frame];}

  void ResizeFeatureMaps();

  /**
   * Get the global camera transform for a keyframe in the map.
   **/
  // TODO(rsoussan): Rename this to GlobalTCid
  const Eigen::Affine3d & GetFrameGlobalTransform(int frame) const
        {return cid_to_cam_t_global_[frame];}
  // TODO(rsoussan): Rename
  void SetFrameGlobalTransform(int frame, const Eigen::Affine3d & transform) {
    cid_to_cam_t_global_[frame] = transform;
  }
  /**
   * Get the keypoint coordinates in the specified frame.
   **/
  // TODO(rsoussan): Rename
  const Eigen::Matrix2Xd & GetFrameKeypoints(int frame) const {return cid_to_keypoint_map_[frame];}
  /**
   * Get the descriptor for a frame and feature.
   **/
  cv::Mat GetDescriptor(int frame, int fid) const { return cid_to_descriptor_map_[frame].row(fid);}
  /**
   * Returns map of feature ids to landmark ids for the specified frame.
   **/
  // TODO(rsoussan): Rename
  const std::map<int, int> & GetFrameFidToPidMap(int frame) const {return cid_fid_to_pid_[frame];}

  // access map landmarks
  /**
   * Get the number of landmark points in the map.
   **/
  size_t GetNumLandmarks() const {return pid_to_xyz_.size();}
  /**
   * Get the global position of the specified landmark.
   **/
  Eigen::Vector3d GetLandmarkPosition(int landmark) const {return pid_to_xyz_[landmark];}
  /**
   * Return a map for a specified landmark, matching the ids of all the keyframes that landmark
   * was seen in to the feature id within that frame.
   **/
  const std::map<int, int> & GetLandmarkCidToFidMap(int landmark) const {return pid_to_cid_fid_[landmark];}

  /**
   * Return the number of observations. 
   **/
  size_t GetNumObservations() const {return std::accumulate(pid_to_cid_fid_.begin(),
                                                                pid_to_cid_fid_.end(),
                                                                0, [](size_t v, std::map<int, int> const& map)
                                                                { return v + map.size(); }); }
  /**
   * Return the transform to real world coordinates.
   **/
  Eigen::Affine3d GetWorldTransform() const {return world_transform_;}

  /**
   * Apply given transform to camera positions and 3D points
   **/
  void ApplyTransform(Eigen::Affine3d const& T) {
    sparse_mapping::TransformCamerasAndPoints(T, &cid_to_cam_t_global_, &pid_to_xyz_);
  }

  // construct from pid_to_cid_fid
  void InitializeCidFidToPid();

  FeatureSet GetCidFeatures(const int cid) const;

  FetaureSets GetAllCidFeatures() const;

  int NumFeatures() const;

  const std::string& filename(const int cid) const { return cid_to_filename_[cid]; }

  const cv::Mat& descriptor_map(const int cid) const { return cid_to_descriptor_map_[cid]; }

  const Eigen::Matrix2Xd& keypoint_map(const int cid) const { return cid_to_keypoint_map_[cid]; }

  int num_points() const { return pid_to_xyz_.size(); }

  int num_features(const int cid) const { return cid_to_keypoint_map_[cid].cols(); }

 protected:
  cv::Mat& descriptor_map(const int cid) { return cid_to_descriptor_map_[cid]; }

  Eigen::Matrix2Xd& keypoint_map(const int cid) { return cid_to_keypoint_map_[cid]; }

  // TODO(rsoussan): These should be private
  // stored in map file
  std::vector<std::string> cid_to_filename_;
  // TODO(bcoltin) replace Eigen2Xd everywhere with one keypoint class
  std::vector<Eigen::Matrix2Xd > cid_to_keypoint_map_;
  std::vector<cv::Mat> cid_to_descriptor_map_;
  std::vector<Eigen::Affine3d > cid_to_cam_t_global_;
  // generated on load
  std::vector<std::map<int, int> > cid_fid_to_pid_;
  std::vector<std::map<int, int> > pid_to_cid_fid_;
  std::vector<Eigen::Vector3d> pid_to_xyz_;

  // If datastructure is available, match only pairs of cids
  // that are present in it (this info can come from example from
  // a map that was built previously with the same images but
  // a different descriptor.
  // TODO(rsoussan): Make new file containing this! store elsewhere!
  std::map< int, std::set<int> > cid_to_cid_;  // TODO(oalexan1): Need not be a member, remove

  // Optional user defined 3D points and image observations.
  // Enables manually registering the sparse map with control points
  // from a 3D model
  std::vector<Eigen::Matrix2Xd> user_cid_to_keypoint_map_;
  std::vector<std::map<int, int> > user_pid_to_cid_fid_;
  std::vector<Eigen::Vector3d> user_pid_to_xyz_;
};
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_SPARSE_MAP_DATABASE_H_
