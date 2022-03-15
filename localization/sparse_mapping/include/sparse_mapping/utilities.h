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

#ifndef SPARSE_MAPPING_UTILITIES_H_
#define SPARSE_MAPPING_UTILITIES_H_

#include <ff_common/eigen_vectors.h>
#include <sparse_mapping/image_database.h>
#include <camera/camera_model.h>
#include <camera/camera_params.h>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include <map>
#include <string>
#include <vector>

namespace sparse_mapping {

// Non-member function InitializeCidFidToPid() that we will use within
// this class and outside of it as well.
void InitializeCidFidToPid(int num_cid,
                           std::vector<std::map<int, int> > const& pid_to_cid_fid,
                           std::vector<std::map<int, int> > * cid_fid_to_pid);

/**
 * Estimate the camera pose for a set of image descriptors and keypoints.
 * Non-member function. We will invoke it both from within
 * the SparseMap class and from outside of it.
 **/
// TODO(rsoussan): Make params file for this
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
              std::vector<int> * cid_list);

}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_UTILITIES_H_
