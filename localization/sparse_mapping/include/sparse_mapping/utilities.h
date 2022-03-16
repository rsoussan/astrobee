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
#include <sparse_mapping/estimate_pose_params.h>
#include <sparse_mapping/estimate_pose_results.h>
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
 **/
EstimatePoseResults EstimatePose(
  const cv::Mat& descriptors,  // TODO(rsoussan): change this to vector of descriptors
                               // TODO(rsoussan): change this to vector of Eigen::Vector2ds
  const Eigen::Matrix2Xd& keypoints, const SparseMap& map, const EstimatePoseParams& params);

}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_UTILITIES_H_
