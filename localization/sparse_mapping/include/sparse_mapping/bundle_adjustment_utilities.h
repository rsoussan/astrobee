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

#ifndef SPARSE_MAPPING_BUNDLE_ADJUSTMENT_UTILITIES_H_
#define SPARSE_MAPPING_BUNDLE_ADJUSTMENT_UTILITIES_H_

#include <ff_common/eigen_vectors.h>
#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace sparse_mapping {
  // TODO(rsoussan): What does this do?
  /**
   * Close a loop with repeated images.
   **/
  void CloseLoop(sparse_mapping::SparseMap * s);

  /**
   * Improve the map with bundle adjustment. Vary only the cameras
   * between given indices.
   **/
  void BundleAdjust(bool fix_all_cameras, sparse_mapping::SparseMap * map,
                    std::set<int> const& fixed_cameras = std::set<int>());

  void BundleAdjustment(sparse_mapping::SparseMap * s,
                        ceres::LossFunction * loss,
                        const ceres::Solver::Options & options,
                        ceres::Solver::Summary * summary,
                        int first = 0, int last = std::numeric_limits<int>::max(),
                        bool fix_all_cameras = false,
                        std::set<int> const& fixed_cameras = std::set<int>());

 /**
 * Perform bundle adjustment. 
 * All poses and point values should be set to initial guesses and are modified to improved guesses when the function returns.
 **/
  void BundleAdjust(const BundleAdjustmentParams& params, const std::vector<Eigen::Matrix2Xd>& cid_to_keypoint_map,
                    const double focal_length, std::vector<Eigen::Affine3d>* cid_to_cam_t_global,
                    std::vector<std::map<int, int> >* pid_to_cid_fid, std::vector<Eigen::Vector3d>* pid_to_xyz,
                    ceres::Solver::Summary* summary);
  /**
   * Perform bundle adjustment.
   *
   * This variant assumes that all cameras see that same points. This is
   * meant to be used to do 2 or 3 camera refinements however it can do
   * N cameras just fine.
   *
   **/
  void BundleAdjustSmallSet(std::vector<Eigen::Matrix2Xd> const& features_n, double focal_length,
                            std::vector<Eigen::Affine3d>* cam_t_global_n, Eigen::Matrix3Xd* pid_to_xyz,
                            ceres::LossFunction* loss, ceres::Solver::Options const& options,
                            ceres::Solver::Summary* summary);


}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_BUNDLE_ADJUSTMENT_UTILITIES_H_
