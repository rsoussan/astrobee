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

#include <sparse_mapping/image_database.h>

#include <opencv2/core/core.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace sparse_mapping {
void InitializeCidFidToPid(int num_cid,
                           std::vector<std::map<int, int> > const& pid_to_cid_fid,
                           std::vector<std::map<int, int> > * cid_fid_to_pid);

cv::Mat LoadImage(const std::string& filename);

void DetectFeatures(const cv::Mat& image,
                      const bool histogram_equalization,
                      interest_point::Detector& detector,
                      cv::Mat* descriptors,
                      Eigen::Matrix2Xd* keypoints);


  // Performs a robust, ransac, solving for the essential matrix
  // between interest point measurements in x1 and x2.
bool RobustEssential(Eigen::Matrix3d const& k1, Eigen::Matrix3d const& k2, Eigen::Matrix2Xd const& x1,
                     Eigen::Matrix2Xd const& x2, Eigen::Matrix3d* e, std::vector<size_t>* vec_inliers,
                     std::pair<size_t, size_t> const& size1, std::pair<size_t, size_t> const& size2, double* error_max,
                     double precision);

// Solves for the RT (Rotation and Translation) from the essential
// matrix and x1 and x2. There are 4 possible, and this returns the
// best of the 4 solutions.
bool EstimateRTFromE(Eigen::Matrix3d const& k1, Eigen::Matrix3d const& k2, Eigen::Matrix2Xd const& x1,
                     Eigen::Matrix2Xd const& x2, Eigen::Matrix3d const& e, std::vector<size_t> const& vec_inliers,
                     Eigen::Matrix3d* r, Eigen::Vector3d* t);


}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_UTILITIES_H_
