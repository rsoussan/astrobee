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

#ifndef VISION_COMMON_MAPPING_UTILITIES_H_
#define VISION_COMMON_MAPPING_UTILITIES_H_

#include <Eigen/Geometry>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace vision_common {
boost::optional<Eigen::Vector3d> Triangulate(const Eigen::Matrix3d& intrinsics,
                                             const std::vector<Eigen::Affine3d>& camera_T_worlds,
                                             const Keypoints& keypoints);

boost::optional<double> AngleBetweenRays(const Eigen::Vector3d& a_t_p, const Eigen::Vector3d& b_t_p);

// Find the maximum angle between n rays intersecting at given point.
double MaxAngleBetweenCameraRays(const FeatureTrack& feature_track, const Eigen::Vector3d& global_t_point,
                                 const CidPoseMap& cid_to_global_t_cam);

void DetectFeatures(const cv::Mat& image, const bool histogram_equalization, DynamicDetector& detector,
                    Descriptors& descriptors, Keypoints& keypoints);

bool EstimateEssentialMatrix(const Eigen::Matrix3d& intrinsics_1, const Eigen::Matrix3d& intrinsics_2,
                             const Keypoints& keypoints_1, const Keypoints& keypoints_2,
                             const std::pair<int, int>& image_size_1, const std::pair<int, int>& image_size_2,
                             const double precision, Eigen::Matrix3d& essential_matrix, std::vector<int>& inliers,
                             double& max_error);

bool EstimateEssentialMatrix(const Eigen::Matrix3d& intrinsics_1, const Eigen::Matrix3d& intrinsics_2,
                             const Eigen::Matrix2Xd& keypoints_1, const Eigen::Matrix2Xd& keypoints_2,
                             const std::pair<int, int>& image_size_1, const std::pair<int, int>& image_size_2,
                             const double precision, Eigen::Matrix3d& essential_matrix, std::vector<int>& inliers,
                             double& max_error);

boost::optional<Eigen::Isometry3d> EstimateNormalizedPoseFromEssentialMatrix(
  const Eigen::Matrix3d& intrinsics_1, const Eigen::Matrix3d& intrinsics_2, const Keypoints& keypoints_1,
  const Keypoints& keypoints_2, const Eigen::Matrix3d& essential_matrix, const std::vector<int>& inliers);

boost::optional<Eigen::Affine3d> MatchImages(const Keypoints& keypoints_a, const Keypoints& keypoints_b,
                                             const Descriptors& descriptors_a, const Descriptors& descriptors_b,
                                             const camera::CameraParameters& camera_params, const int max_num_matches,
                                             const int min_num_inliers_for_valid_match,
                                             std::vector<cv::DMatch>& inlier_matches);

boost::optional<Eigen::Affine3d> EstimateNormalizedRelativeAffine3D(
  const Eigen::Matrix2Xd& keypoints_a, const Eigen::Matrix2Xd& keypoints_b, const std::vector<cv::DMatch>& matches,
  const camera::CameraParameters& camera_params, const int max_num_matches, std::vector<cv::DMatch>& inlier_matches);

Eigen::Affine3d EstimateRelativeAffine3D(const std::vector<Eigen::Vector3d>& points_a,
                                         const std::vector<Eigen::Vector3d>& points_b);

// Assumes each feature is seen in every camera
ceres::Solver::Summary BundleAdjustFeatureSet(const std::vector<Keypoints>& camera_keypoints, const double focal_length,
                                              const ceres::Solver::Options& options,
                                              std::vector<Eigen::Affine3d>& cam_T_globals,
                                              std::vector<Eigen::Vector3d>& global_t_points, ceres::LossFunction* loss);
}  // namespace vision_common

#endif  // VISION_COMMON_MAPPING_UTILITIES_H_
