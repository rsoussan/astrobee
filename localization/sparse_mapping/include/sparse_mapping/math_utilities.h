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

#ifndef SPARSE_MAPPING_MATH_UTILITIES_H_
#define SPARSE_MAPPING_MATH_UTILITIES_H_

#include <sparse_mapping/remove_invalid_points_and_detections_params.h>

#include <Eigen/Geometry>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace sparse_mapping {
  Eigen::Quaternion<double> slerp_n(std::vector<double> const& W,
                                    std::vector<Eigen::Quaternion<double> > const& Q);

boost::optional<Eigen::Vector3d> Triangulate(const Eigen::Matrix3d& intrinsics,
                 const std::vector<Eigen::Affine3d>& camera_T_worlds,
                 const std::vector<Eigen::Matrix2Xd>& keypoints);

// Triangulates all points given camera positions.
void TriangulateAllPoints(const bool remove_invalid_points, const double focal_length,
                          const std::vector<Eigen::Affine3d>& cid_to_cam_t_global,
                          const std::vector<Eigen::Matrix2Xd>& cid_to_keypoint_map,
                          std::vector<std::map<int, int> >* pid_to_cid_fid, std::vector<Eigen::Vector3d>* pid_to_xyz,
                          std::vector<std::map<int, int> >* cid_fid_to_pid = nullptr);

double ReprojectionErrorThreshold(const std::vector<double>& reprojection_errors,
                                  const RemoveInvalidPointsAndDetectionsParams& params);

boost::optional<double> AngleBetweenRays(const Eigen::Vector3d& a_t_p, const Eigen::Vector3d& b_t_p);

// Find the maximum angle between n rays intersecting at given
// point. Must compute the camera centers in the global coordinate
// system before calling this function.
double MaxAngleBetweenCameraRays(const int pid, const std::vector<std::map<int, int> >& pid_to_cid_fid,
                                 const std::vector<Eigen::Vector3d>& global_t_cams,
                                 const std::vector<Eigen::Vector3d>& pid_to_xyz);

// Remove points that don't project at valid camera pixels,
// points behind the camera, and matches having large reprojection error.
void RemoveInvalidPointsAndDetections(const RemoveInvalidPointsAndDetectionsParams& params,
                 const std::vector<Eigen::Affine3d >& cid_to_cam_t_global,
                 const std::vector<Eigen::Matrix2Xd >& cid_to_keypoint_map,
                 std::vector<std::map<int, int> > * pid_to_cid_fid,
                 std::vector<Eigen::Vector3d> * pid_to_xyz, std::vector<std::map<int, int> >* cid_fid_to_pid = nullptr);

void DetectFeatures(const cv::Mat& image,
                      const bool histogram_equalization,
                      vision_common::DynamicDetector& detector,
                      cv::Mat& descriptors,
                      Eigen::Matrix2Xd& keypoints);

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

  // TODO(rsoussan): Remove this/use vision_common code, remove detection of feature type
  // base on descriptor
  void FindMatches(const cv::Mat & img1_descriptor_map,
                   const cv::Mat & img2_descriptor_map,
                   std::vector<cv::DMatch> * matches);

  boost::optional<Eigen::Affine3d> MatchImages(const Eigen::Matrix2Xd& keypoints_a, const Eigen::Matrix2Xd& keypoints_b,
                                               const cv::Mat& descriptors_a, const cv::Mat& descriptors_b,
                                               const camera::CameraParameters& camera_params, const int max_num_matches,
                                               const int min_num_inliers_for_valid_match,
                                               std::vector<cv::DMatch>& inlier_matches);

  boost::optional<Eigen::Affine3d> EstimateRelativeAffine3D(
    const Eigen::Matrix2Xd& keypoints_a, const Eigen::Matrix2Xd& keypoints_b, const std::vector<cv::DMatch>& matches,
    const camera::CameraParameters& camera_params, const int max_num_matches, std::vector<cv::DMatch>& inlier_matches);

// ICP solver that given matching 3D points, finds an affine transform that
// best fits in to out.
void Find3DAffineTransform(Eigen::Matrix3Xd const& in,
                           Eigen::Matrix3Xd const& out,
                           Eigen::Affine3d* result);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_MATH_UTILITIES_H_
