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

#include <Eigen/Geometry>

#include <map>
#include <string>
#include <vector>

namespace sparse_mapping {
  Eigen::Quaternion<double> slerp_n(std::vector<double> const& W,
                                    std::vector<Eigen::Quaternion<double> > const& Q);

  // Triangulate metric camera point
  //     unnormalized point means that the point is:
  //     [px (image loc) - cx (optical center), py - cy, f (f length in px)]
  Eigen::Vector3d
  TriangulatePoint(Eigen::Vector3d const& unnormalized_pt1,
                   Eigen::Vector3d const& unnormalized_pt2,
                   Eigen::Matrix3d const& cam2_r_cam1,
                   Eigen::Vector3d const& cam2_t_cam1,
                   double* error);

  // Decompose Fundamental Matrix into Essential Matrix given known
  // Intrinsics Matrix.
  void DecomposeFMatIntoEMat(Eigen::Matrix3d const& fundamental,
                             Eigen::Matrix3d const& intrinsics,
                             Eigen::Matrix3d * essential);

  // Decompose Essential Matrix into R and T
  void DecomposeEMatIntoRT(Eigen::Matrix3d const& essential,
                           Eigen::Matrix2Xd const& unnormalized_pts1,
                           Eigen::Matrix2Xd const& unnormalized_pts2,
                           std::vector<cv::DMatch> const& matches,
                           double focal_length1,  // Camera 1
                           double focal_length2,  // Camera 2
                           Eigen::Matrix3d * cam2_r_cam1,
                           Eigen::Vector3d * cam2_t_cam1);

  // Apply a given transform to the specified xyz points, and adjust
  // accordingly the cameras for consistency.
  void TransformCamerasAndPoints(Eigen::Affine3d const& A,
                                 std::vector<Eigen::Affine3d> *cid_to_cam_t,
                                 std::vector<Eigen::Vector3d> *xyz);

  // Get the error threshold based on a multiple of a percentile
  double GetErrThresh(const std::vector<double> & errors, double factor);

  // Find the maximum angle between n rays intersecting at given
  // point. Must compute the camera centers in the global coordinate
  // system before calling this function.
  double ComputeRaysAngle(int pid,
                          std::vector<std::map<int, int> > const& pid_to_cid_fid,
                          std::vector<Eigen::Vector3d> const & cam_ctrs,
                          std::vector<Eigen::Vector3d> const& pid_to_xyz);

  // Filter points by reprojection error and other criteria
  void FilterPID(double reproj_thresh,
                 camera::CameraParameters const& camera_params,
                 std::vector<Eigen::Affine3d > const& cid_to_cam_t_global,
                 std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
                 std::vector<std::map<int, int> > * pid_to_cid_fid,
                 std::vector<Eigen::Vector3d> * pid_to_xyz,
                 bool print_stats = true, double multiple_of_median = 3.0);

  // Given a data sequence having camera pose information for
  // a set of timestamps, interpolate those poses at the timestamps
  // given in out_time. We assume timestamps are always in increasing values.
  void PoseInterpolation(std::vector<std::string> const& images, std::vector<double> const& out_time,
                         std::map<std::string, std::vector<double> > const& data,
                         std::vector<Eigen::Affine3d>* cid_to_cam_t, std::vector<std::string>* good_images);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_MATH_UTILITIES_H_
