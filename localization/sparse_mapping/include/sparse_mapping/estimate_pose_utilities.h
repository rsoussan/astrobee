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
#ifndef SPARSE_MAPPING_ESTIMATE_POSE_UTILITIES_H_
#define SPARSE_MAPPING_ESTIMATE_POSE_UTILITIES_H_

#include <ff_common/eigen_vectors.h>
#include <sparse_mapping/estimate_pose_params.h>
#include <sparse_mapping/estimate_pose_results.h>

#include <ceres/ceres.h>

#include <Eigen/Geometry>

#include <map>
#include <vector>
#include <limits>
#include <string>
#include <set>

namespace camera {
  class CameraModel;
}

namespace cv {
  template <class T>
  class Point_;
  template <class T>
  class Point3_;
  typedef Point_<double> Point2d;
  typedef Point3_<double> Point3d;
}

namespace sparse_mapping {
struct ImageMatch {
  std::vector<cv::DMatch> matches;
  int num_valid_matches;
  int cid;
  bool operator<(const ImageMatch& rhs) { return num_valid_matches < rhs.num_valid_matches; }
};

void GetMatchingObservationsAndLandmarks(const std::vector<ImageMatches>& image_matches, const SparseMap& map,
                                         std::vector<Eigen::Vector2d>& observations,
                                         std::vector<Eigen::Vector3d>& landmarks);

std::vector<ImageMatch> FindAndSortMatches(const std::vector<int>& matching_cids, const SparseMap& map,
                                           const int max_num_total_feature_matches = 100,
                                           const bool check_point_3d_exists = true,
                                           const int min_matches_per_image = 5);
/**
 * Estimate the camera pose for a set of image descriptors and keypoints.
 **/
EstimatePoseResults EstimatePose(
  const cv::Mat& descriptors,  // TODO(rsoussan): change this to vector of descriptors
                               // TODO(rsoussan): change this to vector of Eigen::Vector2ds
  const Eigen::Matrix2Xd& keypoints, const SparseMap& map, const EstimatePoseParams& params);

EstimatePoseResults EstimatePose(
  const cv::Mat& image, const EstimatePoseParams& params, SparseMap& map);

EstimatePoseResults EstimatePose(
  const std::string& image_filename, const EstimatePoseParams& params, SparseMap& map);

  ceres::LossFunction* GetLossFunction(std::string cost_fun, double th);

/**
 * Perform bundle adjustment.
 *
 * cid_to_cam_t_global is the camera transforms
 * focal_length is the focal_length
 * pid_to_xyz are landmark locations
 * All should be set to initial guesses and are modified to improved guesses when the function returns.
 *
 * pid_to_cid_fid is maps from landmark id to camera id and feature id
 * cid_to_keypoint_map gives a list of observations for each camera
 * Ceres loss function and options can be specified, and summary returns results from ceres.
 * Optimize only the cameras with indices in [first, last].
 **/
void BundleAdjust(std::vector<std::map<int, int> > const& pid_to_cid_fid,
                  std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
                  double focal_length,
                  std::vector<Eigen::Affine3d> * cid_to_cam_t_global,
                  std::vector<Eigen::Vector3d> * pid_to_xyz,
                  std::vector<std::map<int, int> > const& user_pid_to_cid_fid,
                  std::vector<Eigen::Matrix2Xd > const& user_cid_to_keypoint_map,
                  std::vector<Eigen::Vector3d> * user_pid_to_xyz,
                  ceres::LossFunction * loss,
                  ceres::Solver::Options const& options,
                  ceres::Solver::Summary* summary,
                  int first = 0, int last = std::numeric_limits<int>::max(),
                  bool fix_cameras = false,
                  std::set<int> const& fixed_cameras = std::set<int>());

/**
 * Perform bundle adjustment.
 *
 * This variant assumes that all cameras see that same points. This is
 * meant to be used to do 2 or 3 camera refinements however it can do
 * N cameras just fine.
 *
 **/
void BundleAdjustSmallSet(std::vector<Eigen::Matrix2Xd> const& features_n,
                          double focal_length,
                          std::vector<Eigen::Affine3d> * cam_t_global_n,
                          Eigen::Matrix3Xd * pid_to_xyz,
                          ceres::LossFunction * loss,
                          ceres::Solver::Options const& options,
                          ceres::Solver::Summary * summary);

// Random integer between min (inclusive) and max (exclusive)
int RandomInt(int min, int max);

// Select a Random Observations
void SelectRandomObservations(const std::vector<Eigen::Vector3d> & all_landmarks,
    const std::vector<Eigen::Vector2d> & all_observations, size_t num_selected,
    std::vector<cv::Point3d> * landmarks, std::vector<cv::Point2d> * observations);

// Used to find landmark and observations that best match the current camera
// model.
size_t CountInliers(const std::vector<Eigen::Vector3d> & landmarks,
    const std::vector<Eigen::Vector2d> & observations,
    const camera::CameraModel & camera, int tolerance,
    std::vector<size_t>* inliers);

/**
 * Estimate the camera matrix, with translation and rotation, that maps the points in landmarks
 * to the image coordinates observed in observations. This uses a least squares solver
 * and the initial estimate must be in the neighborhood of the true position for it
 * to converge.
 *
 * After the function is called, camera_estimate is updated to contain the results,
 * and summary summarizes them.
 **/
void EstimateCamera(camera::CameraModel * camera_estimate, std::vector<Eigen::Vector3d> * landmarks,
                    const std::vector<Eigen::Vector2d> & observations,
                    const ceres::Solver::Options & options, ceres::Solver::Summary* summary);

/**
 * Estimate the camera matrix, with translation and rotation, that maps the points in landmarks
 * to the image coordinates observed in observations. This uses ransac with a three
 * point perspective algorithm, and does not use an initial guess for the camera pose.
 *
 * After the function is called, camera_estimate is updated to contain the results.
 *
 * Returns zero on success, nonzero on failure.
 **/
int RansacEstimateCamera(const std::vector<Eigen::Vector3d> & landmarks,
                         const std::vector<Eigen::Vector2d> & observations,
                         int num_tries, int inlier_tolerance, camera::CameraModel * camera_estimate,
                         std::vector<Eigen::Vector3d> * inlier_landmarks_out = NULL,
                         std::vector<Eigen::Vector2d> * inlier_observations_out = NULL,
                         bool verbose = false);

// ICP solver that given matching 3D points, finds an affine transform that
// best fits in to out.
void Find3DAffineTransform(Eigen::Matrix3Xd const& in,
                           Eigen::Matrix3Xd const& out,
                           Eigen::Affine3d* result);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_ESTIMATE_POSE_UTILITIES_H_
