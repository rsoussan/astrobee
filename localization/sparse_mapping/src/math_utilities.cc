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

#include <localization_common/utilities.h>
#include <sparse_mapping/math_utilities.h>

#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic push
#include <openMVG/multiview/conditioning.hpp>
#include <openMVG/multiview/projection.hpp>
#include <openMVG/multiview/triangulation.hpp>
#include <openMVG/multiview/solver_essential_kernel.hpp>
#include <openMVG/robust_estimation/robust_estimator_ACRansac.hpp>
#include <openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp>
#pragma GCC diagnostic pop

namespace sparse_mapping {
Eigen::Matrix2Xd MatrixFromVector(const std::vector<Eigen::Vector2d>& vector) {
  // TODO(rsoussan): Better way to do this?
  Eigen::Matrix2Xd matrix(2, vector.size());
  for (int i = 0; i < static_cast<int>(vector.size()); ++i) {
    matrix.col(i) = vector[i];
  }
}
// Compute the n-weight slerp, analogous to the linear combination
// W[0]*Q[0] + ... + W[n-1]*Q[n-1]. This is experimental.
// We assume the sum of weights is 1.
Eigen::Quaternion<double> slerp_n(std::vector<double> const& W, std::vector<Eigen::Quaternion<double>> const& Q) {
  if (W.size() != Q.size()) LOG(FATAL) << "Expecting as many quaternions as weights.";

  if (Q.empty()) LOG(FATAL) << "Expecting at least one quaternion and weight.";

  if (Q.size() == 1) return Q[0];

  if (Q.size() == 2) {
    if (!(std::abs(W[0] + W[1] - 1.0) < 1e-6 && W[0] >= 0 && W[1] >= 0))
      LOG(FATAL) << "Expecting the weights to be >= 0 and sum up to 1.";
    return Q[0].slerp(W[1], Q[1]);
  }

  // Call recursively this function with fewer terms
  double sum = W[0] + W[1];
  if (sum == 0) sum = 1.0;
  Eigen::Quaternion<double> q = Q[0].slerp(W[1] / sum, Q[1]);
  std::vector<double> W2 = W;
  std::vector<Eigen::Quaternion<double>> Q2 = Q;
  W2.erase(W2.begin());
  Q2.erase(Q2.begin());
  W2[0] = sum;
  Q2[0] = q;
  return slerp_n(W2, Q2);
}

boost::optional<Eigen::Vector3d> Triangulate(const Eigen::Matrix3d& intrinsics,
                                             const std::vector<Eigen::Affine3d>& camera_T_worlds,
                                             const Keypoints& keypoints) {
  std::vector<openMVG::Mat34> projection_matrices(camera_T_worlds.size());
  openMVG::Triangulation triangulation;
  for (int i = 0; i < camera_T_worlds.size(); ++i) {
    const auto& camera_T_world = camera_T_worlds[i];
    auto& projection_matrix = projection_matrices[i];
    // TODO(rsoussan): Issue if rotation contains scaling?
    openMVG::P_From_KRt(intrinsics, camera_T_world.linear(), camera_T_world.translation(), &projection_matrix);
    triangulation.add(projection_matrix, keypoints[i]);
  }

  const Eigen::Vector3d world_t_point = triangulation.compute();
  if (std::isnan(world_t_point.x()) || triangulation.minDepth() < 0) return boost::none;
  return world_t_point;
}

boost::optional<double> AngleBetweenRays(const Eigen::Vector3d& a_t_p, const Eigen::Vector3d& b_t_p) {
  const double a_t_p_norm = a_t_p.norm();
  const double b_t_p_norm = b_t_p.norm();
  if (a_t_p_norm == 0 || b_t_p_norm == 0) return boost::none;

  // a dot b = cos(theta)*||a||*||b||
  // cos(theta) = (a dot b) /(||a||*||b||)
  double cos_angle = a_t_p.dot(b_t_p) / (a_t_p_norm * b_t_p_norm);
  // Avoid numerical errors of cos_angle being slightly larger or smaller than 1/-1
  cos_angle = std::min(1.0, cos_angle);
  cos_angle = std::max(-1.0, cos_angle);
  return (180.0 / M_PI) * std::acos(cos_angle);
}

double MaxAngleBetweenCameraRays(const FeatureTrack& feature_track, const Eigen::Vector3d& global_t_point,
                                 const CidPoseMap& cid_to_global_t_cam) {
  double max_angle = 0;
  int cid = 0;
  for (auto cid_fid_it1 = feature_track.begin(); cid_fid_it1 != feature_track.end(); ++cid_fid_it1) {
    const int cid1 = cid_fid_it1->first;
    for (auto cid_fid_it2 = cid_fid_it1 + 1; cid_fid_it2 != feature_track.end(); ++cid_fid_it2) {
      const int cid2 = cid_fid_it2->first;
      const Eigen::Vector3d cam1_t_point = cid_to_global_t_cam[cid1] - global_t_point;
      const Eigen::Vector3d cam2_t_point = cid_to_global_t_cam[cid2] - global_t_point;
      const auto angle = AngleBetweenRays(cam1_t_point, cam2_t_point);
      if (!angle) continue;
      max_angle = std::max(*angle, max_angle);
    }
  }
  return max_angle;
}

void DetectFeatures(const cv::Mat& image, const bool histogram_equalization, vision_common::DynamicDetector& detector,
                    Descriptors& descriptors, Keypoints& keypoints) {
  cv::Mat hist_image;
  if (histogram_equalization) {
    cv::equalizeHist(image, hist_image);
  }
  const auto& input_image = histogram_equalization ? hist_image : image;

  std::vector<cv::KeyPoint> cv_keypoints;
  detector.DetectAndCompute(input_image, cv_keypoints, descriptors);

  // Convert keypoints to undistorted frame and Eigen type
  for (int i = 0; i < static_cast<int>(storage.size()); ++i) {
    Eigen::Vector2d keypoint;
    camera_params_.Convert<camera::DISTORTED_C, camera::UNDISTORTED_C>(
      Eigen::Vector2d(cv_keypoint[i].pt.x, cv_keypoint[i].pt.y), &keypoint);
    keypoints.emplace_back(keypoint);
  }
}

bool EstimateEssentialMatrix(const Eigen::Matrix3d& intrinsics_1, const Eigen::Matrix3d& intrinsics_2,
                             const Keypoints& keypoints_1, const Keypoints& keypoints_2,
                             const std::pair<int, int>& image_size_1, const std::pair<int, int>& image_size_2,
                             const double precision, Eigen::Matrix3d& essential_matrix, std::vector<int>& inliers,
                             double& max_error) {
  const auto keypoints_1_matrix = MatrixFromVector(keypoints_1);
  const auto keypoints_2_matrix = MatrixFromVector(keypoints_2);
  return EstimateEssentialMatrix(intrinsics_1, intrinsics_2, keypoints_1_matrix, keypoints_2_matrix, image_size_1,
                                 image_size_2, precision, essential_matrix, inliers, max_error);
}

bool EstimateEssentialMatrix(const Eigen::Matrix3d& intrinsics_1, const Eigen::Matrix3d& intrinsics_2,
                             const Eigen::Matrix2Xd& keypoints_1, const Eigen::Matrix2Xd& keypoints_2,
                             const std::pair<int, int>& image_size_1, const std::pair<int, int>& image_size_2,
                             const double precision, Eigen::Matrix3d& essential_matrix, std::vector<int>& inliers,
                             double& max_error) {
  using SolverType = openMVG::essential::kernel::FivePointKernel;
  using KernelType =
    openMVG::robust::ACKernelAdaptorEssential<SolverType, openMVG::fundamental::kernel::EpipolarDistanceError,
                                              Eigen::Matrix3d>;
  KernelType kernel(keypoints_1, image_size_1.first, image_size_1.second, keypoints_2, image_size_2.first,
                    image_size_2.second, intrinsics_1, intrinsics_2);
  const auto ransac_output =
    openMVG::robust::ACRANSAC(kernel, inliers, 4096 /* iterations */, essential_matrix, precision, false);
  max_error = ransac_output.first;
  return inliers.size() > 1.5 * SolverType::MINIMUM_SAMPLES;
}

boost::optional<Eigen::Isometry3d> EstimateNormalizedPoseFromEssentialMatrix(
  const Eigen::Matrix3d& intrinsics_1, const Eigen::Matrix3d& intrinsics_2, const Keypoints& keypoints_1,
  const Keypoints& keypoints_2, const Eigen::Matrix3d& essential_matrix, const std::vector<int>& inliers) {
  std::vector<Eigen::Matrix3d> possible_rotations;
  std::vector<Eigen::Vector3d> possible_translations;
  possible_rotations.reserve(4);
  possible_translations.reserve(4);
  openMVG::MotionFromEssential(essential_matrix, &possible_rotations, &possible_translations);

  if (possible_rotations.size() != 4 || possible_translations.size() != 4) {
    LOG(ERROR) << "Failed to find 4 solutions for R & T";
    return boost::none;
  }

  // Use identity projection matrix as other projection matrix since
  // we are estimating a relative pose
  openMVG::Mat34 identity_projection_matrix, candidate_projection_matrix;
  const Eigen::Matrix3d zero_rotation(Eigen::Matrix3d::Identity());
  const Eigen::Vector3d zero_translation(Eigen::Vector3d::Zero());
  openMVG::P_From_KRt(intrinsics_1, zero_rotation, zero_translation, &identity_projection_matrix);

  // See which pose candidate has the most valid triangulated keypoints
  std::vector<int> candidate_valid_triangulated_points(4, 0);
  for (int i = 0; i < 4; ++i) {
    const auto& rotation_candidate = possible_rotations[i];
    const auto& translation_candidate = possible_translations[i];
    openMVG::P_From_KRt(intrinsics_2, rotation_candidate, translation_candidate, &candidate_projection_matrix);

    Eigen::Vector3d triangulated_point;
    for (int j = 0; j < static_cast<int>(inliers.size()); ++j) {
      const auto& keypoint_1 = keypoints_1[vec_inliers[j]];
      const auto& keypoint_2 = keypoints_2[vec_inliers[j]];
      openMVG::TriangulateDLT(identity_projection_matrix, keypoint_1, candidate_projection_matrix, keypoint_2,
                              &triangulated_point);
      if (openMVG::Depth(zero_rotation, zero_translation, triangulated_point) > 0 &&
          openMVG::Depth(rotation_candidate, translation_candidate, triangulated_point) > 0) {
        ++candidate_valid_triangulated_points[i];
      }
    }
  }

  const auto most_valid_triangulate_points_index =
    std::max_element(candidate_valid_triangulated_points.begin(), candidate_valid_triangulated_points.end());
  if (*most_valid_triangulate_points_index == 0) {
    LOG(ERROR) << "Unable to find right solution for RT, possibly there is none.";
    return boost::none;
  }
  const int best_candidate_index =
    std::distance(candidate_valid_triangulated_points.cbegin(), most_valid_triangulate_points_index);
  const auto best_rotation = possible_rotations[best_candidate_index];
  const auto best_translation = possible_translations[best_candidate_index];
  return lc::Isometry3d(best_translation, best_rotation);
}

std::vector<cv::DMatch> FindMatches(const Descriptors& descriptors_a, const Descriptors& descriptors_b,
                                    const int brisk_hamming_distance, const double surf_goodness_ratio) {
  if (descriptors_a.size() == 0 || descriptors_b.size() == 0) return;

  CHECK(descriptors_a[0].depth() == descriptors_b[0].depth())
    << "Mixed descriptor types. Did you mash BRISK with SIFT/SURF?";

  // Binary descriptor
  if (descriptors_a[0].depth() == CV_8U) {
    // cv::BFMatcher matcher(cv::NORM_HAMMING, true  /* Forward & Backward matching */);
    cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::LshIndexParams>(3, 18, 2));
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors_a, descriptors_b, matches);

    // Select only inlier matches that meet a BRISK threshold of
    // of FLAGS_hamming_distance.
    // TODO(oalexan1) This needs further study.
    std::vector<cv::DMatch> inlier_matches;
    for (const auto& match : matches) {
      if (match.distance < brisk_hamming_distance) {
        inlier_matches.emplace_back(match);
      }
    }
    return inlier_matches;
  } else {  // Floating point descriptor
    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch>> possible_matches;
    matcher.knnMatch(descriptors_a, descriptors_b, possible_matches, 2);
    std::vector<cv::DMatch> matches;
    for (const auto& best_matches : possible_matches) {
      if (best_matches.size() == 1) {
        // This was the only best match, push it.
        matches.emplace_back(best_matches[0]);
      } else {
        // Push back a match only if it is 25% better than the next best.
        if (best_matches[0].distance < surf_goodness_ratio * best_matches[1].distance) {
          matches.emplace_back(best_matches[0]);
        }
      }
    }
    return matches;
  }
}

boost::optional<Eigen::Affine3d> MatchImages(const Keypoints& keypoints_a, const Keypoints& keypoints_b,
                                             const Descriptors& descriptors_a, const Descriptors& descriptors_b,
                                             const camera::CameraParameters& camera_params, const int max_num_matches,
                                             const int min_num_inliers_for_valid_match,
                                             std::vector<cv::DMatch>& inlier_matches) {
  const auto matches = FindMatches(descriptors_a, descriptors_b);
  if (static_cast<int>(matches.size()) < min_num_inliers_for_valid_match) {
    LOG(DEBUG) << "Failed to find enough matches " << matches.size();
    return boost::none;
  }

  const auto relative_pose =
    EstimateNormalizedRelativeAffine3D(keypoints1, keypoints2, matches, camera_params, max_num_matches, inlier_matches);
  if (!relative_pose) return boost::none;

  if (static_cast<int>(inlier_matches.size()) < min_num_inliers_for_valid_match) {
    LOG(DEBUG) << "Failed to find enough inlier matches: " << inlier_matches.size();
    return boost::none;
  }

  return relative_pose;
}

boost::optional<Eigen::Affine3d> EstimateNormalizedRelativeAffine3D(
  const Keypoints& keypoints_a, const Keypoints& keypoints_b, const std::vector<cv::DMatch>& matches,
  const camera::CameraParameters& camera_params, const int max_num_matches, std::vector<cv::DMatch>& inlier_matches,
  const int min_valid_inliers = 20) {
  const int num_matches = matches.size();
  Keypoints matching_keypoints_a;
  Keypoints matching_keypoints_b;
  for (int i = 0; i < num_matches; ++i) {
    matching_keypoints_a.emplace_back(keypoints_a[matches[i].queryIdx]);
    matching_keypoints_b.emplace_back(keypoints_b[matches[i].trainIdx]);
  }

  const std::pair<int, int> image_size(camera_params.GetUndistortedSize()[0], camera_params.GetUndistortedSize()[1]);
  const Eigen::Matrix3d intrinsics = camera_params.GetIntrinsicMatrix<camera::UNDISTORTED_C>();

  Eigen::Matrix3d essential_matrix;
  std::vector<int> inlier_indices;
  const double max_expected_error = 2.5;
  double max_error;
  if (!EstimateEssentialMatrix(intrinsics, intrinsics, matching_keypoints_a, matching_keypoints_b, image_size,
                               image_size, max_expected_error, essential_matrix, inlier_indices, max_error)) {
    LOG(DEBUG) << "Estimation of essential matrix failed!\n";
    return boost::none;
  }

  if (inlier_indices.size() < min_valid_inliers) {
    LOG(DEBUG) << "Failed to get enough inliers " << inlier_indices.size();
    return boost::none;
  }

  const auto cam_2_T_cam_1 = EstimateNormalizedPoseFromEssentialMatrix(
    intrinsics, intrinsics, matching_keypoints_a, matching_keypoints_b, essential_matrix, inlier_indices);
  if (!cam_2_T_cam_1) {
    LOG(DEBUG) << "Failed to get pose from essential matrix.";
    return boost::none;
  }

  LOG(DEBUG) << "Inliers from E: " << inlier_indices.size() << " / " << matching_keypoints_a.size();

  // Triangulate points for matches, only save keypoints and triangulated points
  // for valid triangulated points and inlier points
  std::vector<Eigen::Vector3d> pid_to_cam_1_t_point;
  std::vector<Keypoints> valid_inlier_keypoints_a;
  std::vector<Keypoints> valid_inlier_keypoints_b;
  std::vector<int> valid_inlier_indices;
  int num_pts_behind_camera = 0;
  for (const auto inlier_index : inlier_indices) {
    std::vector<Eigen::Vector2d> keypoints;
    const auto& keypoint_a = matching_keypoints_a[inlier_index];
    const auto& keypoint_b = matching_keypoints_b[inlier_index];
    keypoints.emplace_back(keypoint_a);
    keypoints.emplace_back(keypoint_b);
    const auto cam_1_t_point = Triangulate(intrinsics, cameras, keypoints);
    if (!cam_1_t_point) continue;
    pid_to_cam_1_t_point.emplace_back(*cam_1_t_point);
    valid_inlier_keypoints_a.emplace_back(keypoint_a);
    valid_inlier_keypoints_b.emplace_back(keypoint_b);
    valid_inlier_indices.emplace_back(inlier_index);
    cam_2_t_point = cam_2_T_cam_1 * (*cam_1_t_point);
    if (cam_1_t_point->z() <= 0 || cam_2_t_point.z() <= 0) {
      num_pts_behind_camera++;
    }
  }
  LOG(DEBUG) << "Pair "
             << ": number of points behind cameras: " << num_pts_behind_camera << "/" << valid_keypoints_a.size()
             << " (" << round((100.0 * num_pts_behind_camera) / valid_keypoints_a.size()) << "%)";

  // Refine the pose using bundle adjustment
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.max_num_iterations = 200;
  options.logging_type = ceres::SILENT;
  options.num_threads = FLAGS_num_threads;
  std::vector<Eigen::Affine3d> cameras;
  cameras.emplace_back(Eigen::Affine3d::Identity());
  cameras.emplace_back(*cam_2_T_cam_1);
  const auto summary =
    BundleAdjustFeatureSet({valid_inlier_keypoints_a, valid_inlier_keypoints_b}, camera_params.GetFocalLength(),
                           options, cameras, pid_to_cam_1_t_point, new ceres::CauchyLoss(0.5));

  if (!summary.IsSolutionUsable()) {
    LOG(ERROR) << " Failed to refine pose with bundle adjustment";
    return boost::none;
  }
  LOG(DEBUG) << summary.BriefReport();

  Eigen::Affine3d cam_1_T_cam_2 = cameras[1].inverse();
  cam_1_T_cam_2.translation().normalize();

  const int num_inliers = valid_inlier_indices.size();
  // Filter inliers by distance if neccessary
  if (num_inliers > max_num_matches) {
    std::map<double, int> distance_to_index;
    for (const auto inlier_index : inlier_indices) {
      distance_to_index.emplace_back(matches[inlier_index].distance, inlier_index);
    }
    inlier_indices.clear();
    int count = 0;
    for (const auto& distance_index_pair : distance_to_index) {
      inlier_indices.emplace_back(distance_index_pair.second);
      ++count;
      if (count >= max_num_matches) break;
    }
  }

  inlier_matches.reserve(inlier_indices.size());
  for (const auto inlier_index : inlier_indices) {
    inlier_matches.emplace_back(matches[inlier_index]);
  }

  return cam_1_T_cam_2;
}

// Given two sets of 3D points, find the rotation + translation + scale
// which best maps the first set to the second.
// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm
void Find3DAffineTransform(Eigen::Matrix3Xd const& in, Eigen::Matrix3Xd const& out, Eigen::Affine3d* result) {
  // Default output
  result->linear() = Eigen::Matrix3d::Identity(3, 3);
  result->translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols()) throw "Find3DAffineTransform(): input data mis-match";

  // Local copies we can modify
  Eigen::Matrix3Xd local_in = in, local_out = out;

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < local_in.cols() - 1; col++) {
    dist_in += (local_in.col(col + 1) - local_in.col(col)).norm();
    dist_out += (local_out.col(col + 1) - local_out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0) return;
  double scale = dist_out / dist_in;
  local_out /= scale;

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < local_in.cols(); col++) {
    in_ctr += local_in.col(col);
    out_ctr += local_out.col(col);
  }
  in_ctr /= local_in.cols();
  out_ctr /= local_out.cols();
  for (int col = 0; col < local_in.cols(); col++) {
    local_in.col(col) -= in_ctr;
    local_out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::Matrix3d Cov = local_in * local_out.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(Cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  result->linear() = scale * R;
  result->translation() = scale * (out_ctr - R * in_ctr);
}

ceres::Solver::Summary BundleAdjustFeatureSet(const std::vector<Keypoints>& camera_keypoints, const double focal_length,
                                              const ceres::Solver::Options& options,
                                              std::vector<Eigen::Affine3d>& cam_T_globals,
                                              std::vector<Eigen::Vector3d>& global_t_points,
                                              ceres::LossFunction* loss) {
  CHECK(cam_T_globals.size() == camera_keypoints.size())
    << "Variables features_n and cam_T_globals need to agree on the number of cameras";
  CHECK(cam_T_globals.size() > 1) << "Bundle adjust needs at least 2 or more cameras";
  CHECK(global_t_point.size() == camera_keypoints[0].size())
    << "There should be an equal amount of XYZ points as there are feature observations";
  for (int i = 1; i < static_cast<int>(camera_keypoints.size()); ++i) {
    CHECK(camera_keypoints[0].cols() == camera_keypoints[i].cols())
      << "The same amount of features should be seen in all cameras";
  }

  const int num_cameras = camera_keypoints.size();
  std::vector<Eigen::Matrix<double, 7, 1>> cam_T_global_data_vec;
  cam_T_global_data_vec.reserve(num_cameras);
  for (const auto& cam_T_global : cam_T_globals) {
    cam_T_global_data_vec.emplace_back(oc::VectorFromAffine3d(cam_T_global));
  }

  ceres::Problem problem;
  // Centered, undistored camera
  const Eigen::Vector2d zero_principal_points(Eigen::Vector2d::Zero());
  const Eigen::VectorXd zero_distortion(1);
  const Eigen::Vector2d focal_lengths(focal_length, focal_length);
  oc::AddConstantParameterBlock(2, zero_principal_points.data(), problem);
  oc::AddConstantParameterBlock(1, zero_distortion.data(), problem);
  oc::AddConstantParameterBlock(2, focal_lengths.data(), problem);

  for (int pid = 0; pid < static_cast<int>(global_t_points.size()); ++pid) {
    for (int cid = 0; cid < num_cameras; ++cid) {
      auto& cam_T_global_data = cam_T_global_data_vec[cid];
      oc::AddAffine3ParameterBlock(cam_T_global_data.data(), problem);
      ceres::SubsetParameterization* constant_scale_parameterization = new ceres::SubsetParameterization(7, {6});
      problem.SetParameterization(cam_T_global_data.data(), constant_scale_parameterization);
      oc::ReprojectionError<vc::IdentityDistorter, oc::AffineFunctor>::AddCostFunction(
        camera_keypoints[cid][pid], global_t_points[pid], cam_T_global_data,
        const_cast<Eigen::Vector2d&>(focal_lengths), const_cast<Eigen::Vector2d&>(zero_principal_points),
        const_cast<Eigen::VectorXd&>(zero_distortion), problem, loss);
    }
  }

  ceres::Solver::Summary summary;
  ceres::Solve(params.options, &problem, &summary);

  for (int cid = 0; cid < num_cameras; ++cid) {
    cam_T_globals[cid] = oc::Affine3d(cam_T_global_data_vec[cid]);
  }

  return summary;
}
}  // namespace sparse_mapping
