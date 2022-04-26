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
#include <vision_common/dynamic_detector.h>
#include <vision_common/mapping_utilities.h>

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

namespace vision_common {
Eigen::Matrix2Xd MatrixFromVector(const std::vector<Eigen::Vector2d>& vector) {
  Eigen::Matrix2Xd matrix(2, vector.size());
  for (int i = 0; i < static_cast<int>(vector.size()); ++i) {
    matrix.col(i) = vector[i];
  }
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

void DetectFeatures(const cv::Mat& image, const bool histogram_equalization, DynamicDetector& detector,
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

// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm
Eigen::Affine3d EstimateRelativeAffine3D(const std::vector<Eigen::Vector3d>& points_a,
                                         const std::vector<Eigen::Vector3d>& points_b) {
  if (points_a.size() != points_b.size()) throw "EstimateRelativeAffine3D(): points are different sizes.";
  Eigen::Affine3d b_T_a(Eigen::Affine3d::Identity());

  Eigen::Matrix3Xd points_matrix_a = Eigen::MatrixXd(3, points_a.size());
  Eigen::Matrix3Xd points_matrix_b = Eigen::MatrixXd(3, points_b.size());
  for (int i = 0; i < static_cast<int>(points_a.size()); ++i) {
    points_matrix_a.col(i) = points_a[i];
    points_matrix_b.col(i) = points_b[i];
  }

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double sum_sequential_distances_a = 0, sum_sequential_distances_b = 0;
  for (int i = 0; i < static_cast<int>(points_a.size()) - 1; ++i) {
    sum_sequential_distances_a += (points_matrix_a.col(i + 1) - points_matrix_a.col(i)).norm();
    sum_sequential_distances_b += (points_matrix_b.col(i + 1) - points_matrix_b.col(i)).norm();
  }
  if (sum_sequential_distances_a <= 0 || sum_sequential_distances_b <= 0) return b_T_a;
  const double scale = sum_sequential_distances_b / sum_sequential_distances_a;
  points_matrix_b /= scale;

  // Center points
  Eigen::Vector3d a_centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d b_centroid = Eigen::Vector3d::Zero();
  for (int i = 0; i < static_cast<int>(points_a.size()); ++i) {
    a_centroid += points_matrix_a.col(i);
    b_centroid += points_matrix_b.col(i);
  }
  a_centroid /= static_cast<double>(points_a.size());
  b_centroid /= static_cast<double>(points_b.size());
  for (int i = 0; i < static_cast<int>(points_a.size()); ++i) {
    points_matrix_a.col(i) -= a_centroid;
    points_matrix_b.col(i) -= b_centroid;
  }

  // Find the rotation
  const Eigen::Matrix3d Cov = points_matrix_a * points_matrix_b.transpose();
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(Cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  d = d > 0 ? 1.0 : -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  const Eigen::Matrix3d rotation = svd.matrixV() * I * svd.matrixU().transpose();

  b_T_a.linear() = scale * rotation;
  b_T_a.translation() = scale * (b_centroid - rotation * a_centroid);
  return b_T_a;
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
    CHECK(camera_keypoints[0].size() == camera_keypoints[i].size())
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
}  // namespace vision_common
