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
// Compute the n-weight slerp, analogous to the linear combination
// W[0]*Q[0] + ... + W[n-1]*Q[n-1]. This is experimental.
// We assume the sum of weights is 1.
Eigen::Quaternion<double> slerp_n(std::vector<double> const& W, std::vector<Eigen::Quaternion<double> > const& Q) {
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
  std::vector<Eigen::Quaternion<double> > Q2 = Q;
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

bool RobustEssential(Eigen::Matrix3d const& k1, Eigen::Matrix3d const& k2, Eigen::Matrix2Xd const& x1,
                     Eigen::Matrix2Xd const& x2, Eigen::Matrix3d* e, std::vector<size_t>* vec_inliers,
                     std::pair<size_t, size_t> const& size1, std::pair<size_t, size_t> const& size2, double* error_max,
                     double precision) {
  CHECK(e) << "Missing e argument";
  CHECK(vec_inliers) << "Missing vec inliers argument";

  typedef openMVG::essential::kernel::FivePointKernel SolverType;
  typedef openMVG::robust::ACKernelAdaptorEssential<SolverType, openMVG::fundamental::kernel::EpipolarDistanceError,
                                                    Eigen::Matrix3d>
    KernelType;

  KernelType kernel(x1, size1.first, size1.second, x2, size2.first, size2.second, k1, k2);

  std::pair<double, double> ransac_output =
    openMVG::robust::ACRANSAC(kernel, *vec_inliers, 4096 /* iterations */, e, precision, false);
  *error_max = ransac_output.first;

  return vec_inliers->size() > 1.5 * SolverType::MINIMUM_SAMPLES;
}

bool EstimateRTFromE(Eigen::Matrix3d const& k1, Eigen::Matrix3d const& k2, Eigen::Matrix2Xd const& x1,
                     Eigen::Matrix2Xd const& x2, Eigen::Matrix3d const& e, std::vector<size_t> const& vec_inliers,
                     Eigen::Matrix3d* r, Eigen::Vector3d* t) {
  // Accumulator to find the best solution
  std::vector<size_t> f(4, 0);

  std::vector<Eigen::Matrix3d> possible_r;  // Rotation matrix.
  std::vector<Eigen::Vector3d> possible_t;  // Translation matrix.
  possible_r.reserve(4);
  possible_t.reserve(4);

  // Recover best rotation and translation from E.
  openMVG::MotionFromEssential(e, &possible_r, &possible_t);

  //-> Test the 4 solutions will all the point
  CHECK(possible_r.size() == 4 && possible_t.size() == 4) << "Failed to find 4 solutions for R & T";

  openMVG::Mat34 P1, P2;
  Eigen::Matrix3d r1 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t1 = Eigen::Vector3d::Zero();
  openMVG::P_From_KRt(k1, r1, t1, &P1);

  for (size_t i = 0; i < 4; ++i) {
    const Eigen::Matrix3d& r2 = possible_r[i];
    const Eigen::Vector3d& t2 = possible_t[i];
    openMVG::P_From_KRt(k2, r2, t2, &P2);
    Eigen::Vector3d X;

    for (size_t k = 0; k < vec_inliers.size(); ++k) {
      const Eigen::Vector2d &x1_ = x1.col(vec_inliers[k]), &x2_ = x2.col(vec_inliers[k]);
      openMVG::TriangulateDLT(P1, x1_, P2, x2_, &X);
      // Test if point is front to the two cameras.
      if (openMVG::Depth(r1, t1, X) > 0 && openMVG::Depth(r2, t2, X) > 0) {
        ++f[i];
      }
    }
  }

  // Check the solution:
  std::vector<size_t>::const_iterator iter = std::max_element(f.begin(), f.end());
  if (*iter == 0) {
    LOG(ERROR) << "Unable to find right solution for RT, possibly there is none.";
    return false;
  }
  size_t index = std::distance(f.cbegin(), iter);
  *r = possible_r[index];
  *t = possible_t[index];

  return true;
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
    std::vector<std::vector<cv::DMatch> > possible_matches;
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
    EstimateRelativeAffine3D(keypoints1, keypoints2, matches, camera_params, max_num_matches inlier_matches);
  if (!relative_pose) return boost::none;

  if (static_cast<int>(inlier_matches.size()) < min_num_inliers_for_valid_match) {
    LOG(DEBUG) << "Failed to find enough inlier matches: " << inlier_matches.size();
    return boost::none;
  }

  return relative_pose;
}

// TODO(rsoussan): Clean this up
boost::optional<Eigen::Affine3d> EstimateRelativeAffine3D(
  const Eigen::Matrix2Xd& keypoints_a, const Eigen::Matrix2Xd& keypoints_b, const std::vector<cv::DMatch>& matches,
  const camera::CameraParameters& camera_params, const int max_num_matches, std::vector<cv::DMatch>& inlier_matches) {
  inlier_matches->clear();

  const int num_matches = matches.size();
  Eigen::MatrixXd matching_keypoints_a(2, num_matches);
  Eigen::MatrixXd matching_keypoints_b(2, num_matches);
  for (int i = 0; i < num_matches; ++i) {
    matching_keypoints_a.col(i) = keypoints_a.col(matches[i].queryIdx);
    matching_keypoints_b.col(i) = keypoints_b.col(matches[i].trainIdx);
  }

  const std::pair<size_t, size_t> image_size(camera_params.GetUndistortedSize()[0],
                                             camera_params.GetUndistortedSize()[1]);
  const Eigen::Matrix3d intrinsics = camera_params.GetIntrinsicMatrix<camera::UNDISTORTED_C>();

  Eigen::Matrix3d essential_matrix;
  std::vector<size_t> vec_inliers;
  double error_max = std::numeric_limits<double>::max();
  double max_expected_error = 2.5;
  if (!RobustEssential(intrinsics, intrinsics, matching_keypoints_a, matching_keypoints_b, &essential_matrix,
                       &vec_inliers, image_size, image_size, &error_max, max_expected_error)) {
    LOG(DEBUG) << "Estimation of essential matrix failed!\n";
    return boost::none;
  }

  if (vec_inliers.size() < static_cast<size_t>(FLAGS_min_valid)) {
    LOG(DEBUG) << "Failed to get enough inliers " << vec_inliers.size();
    return boost::none;
  }

  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  if (!EstimateRTFromE(intrinsics, intrinsics, matching_keypoints_a, matching_keypoints_b, essential_matrix,
                       vec_inliers, &r, &t)) {
    LOG(DEBUG) << "Failed to extract RT from E";
    return boost::none;
  }

  LOG(DEBUG) << "Inliers from E: " << vec_inliers.size() << " / " << matching_keypoints_a.cols();

  // Get the matching_keypoints corresponding to inliers
  // TODO(ZACK): We could reuse everything.
  int num_inliers = vec_inliers.size();
  std::vector<Eigen::Matrix2Xd> matching_keypoints_2(2, Eigen::Matrix2Xd(2, num_inliers));
  for (int i = 0; i < num_inliers; i++) {
    matching_keypoints_2[0].col(i) = matching_keypoints_a.col(vec_inliers[i]);
    matching_keypoints_2[1].col(i) = matching_keypoints_b.col(vec_inliers[i]);
  }

  // Refine the found T and R via bundle adjustment
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.max_num_iterations = 200;
  options.logging_type = ceres::SILENT;
  options.num_threads = FLAGS_num_threads;
  ceres::Solver::Summary summary;
  std::vector<Eigen::Affine3d> cameras(2);
  cameras[0].setIdentity();
  cameras[1].linear() = r;
  cameras[1].translation() = t;
  Eigen::Matrix3Xd pid_to_global_t_point(3, matching_keypoints_2[0].cols());
  double error;
  int num_pts_behind_camera = 0;
  for (ptrdiff_t i = 0; i < matching_keypoints_2[0].cols(); i++) {
    pid_to_global_t_point.col(i) = TriangulatePoint(
      Eigen::Vector3d(matching_keypoints_2[0](0, i), matching_keypoints_2[0](1, i), camera_params.GetFocalLength()),
      Eigen::Vector3d(matching_keypoints_2[1](0, i), matching_keypoints_2[1](1, i), camera_params.GetFocalLength()), r,
      t, &error);
    Eigen::Vector3d P = pid_to_global_t_point.col(i);
    Eigen::Vector3d Q = r * P + t;
    if (P[2] <= 0 || Q[2] <= 0) {
      num_pts_behind_camera++;
    }
  }
  LOG(DEBUG) << "Pair "
             << ": number of points behind cameras: " << num_pts_behind_camera << "/" << matching_keypoints_2[0].cols()
             << " (" << round((100.0 * num_pts_behind_camera) / matching_keypoints_2[0].cols()) << "%)";

  BundleAdjustSmallSet(matching_keypoints_2, camera_params.GetFocalLength(), &cameras, &pid_to_global_t_point,
                       new ceres::CauchyLoss(0.5), options, &summary);

  if (!summary.IsSolutionUsable()) {
    LOG(ERROR) << " Failed to refine RT with bundle adjustment";
    return boost::none;
  }
  LOG(DEBUG) << summary.BriefReport();

  // Give the solution
  Eigen::Affine3d result = cameras[1] * cameras[0].inverse();
  result.translation().normalize();

  // TODO(rsoussan): Clean this section up!
  // Return valid inliers, limit number of inliers by provided max
  cv::Mat valid = cv::Mat::zeros(num_matches, 1, CV_8UC1);
  for (int i = 0; i < static_cast<int>(vec_inliers.size()); ++i) {
    valid.at<uint8_t>(vec_inliers[i], 0) = 1;
  }

  // Filter inliers by distance if neccessary
  int num_inliers = std::accumulate(valid.begin<uint8_t>(), valid.end<uint8_t>(), 0);
  if (num_inliers > max_num_matches) {
    std::vector<double> dist;
    for (int query_index = 0; query_index < static_cast<int>(matches.size()); ++query_index) {
      if (valid.at<uint8_t>(query_index, 0) > 0) dist.push_back(matches[query_index].distance);
    }
    std::sort(dist.begin(), dist.end());
    const double max_dist = dist[max_num_matches - 1];
    for (int query_index = 0; query_index < static_cast<int>(matches.size()); ++query_index) {
      if (valid.at<uint8_t>(query_index, 0) > 0 && matches[query_index].distance > max_dist) {
        valid.at<uint8_t>(query_index, 0) = 0;
      }
    }
    num_inliers = std::accumulate(valid.begin<uint8_t>(), valid.end<uint8_t>(), 0);
  }

  // Copy the inliers only
  inlier_matches->clear();
  inlier_matches->reserve(num_of_inliers);
  for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
    if (valid.at<uint8_t>(i, 0) > 0) {
      inlier_matches->push_back(matches[i]);
    }
  }

  return result;
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

// This is a very specialized function
// TODO(rsoussan): Clean this up? Use for ransac affine3d?
void BundleAdjustSmallSet(std::vector<Eigen::Matrix2Xd> const& features_n, double focal_length,
                          std::vector<Eigen::Affine3d>* cam_T_global_n, Eigen::Matrix3Xd* pid_to_global_t_point,
                          ceres::LossFunction* loss, ceres::Solver::Options const& options,
                          ceres::Solver::Summary* summary) {
  CHECK(cam_T_global_n) << "Variable cam_T_global_n needs to be defined";
  CHECK(cam_T_global_n->size() == features_n.size())
    << "Variables features_n and cam_T_global_n need to agree on the number of cameras";
  CHECK(cam_T_global_n->size() > 1) << "Bundle adjust needs at least 2 or more cameras";
  CHECK(pid_to_global_t_point->cols() == features_n[0].cols())
    << "There should be an equal amount of XYZ points as there are feature observations";
  for (size_t i = 1; i < features_n.size(); i++) {
    CHECK(features_n[0].cols() == features_n[i].cols()) << "The same amount of features should be seen in all cameras";
  }

  const size_t n_cameras = features_n.size();

  // Allocate space for the angle axis representation of rotation
  std::vector<Eigen::Vector3d> aa(n_cameras);
  for (size_t cid = 0; cid < n_cameras; cid++) {
    camera::RotationToRodrigues(cam_T_global_n->at(cid).linear(), &aa[cid]);
  }

  // Build the problem
  ceres::Problem problem;
  for (ptrdiff_t pid = 0; pid < pid_to_global_t_point->cols(); pid++) {
    for (size_t cid = 0; cid < n_cameras; cid++) {
      ceres::CostFunction* cost_function = ReprojectionError::Create(features_n[cid].col(pid));
      problem.AddResidualBlock(cost_function, loss, &cam_T_global_n->at(cid).translation()[0], &aa.at(cid)[0],
                               &pid_to_global_t_point->col(pid)[0], &focal_length);
    }
  }
  problem.SetParameterBlockConstant(&focal_length);

  // Solve the problem
  ceres::Solve(options, &problem, summary);

  // Write the rotations back to the transform
  Eigen::Matrix3d r;
  for (size_t cid = 0; cid < n_cameras; cid++) {
    camera::RodriguesToRotation(aa[cid], &r);
    cam_T_global_n->at(cid).linear() = r;
  }
}
}  // namespace sparse_mapping
