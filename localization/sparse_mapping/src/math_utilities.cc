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
Eigen::Quaternion<double> slerp_n(std::vector<double> const& W,
                                                  std::vector<Eigen::Quaternion<double> > const& Q) {
  if (W.size() != Q.size())
    LOG(FATAL) << "Expecting as many quaternions as weights.";

  if (Q.empty())
    LOG(FATAL) << "Expecting at least one quaternion and weight.";

  if (Q.size() == 1)
    return Q[0];

  if (Q.size() == 2) {
    if (!(std::abs(W[0] + W[1] - 1.0) < 1e-6 && W[0] >= 0 && W[1] >= 0))
      LOG(FATAL) << "Expecting the weights to be >= 0 and sum up to 1.";
    return Q[0].slerp(W[1], Q[1]);
  }

  // Call recursively this function with fewer terms
  double sum = W[0] + W[1];
  if (sum == 0) sum = 1.0;
  Eigen::Quaternion<double> q = Q[0].slerp(W[1]/sum, Q[1]);
  std::vector<double> W2 = W;
  std::vector<Eigen::Quaternion<double> > Q2 = Q;
  W2.erase(W2.begin());
  Q2.erase(Q2.begin());
  W2[0] = sum;
  Q2[0] = q;
  return slerp_n(W2, Q2);
}

  Eigen::Vector3d TriangulatePoint(Eigen::Vector3d const& unnormalized_pt1, Eigen::Vector3d const& unnormalized_pt2,
                                   Eigen::Matrix3d const& cam2_r_cam1, Eigen::Vector3d const& cam2_t_cam1,
                                   double* error) {
    // The second camera's center in the coordinate system of the first
    // camera.
    Eigen::Vector3d p2 = -cam2_r_cam1.transpose() * cam2_t_cam1;

    // Calculate the two unit pointing vectors in the domain of cam1
    Eigen::Vector3d unit1 = unnormalized_pt1.normalized();
    Eigen::Vector3d unit2 = cam2_r_cam1.transpose() * unnormalized_pt2.normalized();

    Eigen::Vector3d v12 = unit1.cross(unit2);
    Eigen::Vector3d v1 = v12.cross(unit1);
    Eigen::Vector3d v2 = v12.cross(unit2);

    Eigen::Vector3d closestPoint1 = v2.dot(p2) / v2.dot(unit1) * unit1;
    Eigen::Vector3d closestPoint2 = p2 + v1.dot(-p2) / v1.dot(unit2) * unit2;
    *error = (closestPoint2 - closestPoint1).norm();

    return 0.5 * (closestPoint2 + closestPoint1);
  }

void Triangulate(const bool rm_invalid_xyz, const double focal_length,
                 const std::vector<Eigen::Affine3d>& cid_to_cam_t_global,
                 const std::vector<Eigen::Matrix2Xd>& cid_to_keypoint_map,
                 std::vector<std::map<int, int> > * pid_to_cid_fid,
                 std::vector<Eigen::Vector3d> * pid_to_xyz,
                 std::vector<std::map<int, int> > * cid_fid_to_pid) {
  Eigen::Matrix3d k;
  k << focal_length, 0, 0,
    0, focal_length, 0,
    0, 0, 1;

  // Build p matrices for all of the cameras. openMVG::Triangulation
  // will be holding pointers to all of the cameras.
  std::vector<openMVG::Mat34> cid_to_p(cid_to_cam_t_global.size());
  for (int cid = 0; cid < cid_to_p.size(); ++cid) {
    openMVG::P_From_KRt(k, cid_to_cam_t_global[cid].linear(),
                        cid_to_cam_t_global[cid].translation(), &cid_to_p[cid]);
  }

  pid_to_xyz->resize(pid_to_cid_fid->size());
  for (int pid = pid_to_cid_fid->size() - 1; pid >= 0; --pid) {
    openMVG::Triangulation tri;
    for (const auto& cid_fid : pid_to_cid_fid->at(pid)) {
      tri.add(cid_to_p[cid_fid.first],  // they're holding a pointer to this
              cid_to_keypoint_map[cid_fid.first].col(cid_fid.second));
    }
    const Eigen::Vector3d solution = tri.compute();
    if ( rm_invalid_xyz && (std::isnan(solution[0]) || tri.minDepth() < 0) ) {
      pid_to_xyz->erase(pid_to_xyz->begin() + pid);
      pid_to_cid_fid->erase(pid_to_cid_fid->begin() + pid);
    } else {
      pid_to_xyz->at(pid) = solution;
    }
  }

  // Must always keep the book-keeping correct
  InitializeCidFidToPid(cid_to_cam_t_global.size(),
                                        *pid_to_cid_fid,
                                        cid_fid_to_pid);
}



// Apply a given transform to the specified xyz points, and adjust accordingly the cameras
// for consistency. We assume that the transform is of the form
// A(x) = scale * rotation * x + translation
void TransformCamerasAndPoints(Eigen::Affine3d const& A,
                                               std::vector<Eigen::Affine3d> *cid_to_cam_t,
                                               std::vector<Eigen::Vector3d> *xyz) {
  for (size_t pid = 0; pid < (*xyz).size(); pid++)
    (*xyz)[pid] = A * (*xyz)[pid];

  // Inverse of rotation component
  double scale = pow(A.linear().determinant(), 1.0/3.0);
  Eigen::MatrixXd Ainv = (A.linear()/scale).inverse();

  for (size_t cid = 0; cid < (*cid_to_cam_t).size(); cid++) {
    (*cid_to_cam_t)[cid].linear() = (*cid_to_cam_t)[cid].linear()*Ainv;
    (*cid_to_cam_t)[cid].translation() = scale*(*cid_to_cam_t)[cid].translation() -
      (*cid_to_cam_t)[cid].linear()*A.translation();
  }
}

double ReprojectionErrorThreshold(const std::vector<double>& reprojection_errors,
                                  const RemoveInvalidPointsParams& params) {
  const int num_errors = reprojection_errors.size();
  if (num_errors == 0) return 0;

  std::vector<double> sorted_reprojection_errors = reprojection_errors;
  std::sort(sorted_reprojection_errors.begin(), sorted_reprojection_errors.end());

  if (num_errors <= 2)
    return params.reprojection_error_threshold_scale_factor * sorted_reprojection_errors[num_errors - 1];
  const double scaled_median_reprojection_error =
    params.reprojection_error_threshold_scale_factor * sorted_reprojection_errors[num_errors / 2];
  return std::max(scaled_median_reprojection_error, params.max_reprojection_error);
}

double MaxAngleBetweenCameraRays(const int pid, const std::vector<std::map<int, int> >& pid_to_cid_fid,
                                        const std::vector<Eigen::Vector3d>& global_t_cams,
                                        const std::vector<Eigen::Vector3d>& pid_to_xyz) {
  const auto& track = pid_to_cid_fid[pid];
  const auto global_t_point = pid_to_xyz[pid];
  double max_angle = 0;
  int cid = 0;
  for (auto cid_fid_it1 = track.begin();
       cid_fid_it1 != track.end(); ++cid_fid_it1) {
    const int cid1 = cid_fid_it1->first;
    for (auto cid_fid_it2 = cid_fid_it1+1;
         cid_fid_it2 != track.end(); ++cid_fid_it2) {
      const int cid2 = cid_fid_it2->first;
      const Eigen::Vector3d cam1_t_point = global_t_cams[cid1] - global_t_point;
      const Eigen::Vector3d cam2_t_point = global_t_cams[cid2] - global_t_point;
      // TODO(rsoussan): make function for this next part, call AngleBetweenRays!! (A)
      const double l1 = cam1_t_point.norm();
      const double l2 = cam2_t_point.norm();
      if (l1 == 0 || l2 == 0)
        continue;

      double dot = cam1_t_point.dot(cam2_t_point)/(l1*l2);
      dot = std::min(dot, 1.0);
      dot = std::max(-1.0, dot);
      const double angle = (180.0/M_PI)*std::acos(dot);
      max_angle = std::max(angle, max_angle);
    }
  }
  return max_angle;
}

void RemoveInvalidPoints(const RemoveInvalidPointsParams& params,
                         const std::vector<Eigen::Affine3d>& cid_to_cam_t_global,
                         const std::vector<Eigen::Matrix2Xd>& cid_to_keypoint_map,
                         std::vector<std::map<int, int> >* pid_to_cid_fid, std::vector<Eigen::Vector3d>* pid_to_xyz) {
  std::vector<double> pid_reprojection_errors;
  const int num_cams = cid_to_cam_t_global.size();
  std::vector<Eigen::Vector3d> global_t_cams;
  global_t_cams.reserve(num_cams);
  for (int cid = 0; cid < num_cams; ++cid) {
    global_t_cams.emplace_back(cid_to_cam_t_global[cid].inverse().translation());
  }

  RemoveInvalidPointsStats stats;
  stats.num_points = pid_to_xyz->size();
  std::vector<bool> is_bad(pid_to_xyz->size(), false);
  const Eigen::Vector2d half_size = camera_params.GetUndistortedHalfSize();
  for (int pid = 0; pid < static_cast<int>(pid_to_xyz->size()); ++pid) {
    bool small_angle = false, behind_cam = false, invalid_reproj = false;
    const double max_angle_between_camera_rays
      = MaxAngleBetweenCameraRays(pid, *pid_to_cid_fid,
                                         global_t_cams,  *pid_to_xyz);
    if (max_angle_between_camera_rays < params.min_max_angle_between_camera_rays) {
      small_angle = true;
      is_bad[pid] = true;
    }

    for (const auto cid_fid : (*pid_to_cid_fid)[pid]) {
      const Eigen::Vector2d pix = (cid_to_cam_t_global[cid_fid.first] *
                             (*pid_to_xyz)[pid]).hnormalized() * camera_params.GetFocalLength();
      pid_reprojection_errors.push_back((cid_to_keypoint_map[cid_fid.first].col(cid_fid.second) - pix).norm());
      // Mark points which don't project at valid camera pixels
      // TODO(zmoratto) : This can probably be done with a Eigen Array reduction
      if (pix[0] < -half_size[0] || pix[0] >= half_size[0] || pix[1] < -half_size[1] || pix[1] >= half_size[1]) {
        invalid_reproj = true;
        is_bad[pid] = true;
      }

      // Mark points that are behind the camera
      const Eigen::Vector3d P = cid_to_cam_t_global[cid_fid.first] * (*pid_to_xyz)[pid];
      if (P[2] <= 0) {
        behind_cam = true;
        is_bad[pid] = true;
      }
    }
    stats.small_angle    += static_cast<int>(small_angle);
    stats.behind_cam     += static_cast<int>(behind_cam);
    stats.invalid_reproj += static_cast<int>(invalid_reproj);
  }

  // TODO(rsoussan): Clean this up - why using reverse iterator?
  for (int pid = (*pid_to_xyz).size() - 1; pid < static_cast<int>((*pid_to_xyz).size()); --pid) {
    if (is_bad[pid]) {
      auto cid_fid_it = (*pid_to_cid_fid).begin();
      auto xyz_it = (*pid_to_xyz).begin();
      std::advance(cid_fid_it, pid);
      std::advance(xyz_it, pid);
      pid_to_cid_fid->erase(cid_fid_it);
      pid_to_xyz->erase(xyz_it);
    }
  }

  // TODO(rsoussan): why is this done after first pass??? to get only valid thresh in geterrthresh?
  // Wipe all features who are further than the reprojection of the
  // corresponding 3D point than given threshold.
  const double reprojection_error_threshold = ReprojectionErrorThreshold(pid_reprojection_errors, params);
  LOG(INFO) << "Filtering features with reprojection error higher than: "
            << reprojection_error_threshold << " pixels";
  for (int pid = (*pid_to_xyz).size() - 1; pid < static_cast<int>((*pid_to_xyz).size()); --pid) {
    const auto& cid_fid = (*pid_to_cid_fid)[pid];
    auto itr = cid_fid.begin();
    while (itr != cid_fid.end()) {
      ++stats.num_features;
      const Eigen::Vector2d pix = (cid_to_cam_t_global[itr->first] *
                             (*pid_to_xyz)[pid]).hnormalized() * camera_params.GetFocalLength();
      // TODO(rsoussan): Make function for this!
      const double err
        = (cid_to_keypoint_map[itr->first].col(itr->second) - pix).norm();

      if (err >= reprojection_error_threshold) {
        auto toErase = itr;
        ++itr;
        cid_fid.erase(toErase);
        stats.big_reproj_err++;
      } else {
        ++itr;
      }
    }

    // Wipe a 3D point altogether if it corresponds to less than 2 matches.
    const int total = (*pid_to_cid_fid)[pid].size();
    if (total < 2) {
      auto cid_fid_it
        = pid_to_cid_fid->begin();
      auto xyz_it = pid_to_xyz->begin();
      std::advance(cid_fid_it, pid);
      std::advance(xyz_it, pid);
      pid_to_cid_fid->erase(cid_fid_it);
      pid_to_xyz->erase(xyz_it);
    }
  }

  if (params.print_stats)
    stats.Print();
}

void DetectFeatures(const cv::Mat& image, const bool histogram_equalization,
                                vision_common::DynamicDetector& detector,
                               cv::Mat& descriptors,
                               Eigen::Matrix2Xd& keypoints) {
  cv::Mat hist_image;
  if (histogram_equalization) {
    cv::equalizeHist(image, hist_image);
  }
  const auto& input_image = histogram_equalization ? hist_image : image;

  std::vector<cv::KeyPoint> storage;
  detector.Detect(input_image, &storage, &descriptors);

  keypoints.resize(2, storage.size());
  Eigen::Vector2d output;
  for (int i = 0; i < static_cast<int>(storage.size()); ++i) {
    camera_params_.Convert<camera::DISTORTED_C, camera::UNDISTORTED_C>
      (Eigen::Vector2d(storage[i].pt.x, storage[i].pt.y), &output);
    keypoints.col(i) = output;
  }
}

bool RobustEssential(Eigen::Matrix3d const& k1, Eigen::Matrix3d const& k2,
                                     Eigen::Matrix2Xd const& x1, Eigen::Matrix2Xd const& x2,
                                     Eigen::Matrix3d * e,
                                     std::vector<size_t> * vec_inliers,
                                     std::pair<size_t, size_t> const& size1,
                                     std::pair<size_t, size_t> const& size2,
                                     double * error_max,
                                     double precision) {
  CHECK(e) << "Missing e argument";
  CHECK(vec_inliers) << "Missing vec inliers argument";

  typedef openMVG::essential::kernel::FivePointKernel SolverType;
  typedef openMVG::robust::ACKernelAdaptorEssential<
    SolverType,
    openMVG::fundamental::kernel::EpipolarDistanceError,
    Eigen::Matrix3d>
    KernelType;

  KernelType kernel(x1, size1.first, size1.second,
                    x2, size2.first, size2.second, k1, k2);

  std::pair<double, double> ransac_output =
    openMVG::robust::ACRANSAC(kernel, *vec_inliers, 4096 /* iterations */,
                              e, precision, false);
  *error_max = ransac_output.first;

  return vec_inliers->size() > 1.5 * SolverType::MINIMUM_SAMPLES;
}

bool EstimateRTFromE(Eigen::Matrix3d const& k1, Eigen::Matrix3d const& k2,
                                     Eigen::Matrix2Xd const& x1, Eigen::Matrix2Xd const& x2,
                                     Eigen::Matrix3d const& e, std::vector<size_t> const& vec_inliers,
                                     Eigen::Matrix3d * r, Eigen::Vector3d * t) {
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
    const Eigen::Matrix3d &r2 = possible_r[i];
    const Eigen::Vector3d &t2 = possible_t[i];
    openMVG::P_From_KRt(k2, r2, t2, &P2);
    Eigen::Vector3d X;

    for (size_t k = 0; k < vec_inliers.size(); ++k) {
      const Eigen::Vector2d & x1_ = x1.col(vec_inliers[k]),
        & x2_ = x2.col(vec_inliers[k]);
      openMVG::TriangulateDLT(P1, x1_, P2, x2_, &X);
      // Test if point is front to the two cameras.
      if (openMVG::Depth(r1, t1, X) > 0 &&
          openMVG::Depth(r2, t2, X) > 0) {
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
  void FindMatches(const cv::Mat & img1_descriptor_map,
                   const cv::Mat & img2_descriptor_map, std::vector<cv::DMatch> * matches) {
    CHECK(img1_descriptor_map.depth() ==
          img2_descriptor_map.depth())
      << "Mixed descriptor types. Did you mash BRISK with SIFT/SURF?";

    // Check for early exit conditions
    matches->clear();
    if (img1_descriptor_map.rows == 0 ||
        img2_descriptor_map.rows == 0)
      return;

    if (img1_descriptor_map.depth() == CV_8U) {
      // Binary descriptor

      // cv::BFMatcher matcher(cv::NORM_HAMMING, true  /* Forward & Backward matching */);
      cv::FlannBasedMatcher matcher(cv::makePtr<cv::flann::LshIndexParams>(3, 18, 2));
      matcher.match(img1_descriptor_map, img2_descriptor_map, *matches);

      // Select only inlier matches that meet a BRISK threshold of
      // of FLAGS_hamming_distance.
      // TODO(oalexan1) This needs further study.
      std::vector<cv::DMatch> inlier_matches;
      inlier_matches.reserve(matches->size());  // This saves time in allocation
      for (cv::DMatch const& dmatch : *matches) {
        if (dmatch.distance < FLAGS_hamming_distance) {
          inlier_matches.push_back(dmatch);
        }
      }
      matches->swap(inlier_matches);  // Doesn't invoke a copy of all elements.
    } else {
      // Traditional floating point descriptor
      cv::FlannBasedMatcher matcher;
      std::vector<std::vector<cv::DMatch> > possible_matches;
      matcher.knnMatch(img1_descriptor_map, img2_descriptor_map, possible_matches, 2);
      matches->clear();
      matches->reserve(possible_matches.size());
      for (std::vector<cv::DMatch> const& best_pair : possible_matches) {
        if (best_pair.size() == 1) {
          // This was the only best match, push it.
          matches->push_back(best_pair.at(0));
        } else {
          // Push back a match only if it is 25% better than the next best.
          if (best_pair.at(0).distance < FLAGS_goodness_ratio * best_pair.at(1).distance) {
            matches->push_back(best_pair[0]);
          }
        }
      }
    }
  }

  boost::optional<Eigen::Affine3d> MatchImages(const Eigen::Matrix2Xd& keypoints_a, const Eigen::Matrix2Xd& keypoints_b,
                                               const cv::Mat& descriptors_a, const cv::Mat& descriptors_b,
                                               const camera::CameraParameters& camera_params, const int max_num_matches,
                                               const int min_num_inliers_for_valid_match,
                                               std::vector<cv::DMatch>& inlier_matches) {
    std::vector<cv::DMatch> matches;
    FindMatches(descriptors_a, descriptors_b, &matches);

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
boost::optional<Eigen::Affine3d> EstimateRelativeAffine3D(const Eigen::Matrix2Xd& keypoints_a,
                                     const Eigen::Matrix2Xd& keypoints_b,
                                     const std::vector<cv::DMatch>& matches,
                                     const camera::CameraParameters& camera_params,
                                     const int max_num_matches,
                                     std::vector<cv::DMatch>& inlier_matches) {
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
  if (!RobustEssential(intrinsics, intrinsics, matching_keypoints_a, matching_keypoints_b,
                                       &essential_matrix, &vec_inliers,
                                       image_size, image_size,
                                       &error_max,
                                       max_expected_error)) {
    LOG(DEBUG) << "Estimation of essential matrix failed!\n";
    return boost::none;
  }

  if (vec_inliers.size() < static_cast<size_t>(FLAGS_min_valid)) {
    LOG(DEBUG) << "Failed to get enough inliers " << vec_inliers.size();
    return boost::none;
  }

  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  if (!EstimateRTFromE(intrinsics, intrinsics, matching_keypoints_a, matching_keypoints_b,
                                       essential_matrix, vec_inliers,
                                       &r, &t)) {
    LOG(DEBUG)  << "Failed to extract RT from E";
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
  Eigen::Matrix3Xd pid_to_xyz(3, matching_keypoints_2[0].cols());
  double error;
  int num_pts_behind_camera = 0;
  for (ptrdiff_t i = 0; i < matching_keypoints_2[0].cols(); i++) {
    pid_to_xyz.col(i) =
      TriangulatePoint
      (Eigen::Vector3d(matching_keypoints_2[0](0, i), matching_keypoints_2[0](1, i),
                       camera_params.GetFocalLength()),
       Eigen::Vector3d(matching_keypoints_2[1](0, i), matching_keypoints_2[1](1, i),
                       camera_params.GetFocalLength()),
       r, t, &error);
    Eigen::Vector3d P = pid_to_xyz.col(i);
    Eigen::Vector3d Q = r*P + t;
    if (P[2] <= 0 || Q[2] <= 0) {
      num_pts_behind_camera++;
    }
  }
  LOG(DEBUG) << "Pair "
             << ": number of points behind cameras: " << num_pts_behind_camera << "/" << matching_keypoints_2[0].cols()
             << " (" << round((100.0 * num_pts_behind_camera) / matching_keypoints_2[0].cols()) << "%)";

  BundleAdjustSmallSet(matching_keypoints_2, camera_params.GetFocalLength(), &cameras,
                                       &pid_to_xyz, new ceres::CauchyLoss(0.5), options,
                                       &summary);

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
  int num_inliers =
    std::accumulate(valid.begin<uint8_t>(), valid.end<uint8_t>(), 0);
  if (num_inliers > max_num_matches) {
    std::vector<double> dist;
    for (int query_index = 0; query_index < static_cast<int>(matches.size()); ++query_index) {
      if (valid.at<uint8_t>(query_index, 0) > 0)
        dist.push_back(matches[query_index].distance);
    }
    std::sort(dist.begin(), dist.end());
    const double max_dist = dist[max_num_matches - 1];
    for (int query_index = 0; query_index < static_cast<int>(matches.size()); ++query_index) {
      if (valid.at<uint8_t>(query_index, 0) > 0 &&
          matches[query_index].distance > max_dist) {
        valid.at<uint8_t>(query_index, 0) = 0;
      }
    }
    num_inliers
      = std::accumulate(valid.begin<uint8_t>(), valid.end<uint8_t>(), 0);
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
}  // namespace sparse_mapping
