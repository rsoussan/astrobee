/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http:  //  www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <sparse_mapping/utilities.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
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
// From pid_to_cid_fid, create cid_fid_to_pid for lookup.
void InitializeCidFidToPid(int num_cid,
                           std::vector<std::map<int, int> > const& pid_to_cid_fid,
                           std::vector<std::map<int, int> > * cid_fid_to_pid) {
  cid_fid_to_pid->clear();
  cid_fid_to_pid->resize(num_cid, std::map<int, int>());

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (std::pair<int, int> const& cid_fid : pid_to_cid_fid[pid]) {
      (*cid_fid_to_pid)[cid_fid.first][cid_fid.second] = pid;
    }
  }
}

cv::Mat LoadImage(const std::string& filename) {
  const cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
  if (image.rows == 0 || image.cols == 0)
    LOG(FATAL) << "Found empty image in file: " << filename;
  return image;
}

void DetectFeatures(const cv::Mat& image, const bool histogram_equalization,
                                interest_point::Detector& detector,
                               cv::Mat* descriptors,
                               Eigen::Matrix2Xd* keypoints) {
  cv::Mat hist_image;
  if (histogram_equalization) {
    cv::equalizeHist(image, hist_image);
  }
  const auto& input_image = histogram_equalization ? hist_image : image;

  std::vector<cv::KeyPoint> storage;
  detector.Detect(input_image, &storage, descriptors);

  keypoints->resize(2, storage.size());
  Eigen::Vector2d output;
  for (int i = 0; i < static_cast<int>(storage.size()); ++i) {
    camera_params_.Convert<camera::DISTORTED_C, camera::UNDISTORTED_C>
      (Eigen::Vector2d(storage[i].pt.x, storage[i].pt.y), &output);
    keypoints->col(i) = output;
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
}  // namespace sparse_mapping
