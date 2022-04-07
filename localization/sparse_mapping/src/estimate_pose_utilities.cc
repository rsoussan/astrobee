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

#include <sparse_mapping/estimate_pose_utilities.h>
#include <sparse_mapping/sparse_mapping.h>
#include <sparse_mapping/utilities.h>

#include <ff_common/thread.h>
#include <camera/camera_model.h>

#include <ceres/rotation.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <gflags/gflags.h>

#include <random>
#include <thread>
#include <unordered_map>

DEFINE_uint64(num_min_localization_inliers, 10,
              "If fewer than this many number of inliers, localization has failed.");

namespace sparse_mapping {
std::vector<ImageMatch> FindAndSortMatches(const std::vector<int>& matching_cids, const SparseMap& map,
                                           const int max_num_total_feature_matches, const bool check_point_3d_exists,
                                           const int min_matches_per_image) {
  std::vector<ImageMatch> sorted_image_matches;
  int total_matches = 0;
  // TODO(oalexan1): Use multiple threads here?
  for (const auto cid : matching_cids) {
    const auto& map_image_descriptors = map.cid_to_descriptor_map_[cid];
    ImageMatch image_match;
    image_match.cid = cid;
    FindMatches(descriptors,
                                map_image_descriptors,
                                &image_match.matches);
    int num_valid_matches = 0;
    if (!check_point_3d_exists) {
      image_match.num_valid_matches = image_match.matches.size();
    } else {
      for (const auto& match : image_match.matches) {
       const bool map_point_3d_exists = map.cid_fid_to_pid_[cid].count(match.trainIdx) > 0;
        if (!map_point_3d_exists) continue;
        ++image_match.num_valid_matches;
      }
    }

    sorted_image_matches.emplace_back(image_match);
    total_matches += image_match.num_valid_matches;
    if (total_matches >= max_num_total_feature_matches)
      break;
  }

  std::sort(image_matches.begin(), image_matches.end(), std::greater<>());
  return image_matches;
}

void GetMatchingObservationsAndLandmarks(const std::vector<ImageMatches>& image_matches, const SparseMap& map,
                                         std::vector<Eigen::Vector2d>& observations,
                                         std::vector<Eigen::Vector3d>& landmarks) {
  std::set<int> seen_landmarks;
  for (const auto& image_match : image_matches) {
    for (const auto& match : image_match.matches) {
      const bool map_point_3d_exists = map.cid_fid_to_pid_[cid].count(match.trainIdx) > 0;
      if (!map_point_3d_exists) continue;
      const int landmark_id = map.cid_fid_to_pid_.at(cid).at(match.trainIdx);
      if (seen_landmarks.count(landmark_id) > 0) continue;
      const Eigen::Vector2d observation(keypoints.col(match.queryIdx)[0],
                          keypoints.col(match.queryIdx)[1]);
      observations.emplace_back(observation);
      landmarks.push_back(map.pid_to_xyz_[landmark_id]);
      seen_landmarks.insert(landmark_id);
      ++num_matches;
    }
  }
}

EstimatePoseResults EstimatePose(const cv::Mat& descriptors, const Eigen::Matrix2Xd& keypoints, const SparseMap& map,
                                 const EstimatePoseParams& params) {
  const auto matching_cids = map.image_database().Query(descriptors, params.max_image_matches);
  if (matching_cids.empty()) {
    LOG(FATAL) << "No matching cids found.";
  }

  const auto image_matches =
    FindAndSortMatches(matching_cids, map, params.max_num_total_feature_matches, params.check_point_3d_exists);
  std::vector<Eigen::Vector2d> observations;
  std::vector<Eigen::Vector3d> landmarks;
  GetMatchingObservationsAndLandmarks(image_matches, map, observations, landmarks);

  // TODO(rsoussan): Update this to return estimate pose results or use vision_common function
  std::vector<Eigen::Vector2d> inlier_landmarks_vec;
  std::vector<Eigen::Vector2d>* inlier_landmarks = params.inlier_landmarks ? &inlier_landmarks_vec : nullptr;
  std::vector<Eigen::Vector3d> inlier_observations_vec;
  std::vector<Eigen::Vector3d>* inlier_observations = params.inlier_observations ? &inlier_observations_vec : nullptr;
  camera::CameraModel camera_estimate;
  int ret = RansacEstimateCamera(landmarks, observations, params.num_ransac_iterations, params.ransac_inlier_tolerance,
                                 camera_estimate, inlier_landmarks, inlier_observations,
                                 // TODO(rsoussan): Change this to use LOG(DEBUG)
                                 FLAGS_verbose_localization);
  EstimatePoseResults results;
  if (ret) {
    results.pose = camera_estimate;
    if (params.inlier_landmarks) results.inlier_landmarks = *inlier_landmarks;
    if (params.inlier_observations) results.inlier_observations = *inlier_observations;
}
  return results;
}

EstimatePoseResults EstimatePose(
  const cv::Mat& image, const EstimatePoseParams& params, SparseMap& map) {
  cv::Mat descriptors;
  Eigen::Matrix2Xd keypoints;
  DetectFeatures(image, map.params().histogram_equalization, descriptors, keypoints);
  return EstimatePose(descriptors, keypoints, map, params);
}

EstimatePoseResults EstimatePose(const std::string& image_filename, const EstimatePoseParams& params, SparseMap& map) {
  const auto image = LoadImage(filename);
  return EstimatePose(image, params, map);
}

// if parms is null, don't worry about converting to pixels
struct ReprojectionError {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit ReprojectionError(const Eigen::Vector2d & observed)
    : observed(observed) {}

  template <typename T>
  bool operator()(const T* const camera_p_global,
                  const T* const camera_aa_global,
                  const T* const point_global,
                  const T* const focal_length,
                  T* residuals) const {
    // Project the point into the camera's coordinate frame
    T p[3];
    ceres::AngleAxisRotatePoint(camera_aa_global, point_global, p);
    p[0] += camera_p_global[0];
    p[1] += camera_p_global[1];
    p[2] += camera_p_global[2];

    T xp = (p[0] / p[2]) * focal_length[0];
    T yp = (p[1] / p[2]) * focal_length[0];

    // The error is the difference between the prediction and observed
    residuals[0] = xp - T(observed.x());
    residuals[1] = yp - T(observed.y());

    return true;
  }

  // Helper function ... make the code look nice
  static ceres::CostFunction* Create(const Eigen::Vector2d & observed) {
    return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3, 3, 3, 1>
            (new ReprojectionError(observed)));
  }

  Eigen::Vector2d observed;
};

void EstimateCamera(camera::CameraModel* camera_estimate,
                    std::vector<Eigen::Vector3d>* landmarks,
                    const std::vector<Eigen::Vector2d> & observations,
                    const ceres::Solver::Options & options,
                    ceres::Solver::Summary* summary) {
  Eigen::Affine3d guess = camera_estimate->GetTransform();
  camera::CameraParameters params = camera_estimate->GetParameters();

  // Initialize the angle axis representation of rotation
  Eigen::Vector3d aa;
  camera::RotationToRodrigues(guess.linear(), &aa);

  double focal_length = params.GetFocalLength();

  // Build problem
  ceres::Problem problem;
  for (size_t pid = 0; pid < landmarks->size(); pid++) {
    ceres::CostFunction* cost_function = ReprojectionError::Create(
                Eigen::Vector2d(observations[pid].x(), observations[pid].y()));
    problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(1.0),
                             &guess.translation()[0],
                             &aa[0],
                             &landmarks->at(pid)[0],
                             &focal_length);
    problem.SetParameterBlockConstant(&landmarks->at(pid)[0]);
  }
  problem.SetParameterBlockConstant(&focal_length);

  // Solve the problem
  ceres::Solve(options, &problem, summary);

  // Write the rotations back to the transform
  Eigen::Matrix3d r;
  camera::RodriguesToRotation(aa, &r);
  guess.linear() = r;
  camera_estimate->SetTransform(guess);
}

// random intger in [min, max)
int RandomInt(int min, int max) {
  static std::mt19937 generator;  // should be thread_local for thread safe, gcc 4.6 doesn't support
  std::uniform_int_distribution<int> random_item(min, max - 1);
  return random_item(generator);
}

void SelectRandomObservations(const std::vector<Eigen::Vector3d> & all_landmarks,
        const std::vector<Eigen::Vector2d> & all_observations, size_t num_selected,
        std::vector<cv::Point3d> * landmarks, std::vector<cv::Point2d> * observations) {
  std::unordered_map<int, int> used;
  // not enough observations
  if (all_observations.size() < num_selected)
    return;
  // Reserve space in the output so we don't have to keep reallocating on
  // push_back().
  landmarks->reserve(num_selected);
  observations->reserve(num_selected);
  while (observations->size() < num_selected) {
    int id = RandomInt(0, all_observations.size());
    if (used.count(id) > 0)
      continue;
    Eigen::Vector3d p = all_landmarks[id];
    landmarks->push_back(cv::Point3d(p[0], p[1], p[2]));
    observations->push_back(cv::Point2d(all_observations[id][0], all_observations[id][1]));
    used[id] = 1;
  }
}

bool P3P(const std::vector<cv::Point3d> & landmarks, const std::vector<cv::Point2d> & observations,
         const camera::CameraParameters & params, Eigen::Vector3d * pos, Eigen::Matrix3d * rotation) {
    cv::Mat camera_matrix(3, 3, cv::DataType<double>::type);
    cv::eigen2cv(params.GetIntrinsicMatrix<camera::UNDISTORTED_C>(), camera_matrix);
    cv::Mat rvec(3, 1, cv::DataType<double>::type, cv::Scalar(0));
    cv::Mat tvec(3, 1, cv::DataType<double>::type, cv::Scalar(0));
    cv::Mat distortion(4, 1, cv::DataType<double>::type, cv::Scalar(0));
    bool result = cv::solvePnP(landmarks, observations, camera_matrix, distortion, rvec, tvec, false, cv::SOLVEPNP_P3P);
    if (!result)
      return false;
    cv::cv2eigen(tvec, *pos);
    camera::RodriguesToRotation(Eigen::Vector3d(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)), rotation);
    return true;
}

size_t CountInliers(const std::vector<Eigen::Vector3d> & landmarks, const std::vector<Eigen::Vector2d> & observations,
                 const camera::CameraModel & camera, int tolerance, std::vector<size_t>* inliers) {
  int num_inliers = 0;
  if (inliers) {
    // To save ourselves some allocation time. We'll prealloc for a 50% inlier
    // success rate
    inliers->reserve(observations.size()/2);
  }

  double tolerance_sq = tolerance * tolerance;

  for (size_t i = 0; i < landmarks.size(); i++) {
    Eigen::Vector2d pos = camera.ImageCoordinates(landmarks[i]);
    if ((observations[i] - pos).squaredNorm() <= tolerance_sq) {
      num_inliers++;
      if (inliers)
        inliers->push_back(i);
    }
  }
  return num_inliers;
}

int RansacEstimateCamera(const std::vector<Eigen::Vector3d> & landmarks,
                         const std::vector<Eigen::Vector2d> & observations,
                         int num_tries, int inlier_tolerance, camera::CameraModel * camera_estimate,
                         std::vector<Eigen::Vector3d> * inlier_landmarks_out,
                         std::vector<Eigen::Vector2d> * inlier_observations_out,
                         bool verbose) {
  size_t best_inliers = 0;
  camera::CameraParameters params = camera_estimate->GetParameters();

  // Need the minimum number of observations
  if (observations.size() < 4)
    return 1;

  // RANSAC to find the best camera with P3P
  std::vector<cv::Point3d> subset_landmarks;
  std::vector<cv::Point2d> subset_observations;
  // TODO(oalexan1): Use multiple threads here?
  for (int i = 0; i < num_tries; i++) {
    subset_landmarks.clear();
    subset_observations.clear();
    SelectRandomObservations(landmarks, observations, 4, &subset_landmarks, &subset_observations);

    Eigen::Vector3d pos;
    Eigen::Matrix3d rotation;
    bool result = P3P(subset_landmarks, subset_observations, params, &pos, &rotation);
    if (!result)
      continue;
    Eigen::Affine3d cam_t_global;
    cam_t_global.setIdentity();
    cam_t_global.translate(pos);
    cam_t_global.rotate(rotation);
    camera::CameraModel guess(cam_t_global, camera_estimate->GetParameters());

    size_t inliers = CountInliers(landmarks, observations, guess, inlier_tolerance, NULL);
    if (inliers > best_inliers) {
      best_inliers = inliers;
      *camera_estimate = guess;
    }
  }

  if (verbose)
    std::cout << observations.size() << " Ransac observations "
              << best_inliers << " inliers\n";

  // TODO(bcoltin): Return some sort of confidence?
  if (best_inliers < FLAGS_num_min_localization_inliers)
    return 2;

  std::vector<size_t> inliers;
  CountInliers(landmarks, observations, *camera_estimate, inlier_tolerance, &inliers);
  std::vector<Eigen::Vector3d> inlier_landmarks;
  std::vector<Eigen::Vector2d> inlier_observations;
  inlier_landmarks.reserve(inliers.size());
  inlier_observations.reserve(inliers.size());
  for (size_t idx : inliers) {
    inlier_landmarks.push_back(landmarks[idx]);
    inlier_observations.push_back(observations[idx]);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.num_threads = 1;  // it is no slower with only one thread
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  // improve estimate with CERES solver
  EstimateCamera(camera_estimate, &inlier_landmarks, inlier_observations, options, &summary);

  // find inliers again with refined estimate
  inliers.clear();
  best_inliers = CountInliers(landmarks, observations, *camera_estimate, inlier_tolerance, &inliers);

  if (verbose)
    std::cout << "Number of inliers with refined camera: " << best_inliers << "\n";

  if (best_inliers < FLAGS_num_min_localization_inliers)
    return 2;

  inlier_landmarks.clear();
  inlier_observations.clear();
  inlier_landmarks.reserve(inliers.size());
  inlier_observations.reserve(inliers.size());
  for (size_t idx : inliers) {
    inlier_landmarks.push_back(landmarks[idx]);
    inlier_observations.push_back(observations[idx]);
  }
  if (inlier_landmarks_out) {
    inlier_landmarks_out->reserve(inliers.size());
    std::copy(inlier_landmarks.begin(), inlier_landmarks.end(),
        std::back_inserter(*inlier_landmarks_out));
  }
  if (inlier_observations_out) {
    inlier_observations_out->reserve(inliers.size());
    std::copy(inlier_observations.begin(), inlier_observations.end(),
        std::back_inserter(*inlier_observations_out));
  }

  return 0;
}

// Given two sets of 3D points, find the rotation + translation + scale
// which best maps the first set to the second.
// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm

void Find3DAffineTransform(Eigen::Matrix3Xd const & in,
                           Eigen::Matrix3Xd const & out,
                           Eigen::Affine3d* result) {
  // Default output
  result->linear() = Eigen::Matrix3d::Identity(3, 3);
  result->translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // Local copies we can modify
  Eigen::Matrix3Xd local_in = in, local_out = out;

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < local_in.cols()-1; col++) {
    dist_in  += (local_in.col(col+1) - local_in.col(col)).norm();
    dist_out += (local_out.col(col+1) - local_out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return;
  double scale = dist_out/dist_in;
  local_out /= scale;

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < local_in.cols(); col++) {
    in_ctr  += local_in.col(col);
    out_ctr += local_out.col(col);
  }
  in_ctr /= local_in.cols();
  out_ctr /= local_out.cols();
  for (int col = 0; col < local_in.cols(); col++) {
    local_in.col(col)  -= in_ctr;
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
  result->translation() = scale*(out_ctr - R*in_ctr);
}

}  // namespace sparse_mapping
