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
#include <camera/camera_params.h>
#include <ff_common/init.h>
#include <interest_point/essential.h>
#include <localization_common/averager.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <sparse_mapping/tensor.h>
#include <vision_common/lk_optical_flow_feature_detector_and_matcher.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <mutex>

namespace fs = boost::filesystem;
namespace lc = localization_common;
namespace po = boost::program_options;
namespace sm = sparse_mapping;
namespace vc = vision_common;

boost::optional<vc::FeatureMatches> Matches(const vc::FeatureImage& current_image, const vc::FeatureImage& next_image,
                                            vc::LKOpticalFlowFeatureDetectorAndMatcher& detector_and_matcher) {
  const auto& matches = detector_and_matcher.Match(current_image, next_image);
  if (matches.size() < 5) {
    LogDebug("Too few matches: " << matches.size() << ", current image keypoints: " << current_image.keypoints().size()
                                 << ", next image keypoints: " << next_image.keypoints().size());
    return boost::none;
  }
  LogDebug("Found matches: " << matches.size() << ", current image keypoints: " << current_image.keypoints().size()
                             << ", next image keypoints: " << next_image.keypoints().size());
  return matches;
}

bool LowMovementImageSequence(const vc::FeatureMatches& matches, const double max_low_movement_mean_distance) {
  lc::Averager distance_averager;
  for (const auto& match : matches) {
    distance_averager.Update(match.distance);
  }
  LogDebug("Mean distance: " << distance_averager.average());
  if (distance_averager.average() <= max_low_movement_mean_distance) return true;
  return false;
}

void RelativePose(const vc::FeatureMatches& matches, const camera::CameraParameters& camera_params,
                  Eigen::Affine3d& relative_pose, std::vector<cv::DMatch>& inlier_matches) {
  // TODO(rsoussan): Update this call when surf map branch merged into dev
  // Function expects keypoints in undistorted centered frame
  Eigen::Matrix2Xd keypoints_1(2, matches.size());
  Eigen::Matrix2Xd keypoints_2(2, matches.size());
  int i = 0;
  std::vector<cv::DMatch> cv_matches;
  for (const auto& match : matches) {
    Eigen::Vector2d keypoint_1;
    camera_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(match.source_point, &keypoint_1);
    keypoints_1.col(i) = keypoint_1;
    Eigen::Vector2d keypoint_2;
    camera_params.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(match.target_point, &keypoint_2);
    keypoints_2.col(i) = keypoint_2;
    const cv::DMatch cv_match(i, i, match.distance);
    cv_matches.emplace_back(cv_match);
    ++i;
  }

  sm::CIDPairAffineMap relative_affines;
  std::mutex mutex;
  sm::BuildMapFindEssentialAndInliers(keypoints_1, keypoints_2, cv_matches, camera_params, true, 0, 1, &mutex,
                                      &relative_affines, &inlier_matches, false, nullptr);
  std::pair<int, int> pose_indices(0, 1);
  relative_pose = relative_affines[pose_indices];
}

bool RotationOnlyImageSequence(const vc::FeatureMatches& matches, const camera::CameraParameters& camera_params,
                               const double max_rotation_only_mean_error) {
  Eigen::Affine3d relative_pose;
  std::vector<cv::DMatch> inlier_matches;
  RelativePose(matches, camera_params, relative_pose, inlier_matches);
  Eigen::Matrix3d rotation;
  Eigen::Matrix3d scale_matrix;
  relative_pose.computeRotationScaling(&rotation, &scale_matrix);
  const Eigen::Matrix3d intrinsics = camera_params.GetIntrinsicMatrix<camera::DISTORTED>();
  Eigen::Matrix3d rotation_homography = intrinsics * rotation * intrinsics.inverse();
  rotation_homography /= rotation_homography(2, 2);
  lc::Averager error_averager;
  // TODO(rsoussan): Only use inlier matches???
  for (const auto& match : matches) {
    const Eigen::Vector2d rotated_target_point = (rotation_homography * match.target_point.homogeneous()).hnormalized();
    const double error_norm = (match.source_point - rotated_target_point).norm();
    error_averager.Update(error_norm);
  }
  LogDebug("Mean error: " << error_averager.average());
  if (error_averager.average() <= max_rotation_only_mean_error) return true;
  return false;
}

vc::FeatureImage LoadImage(const int index, const std::vector<std::string>& image_names, cv::Feature2D& detector) {
  auto image = cv::imread(image_names[index], cv::IMREAD_GRAYSCALE);
  if (image.empty()) LogFatal("Failed to load image " << image_names[index]);
  cv::resize(image, image, cv::Size(), 0.5, 0.5);
  // TODO(rsoussan): Add option to undistort image, use histogram equalization
  return vc::FeatureImage(image, detector);
}

vc::LKOpticalFlowFeatureDetectorAndMatcherParams LoadParams() {
  vc::LKOpticalFlowFeatureDetectorAndMatcherParams params;
  // TODO(rsoussan): Add config file for these
  params.max_iterations = 10;
  params.termination_epsilon = 0.03;
  params.window_length = 31;
  params.max_level = 3;
  params.min_eigen_threshold = 0.001;
  params.max_flow_distance = 180;
  params.max_backward_match_distance = 0.5;
  params.good_features_to_track.max_corners = 100;
  params.good_features_to_track.quality_level = 0.01;
  params.good_features_to_track.min_distance = 40;
  params.good_features_to_track.block_size = 3;
  params.good_features_to_track.use_harris_detector = false;
  params.good_features_to_track.k = 0.04;
  return params;
}

int RemoveLowMovementAndRotationOnlyImages(const std::vector<std::string>& image_names,
                                           const camera::CameraParameters& camera_params,
                                           const double max_low_movement_mean_distance,
                                           const double max_rotation_only_mean_error) {
  const vc::LKOpticalFlowFeatureDetectorAndMatcherParams params = LoadParams();
  vc::LKOpticalFlowFeatureDetectorAndMatcher detector_and_matcher(params);
  auto& detector = *(detector_and_matcher.detector());
  // Compare current image with subsequent image and remove subsequent image if it has low or rotation only movement.
  // If a subsequent image is removed, check the next image compared with the current image for low or rotation only
  // movement. If a subsequent image does not have low or rotation only movement, advance current image and start the
  // process over.
  int current_image_index = 0;
  int next_image_index = 1;
  auto current_image = LoadImage(current_image_index, image_names, detector);
  auto next_image = LoadImage(next_image_index, image_names, detector);
  int num_removed_images = 0;
  while (current_image_index < image_names.size()) {
    while (next_image_index < image_names.size()) {
      const auto matches = Matches(current_image, next_image, detector_and_matcher);
      if (matches && (LowMovementImageSequence(*matches, max_low_movement_mean_distance) ||
                      RotationOnlyImageSequence(*matches, camera_params, max_rotation_only_mean_error))) {
        LogDebug("Removing image index: " << next_image_index << ", current image index: " << current_image_index);
        std::remove((image_names[next_image_index++]).c_str());
        ++num_removed_images;
        // Don't load next image if index is past the end of the sequence
        if (next_image_index >= image_names.size()) break;
        next_image = LoadImage(next_image_index, image_names, detector);
      } else {
        break;
      }
    }
    current_image = next_image;
    current_image_index = next_image_index;
    // Exit if current image is the last image in the sequence
    if (current_image_index >= image_names.size() - 1) break;
    next_image = LoadImage(++next_image_index, image_names, detector);
  }

  return num_removed_images;
}

std::vector<std::string> GetImageNames(const std::string& image_directory,
                                       const std::string& image_extension = ".jpg") {
  std::vector<std::string> image_names;
  for (const auto& file : fs::recursive_directory_iterator(image_directory)) {
    if (fs::is_regular_file(file) && file.path().extension() == image_extension)
      image_names.emplace_back(fs::absolute(file.path()).string());
  }
  std::sort(image_names.begin(), image_names.end());
  LogInfo("Found " << image_names.size() << " images.");
  return image_names;
}

int main(int argc, char** argv) {
  double max_low_movement_mean_distance;
  double max_rotation_only_mean_error;
  std::string robot_config_file;
  po::options_description desc(
    "Removes any images with too little movement.  Computes relative movement using sequential images.");
  desc.add_options()("help,h", "produce help message")(
    "image-directory", po::value<std::string>()->required(),
    "Directory containing images. Images are assumed to be named in sequential order.")(
    "--max-low-movement-mean-distance,d", po::value<double>(&max_low_movement_mean_distance)->default_value(0.1),
    "Max mean distance in image space for optical flow tracks between sequential images to be classified as a low "
    "movement pair.")("--max-rotation-only-mean-error,e",
                      po::value<double>(&max_rotation_only_mean_error)->default_value(0.1),
                      "Max mean error in image space for features described by rotation only movement between "
                      "sequential images to be classified as a rotation only pair.")(
    "config-path,c", po::value<std::string>()->required(), "Config path")(
    "robot-config-file,r", po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
    "robot config file");
  po::positional_options_description p;
  p.add("image-directory", 1);
  p.add("config-path", 1);
  po::variables_map vm;
  try {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    if (vm.count("help") || (argc <= 1)) {
      std::cout << desc << "\n";
      return 1;
    }
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  const std::string image_directory = vm["image-directory"].as<std::string>();
  const std::string config_path = vm["config-path"].as<std::string>();

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);
  lc::SetEnvironmentConfigs(config_path, "iss", robot_config_file);
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  // TODO(rsoussan): Allow for other cameras?
  const camera::CameraParameters camera_parameters(&config, "nav_cam");

  if (!fs::exists(image_directory) || !fs::is_directory(image_directory)) {
    LogFatal("Image directory " << image_directory << " not found.");
  }

  const auto image_names = GetImageNames(image_directory);
  if (image_names.empty()) LogFatal("No images found.");

  const int num_original_images = image_names.size();
  const int num_removed_images = RemoveLowMovementAndRotationOnlyImages(
    image_names, camera_parameters, max_low_movement_mean_distance, max_rotation_only_mean_error);
  LogInfo("Removed " << num_removed_images << " of " << num_original_images << " images.");
}
