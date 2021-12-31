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

#include <depth_odometry/fpfh_features_with_known_correspondences_aligner_depth_odometry.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <point_cloud_common/correspondences_3d.h>
#include <point_cloud_common/utilities.h>

#include <pcl/registration/correspondence_estimation.h>

namespace depth_odometry {
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace pc = point_cloud_common;

FPFHFeaturesWithKnownCorrespondencesAlignerDepthOdometry::FPFHFeaturesWithKnownCorrespondencesAlignerDepthOdometry(
  const FPFHFeaturesWithKnownCorrespondencesAlignerDepthOdometryParams& params)
    : params_(params), aligner_(params.aligner) {}

boost::optional<PoseWithCovarianceAndCorrespondences>
FPFHFeaturesWithKnownCorrespondencesAlignerDepthOdometry::DepthImageCallback(
  const lm::DepthImageMeasurement& depth_image_measurement) {
  if (!previous_fpfh_features_ && !latest_fpfh_features_) {
    latest_point_cloud_with_normals_ = pc::DownsampledFilteredCloudWithNormals<pcl::PointXYZI, pcl::PointXYZINormal>(
      depth_image_measurement.depth_image.unfiltered_point_cloud(), params_.search_radius, params_.downsample,
      params_.downsample_leaf_size);
    latest_fpfh_features_ = pc::EstimateHistogramFeatures(latest_point_cloud_with_normals_);
    latest_timestamp_ = depth_image_measurement.timestamp;
    return boost::none;
  }
  const lc::Time timestamp = depth_image_measurement.timestamp;
  if (timestamp < latest_timestamp_) {
    LogWarning("DepthImageCallback: Out of order measurement received.");
    return boost::none;
  }

  previous_point_cloud_with_normals_ = latest_point_cloud_with_normals_;
  previous_fpfh_features_ = latest_fpfh_features_;
  previous_timestamp_ = latest_timestamp_;
  latest_point_cloud_with_normals_ = pc::DownsampledFilteredCloudWithNormals<pcl::PointXYZI, pcl::PointXYZINormal>(
    depth_image_measurement.depth_image.unfiltered_point_cloud(), params_.search_radius, params_.downsample,
    params_.downsample_leaf_size);
  latest_fpfh_features_ = pc::EstimateHistogramFeatures(latest_point_cloud_with_normals_);
  latest_timestamp_ = depth_image_measurement.timestamp;

  const double time_diff = latest_timestamp_ - previous_timestamp_;
  if (time_diff > params_.max_time_diff) {
    LogWarning("DepthImageCallback: Time difference too large, time diff: " << time_diff);
    return boost::none;
  }

  pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> correspondence_estimator;
  pcl::Correspondences pcl_correspondences;
  correspondence_estimator.setInputSource(previous_fpfh_features_);
  correspondence_estimator.setInputTarget(latest_fpfh_features_);
  correspondence_estimator.determineCorrespondences(pcl_correspondences);
  pc::Correspondences3d correspondences(pcl_correspondences, *previous_point_cloud_with_normals_,
                                        *latest_point_cloud_with_normals_);
  LogDebug("DepthImageCallback: Correspondences: " << correspondences.size());
  if (correspondences.target_points.size() < 4) {
    LogError("DepthImageCallback: Too few points provided, need 4 but given " << correspondences.target_points.size()
                                                                              << ".");
    return boost::none;
  }

  const auto target_T_source =
    aligner_.ComputeRelativeTransform(correspondences.source_points, correspondences.target_points);
  if (!target_T_source) {
    LogWarning("DepthImageCallback: Failed to get relative transform.");
    return boost::none;
  }

  const auto source_T_target = lc::InvertPoseWithCovariance(*target_T_source);

  if (!lc::PoseCovarianceSane(source_T_target.covariance, params_.position_covariance_threshold,
                              params_.orientation_covariance_threshold)) {
    LogWarning("DepthImageCallback: Sanity check failed - invalid covariance.");
    return boost::none;
  }

  return PoseWithCovarianceAndCorrespondences(
    source_T_target, lm::DepthCorrespondences(correspondences.source_points, correspondences.target_points),
    previous_timestamp_, latest_timestamp_);
}
}  // namespace depth_odometry
