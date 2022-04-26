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

#include <localization_common/logger.h>
#include <localization_measurements/measurement_conversions.h>
#include <sparse_map_matcher/sparse_map_matcher_wrapper.h>

namespace sparse_map_matcher {
namespace lc = localization_common;
namespace lm = localization_measurements;

SparseMapMatcherWrapper::SparseMapMatcherWrapper() : vl_msg_count_(0) {
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("sparse_map_matcher.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  SparseMapMatcherParams params;
  LoadSparseMapMatcherParams(config, params);
  cv::setNumThreads(params.num_cv_threads);
  matcher_.reset(std::make_unique<SparseMapMatcher>(params));
}

ff_msgs::VisualLandmarks SparseMapMatcherWrapper::VlMsg(const Eigen::Isometry3d& world_T_camera,
                                                        const lc::Time& timestamp,
                                                        const std::vector<Eigen::Vector2d>& observations,
                                                        const std::vector<Eigen::Vector3d>& landmarks) const {
  ff_msgs::VisualLandmarks vl_msg;
  lc::TimeToHeader(timestamp, vl_msg.header);
  vl_msg.header.frame_id = "world";
  vl.camera_id = vl_msg_count_++;

  mc::EigenPoseToMsg(world_T_camera, vl_msg.pose);
  vl_msg.landmarks.reserve(landmarks.size());
  for (int i = 0; i < static_cast<int>(landmarks.size()); ++i) {
    ff_msgs::VisualLandmark l;
    l.x = landmarks[i].x();
    l.y = landmarks[i].y();
    l.z = landmarks[i].z();
    l.u = observations[i].x();
    l.v = observations[i].y();
    vl_msg.landmarks.push_back(l);
  }
  return vl_msg;
}

boost::optional<ff_msgs::VisualLandmarks> SparseMapMatcherWrapper::ImageCallback(
  const sensor_msgs::ImageConstPtr& image_msg) {
  const auto image_measurement = lm::MakeImageMeasurement(image_msg);
  const auto pose_estimate = matcher_.ImageCallback(image_measurement.image);
  if (!pose_estimate) return boost::none;
  const Eigen::Isometry3d world_T_camera(pose_estimate->pose.GetTransform().matrix());
  return VlMsg(world_T_camera, image_measurement.timestamp, pose_estimate->inlier_observations,
               pose_estimate->inlier_landmarks);
}
}  // namespace sparse_map_matcher
