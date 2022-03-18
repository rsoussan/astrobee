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

#include <sparse_map_matcher/sparse_map_matcher.h>
#include <sparse_mapping/estimate_pose_utilities.h>
#include <sparse_mapping/sparse_map_utilities.h>

namespace sparse_map_matcher {
namespace mc = msg_conversions;
namespace sm = sparse_mapping;
namespace vc = vision_common;

SparseMapMatcher::SparseMapMatcher(const SparseMapMatcherParams& params, std::shared_ptr<sparse_mapping::SparseMap> map)
    : params_(params), map_(std::move(map)), detector_(params.detector) {
  sm::HistogramEqualizationCheck(map_->GetHistogramEqualization(),
                                             params_.histogram_equalization);
}

ff_msgs::VisualLandmarks SparseMapMatcher::VlMsg(const Eigen::Isometry3d& world_T_camera, const ros::Time& timestamp,
                                                 const std::vector<Eigen::Vector2d>& observations,
                                                 const std::vector<Eigen::Vector3d>& landmarks) {
  ff_msgs::VisualLandmarks vl_msg;
  vl_msg.header = std_msgs::Header();
  vl_msg.header.stamp = timestamp;
  vl_msg.header.frame_id = "world";

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

boost::optional<ff_msgs::VisualLandmarks>  SparseMapMatcher::Match(const cv::Mat& image, const ros::Time& timestamp) {
  cv::Mat descriptors;
  Eigen::Matrix2Xd keypoints;
  sm::DetectFeatures(image, params_.histogram_equalization, detector_, &descriptors, &keypoints);
  camera::CameraModel camera(Eigen::Vector3d(),
                             Eigen::Matrix3d::Identity(),
                             map_->GetCameraParameters());
  std::vector<Eigen::Vector3d> landmarks;
  std::vector<Eigen::Vector2d> observations;
  const auto pose_estimate = sm::EstimatePose(descriptors, keypoints, *map_, params_.estimate_pose);
  if (!pose_estimate.pose) return boost::none;
  const Eigen::Isometry3d world_T_camera(pose_estimate.pose->GetTransform().matrix());
  *vl = VlMsg(world_T_camera, *(pose_estimate->inlier_observations), *(pose_estimate->inlier_landmarks));
  return true;
}
}  // namespace sparse_map_matcher
