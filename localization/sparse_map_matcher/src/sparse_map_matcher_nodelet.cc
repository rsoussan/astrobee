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

#include <ff_msgs/VisualLandmarks.h>
#include <sparse_map_matcher/sparse_map_matcher_nodelet.h>

#include <pluginlib/class_list_macros.h>

namespace sparse_map_matcher {
SparseMapMatcherNodelet::SparseMapMatcherNodelet()
    : ff_util::FreeFlyerNodelet(NODE_MAPPED_LANDMARKS), enabled_(false) {}

void SparseMapMatcherNodelet::Initialize(ros::NodeHandle* nh) { SubscribeAndAdvertise(nh); }

void SparseMapMatcherNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  image_transport::ImageTransport image_transport(*nh);
  image_sub_ = image_transport.subscribe(TOPIC_HARDWARE_NAV_CAM, 1, &SparseMapMatcherNodelet::ImageCallback, this);
  vl_pub_ = nh->advertise<ff_msgs::VisualLandmarks>(TOPIC_LOCALIZATION_ML_FEATURES, 10);
  enable_srv_ = nh->advertiseService(SERVICE_LOCALIZATION_ML_ENABLE, &SparseMapMatcherNodelet::EnableService, this);
}

bool SparseMapMatcherNodelet::EnableService(ff_msgs::SetBool::Request& req, ff_msgs::SetBool::Response& res) {
  enabled_ = req.enable;
  res.success = true;
  return true;
}

void SparseMapMatcherNodelet::ImageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
  if (!enabled_) return;
  const auto vl_msg = map_matcher_wrapper_.ImageCallback(image_msg);
  if (vl_msg) vl_pub_.publish(*vl_msg);
}
}  // namespace sparse_map_matcher

PLUGINLIB_EXPORT_CLASS(sparse_map_matcher::SparseMapMatcherNodelet, nodelet::Nodelet)
