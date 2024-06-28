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

#include <localization_node/localization_nodelet.h>

#include <ff_common/init.h>
#include <sparse_mapping/sparse_map.h>

#include <ros/ros.h>
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/ResetMap.h>
#include <ff_msgs/VisualLandmarks.h>
#include <geometry_msgs/TransformStamped.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <msg_conversions/msg_conversions.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/filesystem.hpp>

namespace localization_node {

LocalizationNodelet::LocalizationNodelet() : ff_util::FreeFlyerNodelet(NODE_MAPPED_LANDMARKS),
        enabled_(false), count_(0) {
  private_nh_.setCallbackQueue(&private_queue_);
  heartbeat_.node = GetName();
  heartbeat_.nodelet_manager = ros::this_node::getName();
  last_heartbeat_time_ = ros::Time::now();
}

bool LocalizationNodelet::ResetMap(const std::string& map_file) {
  if (!boost::filesystem::exists(map_file)) {
    LOG(ERROR) << "Map file " << map_file << " does not exist, failed to reset map.";
    return false;
  }
  map_.reset(new sparse_mapping::SparseMap(map_file, true));
  inst_.reset(new Localizer(map_.get()));
  // Check to see if any params were changed when map was reset
  ReadParams();
  enabled_ = true;
  return true;
}

void LocalizationNodelet::Initialize(ros::NodeHandle* nh) {
  ff_common::InitFreeFlyerApplication(getMyArgv());
  config_.AddFile("cameras.config");
  config_.AddFile("localization.config");
  if (!config_.ReadFiles()) {
    ROS_FATAL("Failed to read config files.");
  }

  // Resolve the full path to the AR tag file specified for the current world
  std::string map_file;
  if (!config_.GetStr("world_vision_map_filename", &map_file))
    ROS_ERROR("Cannot read world_vision_map_filename from LUA config");

  // Reset all internal shared pointers
  it_.reset(new image_transport::ImageTransport(private_nh_));
  map_.reset(new sparse_mapping::SparseMap(map_file, true));
  inst_.reset(new Localizer(map_.get()));

  landmark_publisher_     = nh->advertise<ff_msgs::VisualLandmarks>(
      TOPIC_LOCALIZATION_ML_FEATURES, 10);
  heartbeat_pub_ = nh->advertise<ff_msgs::Heartbeat>(TOPIC_HEARTBEAT, 5, true);

  // Subscribe to input video feed and publish output odometry info
  image_sub_ = it_->subscribe(TOPIC_HARDWARE_NAV_CAM, 1, &LocalizationNodelet::ImageCallback, this);

  matched_features_on_ = false;
  all_features_on_ = false;
  config_.GetBool("matched_features_on", &matched_features_on_);
  config_.GetBool("all_features_on", &all_features_on_);

  if (matched_features_on_) {
    used_features_publisher_ = nh->advertise<sensor_msgs::Image>("rviz/used_features", 10);
  }
  if (all_features_on_) {
    detected_features_publisher_ = nh->advertise<sensor_msgs::Image>("rviz/detected_features", 10);
  }

  ReadParams();
  // only do this once, will cause a crash if done in middle of thread execution
  int num_threads;
  if (!config_.GetInt("num_threads", &num_threads))
    ROS_FATAL("num_threads not specified in localization.");
  cv::setNumThreads(num_threads);

  enable_srv_ = private_nh_.advertiseService(SERVICE_LOCALIZATION_ML_ENABLE, &LocalizationNodelet::EnableService, this);
  reset_map_srv_ =
    private_nh_.advertiseService(SERVICE_LOCALIZATION_RESET_MAP, &LocalizationNodelet::ResetMapService, this);
  reset_map_loc_client_ = private_nh_.serviceClient<ff_msgs::ResetMap>(
                                                SERVICE_LOCALIZATION_RESET_MAP_LOC);
  Run();
}

void LocalizationNodelet::ReadParams(void) {
  if (inst_) inst_->ReadParams(config_);
}

bool LocalizationNodelet::EnableService(ff_msgs::SetBool::Request & req, ff_msgs::SetBool::Response & res) {
  enabled_ = req.enable;
  res.success = true;
  return true;
}

bool LocalizationNodelet::ResetMapService(ff_msgs::ResetMap::Request& req, ff_msgs::ResetMap::Response& res) {
  std::string map_file;
  if (req.map_file == "default") {
    if (!config_.GetStr("world_vision_map_filename", &map_file)) {
      ROS_ERROR("Cannot read world_vision_map_filename from LUA config");
      return false;
    }
  } else {
    map_file = req.map_file;
  }
  LOG(INFO) << "Resetting map to " << map_file;

  res.success = ResetMap(map_file);

  ff_msgs::ResetMap map_srv;
  if (!reset_map_loc_client_.call(map_srv)) {
    res.success = false;
  }
  return true;
}

void LocalizationNodelet::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ros::Time timestamp = ros::Time::now();
  try {
    image_ptr_ = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void LocalizationNodelet::Localize(void) {
  ff_msgs::VisualLandmarks vl;
  Eigen::Matrix2Xd image_keypoints;

  bool success = inst_->Localize(image_ptr_, &vl, &image_keypoints);

  vl.camera_id = count_;
  if (enabled_) landmark_publisher_.publish(vl);

  // only send transform if succeeded
  if (!success)
    return;

  // send rviz feature overlay messages
  sensor_msgs::ImagePtr image_pointer;
  if (matched_features_on_ || all_features_on_) {
    image_pointer = (*image_ptr_).toImageMsg();
  }
  if (matched_features_on_) {
    cv_bridge::CvImagePtr used_image = cv_bridge::toCvCopy(image_pointer);
    for (size_t i = 0; i < vl.landmarks.size(); i++) {
      Eigen::Vector2d undistorted, distorted;
      undistorted[0] = vl.landmarks[i].u;
      undistorted[1] = vl.landmarks[i].v;
      (map_->GetCameraParameters()).Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undistorted, &distorted);
      cv::circle(used_image->image, cv::Point(distorted[0], distorted[1]), 10, CV_RGB(255, 255, 255), 3, 8);
      cv::circle(used_image->image, cv::Point(distorted[0], distorted[1]), 6, CV_RGB(0, 0, 0), 3, 8);
    }
    used_features_publisher_.publish(*used_image);
  }
  if (all_features_on_) {
    cv_bridge::CvImagePtr detected_image = cv_bridge::toCvCopy(image_pointer);
    for (int i = 0; i < image_keypoints.cols(); i++) {
      Eigen::Vector2d undistorted, distorted;
      undistorted[0] = image_keypoints.col(i)[0];
      undistorted[1] = image_keypoints.col(i)[1];
      (map_->GetCameraParameters()).Convert<camera::UNDISTORTED_C, camera::DISTORTED>(undistorted, &distorted);
      cv::circle(detected_image->image, cv::Point(distorted[0], distorted[1]), 10, CV_RGB(255, 255, 255), 3, 8);
      cv::circle(detected_image->image, cv::Point(distorted[0], distorted[1]), 6, CV_RGB(0, 0, 0), 2, 8);
    }
    detected_features_publisher_.publish(*detected_image);
  }

  // now publish the transform
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.seq = count_;
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "localization";
  transformStamped.transform.translation.x = vl.pose.position.x;
  transformStamped.transform.translation.y = vl.pose.position.y;
  transformStamped.transform.translation.z = vl.pose.position.z;
  transformStamped.transform.rotation = vl.pose.orientation;

  br.sendTransform(transformStamped);
}

void LocalizationNodelet::PublishHeartbeat() {
  heartbeat_.header.stamp = ros::Time::now();
  if ((heartbeat_.header.stamp - last_heartbeat_time_).toSec() < 1.0) return;
  heartbeat_pub_.publish(heartbeat_);
  last_heartbeat_time_ = heartbeat_.header.stamp;
}

void LocalizationNodelet::Run() {
  ros::Rate rate(100);
  while (ros::ok()) {
    private_queue_.callAvailable();
    if (enabled_) {
        Localize();
       count_++;
    }
    PublishHeartbeat();
    rate.sleep();
  }
}
};  // namespace localization_node

PLUGINLIB_EXPORT_CLASS(localization_node::LocalizationNodelet, nodelet::Nodelet)
