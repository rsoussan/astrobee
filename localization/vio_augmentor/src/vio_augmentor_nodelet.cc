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

#include <vio_augmentor/vio_augmentor_nodelet.h>

#include <ff_common/init.h>
#include <ff_util/ff_nodelet.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <gnc_autocode/autocode.h>

#include <thread>
#include <atomic>
#include <memory>

namespace vio_augmentor {

VIOAugmentorNodelet::VIOAugmentorNodelet()
    : ff_util::FreeFlyerNodelet(NODE_VIO_AUG, true),
      vio_augmentor_wrapper_(this->GetPlatformHandle(true)),
      killed_(false),
      nh_(this->GetPlatformHandle(true)),
      platform_name_(GetPlatform()) {}

VIOAugmentorNodelet::~VIOAugmentorNodelet() {
  killed_ = true;
  thread_->join();
}

// This is called when the nodelet is loaded into the nodelet manager
void VIOAugmentorNodelet::Initialize(ros::NodeHandle* nh) {
  // Bootstrap our environment
  ff_common::InitFreeFlyerApplication(getMyArgv());
  gnc_autocode::InitializeAutocode(this);
  imu_sub_ =
    nh_->subscribe(TOPIC_HARDWARE_IMU, 5, &VIOAugmentorNodelet::ImuCallBack, this, ros::TransportHints().tcpNoDelay());
  flight_mode_sub_ = nh_->subscribe(TOPIC_MOBILITY_FLIGHT_MODE, 1, &VIOAugmentorNodelet::FlightModeCallback, this);
  state_pub_ = nh_->advertise<ff_msgs::EkfState>(TOPIC_GNC_EKF, 1);
  pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(TOPIC_LOCALIZATION_POSE, 1);
  twist_pub_ = nh_->advertise<geometry_msgs::TwistStamped>(TOPIC_LOCALIZATION_TWIST, 1);
  reset_pub_ = nh_->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 1);

  of_sub_ = nh_->subscribe(TOPIC_LOCALIZATION_OF_FEATURES, 1, &VIOAugmentorNodelet::OpticalFlowCallBack, this,
                           ros::TransportHints().tcpNoDelay());
  of_reg_sub_ = nh_->subscribe(TOPIC_LOCALIZATION_OF_REGISTRATION, 1, &VIOAugmentorNodelet::RegisterOpticalFlowCamera,
                               this, ros::TransportHints().tcpNoDelay());
  state_sub_ = nh_->subscribe(TOPIC_GRAPH_LOC_STATE, 1, &VIOAugmentorNodelet::LocalizationStateCallback, this,
                              ros::TransportHints().tcpNoDelay());
  reset_srv_ = nh_->advertiseService(SERVICE_GNC_EKF_RESET, &VIOAugmentorNodelet::ResetService, this);

  thread_.reset(new std::thread(&vio_augmentor::VIOAugmentorWrapper::Run, &vio_augmentor_wrapper_, std::ref(killed_)));
}

bool VIOAugmentorNodelet::ResetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  return vio_augmentor_wrapper_.ResetService(req, res);
}

void VIOAugmentorNodelet::LocalizationStateCallback(const ff_msgs::GraphState::ConstPtr& loc_msg) {
  vio_augmentor_wrapper_.LocalizationStateCallback(*loc_msg);
}

void VIOAugmentorNodelet::ImuCallBack(const sensor_msgs::Imu::ConstPtr& imu) {
  vio_augmentor_wrapper_.ImuCallBack(*imu);
  PublishState();
}

void VIOAugmentorNodelet::OpticalFlowCallBack(const ff_msgs::Feature2dArray::ConstPtr& of) {
  vio_augmentor_wrapper_.OpticalFlowCallBack(*of);
}

void VIOAugmentorNodelet::RegisterOpticalFlowCamera(const ff_msgs::CameraRegistration::ConstPtr& cr) {
  vio_augmentor_wrapper_.RegisterOpticalFlowCamera(*cr);
}

void VIOAugmentorNodelet::FlightModeCallback(const ff_msgs::FlightMode::ConstPtr& mode) {
  vio_augmentor_wrapper_.FlightModeCallback(*mode);
}

void VIOAugmentorNodelet::PublishState() {
  std::function<void(const ff_msgs::EkfState&)> f =
    std::bind(&VIOAugmentorNodelet::PublishStateHelper, this, std::placeholders::_1);
  vio_augmentor_wrapper_.LatestVIOAugmentedLocMsgVisitor(f);
}

void VIOAugmentorNodelet::PublishStateHelper(const ff_msgs::EkfState& latest_vio_augmented_loc_msg) {
  state_pub_.publish(latest_vio_augmented_loc_msg);
  // Publish pose
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = latest_vio_augmented_loc_msg.header;
  pose_msg.pose = latest_vio_augmented_loc_msg.pose;
  pose_pub_.publish(pose_msg);

  // Publish twist
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header = latest_vio_augmented_loc_msg.header;
  twist_msg.twist.linear = latest_vio_augmented_loc_msg.velocity;
  twist_msg.twist.angular = latest_vio_augmented_loc_msg.omega;
  twist_pub_.publish(twist_msg);

  // Publish TF
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header = latest_vio_augmented_loc_msg.header;
  transform_msg.child_frame_id = platform_name_ + "body";
  transform_msg.transform.translation.x = latest_vio_augmented_loc_msg.pose.position.x;
  transform_msg.transform.translation.y = latest_vio_augmented_loc_msg.pose.position.y;
  transform_msg.transform.translation.z = latest_vio_augmented_loc_msg.pose.position.z;
  transform_msg.transform.rotation = latest_vio_augmented_loc_msg.pose.orientation;
  transform_pub_.sendTransform(transform_msg);
}
}  // namespace vio_augmentor

PLUGINLIB_EXPORT_CLASS(vio_augmentor::VIOAugmentorNodelet, nodelet::Nodelet);
