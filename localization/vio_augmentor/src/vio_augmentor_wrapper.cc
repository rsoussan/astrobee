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

#include <vio_augmentor/vio_augmentor_wrapper.h>
#include <camera/camera_params.h>
#include <camera/camera_model.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <msg_conversions/msg_conversions.h>
#include <ros/package.h>

#include <ff_util/ff_names.h>

namespace vio_augmentor {

VIOAugmentorWrapper::VIOAugmentorWrapper(ros::NodeHandle* nh, std::string const& platform_name)
    : ekf_initialized_(false), imus_dropped_(0), have_imu_(false), nh_(nh), killed_(false) {
  platform_name_ = (platform_name.empty() ? "" : platform_name + "/");

  config_.AddFile("gnc.config");
  config_.AddFile("cameras.config");
  config_.AddFile("geometry.config");
  ReadParams();
  config_timer_ = nh->createTimer(
    ros::Duration(1),
    [this](ros::TimerEvent e) { config_.CheckFilesUpdated(std::bind(&VIOAugmentorWrapper::ReadParams, this)); }, false,
    true);
  pt_ekf_.Initialize("ekf");

  // Register to receive a callback when the EKF resets
  vio_augmentor_.SetResetCallback(std::bind(&VIOAugmentorWrapper::ResetCallback, this));
  // subscribe to IMU first, then rest once IMU is ready
  // this is so localization manager doesn't timeout
  imu_sub_ =
    nh_->subscribe(TOPIC_HARDWARE_IMU, 5, &VIOAugmentorWrapper::ImuCallBack, this, ros::TransportHints().tcpNoDelay());
}

VIOAugmentorWrapper::~VIOAugmentorWrapper() { killed_ = true; }

// wait to start up until the IMU is ready
void VIOAugmentorWrapper::InitializeEkf(void) {
  state_pub_ = nh_->advertise<ff_msgs::EkfState>(TOPIC_GNC_EKF, 1);
  pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(TOPIC_LOCALIZATION_POSE, 1);
  twist_pub_ = nh_->advertise<geometry_msgs::TwistStamped>(TOPIC_LOCALIZATION_TWIST, 1);
  reset_pub_ = nh_->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 1);

  of_sub_ = nh_->subscribe(TOPIC_LOCALIZATION_OF_FEATURES, 1, &VIOAugmentorWrapper::OpticalFlowCallBack, this,
                           ros::TransportHints().tcpNoDelay());
  of_reg_sub_ = nh_->subscribe(TOPIC_LOCALIZATION_OF_REGISTRATION, 1, &VIOAugmentorWrapper::RegisterOpticalFlowCamera,
                               this, ros::TransportHints().tcpNoDelay());
  reset_srv_ = nh_->advertiseService(SERVICE_GNC_EKF_RESET, &VIOAugmentorWrapper::ResetService, this);

  ekf_initialized_ = true;
}

void VIOAugmentorWrapper::ReadParams(void) {
  if (!config_.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }
  vio_augmentor_.ReadParams(&config_);
}

void VIOAugmentorWrapper::ResetCallback() {
  static std_msgs::Empty msg;
  reset_pub_.publish(msg);
}

void VIOAugmentorWrapper::ImuCallBack(sensor_msgs::Imu::ConstPtr const& imu) {
  // concurrency protection
  std::unique_lock<std::mutex> lock(mutex_imu_msg_);
  while (have_imu_ && !killed_) cv_imu_.wait_for(lock, std::chrono::milliseconds(8));

  // copy IMU data
  imu_ = *imu;

  have_imu_ = true;
  // now notify the condition variable waiting for IMU that we have it
  lock.unlock();
  cv_imu_.notify_all();
}

void VIOAugmentorWrapper::OpticalFlowCallBack(ff_msgs::Feature2dArray::ConstPtr const& of) {
  std::lock_guard<std::mutex> lock(mutex_of_msg_);
  vio_augmentor_.OpticalFlowUpdate(*of.get());
}

void VIOAugmentorWrapper::RegisterOpticalFlowCamera(ff_msgs::CameraRegistration::ConstPtr const& cr) {
  std::lock_guard<std::mutex> lock(mutex_of_msg_);
  vio_augmentor_.OpticalFlowRegister(*cr.get());
}

void VIOAugmentorWrapper::Run(std::atomic<bool> const& killed) {
  // Kill the step thread
  while (!killed) {
    ros::spinOnce();
    Step();
  }
  // Make sure the IMU thread also gets killed
  killed_ = true;
}

int VIOAugmentorWrapper::Step() {
  // don't modify anything while we're copying the data
  {
    // wait until we get an imu reading with the condition variable
    std::unique_lock<std::mutex> lk(mutex_imu_msg_);
    if (!have_imu_) {
      cv_imu_.wait_for(lk, std::chrono::milliseconds(8));
      imus_dropped_++;
      if (imus_dropped_ > 10 && ekf_initialized_) {
        vio_augmentor_.Reset();
      }
      return 0;  // Changed by Andrew due to 250Hz ctl messages when sim blocks (!)
    }
    imus_dropped_ = 0;
    if (!ekf_initialized_) InitializeEkf();

    std::lock_guard<std::mutex> lock(mutex_of_msg_);
    // copy everything in EKF, so data structures can be modified for next
    // step while current step processes. We pass the ground truth quaternion
    // representing the latest ISS2BODY rotation, which is used in certain
    // testing contexts to remove the effect of Earth's gravity.
    vio_augmentor_.PrepareStep(imu_, quat_);
    // don't reuse imu reading
    have_imu_ = false;
  }
  cv_imu_.notify_all();

  // All other pipelines get stepped forward normally
  pt_ekf_.Tick();
  const int ret = vio_augmentor_.Step(&state_);
  pt_ekf_.Tock();
  // Only send the state if the Step() function was successful
  if (ret) PublishState(state_);
  return ret;
}  // namespace vio_augmentor

void VIOAugmentorWrapper::PublishState(const ff_msgs::EkfState& state) { state_pub_.publish<ff_msgs::EkfState>(state); }

bool VIOAugmentorWrapper::ResetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {  // NOLINT
  vio_augmentor_.Reset();
  return true;
}
}  // end namespace vio_augmentor
