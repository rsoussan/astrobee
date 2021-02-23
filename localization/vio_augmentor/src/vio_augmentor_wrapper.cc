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
#include <camera/camera_model.h>
#include <localization_common/utilities.h>
#include <vio_augmentor/vio_augmentor_wrapper.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <msg_conversions/msg_conversions.h>
#include <ros/package.h>

#include <ff_util/ff_names.h>

namespace vio_augmentor {
namespace lc = localization_common;

VIOAugmentorWrapper::VIOAugmentorWrapper(ros::NodeHandle* nh)
    : ekf_initialized_(false), imus_dropped_(0), have_imu_(false), killed_(false) {
  config_.AddFile("gnc.config");
  config_.AddFile("cameras.config");
  config_.AddFile("geometry.config");
  ReadParams();
  // TODO(rsoussan): Does this still need to be here?
  config_timer_ = nh->createTimer(
    ros::Duration(1),
    [this](ros::TimerEvent e) { config_.CheckFilesUpdated(std::bind(&VIOAugmentorWrapper::ReadParams, this)); }, false,
    true);
  pt_ekf_.Initialize("ekf");
}

VIOAugmentorWrapper::~VIOAugmentorWrapper() { killed_ = true; }

// wait to start up until the IMU is ready
void VIOAugmentorWrapper::InitializeEkf() { ekf_initialized_ = true; }

void VIOAugmentorWrapper::ReadParams() {
  if (!config_.ReadFiles()) {
    ROS_ERROR("Failed to read config files.");
    return;
  }
  ekf_.ReadParams(&config_);
}

void VIOAugmentorWrapper::LocalizationStateCallback(const ff_msgs::GraphState& loc_msg) {
  std::lock_guard<std::mutex> lock(mutex_loc_msg_);
  latest_combined_nav_state_ = lc::CombinedNavStateFromMsg(loc_msg);
  latest_covariances_ = lc::CombinedNavStateCovariancesFromMsg(loc_msg);
  latest_loc_msg_ = loc_msg;
}

void VIOAugmentorWrapper::ImuCallBack(const sensor_msgs::Imu& imu) {
  // concurrency protection
  std::unique_lock<std::mutex> lock(mutex_imu_msg_);
  while (have_imu_ && !killed_) cv_imu_.wait_for(lock, std::chrono::milliseconds(8));
  // copy IMU data
  imu_ = imu;
  have_imu_ = true;
  // now notify the condition variable waiting for IMU that we have it
  lock.unlock();
  cv_imu_.notify_all();
}

void VIOAugmentorWrapper::OpticalFlowCallBack(const ff_msgs::Feature2dArray& of) {
  std::lock_guard<std::mutex> lock(mutex_of_msg_);
  ekf_.OpticalFlowUpdate(of);
}

void VIOAugmentorWrapper::RegisterOpticalFlowCamera(const ff_msgs::CameraRegistration& cr) {
  std::lock_guard<std::mutex> lock(mutex_of_msg_);
  ekf_.OpticalFlowRegister(cr);
}

void VIOAugmentorWrapper::FlightModeCallback(const ff_msgs::FlightMode& mode) { ekf_.SetSpeedGain(mode.speed); }

void VIOAugmentorWrapper::Run(std::atomic<bool> const& killed) {
  // Kill the step thread
  while (!killed) {
    ros::spinOnce();
    Step();
  }
  // Make sure the IMU thread also gets killed
  killed_ = true;
}

void VIOAugmentorWrapper::Step() {
  // don't modify anything while we're copying the data
  {
    // wait until we get an imu reading with the condition variable
    std::unique_lock<std::mutex> lk(mutex_imu_msg_);
    if (!have_imu_) {
      cv_imu_.wait_for(lk, std::chrono::milliseconds(8));
      imus_dropped_++;
      if (imus_dropped_ > 10 && ekf_initialized_) {
        ekf_.Reset();
      }
      return;  // Changed by Andrew due to 250Hz ctl messages when sim blocks (!)
    }
    imus_dropped_ = 0;
    if (!ekf_initialized_) InitializeEkf();

    std::lock_guard<std::mutex> lock(mutex_of_msg_);
    // copy everything in EKF, so data structures can be modified for next
    // step while current step processes. We pass the ground truth quaternion
    // representing the latest ISS2BODY rotation, which is used in certain
    // testing contexts to remove the effect of Earth's gravity.
    ekf_.PrepareStep(imu_, quat_);
    // don't reuse imu reading
    have_imu_ = false;
  }
  cv_imu_.notify_all();

  // All other pipelines get stepped forward normally
  pt_ekf_.Tick();
  vio_augmentor_.AddState(ekf_.Step());
  pt_ekf_.Tock();
  PublishState();
}

void VIOAugmentorWrapper::PublishState() {
  std::lock_guard<std::mutex> lock(mutex_loc_msg_);
  if (!latest_combined_nav_state_ || !latest_loc_msg_) return;
  vio_augmentor_.RemoveOldPoses(latest_combined_nav_state_->timestamp());
  const auto extrapolated_pose = vio_augmentor_.ExtrapolatePose(latest_combined_nav_state_->timestamp(),
                                                                lc::EigenPose(latest_combined_nav_state_->pose()));
  // TODO(rsoussan): publish stuff
}

bool VIOAugmentorWrapper::ResetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ekf_.Reset();
  return true;
}
}  // end namespace vio_augmentor
