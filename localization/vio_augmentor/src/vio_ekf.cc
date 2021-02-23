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
#include <vio_augmentor/vio_ekf.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ff_common/init.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/SetEkfInput.h>
#include <gflags/gflags.h>
#include <msg_conversions/msg_conversions.h>
#include <ros/package.h>

namespace vio_augmentor {
VIOEkf::VIOEkf()
    : reset_ekf_(true),
      reset_ready_(false),
      reset_callback_(nullptr),
      processing_of_reg_(false),
      of_inputs_delayed_(false),
      of_camera_id_(0) {
  gnc_.cmc_.speed_gain_cmd = 1;  // prevent from being invalid when running bags
  of_history_size_ = ASE_OF_NUM_AUG;
  of_max_features_ = ASE_OF_NUM_FEATURES;
  memset(&reg_, 0, sizeof(cvs_registration_pulse));
  memset(&of_, 0, sizeof(cvs_optical_flow_msg));
  memset(&imu_, 0, sizeof(imu_msg));
  ResetPose();
}

void VIOEkf::ReadParams(config_reader::ConfigReader* config) {
  gnc_.ReadParams(config);
  if (!config->GetInt("min_of_observations", &min_of_observations_)) ROS_FATAL("Unspecified min_of_observations.");
  // get camera transform
  Eigen::Vector3d trans;
  Eigen::Quaterniond rot;
  if (!msg_conversions::config_read_transform(config, "nav_cam_transform", &trans, &rot))
    ROS_FATAL("Unspecified nav_cam_transform.");
  nav_cam_to_body_ = Eigen::Affine3d(Eigen::Translation3d(trans.x(), trans.y(), trans.z())) * Eigen::Affine3d(rot);
  if (!msg_conversions::config_read_transform(config, "imu_transform", &trans, &rot))
    ROS_FATAL("Unspecified imu_transform.");
  imu_to_body_ = Eigen::Affine3d(Eigen::Translation3d(trans.x(), trans.y(), trans.z())) * Eigen::Affine3d(rot);
}

void VIOEkf::SetBias(Eigen::Vector3f gyro_bias, Eigen::Vector3f accel_bias) {
  Eigen::Map<Eigen::Vector3f> gyro_fixed_bias(gnc_.est_->defaultParam->ase_gyro_fixed_bias),
    accel_fixed_bias(gnc_.est_->defaultParam->ase_accel_fixed_bias);
  gyro_fixed_bias = gyro_bias;
  accel_fixed_bias = accel_bias;
}

void VIOEkf::OpticalFlowUpdate(const ff_msgs::Feature2dArray& of) {
  // check that the camera id matches our registration
  if (of_camera_id_ != of.camera_id) {
    // ROS_DEBUG_THROTTLE(1, "Registered optical flow camera not found.");
    return;
  }

  // put new features in tree to make it faster
  std::map<int, OFObservation> new_features;
  for (size_t i = 0; i < of.feature_array.size(); i++) {
    new_features.insert(
      std::pair<int, OFObservation>(of.feature_array[i].id, {of.feature_array[i].x, of.feature_array[i].y}));
  }

  // delete all observations that we didn't see
  std::map<int, OFFeature>::iterator of_iter = optical_flow_features_.begin();
  while (of_iter != optical_flow_features_.end()) {
    if (new_features.count((*of_iter).first) == 0) {
      OFFeature& f = (*of_iter).second;
      f.missing_frames++;
    }
    of_iter++;
  }

  // now, add new features to the list of existing features
  for (size_t i = 0; i < of.feature_array.size(); i++) {
    uint16_t feature_id = of.feature_array[i].id;
    float x = of.feature_array[i].x;
    float y = of.feature_array[i].y;
    // add new feature to list
    if (optical_flow_features_.count(feature_id) == 0) {
      optical_flow_features_.insert(
        std::pair<int, OFFeature>(feature_id, {feature_id, std::vector<OFObservation>(), 0}));
    }
    OFFeature& f = optical_flow_features_[feature_id];
    // add the new observations
    f.obs.insert(f.obs.begin(), {x, y});
    optical_flow_augs_feature_counts_[0]++;
  }
  of_camera_id_ = 0;

  if (deleting_augs_.size() > 0) return;
  if (optical_flow_augs_feature_counts_.size() >= of_history_size_ &&
      optical_flow_augs_feature_counts_[of_history_size_ - 1] > 3) {
    deleting_augs_.push_back(0);
    if (optical_flow_augs_feature_counts_[of_history_size_ - 3] < 10) {
      deleting_augs_.push_back(of_history_size_ - 3);
      deleting_augs_.push_back(of_history_size_ - 2);
      deleting_augs_.push_back(of_history_size_ - 1);
    } else {
      deleting_augs_.push_back(2);
      if (optical_flow_augs_feature_counts_[of_history_size_ - 2] < 10) {
        deleting_augs_.push_back(of_history_size_ - 2);
        deleting_augs_.push_back(of_history_size_ - 1);
      } else {
        deleting_augs_.push_back(6);
        if (optical_flow_augs_feature_counts_[of_history_size_ - 1] < 10) {
          deleting_augs_.push_back(of_history_size_ - 1);
        } else {
          ros::Time now = of.header.stamp;
          double age_1 = (now - optical_flow_augs_times_[of_history_size_ - 1]).toSec();
          double age_2 = (now - optical_flow_augs_times_[of_history_size_ - 2]).toSec();
          double age_3 = (now - optical_flow_augs_times_[of_history_size_ - 3]).toSec();
          double age_4 = (now - optical_flow_augs_times_[of_history_size_ - 4]).toSec();
          if (age_1 < 2 * age_2)
            deleting_augs_.push_back(of_history_size_ - 2);
          else if (age_2 < 2 * age_3)
            deleting_augs_.push_back(of_history_size_ - 3);
          else if (age_3 < 2 * age_4)
            deleting_augs_.push_back(of_history_size_ - 4);
          else
            deleting_augs_.push_back(of_history_size_ - 5);
        }
      }
    }
  } else {
    return;
  }

  // clear all valid flags
  unsigned int index = 0;
  memset(of_.cvs_valid_flag, 0, of_history_size_ * of_max_features_);
  // we have to wait until the next timestep
  if (!processing_of_reg_) {
    of_.cvs_timestamp_sec = of.header.stamp.sec;
    of_.cvs_timestamp_nsec = of.header.stamp.nsec;
  } else {
    of_inputs_delayed_ = true;
  }
  std::map<int, OFFeature>::iterator iter = optical_flow_features_.begin();
  while (iter != optical_flow_features_.end()) {
    OFFeature& f = (*iter).second;
    if (f.missing_frames > 0) {
      // We are no longer using these observations because we can greatly speed up
      // computation in the EKF with a sparse block H matrix
      for (unsigned int i = 0; i < f.obs.size(); i++) {
        unsigned int aug = i + f.missing_frames;
        if (aug >= of_history_size_) break;
        // of_.cvs_observations[aug * of_max_features_ * 2 + index] = f.obs[i].x;
        // of_.cvs_observations[aug * of_max_features_ * 2 + index + of_max_features_] = f.obs[i].y;
        // of_.cvs_valid_flag[aug * of_max_features_ + index] = 1;
        if (optical_flow_augs_feature_counts_.size() > aug) optical_flow_augs_feature_counts_[aug]--;
      }
      iter = optical_flow_features_.erase(iter);
    } else {
      // only use oldest features, and choose three oldest frames and newest
      if (f.obs.size() >= of_history_size_) {
        of_.cvs_observations[0 * of_max_features_ * 2 + index] = f.obs[0].x;
        of_.cvs_observations[0 * of_max_features_ * 2 + index + of_max_features_] = f.obs[0].y;
        of_.cvs_valid_flag[0 * of_max_features_ + index] = 1;
        for (unsigned int aug = of_history_size_ - 3; aug < of_history_size_; aug++) {
          of_.cvs_observations[aug * of_max_features_ * 2 + index] = f.obs[aug].x;
          of_.cvs_observations[aug * of_max_features_ * 2 + index + of_max_features_] = f.obs[aug].y;
          of_.cvs_valid_flag[aug * of_max_features_ + index] = 1;
        }
      }
      iter++;
      index++;
    }
    if (index >= of_max_features_) break;
  }
  // just delete any missing features we didn't have space for
  while (iter != optical_flow_features_.end()) {
    OFFeature& f = (*iter).second;
    if (f.missing_frames > 0) {
      for (unsigned int i = 0; i < f.obs.size(); i++) {
        unsigned int aug = i + f.missing_frames;
        if (aug >= of_history_size_) break;
        //   of_.cvs_observations[aug * of_max_features_ * 2 + index] = f.obs[i].x;
        //   of_.cvs_observations[aug * of_max_features_ * 2 + index + of_max_features_] = f.obs[i].y;
        //   of_.cvs_valid_flag[aug * of_max_features_ + index] = 1;
        if (optical_flow_augs_feature_counts_.size() > aug) optical_flow_augs_feature_counts_[aug]--;
      }
      iter = optical_flow_features_.erase(iter);
    }
    iter++;
  }
}

void VIOEkf::OpticalFlowRegister(const ff_msgs::CameraRegistration& cr) {
  processing_of_reg_ = true;
  unsigned int erased_aug = 0;
  if (of_camera_id_ != 0) {
    // ROS_WARN("Failed to get observations on time. Tossing last frame.");
    erased_aug = 0;
  } else {
    if (deleting_augs_.size() > 0) {
      erased_aug = deleting_augs_[0];
      deleting_augs_.erase(deleting_augs_.begin());
    } else {
      // choose the augmentation to delete
      if (optical_flow_augs_feature_counts_.size() < of_history_size_) {
        erased_aug = of_history_size_ - 1;
      } else if (optical_flow_augs_feature_counts_[of_history_size_ - 1] < 10) {
        erased_aug = of_history_size_ - 1;
      } else {
        erased_aug = 0;
      }
    }
    // delete the features from the deleted augmented state
    std::map<int, OFFeature>::iterator of_iter = optical_flow_features_.begin();
    while (of_iter != optical_flow_features_.end()) {
      OFFeature& f = (*of_iter).second;
      if (f.obs.size() > erased_aug) {
        f.obs.erase(f.obs.begin() + erased_aug);
      }
      if (f.obs.size() == 0)
        of_iter = optical_flow_features_.erase(of_iter);
      else
        of_iter++;
    }

    // update arrays of times and counts
    if (optical_flow_augs_feature_counts_.size() > of_history_size_) {
      optical_flow_augs_feature_counts_.erase(optical_flow_augs_feature_counts_.begin() + erased_aug);
      optical_flow_augs_times_.erase(optical_flow_augs_times_.begin() + erased_aug);
    }
    optical_flow_augs_feature_counts_.insert(optical_flow_augs_feature_counts_.begin(), 0);
    optical_flow_augs_times_.insert(optical_flow_augs_times_.begin(), cr.header.stamp);
  }

  // output to GNC
  reg_.cvs_optical_flow_pulse = of_history_size_ - erased_aug;
  of_camera_id_ = cr.camera_id;
}

void VIOEkf::SetSpeedGain(const uint8_t gain) { gnc_.cmc_.speed_gain_cmd = gain; }

void VIOEkf::SetResetCallback(std::function<void(void)> callback) { reset_callback_ = callback; }

void VIOEkf::PrepareStep(const sensor_msgs::Imu& imu, const geometry_msgs::Quaternion& quat) {
  // set IMU values
  // set timestamp
  imu_.imu_timestamp_sec = imu.header.stamp.sec;
  imu_.imu_timestamp_nsec = imu.header.stamp.nsec;

  // set angular vel, ros message is double cast into float
  imu_.imu_omega_B_ECI_sensor[0] = static_cast<float>(imu.angular_velocity.x);
  imu_.imu_omega_B_ECI_sensor[1] = static_cast<float>(imu.angular_velocity.y);
  imu_.imu_omega_B_ECI_sensor[2] = static_cast<float>(imu.angular_velocity.z);

  // set linear accel
  imu_.imu_A_B_ECI_sensor[0] = static_cast<float>(imu.linear_acceleration.x);
  imu_.imu_A_B_ECI_sensor[1] = static_cast<float>(imu.linear_acceleration.y);
  imu_.imu_A_B_ECI_sensor[2] = static_cast<float>(imu.linear_acceleration.z);

  // set validity
  imu_.imu_validity_flag = 1;

  // set saturation
  imu_.imu_sat_flag = 0;

  // set the ISS2BODY quaternion in preparation for step - this is effectively ignored
  // if tun_ase_gravity_removal = true in gnc.config
  gnc_.quat_[0] = quat.x;
  gnc_.quat_[1] = quat.y;
  gnc_.quat_[2] = quat.z;
  gnc_.quat_[3] = quat.w;

  // then copy all other values in preparation for step
  memcpy(&gnc_.reg_, &reg_, sizeof(cvs_registration_pulse));
  memcpy(&gnc_.of_, &of_, sizeof(cvs_optical_flow_msg));
  memcpy(&gnc_.imu_, &imu_, sizeof(imu_msg));
  gnc_.cmc_.localization_mode_cmd = cmc_mode_;

  // prevent double registrations
  reg_.cvs_optical_flow_pulse = false;
  // registration complete, now update next time
  if (of_inputs_delayed_) {
    of_.cvs_timestamp_sec = imu.header.stamp.sec;
    of_.cvs_timestamp_nsec = imu.header.stamp.nsec;
    of_inputs_delayed_ = false;
  }
  processing_of_reg_ = false;

  ApplyReset();
}

void VIOEkf::Step() {
  gnc_.Step();
  // if (gnc_.kfl_.confidence == 2) reset_ekf_ = true;

  /*  state->header.stamp.sec  = imu_.imu_timestamp_sec;
    state->header.stamp.nsec = imu_.imu_timestamp_nsec;
    state->pose.position    = msg_conversions::array_to_ros_point(gnc_.kfl_.P_B_ISS_ISS);
    state->velocity         = msg_conversions::array_to_ros_vector(gnc_.kfl_.V_B_ISS_ISS);
    state->pose.orientation = msg_conversions::array_to_ros_quat(gnc_.kfl_.quat_ISS2B);
  */
}

void VIOEkf::Reset() { reset_ekf_ = true; }

void VIOEkf::ResetPose() {
  reset_camera_to_body_ = nav_cam_to_body_;
  // Initial frame is identity frame
  reset_pose_ = Eigen::Isometry3d::Identity();
  reset_ready_ = true;
}

// reset ekf, during step function to prevent race conditions
void VIOEkf::ApplyReset() {
  if (!reset_ready_) return;

  // set the robot's position based on the pose
  Eigen::Quaterniond world_q_body(reset_pose_.linear());
  Eigen::Quaterniond camera_to_body_rotation(reset_camera_to_body_.linear());
  Eigen::Vector3d world_r_body(reset_pose_.translation());
  world_q_body = world_q_body * camera_to_body_rotation.conjugate();
  Eigen::Quaterniond q1(0, reset_camera_to_body_.translation().x(), reset_camera_to_body_.translation().y(),
                        reset_camera_to_body_.translation().z());
  Eigen::Quaterniond temp = world_q_body * q1 * world_q_body.conjugate();
  world_r_body = world_r_body - Eigen::Vector3d(temp.x(), temp.y(), temp.z());
  Eigen::Vector3d world_r_imu = world_r_body + world_q_body * imu_to_body_.translation();
  auto& quat_ISS2B = gnc_.est_->defaultParam->tun_ase_state_ic_quat_ISS2B;
  auto& P_B_ISS_ISS = gnc_.est_->defaultParam->tun_ase_state_ic_P_B_ISS_ISS;
  auto& P_EST_ISS_ISS = gnc_.est_->defaultParam->tun_ase_state_ic_P_EST_ISS_ISS;
  auto& V_B_ISS_ISS = gnc_.est_->defaultParam->tun_ase_state_ic_V_B_ISS_ISS;
  quat_ISS2B[0] = world_q_body.x();
  quat_ISS2B[1] = world_q_body.y();
  quat_ISS2B[2] = world_q_body.z();
  quat_ISS2B[3] = world_q_body.w();
  P_B_ISS_ISS[0] = world_r_body[0];
  P_B_ISS_ISS[1] = world_r_body[1];
  P_B_ISS_ISS[2] = world_r_body[2];
  P_EST_ISS_ISS[0] = world_r_imu[0];
  P_EST_ISS_ISS[1] = world_r_imu[1];
  P_EST_ISS_ISS[2] = world_r_imu[2];
  V_B_ISS_ISS[0] = 0.0;
  V_B_ISS_ISS[1] = 0.0;
  V_B_ISS_ISS[2] = 0.0;

  ROS_INFO("Reset EKF.");

  // reset the EKF (especially for the covariance)
  gnc_.Initialize();

  // reset optical flow too
  optical_flow_features_.clear();
  optical_flow_augs_feature_counts_.clear();
  optical_flow_augs_times_.clear();
  of_camera_id_ = 0;
  processing_of_reg_ = false;
  of_inputs_delayed_ = false;

  reset_ready_ = false;
  reset_ekf_ = false;

  // If we have set the reset callback, call it now.
  if (reset_callback_) reset_callback_();
}

}  // end namespace vio_augmentor
