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

#ifndef VIO_AUGMENTOR_VIO_AUGMENTOR_H_
#define VIO_AUGMENTOR_VIO_AUGMENTOR_H_

#include <gnc_autocode/ekf.h>

#include <Eigen/Geometry>
#include <config_reader/config_reader.h>

#include <ros/node_handle.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/Imu.h>

#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/Feature2dArray.h>

#include <list>
#include <map>
#include <string>
#include <vector>
#include <functional>

namespace vio_augmentor {

typedef struct {
  float x;
  float y;
} OFObservation;

typedef struct {
  uint32_t feature_id;
  std::vector<OFObservation> obs;
  int missing_frames;  // number of frames skipped
} OFFeature;

/**
 * @brief Ekf implementation using GNC module
 * @details Ekf implementation using GNC module
 */
class VIOAugmentor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VIOAugmentor(void);

  void Reset();

  kfl_msg* GetOutput(void) { return &gnc_.kfl_; }

  void ReadParams(config_reader::ConfigReader* config);
  void SetBias(Eigen::Vector3f gyro_bias, Eigen::Vector3f accel_bias);

  void PrepareStep(const sensor_msgs::Imu& imu, const geometry_msgs::Quaternion& quat);
  int Step(ff_msgs::EkfState* state);

  void OpticalFlowUpdate(const ff_msgs::Feature2dArray& of);

  void OpticalFlowRegister(const ff_msgs::CameraRegistration& reg);

  void SetSpeedGain(const uint8_t gain);

  void SetResetCallback(std::function<void()> callback);

  Eigen::Affine3d GetNavCamToBody(void) const { return nav_cam_to_body_; }

  bool HasPoseEstimate() const { return has_pose_estimate_; }

 protected:
  /** Functions **/
  /**
   * If we're totally lose, this function reinitializes
   * the EKF with a pose gained from visual landmarks.
   **/
  void ResetPose(const Eigen::Affine3d& camera_to_body, geometry_msgs::Pose const& pose);
  void ApplyReset(void);

  void UpdateState(ff_msgs::EkfState* state);

  /** Variables **/
  // this calls the simulink code that actually runs the EKF
  gnc_autocode::GncEkfAutocode gnc_;

  // we write to these, then copy to gnc_
  cvs_registration_pulse reg_;
  cvs_optical_flow_msg of_;
  imu_msg imu_;
  int cmc_mode_;

  // EKF wrapper needs to capture the first landmark message to do PnP before
  // it can initialize the EKF autocode and let it run. This variable holds
  // that state and is atomic so that it doesn't require a mutex.
  bool reset_ekf_;
  // remember pose to reset to do in step function to avoid race conditions
  Eigen::Affine3d reset_camera_to_body_;
  geometry_msgs::Pose reset_pose_;
  bool reset_ready_;

  // optional: called when a reset occurs
  std::function<void()> reset_callback_;

  // vector of feature ids and observations, observations are features ids and positions
  // this list is sorted by feature id
  std::map<int, OFFeature> optical_flow_features_;
  // the number of features for each augmentation
  std::vector<int> optical_flow_augs_feature_counts_;
  std::vector<unsigned int> deleting_augs_;
  // the times of each augmentation
  std::vector<ros::Time> optical_flow_augs_times_;
  bool processing_of_reg_, of_inputs_delayed_;

  // only save this for writing to a file later
  geometry_msgs::Pose last_estimate_pose_;
  bool has_pose_estimate_ = false;

  /** Configuration Constants **/

  // transform from camera frame to body frame
  Eigen::Affine3d camera_to_body_, nav_cam_to_body_, imu_to_body_;
  float nav_cam_focal_length_;
  int min_of_observations_;
  unsigned int of_history_size_, of_max_features_;

  // camera ids currently stored in the EKF's augmented state
  uint32_t of_camera_id_;
};
}  // end namespace vio_augmentor

#endif  // VIO_AUGMENTOR_VIO_AUGMENTOR_H_
