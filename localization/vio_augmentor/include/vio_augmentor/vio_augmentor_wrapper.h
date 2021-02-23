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

#ifndef VIO_AUGMENTOR_VIO_AUGMENTOR_WRAPPER_H_
#define VIO_AUGMENTOR_VIO_AUGMENTOR_WRAPPER_H_

#include <ff_msgs/GraphState.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <vio_augmentor/vio_ekf.h>

#include <Eigen/Geometry>
#include <config_reader/config_reader.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/Imu.h>
#include <ff_msgs/CameraRegistration.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/Feature2dArray.h>
#include <ff_msgs/FlightMode.h>
#include <ff_util/perf_timer.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include <atomic>
#include <condition_variable>  // NOLINT
#include <list>
#include <mutex>
#include <map>
#include <string>
#include <vector>
#include <functional>

namespace vio_augmentor {

/**
 * @brief ROS wrapper around EKF GNC code.
 * @details ROS wrapper around EKF GNC code.
 */
class VIOAugmentorWrapper {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit VIOAugmentorWrapper(ros::NodeHandle* nh);
  ~VIOAugmentorWrapper();

  void Run(std::atomic<bool> const& killed);

  /**
   * This moves the EKF forward one time step
   * and publishes the EKF pose.
   *
   */
  void Step();

 protected:
  void ReadParams();
  /**
   * Initialize services and topics besides IMU.
   **/
  void InitializeEkf();
  /**
   * Publishes a ROS message containing the state of the EKF.
   **/
  void PublishState();

  /**
   * Resets the EKF.
   **/
  bool ResetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);  // NOLINT

  /**
   * Callback functions. These all record the information
   * and then pass it on to the EKF the next time the Step function
   * is called.
   **/
  void ImuCallBack(sensor_msgs::Imu::ConstPtr const& imu);
  void OpticalFlowCallBack(ff_msgs::Feature2dArray::ConstPtr const& of);
  void RegisterOpticalFlowCamera(ff_msgs::CameraRegistration::ConstPtr const& cr);
  void FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode);
  void LocalizationStateCallback(const ff_msgs::GraphState::ConstPtr& loc_msg);

  /**
   * Callback when the EKF resets
   **/
  void ResetCallback();

  /** Variables **/
  // the actual EKF
  VIOEkf ekf_;
  ff_msgs::EkfState state_;

  bool ekf_initialized_;

  // most recent imu message
  sensor_msgs::Imu imu_;
  geometry_msgs::Quaternion quat_;
  int imus_dropped_;

  // EKF wrapper needs to capture the first landmark message to do PnP before
  // it can initialize the EKF autocode and let it run. This variable holds
  // that state and is atomic so that it doesn't require a mutex.
  std::atomic<bool> have_imu_;
  std::atomic<int> input_mode_;

  /** Configuration Constants **/

  /** Ros **/
  config_reader::ConfigReader config_;
  ff_util::PerfTimer pt_ekf_;
  ros::Timer config_timer_;

  ros::NodeHandle* nh_;
  // topic subscribers
  ros::Subscriber imu_sub_, of_sub_, state_sub_;
  ros::Subscriber of_reg_sub_;
  ros::Subscriber flight_mode_sub_;

  // publisher
  ros::Publisher state_pub_;
  ros::Publisher pose_pub_, twist_pub_;
  ros::Publisher reset_pub_;
  ros::ServiceServer reset_srv_;

  /** Threading **/

  // mutex for msgs, concurrency protection
  std::mutex mutex_imu_msg_;
  std::mutex mutex_of_msg_;

  // cv to wait for an imu reading
  std::condition_variable cv_imu_;

  // Prevents needing to call ros::ok() from a thread
  std::atomic<bool> killed_;

  boost::optional<localization_common::CombinedNavState> latest_combined_nav_state_;
  boost::optional<localization_common::CombinedNavStateCovariances> latest_covariances_;
  boost::optional<ff_msgs::GraphState> latest_loc_msg_;
};

}  // end namespace vio_augmentor

#endif  // VIO_AUGMENTOR_VIO_AUGMENTOR_WRAPPER_H_
