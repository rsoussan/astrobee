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

#ifndef VIO_AUGMENTOR_VIO_AUGMENTOR_NODELET_H_
#define VIO_AUGMENTOR_VIO_AUGMENTOR_NODELET_H_

#include <config_reader/config_reader.h>
#include <ff_util/ff_nodelet.h>
#include <vio_augmentor/vio_augmentor_wrapper.h>

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
#include <ff_msgs/GraphState.h>
#include <ff_msgs/InitialIMUBiases.h>
#include <ff_util/perf_timer.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include <string>

namespace vio_augmentor {
class VIOAugmentorNodelet : public ff_util::FreeFlyerNodelet {
 public:
  VIOAugmentorNodelet();
  ~VIOAugmentorNodelet();
  void Initialize(ros::NodeHandle* nh);

 private:
  void ImuCallBack(sensor_msgs::Imu::ConstPtr const& imu);
  void OpticalFlowCallBack(ff_msgs::Feature2dArray::ConstPtr const& of);
  void RegisterOpticalFlowCamera(ff_msgs::CameraRegistration::ConstPtr const& cr);
  void FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode);
  void LocalizationStateCallback(const ff_msgs::GraphState::ConstPtr& loc_msg);
  void InitialIMUBiasesCallback(const ff_msgs::InitialIMUBiases::ConstPtr& biases);
  bool ResetService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void PublishState();
  void PublishStateHelper(const ff_msgs::EkfState& latest_vio_augmented_loc_msg);

  vio_augmentor::VIOAugmentorWrapper vio_augmentor_wrapper_;
  std::shared_ptr<std::thread> thread_;
  std::atomic<bool> killed_;

  ros::NodeHandle* nh_;
  ros::Subscriber imu_sub_, of_sub_, state_sub_, of_reg_sub_, flight_mode_sub_, biases_sub_;
  ros::Publisher state_pub_, pose_pub_, twist_pub_;
  ros::ServiceServer reset_srv_;
  tf2_ros::TransformBroadcaster transform_pub_;
  std::string platform_name_;
};

}  // namespace vio_augmentor
#endif  // VIO_AUGMENTOR_VIO_AUGMENTOR_NODELET_H_
