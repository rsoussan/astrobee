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

#ifndef SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_NODELET_H_
#define SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_NODELET_H_

#include <ff_util/ff_nodelet.h>
#include <sparse_map_matcher/sparse_map_matcher_wrapper.h>

#include <image_transport/image_transport.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace sparse_map_matcher {

class SparseMapMatcherNodelet : public ff_util::FreeFlyerNodelet {
 public:
  SparseMapMatcherNodelet();

 private:
  void Initialize(ros::NodeHandle* nh) final;
  void SubscribeAndAdvertise(ros::NodeHandle* nh);
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
  bool EnableService(ff_msgs::SetBool::Request & req, ff_msgs::SetBool::Response & res);

  SparseMapMatcherWrapper matcher_wrapper_;
  image_transport::Subscriber image_sub_;
  ros::ServiceServer enable_srv_;
  ros::Publisher vl_pub_;
  bool enabled_;
};
}  // namespace sparse_map_matcher

#endif  // SPARSE_MAP_MATCHER_SPARSE_MAP_MATCHER_NODELET_H_

