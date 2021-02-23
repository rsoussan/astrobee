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

#include <ff_common/init.h>
#include <ff_util/ff_nodelet.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <gnc_autocode/autocode.h>

#include <thread>
#include <atomic>
#include <memory>

namespace vio_augmentor {

class VIOAugmentorNodelet : public ff_util::FreeFlyerNodelet {
 public:
  VIOAugmentorNodelet() : ff_util::FreeFlyerNodelet(NODE_VIO_AUG, true), killed_(false) {}
  ~VIOAugmentorNodelet() {
    killed_ = true;
    thread_->join();
  }
  // This is called when the nodelet is loaded into the nodelet manager
  void Initialize(ros::NodeHandle* nh) {
    // Bootstrap our environment
    ff_common::InitFreeFlyerApplication(getMyArgv());
    gnc_autocode::InitializeAutocode(this);
    vio_augmentor_.reset(new vio_augmentor::VIOAugmentorWrapper(this->GetPlatformHandle(true)));
    thread_.reset(new std::thread(&vio_augmentor::VIOAugmentorWrapper::Run, vio_augmentor_.get(), std::ref(killed_)));
  }

 private:
  std::shared_ptr<vio_augmentor::VIOAugmentorWrapper> vio_augmentor_;
  std::shared_ptr<std::thread> thread_;
  std::atomic<bool> killed_;
};

}  // end namespace vio_augmentor

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(vio_augmentor::VIOAugmentorNodelet, nodelet::Nodelet);
