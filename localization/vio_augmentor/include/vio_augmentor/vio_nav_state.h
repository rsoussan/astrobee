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

#ifndef VIO_AUGMENTOR_VIO_NAV_STATE_H_
#define VIO_AUGMENTOR_VIO_NAV_STATE_H_

#include <localization_common/time.h>

#include <Eigen/Geometry>

namespace vio_augmentor {
struct VIONavState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  localization_common::Time timestamp;
  Eigen::Isometry3d pose;
  Eigen::Vector3d velocity;
};
}  // end namespace vio_augmentor

#endif  // VIO_AUGMENTOR_VIO_NAV_STATE_H_
