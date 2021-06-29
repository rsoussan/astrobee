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

#ifndef LOCALIZATION_MEASUREMENTS_ACCELERATION_COMMAND_H_
#define LOCALIZATION_MEASUREMENTS_ACCELERATION_COMMAND_H_

#include <localization_measurements/measurement.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
// Add VectorSpace header as this is where traits::Equal is defined for
// Vector as well.  Remove when fixed in GTSAM
#include <gtsam/base/VectorSpace.h>

#include <iostream>
#include <string>

namespace localization_measurements {
struct AccelerationCommand : public Measurement {
  gtsam::Vector3 linear_acceleration;
  gtsam::Vector3 angular_acceleration;

  void print(const std::string& s = "") const {
    std::cout << (s.empty() ? s : s + " ") << linear_acceleration << ", " << angular_acceleration << ", " << timestamp
              << std::endl;
  }
  bool equals(const AccelerationCommand& acceleration_command, double tol = 1e-9) const {
    return gtsam::traits<gtsam::Vector3>::Equals(this->linear_acceleration, acceleration_command.linear_acceleration,
                                                 tol) &&
           gtsam::traits<gtsam::Vector3>::Equals(this->angular_acceleration, acceleration_command.angular_acceleration,
                                                 tol) &&
           std::abs(timestamp - acceleration_command.timestamp) < tol;
  }

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(linear_acceleration);
    ar& BOOST_SERIALIZATION_NVP(angular_acceleration);
    ar& BOOST_SERIALIZATION_NVP(timestamp);
  }
};
}  // namespace localization_measurements

namespace gtsam {
template <>
struct traits<localization_measurements::AccelerationCommand>
    : public Testable<localization_measurements::AccelerationCommand> {};
}  // namespace gtsam
#endif  // LOCALIZATION_MEASUREMENTS_ACCELERATION_COMMAND_H_
