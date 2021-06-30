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

#include "test_utilities.h"  // NOLINT
#include <graph_localizer/acceleration_command_factor.h>
#include <localization_common/logger.h>
#include <localization_measurements/acceleration_command.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <gtest/gtest.h>

#include <iostream>

namespace {
localization_measurements::AccelerationCommand RandomAccelerationCommand() {
  localization_measurements::AccelerationCommand random_acceleration_measurement;
  random_acceleration_measurement.linear_acceleration = graph_localizer::RandomVector();
  random_acceleration_measurement.angular_acceleration = graph_localizer::RandomVector();
  random_acceleration_measurement.timestamp = graph_localizer::RandomDouble();
  return random_acceleration_measurement;
}

class TestErrorHelper {
 public:
  TestErrorHelper(const gtsam::AccelerationCommandFactor& factor) : factor_(factor) {}
  // Workaround to test factor Jacobians since boost::bind can't accept more than 9 arguments.
  // evaluateError has 10 including the optional Jacobians, but these count towards the boost bind limit.
  gtsam::Vector evaluateError(const gtsam::Pose3& world_T_body_a, const gtsam::Velocity3& world_F_world_v_body_a,
                              const gtsam::imuBias::ConstantBias& imu_biases_a, const gtsam::Pose3& world_T_body_b,
                              const gtsam::Velocity3& world_F_world_v_body_b) const {
    return factor_.evaluateError(world_T_body_a, world_F_world_v_body_a, imu_biases_a, world_T_body_b,
                                 world_F_world_v_body_b);
  }

 private:
  gtsam::AccelerationCommandFactor factor_;
};
}  // namespace

namespace gl = graph_localizer;
namespace lm = localization_measurements;
namespace sym = gtsam::symbol_shorthand;
TEST(AccelerationCommandFactorTester, Jacobian) {
  for (int i = 0; i < 500; ++i) {
    const auto acceleration_measurement = RandomAccelerationCommand();
    const double dt = gl::RandomDouble();
    const gtsam::Pose3 world_T_body_a = gl::RandomPose();
    const gtsam::Pose3 world_T_body_b = gl::RandomPose();
    const gtsam::Velocity3 world_F_world_v_body_a = gl::RandomVector();
    const gtsam::Velocity3 world_F_world_v_body_b = gl::RandomVector();
    const gtsam::imuBias::ConstantBias biases_a = gl::RandomIMUBias();
    const auto noise = gtsam::noiseModel::Unit::Create(3);
    const gtsam::AccelerationCommandFactor factor(acceleration_measurement, dt, noise, sym::P(0), sym::V(0), sym::B(0),
                                                  sym::P(1), sym::V(1));
    gtsam::Matrix H1, H2, H3, H4, H5;
    const auto factor_error = factor.evaluateError(world_T_body_a, world_F_world_v_body_a, biases_a, world_T_body_b,
                                                   world_F_world_v_body_b, H1, H2, H3, H4, H5);
    const TestErrorHelper test_error_helper(factor);
    const auto function =
      boost::function<gtsam::Vector(const gtsam::Pose3&, const gtsam::Velocity3&, const gtsam::imuBias::ConstantBias&,
                                    const gtsam::Pose3&, const gtsam::Velocity3&)>(
        boost::bind(&TestErrorHelper::evaluateError, test_error_helper, _1, _2, _3, _4, _5));
    const auto numerical_H1 =
      gtsam::numericalDerivative51<gtsam::Vector, gtsam::Pose3, gtsam::Velocity3, gtsam::imuBias::ConstantBias,
                                   gtsam::Pose3, gtsam::Velocity3>(
        function, world_T_body_a, world_F_world_v_body_a, biases_a, world_T_body_b, world_F_world_v_body_b, 1e-5);
    ASSERT_TRUE(numerical_H1.isApprox(H1.matrix(), 1e-6));
    const auto numerical_H2 =
      gtsam::numericalDerivative52<gtsam::Vector, gtsam::Pose3, gtsam::Velocity3, gtsam::imuBias::ConstantBias,
                                   gtsam::Pose3, gtsam::Velocity3>(
        function, world_T_body_a, world_F_world_v_body_a, biases_a, world_T_body_b, world_F_world_v_body_b, 1e-5);
    ASSERT_TRUE(numerical_H2.isApprox(H2.matrix(), 1e-6));
    const auto numerical_H3 =
      gtsam::numericalDerivative53<gtsam::Vector, gtsam::Pose3, gtsam::Velocity3, gtsam::imuBias::ConstantBias,
                                   gtsam::Pose3, gtsam::Velocity3>(
        function, world_T_body_a, world_F_world_v_body_a, biases_a, world_T_body_b, world_F_world_v_body_b, 1e-5);
    ASSERT_TRUE(numerical_H3.isApprox(H3.matrix(), 1e-6));
    const auto numerical_H4 =
      gtsam::numericalDerivative54<gtsam::Vector, gtsam::Pose3, gtsam::Velocity3, gtsam::imuBias::ConstantBias,
                                   gtsam::Pose3, gtsam::Velocity3>(
        function, world_T_body_a, world_F_world_v_body_a, biases_a, world_T_body_b, world_F_world_v_body_b, 1e-5);
    ASSERT_TRUE(numerical_H4.isApprox(H4.matrix(), 1e-6));
    const auto numerical_H5 =
      gtsam::numericalDerivative55<gtsam::Vector, gtsam::Pose3, gtsam::Velocity3, gtsam::imuBias::ConstantBias,
                                   gtsam::Pose3, gtsam::Velocity3>(
        function, world_T_body_a, world_F_world_v_body_a, biases_a, world_T_body_b, world_F_world_v_body_b, 1e-5);
    ASSERT_TRUE(numerical_H5.isApprox(H5.matrix(), 1e-6));
  }
}
