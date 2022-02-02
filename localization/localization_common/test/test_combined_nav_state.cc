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

#include <localization_common/combined_nav_state.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
TEST(CombinedNavStateTester, EqualityCheck) {
  const auto combined_nav_state_1 = lc::RandomCombinedNavState();
  EXPECT_TRUE(combined_nav_state_1.Equals(combined_nav_state_1));
  // Vary pose
  {
    const gtsam::Pose3 pose_2(combined_nav_state_1.pose().rotation(),
                              combined_nav_state_1.pose().translation() + Eigen::Vector3d(1, 2, 3));
    const auto combined_nav_state_2 = lc::CombinedNavState(
      pose_2, combined_nav_state_1.velocity(), combined_nav_state_1.bias(), combined_nav_state_1.timestamp());
    EXPECT_FALSE(combined_nav_state_1.Equals(combined_nav_state_2));
  }
  // Vary velocity
  {
    const gtsam::Velocity3 velocity_2(combined_nav_state_1.velocity() + Eigen::Vector3d(1, 2, 3));
    const auto combined_nav_state_2 = lc::CombinedNavState(
      combined_nav_state_1.pose(), velocity_2, combined_nav_state_1.bias(), combined_nav_state_1.timestamp());
    EXPECT_FALSE(combined_nav_state_1.Equals(combined_nav_state_2));
  }
  // Vary accel bias
  {
    Eigen::Matrix<double, 6, 1> bias_offset(Eigen::Matrix<double, 6, 1>::Zero());
    bias_offset.head<3>() += Eigen::Vector3d(1, 2, 3);
    const gtsam::imuBias::ConstantBias bias_2(combined_nav_state_1.bias() + bias_offset);
    const auto combined_nav_state_2 = lc::CombinedNavState(combined_nav_state_1.pose(), combined_nav_state_1.velocity(),
                                                           bias_2, combined_nav_state_1.timestamp());
    EXPECT_FALSE(combined_nav_state_1.Equals(combined_nav_state_2));
  }
  // Vary gyro bias
  {
    Eigen::Matrix<double, 6, 1> bias_offset(Eigen::Matrix<double, 6, 1>::Zero());
    bias_offset.tail<3>() += Eigen::Vector3d(1, 2, 3);
    const gtsam::imuBias::ConstantBias bias_2(combined_nav_state_1.bias() + bias_offset);
    const auto combined_nav_state_2 = lc::CombinedNavState(combined_nav_state_1.pose(), combined_nav_state_1.velocity(),
                                                           bias_2, combined_nav_state_1.timestamp());
    EXPECT_FALSE(combined_nav_state_1.Equals(combined_nav_state_2));
  }
  // Vary timestamp
  {
    const auto combined_nav_state_2 =
      lc::CombinedNavState(combined_nav_state_1.pose(), combined_nav_state_1.velocity(), combined_nav_state_1.bias(),
                           combined_nav_state_1.timestamp() + 1);
    EXPECT_FALSE(combined_nav_state_1.Equals(combined_nav_state_2));
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
