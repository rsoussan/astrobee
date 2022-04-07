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

#include <localization_common/test_utilities.h>
#include <optimization_common/affine3_local_parameterization.h>
#include <optimization_common/utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;
namespace oc = optimization_common;
TEST(Affine3LocalParameterizationTester, Affine3Plus) {
  oc::Affine3Plus affine3_plus;
  for (int i = 0; i < 500; ++i) {
    const Eigen::Affine3d pose = lc::RandomAffine3d();
    const Eigen::Affine3d delta = lc::RandomAffine3d();
    const Eigen::Affine3d updated_pose = pose * delta;
    const Eigen::Matrix<double, 7, 1> pose_vector = oc::VectorFromAffine3d(pose);
    const Eigen::Matrix<double, 7, 1> delta_vector = oc::VectorFromAffine3d(delta);
    Eigen::Matrix<double, 7, 1> updated_pose_vector;
    affine3_plus(pose_vector.data(), delta_vector.data(), updated_pose_vector.data());
    const Eigen::Affine3d updated_pose_again = oc::Affine3d(updated_pose_vector);
    EXPECT_MATRIX_NEAR(updated_pose, updated_pose_again, 1e-6);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
