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

#include <graph_optimizer/graph_optimizer.h>
#include <graph_optimizer/test_utilities.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>

#include <gtsam/slam/PriorFactor.h>

#include <gtest/gtest.h>

namespace go = graph_optimizer;
namespace lc = localization_common;

using PosePrior = gtsam::PriorFactor<gtsam::Pose3>;
using VelocityPrior = gtsam::PriorFactor<gtsam::Velocity3>;

TEST(GraphOptimizerTester, AddFactors) {
  const auto nodes = std::make_shared<go::Nodes>();
  const auto params = go::DefaultGraphOptimizerParams();
  go::GraphOptimizer optimizer(params, nodes);
  EXPECT_EQ(optimizer.TotalNumFactors(), 0);
  EXPECT_EQ(optimizer.NumFactors<PosePrior>(), 0);

  // Add prior 1
  gtsam::Pose3 pose_1;
  const auto noisy_pose_1 = lc::AddNoiseToPose(pose_1, 1, 1);
  const auto key_1 = nodes->Add(noisy_pose_1);
  const gtsam::Vector6 pose_prior_noise_sigmas((gtsam::Vector(6) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished());
  const auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(pose_prior_noise_sigmas);
  PosePrior pose_factor_1(key_1, pose_1, pose_noise);
  optimizer.AddFactor(pose_factor_1);
  {
    EXPECT_EQ(optimizer.TotalNumFactors(), 1);
    EXPECT_EQ(optimizer.NumFactors<PosePrior>(), 1);
    EXPECT_EQ(optimizer.NumFactors<VelocityPrior>(), 0);
    ASSERT_TRUE(optimizer.Optimize());
    const auto optimized_pose_1 = nodes->Get<gtsam::Pose3>(key_1);
    ASSERT_TRUE(optimized_pose_1 != boost::none);
    EXPECT_MATRIX_NEAR(pose_1, (*optimized_pose_1), 1e-6);
  }

  // Add prior 2
  gtsam::Pose3 pose_2;
  const auto noisy_pose_2 = lc::AddNoiseToPose(pose_2, 1, 1);
  const auto key_2 = nodes->Add(noisy_pose_2);
  PosePrior pose_factor_2(key_2, pose_2, pose_noise);
  optimizer.AddFactor(pose_factor_2);
  {
    EXPECT_EQ(optimizer.TotalNumFactors(), 2);
    EXPECT_EQ(optimizer.NumFactors<PosePrior>(), 2);
    EXPECT_EQ(optimizer.NumFactors<VelocityPrior>(), 0);
    ASSERT_TRUE(optimizer.Optimize());
    const auto optimized_pose_1 = nodes->Get<gtsam::Pose3>(key_1);
    ASSERT_TRUE(optimized_pose_1 != boost::none);
    EXPECT_MATRIX_NEAR(pose_1, (*optimized_pose_1), 1e-6);
    const auto optimized_pose_2 = nodes->Get<gtsam::Pose3>(key_2);
    ASSERT_TRUE(optimized_pose_2 != boost::none);
    EXPECT_MATRIX_NEAR(pose_2, (*optimized_pose_2), 1e-6);
  }
}

TEST(GraphOptimizerTester, RemoveFactorsWithType) {
  const auto nodes = std::make_shared<go::Nodes>();
  const auto params = go::DefaultGraphOptimizerParams();
  go::GraphOptimizer optimizer(params, nodes);
  gtsam::Pose3 pose;
  const auto key = nodes->Add(pose);
  const gtsam::Vector6 pose_prior_noise_sigmas((gtsam::Vector(6) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished());
  const auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(pose_prior_noise_sigmas);
  {
    PosePrior pose_factor(key, pose, pose_noise);
    optimizer.AddFactor(pose_factor);
  }
  {
    PosePrior pose_factor(key, pose, pose_noise);
    optimizer.AddFactor(pose_factor);
  }
  {
    PosePrior pose_factor(key, pose, pose_noise);
    optimizer.AddFactor(pose_factor);
  }
  gtsam::Vector3 velocity;
  const auto velocity_key = nodes->Add(velocity);
  const gtsam::Vector3 velocity_prior_noise_sigmas((gtsam::Vector(3) << 1.0, 1.0, 1.0).finished());
  const auto velocity_noise = gtsam::noiseModel::Diagonal::Sigmas(velocity_prior_noise_sigmas);
  {
    VelocityPrior velocity_factor(velocity_key, velocity, velocity_noise);
    optimizer.AddFactor(velocity_factor);
  }
  EXPECT_EQ(optimizer.NumFactors<PosePrior>(), 3);
  EXPECT_EQ(optimizer.NumFactors<VelocityPrior>(), 1);
  EXPECT_EQ(optimizer.TotalNumFactors(), 4);
  optimizer.RemoveFactors<PosePrior>();
  EXPECT_EQ(optimizer.NumFactors<PosePrior>(), 0);
  EXPECT_EQ(optimizer.TotalNumFactors(), 1);
}

TEST(GraphOptimizerTester, RemoveFactorsWithKey) {
  const auto nodes = std::make_shared<go::Nodes>();
  const auto params = go::DefaultGraphOptimizerParams();
  go::GraphOptimizer optimizer(params, nodes);
  gtsam::Pose3 pose;
  const auto pose_key_1 = nodes->Add(pose);
  const gtsam::Vector6 pose_prior_noise_sigmas((gtsam::Vector(6) << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0).finished());
  const auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(pose_prior_noise_sigmas);
  {
    PosePrior pose_factor(pose_key_1, pose, pose_noise);
    optimizer.AddFactor(pose_factor);
  }
  {
    PosePrior pose_factor(pose_key_1, pose, pose_noise);
    optimizer.AddFactor(pose_factor);
  }
  const auto pose_key_2 = nodes->Add(pose);
  {
    PosePrior pose_factor(pose_key_2, pose, pose_noise);
    optimizer.AddFactor(pose_factor);
  }
  gtsam::Vector3 velocity;
  const auto velocity_key = nodes->Add(velocity);
  const gtsam::Vector3 velocity_prior_noise_sigmas((gtsam::Vector(3) << 1.0, 1.0, 1.0).finished());
  const auto velocity_noise = gtsam::noiseModel::Diagonal::Sigmas(velocity_prior_noise_sigmas);
  {
    VelocityPrior velocity_factor(velocity_key, velocity, velocity_noise);
    optimizer.AddFactor(velocity_factor);
  }
  EXPECT_EQ(optimizer.NumFactors<PosePrior>(), 3);
  EXPECT_EQ(optimizer.NumFactors<VelocityPrior>(), 1);
  EXPECT_EQ(optimizer.TotalNumFactors(), 4);
  optimizer.RemoveFactors(pose_key_2);
  EXPECT_EQ(optimizer.NumFactors<PosePrior>(), 2);
  EXPECT_EQ(optimizer.NumFactors<VelocityPrior>(), 1);
  EXPECT_EQ(optimizer.TotalNumFactors(), 3);
  gtsam::NonlinearFactorGraph removed_pose_factors;
  optimizer.RemoveFactors(pose_key_1, removed_pose_factors);
  EXPECT_EQ(removed_pose_factors.size(), 1);
  EXPECT_EQ(optimizer.NumFactors<PosePrior>(), 0);
  EXPECT_EQ(optimizer.NumFactors<VelocityPrior>(), 1);
  EXPECT_EQ(optimizer.TotalNumFactors(), 1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
