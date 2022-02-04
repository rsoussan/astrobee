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
#include <graph_optimizer/node_updater_with_priors.h>
#include <graph_optimizer/test_utilities.h>
#include <graph_optimizer/timestamped_node_updater.h>
#include <graph_optimizer/timestamped_nodes.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtest/gtest.h>

namespace go = graph_optimizer;
namespace lc = localization_common;

using PosePrior = gtsam::PriorFactor<gtsam::Pose3>;

class ConstantVelocityNodeUpdater
    : public go::NodeUpdaterWithPriors<go::TimestampedNodeUpdater, gtsam::Pose3, gtsam::SharedNoiseModel> {
 public:
  explicit ConstantVelocityNodeUpdater(std::shared_ptr<go::Nodes> nodes) : nodes_(std::move(nodes)) {}
  virtual ~ConstantVelocityNodeUpdater() {}
  void AddInitialNodesAndPriors(const gtsam::Pose3& initial_pose, const gtsam::SharedNoiseModel& noise,
                                gtsam::NonlinearFactorGraph& factors) final {
    // TODO(rsoussan): how to pass initial timestamp????
    const auto initial_key = nodes_.Add(0.0, initial_pose);
    ASSERT_TRUE(initial_key != boost::none);
    AddPriors(initial_pose, {*initial_key}, noise, factors);
  }

  void AddPriors(const gtsam::Pose3& pose, const gtsam::KeyVector& keys, const gtsam::SharedNoiseModel& noise,
                 gtsam::NonlinearFactorGraph& factors) final {
    const PosePrior pose_prior_factor(keys[0], pose, noise);
    factors.push_back(pose_prior_factor);
  }

  bool AddNodes(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final {
    const Eigen::Vector3d current_position = (timestamp - starting_timestamp_) * velocity_;
    const gtsam::Pose3 current_pose(gtsam::Rot3(), current_position);
    const auto current_key = nodes_.Add(timestamp, current_pose);
    // ASSERT_TRUE(current_key != boost::none);
    const auto latest_timestamp = nodes_.LatestTimestamp();
    // ASSERT_TRUE(latest_timestamp != boost::none);
    const auto previous_key = nodes_.Key(*latest_timestamp);
    // ASSERT_TRUE(previous_key != boost::none);
    const gtsam::Vector6 pose_between_noise_sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());
    const auto pose_between_noise =
      gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_between_noise_sigmas));
    const gtsam::Pose3 relative_pose = last_pose_.inverse() * current_pose;
    gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr pose_between_factor(
      new gtsam::BetweenFactor<gtsam::Pose3>(*previous_key, *current_key, relative_pose, pose_between_noise));
    factors.push_back(pose_between_factor);
    last_pose_ = current_pose;
  }

  boost::optional<localization_common::Time> OldestTimestamp() const final { return nodes_.OldestTimestamp(); }

  boost::optional<localization_common::Time> LatestTimestamp() const final { return nodes_.LatestTimestamp(); }

 private:
  go::TimestampedNodes<gtsam::Pose3> nodes_;
  gtsam::Pose3 last_pose_;
  const Eigen::Vector3d velocity_ = Eigen::Vector3d(1.0, 1.0, 1.0);
  const lc::Time starting_timestamp_ = 0.0;
  const Eigen::Vector3d starting_positon_ = Eigen::Vector3d::Zero();
};

TEST(TimestampedNodeUpdaterTester, AddFactors) {
  const auto nodes = std::make_shared<go::Nodes>();
  const auto params = go::DefaultGraphOptimizerParams();
  go::GraphOptimizer optimizer(params, nodes);
  ConstantVelocityNodeUpdater node_updater(optimizer.nodes());
  const gtsam::Vector6 pose_between_noise_sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());
  const auto pose_noise =
    gtsam::noiseModel::Diagonal::Sigmas(Eigen::Ref<const Eigen::VectorXd>(pose_between_noise_sigmas));
  node_updater.AddInitialNodesAndPriors(gtsam::Pose3(), pose_noise, optimizer.factors());
  // TODO(rsoussan): loop through timestamps, call node_updater.AddNode(), test that factors and nodes are added!
  // test other functins?
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
