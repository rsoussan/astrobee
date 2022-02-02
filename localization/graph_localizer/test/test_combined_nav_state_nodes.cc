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

#include <graph_localizer/combined_nav_state_nodes.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>

#include <gtest/gtest.h>

namespace gl = graph_localizer;
namespace go = graph_optimizer;
namespace lc = localization_common;

TEST(CombinedNavStateNodesTester, AddGet) {
  std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
  gl::CombinedNavStateNodes nodes(graph_nodes);
  EXPECT_TRUE(nodes.empty());
  EXPECT_EQ(nodes.size(), 0);

  // Add random node 0
  const auto n0 = lc::RandomCombinedNavState();
  nodes.Add(n0);
  EXPECT_FALSE(nodes.empty());
  EXPECT_EQ(nodes.size(), 1);
  {
    const auto node = nodes.Get(n0.timestamp());
    ASSERT_TRUE(node != boost::none);
    EXPECT_TRUE(node->Equals(n0));
  }

  // Add random node 1
  const auto n1 = lc::RandomCombinedNavState();
  nodes.Add(n1);
  EXPECT_FALSE(nodes.empty());
  EXPECT_EQ(nodes.size(), 2);
  {
    const auto node = nodes.Get(n1.timestamp());
    ASSERT_TRUE(node != boost::none);
    EXPECT_TRUE(node->Equals(n1));
  }
}

TEST(CombinedNavStateNodesTester, OldestLatest) {
  std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
  gl::CombinedNavStateNodes nodes(graph_nodes);
  // No elements
  {
    EXPECT_TRUE(nodes.OldestTimestamp() == boost::none);
    EXPECT_TRUE(nodes.OldestNode() == boost::none);
    EXPECT_TRUE(nodes.LatestTimestamp() == boost::none);
    EXPECT_TRUE(nodes.LatestNode() == boost::none);
  }

  const localization_common::Time timestamp_1 = 1.0;
  const auto node_1 = lc::RandomCombinedNavState(timestamp_1);
  ASSERT_TRUE(nodes.Add(node_1));
  // 1 element
  {
    const auto oldest_timestamp = nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_1);
    const auto latest_timestamp = nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_1);

    const auto oldest_node = nodes.OldestNode();
    ASSERT_TRUE(oldest_node != boost::none);
    EXPECT_TRUE(oldest_node->Equals(node_1));
    const auto latest_node = nodes.LatestNode();
    ASSERT_TRUE(latest_node != boost::none);
    EXPECT_TRUE(latest_node->Equals(node_1));
  }

  const localization_common::Time timestamp_2 = 3.23;
  const auto node_2 = lc::RandomCombinedNavState(timestamp_2);
  ASSERT_TRUE(nodes.Add(node_2));
  // 2 elements
  {
    const auto oldest_timestamp = nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_1);
    const auto latest_timestamp = nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_2);

    const auto oldest_node = nodes.OldestNode();
    ASSERT_TRUE(oldest_node != boost::none);
    EXPECT_TRUE(oldest_node->Equals(node_1));
    const auto latest_node = nodes.LatestNode();
    ASSERT_TRUE(latest_node != boost::none);
    EXPECT_TRUE(latest_node->Equals(node_2));
  }

  const localization_common::Time timestamp_3 = 21.11;
  const auto node_3 = lc::RandomCombinedNavState(timestamp_3);
  ASSERT_TRUE(nodes.Add(node_3));
  // 3 elements
  {
    const auto oldest_timestamp = nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_1);
    const auto latest_timestamp = nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_3);

    const auto oldest_node = nodes.OldestNode();
    ASSERT_TRUE(oldest_node != boost::none);
    EXPECT_TRUE(oldest_node->Equals(node_1));
    const auto latest_node = nodes.LatestNode();
    ASSERT_TRUE(latest_node != boost::none);
    EXPECT_TRUE(latest_node->Equals(node_3));
  }

  ASSERT_TRUE(nodes.Remove(timestamp_1));
  {
    const auto oldest_timestamp = nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_2);
    const auto latest_timestamp = nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_3);

    const auto oldest_node = nodes.OldestNode();
    ASSERT_TRUE(oldest_node != boost::none);
    EXPECT_TRUE(oldest_node->Equals(node_2));
    const auto latest_node = nodes.LatestNode();
    ASSERT_TRUE(latest_node != boost::none);
    EXPECT_TRUE(latest_node->Equals(node_3));
  }

  ASSERT_TRUE(nodes.Remove(timestamp_3));
  {
    const auto oldest_timestamp = nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_2);
    const auto latest_timestamp = nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_2);

    const auto oldest_node = nodes.OldestNode();
    ASSERT_TRUE(oldest_node != boost::none);
    EXPECT_TRUE(oldest_node->Equals(node_2));
    const auto latest_node = nodes.LatestNode();
    ASSERT_TRUE(latest_node != boost::none);
    EXPECT_TRUE(latest_node->Equals(node_2));
  }

  ASSERT_TRUE(nodes.Remove(timestamp_2));
  {
    EXPECT_TRUE(nodes.OldestTimestamp() == boost::none);
    EXPECT_TRUE(nodes.OldestNode() == boost::none);
    EXPECT_TRUE(nodes.LatestTimestamp() == boost::none);
    EXPECT_TRUE(nodes.LatestNode() == boost::none);
  }
}

TEST(CombinedNavStateNodesTester, LowerAndUpperBounds) {
  std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
  gl::CombinedNavStateNodes nodes(graph_nodes);
  // No elements
  {
    const auto lower_and_upper_bound_timestamps = nodes.LowerAndUpperBoundTimestamps(1.0);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.first == boost::none);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.second == boost::none);
    const auto lower_and_upper_bound_nodes = nodes.LowerAndUpperBoundNodes(1.0);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first == boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second == boost::none);
  }

  // 1 element
  const localization_common::Time timestamp_1 = 37.001;
  const auto node_1 = lc::RandomCombinedNavState(timestamp_1);
  ASSERT_TRUE(nodes.Add(node_1));
  // 1 element below
  {
    const auto lower_and_upper_bound_timestamps = nodes.LowerAndUpperBoundTimestamps(10.0);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = nodes.LowerAndUpperBoundNodes(10.0);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second->Equals(node_1));
  }
  // 1 element above
  {
    const auto lower_and_upper_bound_timestamps = nodes.LowerAndUpperBoundTimestamps(57.3);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_1);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.second == boost::none);
    const auto lower_and_upper_bound_nodes = nodes.LowerAndUpperBoundNodes(57.3);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first->Equals(node_1));
    EXPECT_TRUE(lower_and_upper_bound_nodes.second == boost::none);
  }
  // 1 element equal
  {
    const auto lower_and_upper_bound_timestamps = nodes.LowerAndUpperBoundTimestamps(timestamp_1);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = nodes.LowerAndUpperBoundNodes(timestamp_1);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second->Equals(node_1));
  }

  // 2 elements
  const localization_common::Time timestamp_2 = 2.221;
  const auto node_2 = lc::RandomCombinedNavState(timestamp_2);
  ASSERT_TRUE(nodes.Add(node_2));

  // 2 elements below
  {
    const auto lower_and_upper_bound_timestamps = nodes.LowerAndUpperBoundTimestamps(1.1);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_2);
    const auto lower_and_upper_bound_nodes = nodes.LowerAndUpperBoundNodes(1.1);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second->Equals(node_2));
  }
  // 2 elements above
  {
    const auto lower_and_upper_bound_timestamps = nodes.LowerAndUpperBoundTimestamps(111.3);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_1);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.second == boost::none);
    const auto lower_and_upper_bound_nodes = nodes.LowerAndUpperBoundNodes(111.3);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first->Equals(node_1));
    EXPECT_TRUE(lower_and_upper_bound_nodes.second == boost::none);
  }
  // 2 elements equal lower
  {
    const auto lower_and_upper_bound_timestamps = nodes.LowerAndUpperBoundTimestamps(timestamp_2);
    EXPECT_TRUE(lower_and_upper_bound_timestamps.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_2);
    const auto lower_and_upper_bound_nodes = nodes.LowerAndUpperBoundNodes(timestamp_2);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first == boost::none);
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second->Equals(node_2));
  }
  // 2 elements equal upper
  {
    const auto lower_and_upper_bound_timestamps = nodes.LowerAndUpperBoundTimestamps(timestamp_1);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_2);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = nodes.LowerAndUpperBoundNodes(timestamp_1);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first->Equals(node_2));
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second->Equals(node_1));
  }
  // 2 elements between
  {
    const auto lower_and_upper_bound_timestamps = nodes.LowerAndUpperBoundTimestamps(15.1);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_2);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = nodes.LowerAndUpperBoundNodes(15.1);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first->Equals(node_2));
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second->Equals(node_1));
  }

  // 3 elements
  const localization_common::Time timestamp_3 = 14.076;
  const auto node_3 = lc::RandomCombinedNavState(timestamp_3);
  ASSERT_TRUE(nodes.Add(node_3));
  // 3 elements lower between
  {
    const auto lower_and_upper_bound_timestamps = nodes.LowerAndUpperBoundTimestamps(7.11);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_2);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_3);
    const auto lower_and_upper_bound_nodes = nodes.LowerAndUpperBoundNodes(7.11);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first->Equals(node_2));
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second->Equals(node_3));
  }
  // 3 elements upper between
  {
    const auto lower_and_upper_bound_timestamps = nodes.LowerAndUpperBoundTimestamps(22.22);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.first != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.first), timestamp_3);
    ASSERT_TRUE(lower_and_upper_bound_timestamps.second != boost::none);
    EXPECT_EQ(*(lower_and_upper_bound_timestamps.second), timestamp_1);
    const auto lower_and_upper_bound_nodes = nodes.LowerAndUpperBoundNodes(22.22);
    ASSERT_TRUE(lower_and_upper_bound_nodes.first != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.first->Equals(node_3));
    ASSERT_TRUE(lower_and_upper_bound_nodes.second != boost::none);
    EXPECT_TRUE(lower_and_upper_bound_nodes.second->Equals(node_1));
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
