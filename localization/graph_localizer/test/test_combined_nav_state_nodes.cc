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

TEST(CombinedNavStateNodesTester, LowerBoundOrEqual) {
  std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
  gl::CombinedNavStateNodes nodes(graph_nodes);
  const localization_common::Time timestamp_1 = 3.098;
  const auto node_1 = lc::RandomCombinedNavState(timestamp_1);
  ASSERT_TRUE(nodes.Add(node_1));
  const localization_common::Time timestamp_2 = 5.777;
  const auto node_2 = lc::RandomCombinedNavState(timestamp_2);
  ASSERT_TRUE(nodes.Add(node_2));
  const localization_common::Time timestamp_3 = 7.902;
  const auto node_3 = lc::RandomCombinedNavState(timestamp_3);
  ASSERT_TRUE(nodes.Add(node_3));
  const auto too_low_timestamp = nodes.LowerBoundOrEqualTimestamp(1.23);
  EXPECT_TRUE(too_low_timestamp == boost::none);
  const auto lowest_timestamp = nodes.LowerBoundOrEqualTimestamp(4.11);
  ASSERT_TRUE(lowest_timestamp != boost::none);
  EXPECT_EQ(*lowest_timestamp, timestamp_1);
  const auto middle_timestamp = nodes.LowerBoundOrEqualTimestamp(6.61);
  ASSERT_TRUE(middle_timestamp != boost::none);
  EXPECT_EQ(*middle_timestamp, timestamp_2);
  const auto upper_timestamp = nodes.LowerBoundOrEqualTimestamp(900);
  ASSERT_TRUE(upper_timestamp != boost::none);
  EXPECT_EQ(*upper_timestamp, timestamp_3);
  const auto equal_timestamp = nodes.LowerBoundOrEqualTimestamp(timestamp_2);
  ASSERT_TRUE(equal_timestamp != boost::none);
  EXPECT_EQ(*equal_timestamp, timestamp_2);

  const auto too_low_node = nodes.LowerBoundOrEqualNode(1.23);
  EXPECT_TRUE(too_low_node == boost::none);
  const auto lowest_node = nodes.LowerBoundOrEqualNode(4.11);
  ASSERT_TRUE(lowest_node != boost::none);
  EXPECT_TRUE(lowest_node->Equals(node_1));
  const auto middle_node = nodes.LowerBoundOrEqualNode(6.61);
  ASSERT_TRUE(middle_node != boost::none);
  EXPECT_TRUE(middle_node->Equals(node_2));
  const auto upper_node = nodes.LowerBoundOrEqualNode(900);
  ASSERT_TRUE(upper_node != boost::none);
  EXPECT_TRUE(upper_node->Equals(node_3));
  const auto equal_node = nodes.LowerBoundOrEqualNode(timestamp_2);
  ASSERT_TRUE(equal_node != boost::none);
  EXPECT_TRUE(equal_node->Equals(node_2));
}

TEST(CombinedNavStateNodesTester, Closest) {
  std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
  gl::CombinedNavStateNodes nodes(graph_nodes);
  const localization_common::Time timestamp_1 = 3.098;
  const auto node_1 = lc::RandomCombinedNavState(timestamp_1);
  ASSERT_TRUE(nodes.Add(node_1));
  const localization_common::Time timestamp_2 = 5.777;
  const auto node_2 = lc::RandomCombinedNavState(timestamp_2);
  ASSERT_TRUE(nodes.Add(node_2));
  const localization_common::Time timestamp_3 = 7.902;
  const auto node_3 = lc::RandomCombinedNavState(timestamp_3);
  ASSERT_TRUE(nodes.Add(node_3));
  const auto below_lowest_timestamp = nodes.ClosestTimestamp(1.23);
  ASSERT_TRUE(below_lowest_timestamp != boost::none);
  EXPECT_EQ(*below_lowest_timestamp, timestamp_1);
  const auto above_lowest_timestamp = nodes.ClosestTimestamp(4.11);
  ASSERT_TRUE(above_lowest_timestamp != boost::none);
  EXPECT_EQ(*above_lowest_timestamp, timestamp_1);
  const auto below_middle_timestamp = nodes.ClosestTimestamp(5.61);
  ASSERT_TRUE(below_middle_timestamp != boost::none);
  EXPECT_EQ(*below_middle_timestamp, timestamp_2);
  const auto above_middle_timestamp = nodes.ClosestTimestamp(6.61);
  ASSERT_TRUE(above_middle_timestamp != boost::none);
  EXPECT_EQ(*above_middle_timestamp, timestamp_2);
  const auto below_upper_timestamp = nodes.ClosestTimestamp(7.61);
  ASSERT_TRUE(below_upper_timestamp != boost::none);
  EXPECT_EQ(*below_upper_timestamp, timestamp_3);
  const auto above_upper_timestamp = nodes.ClosestTimestamp(8.61);
  ASSERT_TRUE(above_upper_timestamp != boost::none);
  EXPECT_EQ(*above_upper_timestamp, timestamp_3);
  const auto equal_timestamp = nodes.ClosestTimestamp(timestamp_2);
  ASSERT_TRUE(equal_timestamp != boost::none);
  EXPECT_EQ(*equal_timestamp, timestamp_2);

  const auto below_lowest_node = nodes.ClosestNode(1.23);
  EXPECT_TRUE(below_lowest_node != boost::none);
  EXPECT_TRUE(below_lowest_node->Equals(node_1));
  const auto above_lowest_node = nodes.ClosestNode(4.11);
  ASSERT_TRUE(above_lowest_node != boost::none);
  EXPECT_TRUE(above_lowest_node->Equals(node_1));
  const auto below_middle_node = nodes.ClosestNode(5.61);
  ASSERT_TRUE(below_middle_node != boost::none);
  EXPECT_TRUE(below_middle_node->Equals(node_2));
  const auto above_middle_node = nodes.ClosestNode(6.61);
  ASSERT_TRUE(above_middle_node != boost::none);
  EXPECT_TRUE(above_middle_node->Equals(node_2));
  const auto below_upper_node = nodes.ClosestNode(7.61);
  ASSERT_TRUE(below_upper_node != boost::none);
  EXPECT_TRUE(below_upper_node->Equals(node_3));
  const auto above_upper_node = nodes.ClosestNode(8.61);
  ASSERT_TRUE(above_upper_node != boost::none);
  EXPECT_TRUE(above_upper_node->Equals(node_3));
  const auto equal_node = nodes.ClosestNode(timestamp_2);
  ASSERT_TRUE(equal_node != boost::none);
  EXPECT_TRUE(equal_node->Equals(node_2));
}

TEST(CombinedNavStateNodesTester, OldKeysTimestampsAndNodes) {
  std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
  gl::CombinedNavStateNodes nodes(graph_nodes);
  const double t0 = 0;
  const auto n0 = lc::RandomCombinedNavState(t0);
  const int k0 = 1;
  const double t1 = 1.001;
  const auto n1 = lc::RandomCombinedNavState(t1);
  const int k1 = 2;
  const double t2 = 2.100;
  const auto n2 = lc::RandomCombinedNavState(t2);
  const int k2 = 3;
  const double t3 = 3.0222;
  const auto n3 = lc::RandomCombinedNavState(t3);
  const int k3 = 4;
  ASSERT_TRUE(nodes.Add(n0));
  ASSERT_TRUE(nodes.Add(n1));
  ASSERT_TRUE(nodes.Add(n2));
  ASSERT_TRUE(nodes.Add(n3));
  {
    const auto old_keys = nodes.OldKeys(0);
    EXPECT_EQ(old_keys.size(), 0);
    const auto old_nodes = nodes.OldNodes(0);
    EXPECT_EQ(old_nodes.size(), 0);
    const auto old_timestamps = nodes.OldTimestamps(0);
    EXPECT_EQ(old_timestamps.size(), 0);
  }
  {
    const auto old_keys = nodes.OldKeys(0.1);
    EXPECT_EQ(old_keys.size(), 3);
    const auto old_nodes = nodes.OldNodes(0.1);
    ASSERT_EQ(old_nodes.size(), 1);
    EXPECT_TRUE(old_nodes[0].Equals(n0));
    const auto old_timestamps = nodes.OldTimestamps(0.1);
    ASSERT_EQ(old_timestamps.size(), 1);
    EXPECT_EQ(old_timestamps[0], t0);
  }
  {
    const auto old_keys = nodes.OldKeys(1.7);
    EXPECT_EQ(old_keys.size(), 6);
    const auto old_nodes = nodes.OldNodes(1.7);
    ASSERT_EQ(old_nodes.size(), 2);
    EXPECT_TRUE(old_nodes[0].Equals(n0));
    EXPECT_TRUE(old_nodes[1].Equals(n1));
    const auto old_timestamps = nodes.OldTimestamps(1.7);
    ASSERT_EQ(old_timestamps.size(), 2);
    EXPECT_EQ(old_timestamps[0], t0);
    EXPECT_EQ(old_timestamps[1], t1);
  }
  {
    const auto old_keys = nodes.OldKeys(2.333);
    EXPECT_EQ(old_keys.size(), 9);
    const auto old_nodes = nodes.OldNodes(2.333);
    ASSERT_EQ(old_nodes.size(), 3);
    EXPECT_TRUE(old_nodes[0].Equals(n0));
    EXPECT_TRUE(old_nodes[1].Equals(n1));
    EXPECT_TRUE(old_nodes[2].Equals(n2));
    const auto old_timestamps = nodes.OldTimestamps(2.333);
    ASSERT_EQ(old_timestamps.size(), 3);
    EXPECT_EQ(old_timestamps[0], t0);
    EXPECT_EQ(old_timestamps[1], t1);
    EXPECT_EQ(old_timestamps[2], t2);
  }
  {
    const auto old_keys = nodes.OldKeys(1999);
    EXPECT_EQ(old_keys.size(), 12);
    const auto old_nodes = nodes.OldNodes(1999);
    ASSERT_EQ(old_nodes.size(), 4);
    EXPECT_TRUE(old_nodes[0].Equals(n0));
    EXPECT_TRUE(old_nodes[1].Equals(n1));
    EXPECT_TRUE(old_nodes[2].Equals(n2));
    EXPECT_TRUE(old_nodes[3].Equals(n3));
    const auto old_timestamps = nodes.OldTimestamps(1999);
    ASSERT_EQ(old_timestamps.size(), 4);
    EXPECT_EQ(old_timestamps[0], t0);
    EXPECT_EQ(old_timestamps[1], t1);
    EXPECT_EQ(old_timestamps[2], t2);
    EXPECT_EQ(old_timestamps[3], t3);
  }
}

TEST(CombinedNavStateNodesTester, RemoveOldNodes) {
  {
    std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
    gl::CombinedNavStateNodes nodes(graph_nodes);
    const double t0 = 0;
    const auto n0 = lc::RandomCombinedNavState(t0);
    const double t1 = 1.001;
    const auto n1 = lc::RandomCombinedNavState(t1);
    const double t2 = 2.100;
    const auto n2 = lc::RandomCombinedNavState(t2);
    const double t3 = 3.0222;
    const auto n3 = lc::RandomCombinedNavState(t3);
    ASSERT_TRUE(nodes.Add(n0));
    ASSERT_TRUE(nodes.Add(n1));
    ASSERT_TRUE(nodes.Add(n2));
    ASSERT_TRUE(nodes.Add(n3));
    const int num_nodes_removed = nodes.RemoveOldNodes(0);
    EXPECT_EQ(num_nodes_removed, 0);
    EXPECT_EQ(nodes.size(), 4);
  }

  {
    std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
    gl::CombinedNavStateNodes nodes(graph_nodes);
    const double t0 = 0;
    const auto n0 = lc::RandomCombinedNavState(t0);
    const double t1 = 1.001;
    const auto n1 = lc::RandomCombinedNavState(t1);
    const double t2 = 2.100;
    const auto n2 = lc::RandomCombinedNavState(t2);
    const double t3 = 3.0222;
    const auto n3 = lc::RandomCombinedNavState(t3);
    ASSERT_TRUE(nodes.Add(n0));
    ASSERT_TRUE(nodes.Add(n1));
    ASSERT_TRUE(nodes.Add(n2));
    ASSERT_TRUE(nodes.Add(n3));
    const int num_nodes_removed = nodes.RemoveOldNodes(0.1);
    EXPECT_EQ(num_nodes_removed, 1);
    EXPECT_EQ(nodes.size(), 3);
    const auto timestamps = nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t1);
    EXPECT_EQ(timestamps[1], t2);
    EXPECT_EQ(timestamps[2], t3);
  }
  {
    std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
    gl::CombinedNavStateNodes nodes(graph_nodes);
    const double t0 = 0;
    const auto n0 = lc::RandomCombinedNavState(t0);
    const double t1 = 1.001;
    const auto n1 = lc::RandomCombinedNavState(t1);
    const double t2 = 2.100;
    const auto n2 = lc::RandomCombinedNavState(t2);
    const double t3 = 3.0222;
    const auto n3 = lc::RandomCombinedNavState(t3);
    ASSERT_TRUE(nodes.Add(n0));
    ASSERT_TRUE(nodes.Add(n1));
    ASSERT_TRUE(nodes.Add(n2));
    ASSERT_TRUE(nodes.Add(n3));
    const int num_nodes_removed = nodes.RemoveOldNodes(1.334);
    EXPECT_EQ(num_nodes_removed, 2);
    EXPECT_EQ(nodes.size(), 2);
    const auto timestamps = nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t2);
    EXPECT_EQ(timestamps[1], t3);
  }

  {
    std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
    gl::CombinedNavStateNodes nodes(graph_nodes);
    const double t0 = 0;
    const auto n0 = lc::RandomCombinedNavState(t0);
    const double t1 = 1.001;
    const auto n1 = lc::RandomCombinedNavState(t1);
    const double t2 = 2.100;
    const auto n2 = lc::RandomCombinedNavState(t2);
    const double t3 = 3.0222;
    const auto n3 = lc::RandomCombinedNavState(t3);
    ASSERT_TRUE(nodes.Add(n0));
    ASSERT_TRUE(nodes.Add(n1));
    ASSERT_TRUE(nodes.Add(n2));
    ASSERT_TRUE(nodes.Add(n3));
    const int num_nodes_removed = nodes.RemoveOldNodes(2.78);
    EXPECT_EQ(num_nodes_removed, 3);
    EXPECT_EQ(nodes.size(), 1);
    const auto timestamps = nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t3);
  }

  {
    std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
    gl::CombinedNavStateNodes nodes(graph_nodes);
    const double t0 = 0;
    const auto n0 = lc::RandomCombinedNavState(t0);
    const double t1 = 1.001;
    const auto n1 = lc::RandomCombinedNavState(t1);
    const double t2 = 2.100;
    const auto n2 = lc::RandomCombinedNavState(t2);
    const double t3 = 3.0222;
    const auto n3 = lc::RandomCombinedNavState(t3);
    ASSERT_TRUE(nodes.Add(n0));
    ASSERT_TRUE(nodes.Add(n1));
    ASSERT_TRUE(nodes.Add(n2));
    ASSERT_TRUE(nodes.Add(n3));
    const int num_nodes_removed = nodes.RemoveOldNodes(1923.78);
    EXPECT_EQ(num_nodes_removed, 4);
    EXPECT_EQ(nodes.size(), 0);
  }
}

TEST(CombinedNavStateNodesTester, Duration) {
  std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
  gl::CombinedNavStateNodes nodes(graph_nodes);
  EXPECT_EQ(nodes.Duration(), 0);
  ASSERT_TRUE(nodes.Add(lc::RandomCombinedNavState(1.0)));
  EXPECT_EQ(nodes.Duration(), 0);
  ASSERT_TRUE(nodes.Add(lc::RandomCombinedNavState(2.0)));
  EXPECT_NEAR(nodes.Duration(), 1, 1e-6);
  ASSERT_TRUE(nodes.Add(lc::RandomCombinedNavState(3.0)));
  EXPECT_NEAR(nodes.Duration(), 2, 1e-6);
}

TEST(CombinedNavStateNodesTester, Timestamps) {
  std::shared_ptr<go::Nodes> graph_nodes(new go::Nodes());
  gl::CombinedNavStateNodes nodes(graph_nodes);
  {
    const auto timestamps = nodes.Timestamps();
    EXPECT_EQ(timestamps.size(), 0);
  }
  const double t0 = 0;
  const double t1 = 1;
  const double t2 = 2;
  const double t3 = 3;
  ASSERT_TRUE(nodes.Add(lc::RandomCombinedNavState(t0)));
  ASSERT_TRUE(nodes.Add(lc::RandomCombinedNavState(t1)));
  ASSERT_TRUE(nodes.Add(lc::RandomCombinedNavState(t2)));
  ASSERT_TRUE(nodes.Add(lc::RandomCombinedNavState(t3)));
  {
    const auto timestamps = nodes.Timestamps();
    EXPECT_EQ(timestamps[0], t0);
    EXPECT_EQ(timestamps[1], t1);
    EXPECT_EQ(timestamps[2], t2);
    EXPECT_EQ(timestamps[3], t3);
  }
}

TEST(CombinedNavStateNodesTester, Serialization) {
  const gl::CombinedNavStateNodes nodes;
  const auto serialized_nodes = gtsam::serializeBinary(nodes);
  gl::CombinedNavStateNodes deserialized_nodes;
  gtsam::deserializeBinary(serialized_nodes, deserialized_nodes);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
