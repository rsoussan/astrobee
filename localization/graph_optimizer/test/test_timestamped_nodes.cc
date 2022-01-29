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

#include <graph_optimizer/timestamped_nodes.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>

#include <gtest/gtest.h>

namespace go = graph_optimizer;
namespace lc = localization_common;

TEST(TimestampedNodesTester, AddRemove) {
  std::shared_ptr<go::Nodes> nodes(new go::Nodes());
  go::TimestampedNodes<double> timestamped_nodes(nodes);
  EXPECT_EQ(timestamped_nodes.size(), 0);
  EXPECT_TRUE(timestamped_nodes.empty());

  // Add element 1
  const double element_1 = 100.3;
  const localization_common::Time timestamp_1 = 1.0;
  EXPECT_TRUE(timestamped_nodes.Add(timestamp_1, element_1));
  EXPECT_EQ(timestamped_nodes.size(), 1);
  EXPECT_FALSE(timestamped_nodes.empty());
  {
    EXPECT_TRUE(timestamped_nodes.Get(2.0) == boost::none);
    const auto accessed_node = timestamped_nodes.Get(timestamp_1);
    ASSERT_TRUE(accessed_node != boost::none);
    EXPECT_EQ(*accessed_node, element_1);
  }

  // Add element 2
  const double element_2 = 100.3;
  const localization_common::Time timestamp_2 = 3.3;
  EXPECT_TRUE(timestamped_nodes.Add(timestamp_2, element_2));
  EXPECT_EQ(timestamped_nodes.size(), 2);
  EXPECT_FALSE(timestamped_nodes.empty());
  {
    EXPECT_TRUE(timestamped_nodes.Get(7.0) == boost::none);
    const auto accessed_node_1 = timestamped_nodes.Get(timestamp_1);
    ASSERT_TRUE(accessed_node_1 != boost::none);
    EXPECT_EQ(*accessed_node_1, element_1);
    const auto accessed_node_2 = timestamped_nodes.Get(timestamp_2);
    ASSERT_TRUE(accessed_node_2 != boost::none);
    EXPECT_EQ(*accessed_node_2, element_2);
  }

  // Remove element 1
  EXPECT_TRUE(timestamped_nodes.Remove(timestamp_1));
  EXPECT_TRUE(timestamped_nodes.Get(timestamp_1) == boost::none);
  EXPECT_TRUE(timestamped_nodes.Get(timestamp_2) != boost::none);
  EXPECT_EQ(timestamped_nodes.size(), 1);
  EXPECT_FALSE(timestamped_nodes.empty());
  {
    const auto good_val = timestamped_nodes.Get(timestamp_2);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(*good_val, element_2);
  }

  // Bad Remove
  EXPECT_FALSE(timestamped_nodes.Remove(timestamp_1));
  EXPECT_FALSE(timestamped_nodes.Remove(100));

  // Remove element 2
  EXPECT_TRUE(timestamped_nodes.Remove(timestamp_2));
  EXPECT_TRUE(timestamped_nodes.Get(timestamp_1) == boost::none);
  EXPECT_TRUE(timestamped_nodes.Get(timestamp_2) == boost::none);
  EXPECT_EQ(timestamped_nodes.size(), 0);
  EXPECT_TRUE(timestamped_nodes.empty());
  {
    const auto bad_val = timestamped_nodes.Get(timestamp_2);
    EXPECT_TRUE(bad_val == boost::none);
  }
}

TEST(TimestampedNodesTester, OldestLatest) {
  std::shared_ptr<go::Nodes> nodes(new go::Nodes());
  go::TimestampedNodes<double> timestamped_nodes(nodes);
  // No elements
  {
    EXPECT_TRUE(timestamped_nodes.OldestTimestamp() == boost::none);
    //    EXPECT_TRUE(timestamped_nodes.OldestNode() == boost::none);
    EXPECT_TRUE(timestamped_nodes.LatestTimestamp() == boost::none);
    //    EXPECT_TRUE(timestamped_nodes.LatestNode() == boost::none);
  }
  const double element_1 = 101.0;
  const localization_common::Time timestamp_1 = 1.0;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_1, element_1));
  // 1 element
  {
    const auto oldest_timestamp = timestamped_nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_1);
    const auto latest_timestamp = timestamped_nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_1);
  }
  // 2 elements
  const double element_2 = 100.3;
  const localization_common::Time timestamp_2 = 3.3;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_2, element_2));
  {
    const auto oldest_timestamp = timestamped_nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_1);
    const auto latest_timestamp = timestamped_nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_2);
  }

  // 3 elements
  const double element_3 = 2100.3;
  const localization_common::Time timestamp_3 = 19.3;
  ASSERT_TRUE(timestamped_nodes.Add(timestamp_3, element_3));
  {
    const auto oldest_timestamp = timestamped_nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_1);
    const auto latest_timestamp = timestamped_nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_3);
  }

  ASSERT_TRUE(timestamped_nodes.Remove(timestamp_1));
  {
    const auto oldest_timestamp = timestamped_nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_2);
    const auto latest_timestamp = timestamped_nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_3);
  }

  ASSERT_TRUE(timestamped_nodes.Remove(timestamp_3));
  {
    const auto oldest_timestamp = timestamped_nodes.OldestTimestamp();
    ASSERT_TRUE(oldest_timestamp != boost::none);
    EXPECT_EQ(*oldest_timestamp, timestamp_2);
    const auto latest_timestamp = timestamped_nodes.LatestTimestamp();
    ASSERT_TRUE(latest_timestamp != boost::none);
    EXPECT_EQ(*latest_timestamp, timestamp_2);
  }

  ASSERT_TRUE(timestamped_nodes.Remove(timestamp_2));
  {
    EXPECT_TRUE(timestamped_nodes.OldestTimestamp() == boost::none);
    //    EXPECT_TRUE(timestamped_nodes.OldestNode() == boost::none);
    EXPECT_TRUE(timestamped_nodes.LatestTimestamp() == boost::none);
    //    EXPECT_TRUE(timestamped_nodes.LatestNode() == boost::none);
  }
}

TEST(TimestampedNodesTester, Serialization) {
  const go::TimestampedNodes<double> nodes;
  const auto serialized_nodes = gtsam::serializeBinary(nodes);
  go::TimestampedNodes<double> deserialized_nodes;
  gtsam::deserializeBinary(serialized_nodes, deserialized_nodes);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
