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

  // Add element
  const double element_1 = 100.3;
  const localization_common::Time timestamp_1 = 1.0;
  EXPECT_TRUE(timestamped_nodes.Add(timestamp_1, element_1));
  EXPECT_EQ(timestamped_nodes.size(), 1);
  {
    EXPECT_TRUE(timestamped_nodes.Get(2.0) == boost::none);
    const auto accessed_node = timestamped_nodes.Get(timestamp_1);
    ASSERT_TRUE(accessed_node != boost::none);
    EXPECT_EQ(*accessed_node, element_1);
  }

  /*  // Add element
    const double element_2 = 37.1;
    const auto key_2 = timestamped_nodes.Add(element_2);
    EXPECT_TRUE(timestamped_nodes.Contains(key_1));
    EXPECT_TRUE(timestamped_nodes.Contains(key_2));
    EXPECT_FALSE(timestamped_nodes.Contains(300));
    EXPECT_EQ(timestamped_nodes.size(), 2);
    {
      const auto bad_key_val = timestamped_nodes.Get<double>(3);
      EXPECT_TRUE(bad_key_val == boost::none);
      const auto bad_type_val = timestamped_nodes.Get<int>(key_2);
      EXPECT_TRUE(bad_type_val == boost::none);
      const auto good_val = timestamped_nodes.Get<double>(key_1);
      ASSERT_TRUE(good_val != boost::none);
      EXPECT_EQ(good_val, element_1);
    }
    {
      const auto good_val = timestamped_nodes.Get<double>(key_2);
      ASSERT_TRUE(good_val != boost::none);
      EXPECT_EQ(good_val, element_2);
    }

    // Remove
    EXPECT_TRUE(timestamped_nodes.Remove(key_1));
    EXPECT_FALSE(timestamped_nodes.Contains(key_1));
    EXPECT_TRUE(timestamped_nodes.Contains(key_2));
    EXPECT_EQ(timestamped_nodes.size(), 1);
    {
      const auto good_val = timestamped_nodes.Get<double>(key_2);
      ASSERT_TRUE(good_val != boost::none);
      EXPECT_EQ(good_val, element_2);
    }

    // Bad Remove
    EXPECT_FALSE(timestamped_nodes.Remove(key_1));
    EXPECT_FALSE(timestamped_nodes.Remove(100));

    // Remove
    EXPECT_TRUE(timestamped_nodes.Remove(key_2));
    EXPECT_FALSE(timestamped_nodes.Contains(key_1));
    EXPECT_FALSE(timestamped_nodes.Contains(key_2));
    EXPECT_EQ(timestamped_nodes.size(), 0);
    {
      const auto bad_val = timestamped_nodes.Get<double>(key_2);
      EXPECT_TRUE(bad_val == boost::none);
    }*/
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
