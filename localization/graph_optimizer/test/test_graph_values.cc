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

#include <graph_optimizer/graph_values.h>
#include <localization_common/logger.h>
#include <localization_common/test_utilities.h>

#include <gtest/gtest.h>

namespace go = graph_optimizer;
namespace lc = localization_common;

TEST(GraphValuesTester, Test) {
  go::GraphValues graph_values;
  EXPECT_EQ(graph_values.size(), 0);

  // Add element
  const double element_1 = 100.3;
  graph_values.Add(1, element_1);
  EXPECT_TRUE(graph_values.Contains(1));
  EXPECT_FALSE(graph_values.Contains(2));
  EXPECT_EQ(graph_values.size(), 1);
  {
    const auto bad_key_val = graph_values.Get<double>(2);
    EXPECT_TRUE(bad_key_val == boost::none);
    const auto bad_type_val = graph_values.Get<int>(2);
    EXPECT_TRUE(bad_type_val == boost::none);
    const auto good_val = graph_values.Get<double>(1);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_1);
  }

  // Add element
  const double element_2 = 37.1;
  graph_values.Add(7, element_2);
  EXPECT_TRUE(graph_values.Contains(1));
  EXPECT_TRUE(graph_values.Contains(7));
  EXPECT_FALSE(graph_values.Contains(300));
  EXPECT_EQ(graph_values.size(), 2);
  {
    const auto bad_key_val = graph_values.Get<double>(3);
    EXPECT_TRUE(bad_key_val == boost::none);
    const auto bad_type_val = graph_values.Get<int>(7);
    EXPECT_TRUE(bad_type_val == boost::none);
    const auto good_val = graph_values.Get<double>(1);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_1);
  }
  {
    const auto good_val = graph_values.Get<double>(7);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_2);
  }

  // Remove
  EXPECT_TRUE(graph_values.Remove(1));
  EXPECT_FALSE(graph_values.Contains(1));
  EXPECT_TRUE(graph_values.Contains(7));
  EXPECT_EQ(graph_values.size(), 1);
  {
    const auto good_val = graph_values.Get<double>(7);
    ASSERT_TRUE(good_val != boost::none);
    EXPECT_EQ(good_val, element_2);
  }

  // Bad Remove
  EXPECT_FALSE(graph_values.Remove(1));
  EXPECT_FALSE(graph_values.Remove(100));

  // Remove
  EXPECT_TRUE(graph_values.Remove(7));
  EXPECT_FALSE(graph_values.Contains(1));
  EXPECT_FALSE(graph_values.Contains(7));
  EXPECT_EQ(graph_values.size(), 0);
  {
    const auto bad_val = graph_values.Get<double>(7);
    EXPECT_TRUE(bad_val == boost::none);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
