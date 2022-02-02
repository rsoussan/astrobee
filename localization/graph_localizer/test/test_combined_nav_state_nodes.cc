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

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
