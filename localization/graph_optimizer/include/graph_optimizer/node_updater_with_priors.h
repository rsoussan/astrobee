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

#ifndef GRAPH_OPTIMIZER_NODE_UPDATER_WITH_PRIORS_H_
#define GRAPH_OPTIMIZER_NODE_UPDATER_WITH_PRIORS_H_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace graph_optimizer {
template <typename NodeUpdaterType, typename NodeType, typename NoiseType = gtsam::SharedNoiseModel>
class NodeUpdaterWithPriors : public NodeUpdaterType {
 public:
  virtual ~NodeUpdaterWithPriors() {}

  virtual void AddInitialNodesAndPriors(const NodeType& node, const NoiseType& noise,
                                        gtsam::NonlinearFactorGraph& factors) = 0;

  virtual void AddPriors(const NodeType& node, const gtsam::KeyVector& keys, const NoiseType& noise,
                         gtsam::NonlinearFactorGraph& factors) = 0;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_NODE_UPDATER_WITH_PRIORS_H_
