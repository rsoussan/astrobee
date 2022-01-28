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

#ifndef GRAPH_OPTIMIZER_TIMESTAMPED_NODES_H_
#define GRAPH_OPTIMIZER_TIMESTAMPED_NODES_H_

#include <graph_optimizer/nodes.h>
#include <graph_optimizer/key_info.h>
#include <localization_common/time.h>
#include <localization_measurements/feature_point.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>

namespace graph_optimizer {
class TimestampedNodes : public Nodes {
 public:
  TimestampedNodes(std::shared_ptr<gtsam::Values> values = std::shared_ptr<gtsam::Values>(new gtsam::Values()))
      : Nodes(values) {}

  // Returns the oldest time that will be in graph values once the window is slid using params
  virtual boost::optional<localization_common::Time> SlideWindowNewOldestTime() const = 0;

  virtual gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                                   const gtsam::NonlinearFactorGraph& graph) const = 0;

  virtual boost::optional<gtsam::Key> GetKey(KeyCreatorFunction key_creator_function,
                                             const localization_common::Time timestamp) const = 0;

  // TODO(rsoussan): Move implementations from CombinedNavSTateGraphValues to here, make generic, store timestamp map
  // here
  virtual boost::optional<localization_common::Time> OldestTimestamp() const = 0;

  virtual boost::optional<localization_common::Time> LatestTimestamp() const = 0;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Nodes);
  }
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_TIMESTAMPED_NODES_H_
