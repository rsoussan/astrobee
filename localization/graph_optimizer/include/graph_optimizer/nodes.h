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

#ifndef GRAPH_OPTIMIZER_NODES_H_
#define GRAPH_OPTIMIZER_NODES_H_

#include <localization_common/logger.h>

#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>

namespace graph_optimizer {
class Nodes {
 public:
  Nodes(std::shared_ptr<gtsam::Values> values = std::shared_ptr<gtsam::Values>(new gtsam::Values()));

  template <typename ValueType>
  boost::optional<ValueType> Get(const gtsam::Key& key) const;

  template <typename ValueType>
  bool Add(const gtsam::Key& key, const ValueType& value);

  bool Remove(const gtsam::Key& key);

  bool Contains(const gtsam::Key& key) const;

  size_t size() const;

  const gtsam::Values& values() const { return *values_; }

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(values_);
  }

  std::shared_ptr<gtsam::Values> values_;
};

// Implementation
template <typename ValueType>
boost::optional<ValueType> Nodes::Get(const gtsam::Key& key) const {
  try {
    return values_->at<ValueType>(key);
  } catch (...) {
    return boost::none;
  }
}

template <typename ValueType>
bool Nodes::Add(const gtsam::Key& key, const ValueType& value) {
  if (Contains(key)) return false;
  values_->insert(key, value);
  return true;
}
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_NODES_H_
