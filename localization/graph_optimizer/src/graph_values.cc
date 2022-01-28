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

namespace graph_optimizer {
GraphValues::GraphValues(std::shared_ptr<gtsam::Values> values) : values_(std::move(values)) {}

bool GraphValues::Contains(const gtsam::Key& key) const { return values_->exists(key); }

bool GraphValues::Remove(const gtsam::Key& key) {
  if (!Contains(key)) return false;
  values_->erase(key);
  return true;
}

size_t GraphValues::size() const { return values_->size(); }
}  // namespace graph_optimizer
