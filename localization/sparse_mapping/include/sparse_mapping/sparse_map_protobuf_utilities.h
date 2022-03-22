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

#ifndef SPARSE_MAPPING_SPARSE_MAP_PROTOBUF_UTILITIES_H_
#define SPARSE_MAPPING_SPARSE_MAP_PROTOBUF_UTILITIES_H_

#include <sparse_mapping/sparse_map.h>

#include <string>

namespace sparse_mapping {
// TODO(rsoussan): Make these non member functions, add const accesors and new constructors for sparse map taking
// all required datatypes as needed
/**
 * Constructs a new sparse map from a protobuf file, with specified
 * vocabulary tree and optional parameters.
 **/
/*SparseMap(const std::string & protobuf_file,
          bool localization = false);*/

/**
 * Save the map to a protobuf file.
 **/
void Save(const std::string& protobuf_file) const;

// Load map. If localization is true, load only the parts of the map
// needed for localization.
void Load(const std::string& protobuf_file, bool localization = false);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_SPARSE_MAP_PROTOBUF_UTILITIES_H_
