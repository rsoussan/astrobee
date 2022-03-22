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

#ifndef SPARSE_MAPPING_THIRD_PARTY_MAP_UTILITIES_H_
#define SPARSE_MAPPING_THIRD_PARTY_MAP_UTILITIES_H_

#include <sparse_mapping/sparse_map.h>

#include <map>
#include <string>
#include <vector>

namespace sparse_mapping {
SparseMap(std::string const& filename, std::vector<std::string> const& files);
// Reorder the images in the map and the rest of the data accordingly
void reorderMap(std::map<int, int> const& old_cid_to_new_cid);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_THIRD_PARTY_MAP_UTILITIES_H_
