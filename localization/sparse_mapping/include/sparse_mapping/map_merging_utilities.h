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

#ifndef SPARSE_MAPPING_MAP_MERGING_UTILITIES_H_
#define SPARSE_MAPPING_MAP_MERGING_UTILITIES_H_

#include <string>
#include <vector>

namespace sparse_mapping {
  /**
     Append map file.
  **/
  void AppendMapFile(std::string const& mapOut, std::string const& mapIn,
                     int num_image_overlaps_at_endpoints,
                     double outlier_factor,
                     bool bundle_adjust, bool fix_first_map);

  /**
     Merge two maps.
  **/
  void MergeMaps(sparse_mapping::SparseMap * A_in,
                 sparse_mapping::SparseMap * B_in,
                 int num_image_overlaps_at_endpoints,
                 double outlier_factor,
                 std::string const& output_map,
                 sparse_mapping::SparseMap * C_out);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_MAP_MERGING_UTILITIES_H_
