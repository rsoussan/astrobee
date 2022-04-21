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

#ifndef SPARSE_MAPPING_UTILITIES_H_
#define SPARSE_MAPPING_UTILITIES_H_

#include <map>
#include <string>
#include <vector>

namespace sparse_mapping {
  // TODO(rsoussan): Make this to a sparse map function
  /**
     Take a map. Form a map with only a subset of the images.
     Bundle adjustment will happen later.
  */
  void ExtractSubmap(std::vector<std::string> * keep_ptr,
                     sparse_mapping::SparseMap * map_ptr);

  // TODO(rsoussan): Make this to a sparse map function
  /**
   * Register the map to the world coordinate system or verify
   * how well registration did.
   **/
  double RegistrationOrVerification(std::vector<std::string> const& data_files,
                                  bool verification,
                                  sparse_mapping::SparseMap * s);

}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_UTILITIES_H_
