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

#ifndef SPARSE_MAPPING_REMOVE_INVALID_POINTS_PARAMS_H_
#define SPARSE_MAPPING_REMOVE_INVALID_POINTS_PARAMS_H_

namespace sparse_mapping {
struct RemoveInvalidPointsParams {
  double max_reprojection_error = 5.0;
  // Multiplied by median reprojection error to find threshold for removal
  double reprojection_error_threshold_scale_factor = 3.0;
  double min_valid_ray_angle = 1e-2;
  camera::CameraParameters camera;
  bool print_stats = true;
};
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_REMOVE_INVALID_POINTS_PARAMS_H_
