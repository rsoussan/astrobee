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

#include <sparse_mapping/remove_invalid_points_and_detections_stats.h>

#include <glog>

namespace sparse_mapping {
RemoveInvalidPointsAndDetectionsStats::RemoveInvalidPointsAndDetectionsStats()
    : total(0), small_angle(0), behind_cam(0), invalid_reproj(0), big_reproj_err(0), num_features(0) {}
RemoveInvalidPointsAndDetectionsStats::Print() {
  LOG(INFO) << "Statistics of points to filter out.";
  LOG(INFO) << "Total num points: " << num_points;
  LOG(INFO) << "xyz points with small ray angles:     " << small_angle << " (" << (100.0 * small_angle) / total
            << " %)";
  LOG(INFO) << "xyz points behind camera:             " << behind_cam << " (" << (100.0 * behind_cam) / total << " %)";
  LOG(INFO) << "Reprojected outside of image:         " << invalid_reproj << " (" << (100.0 * invalid_reproj) / total
            << " %)";
  LOG(INFO) << "Features with big reprojection error: " << big_reproj_err << " ("
            << (100.0 * big_reproj_err) / num_features << " %)";
}
}  // namespace sparse_mapping
