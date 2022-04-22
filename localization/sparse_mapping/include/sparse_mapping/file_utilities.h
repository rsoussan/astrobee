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

#ifndef SPARSE_MAPPING_FILE_UTILITIES_H_
#define SPARSE_MAPPING_FILE_UTILITIES_H_

#include <map>
#include <string>
#include <vector>

namespace sparse_mapping {
  struct ControlPoint {
    int cid_left;
    int cid_right;
    Eigen::Vector2d keypoint_left;
    Eigen::Vector2d keypoint_right;
    Eigen::Vector3d global_t_point;
  };

  void LoadHuginControlPoints(const std::string& hugin_file, std::vector<ControlPoint>& control_points,
                              std::vector<std::string>& image_names);

  void LoadPoints(const std::string& points_file, std::vector<Eigen::Vector3d>& points);

  void LoadControlPoints(const std::vector<std::string>& files, std::vector<ControlPoint>& control_points,
                         std::vector < std::string & image_names);

  cv::Mat LoadImage(const std::string& filename);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_FILE_UTILITIES_H_
