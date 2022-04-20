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

  // Writes the NVM control network format.
  void WriteNVM(std::vector<Eigen::Matrix2Xd > const& cid_to_keypoints,
                std::vector<std::string> const& cid_to_filename,
                std::vector<std::map<int, int> > const& pid_to_feature_track,
                std::vector<Eigen::Vector3d> const& pid_to_xyz,
                std::vector<Eigen::Affine3d> const& cid_to_cam_T_global,
                double focal_length,
                std::string const& output_filename);
  // Reads the NVM control network format.
  void ReadNVM(std::string const& input_filename,
               std::vector<Eigen::Matrix2Xd > * cid_to_keypoints,
               std::vector<std::string> * cid_to_filename,
               std::vector<std::map<int, int> > * pid_to_feature_track,
               std::vector<Eigen::Vector3d> * pid_to_xyz,
               std::vector<Eigen::Affine3d> * cid_to_cam_T_global);

  // Adds yaml.gz or .txt extension, depending on descriptor
  std::string ImageToFeatureFile(std::string const& image_file,
                                 std::string const& detector_name);

  // Write features yaml file
  void WriteFeatures(std::string const& detector_name,
                     std::vector<cv::KeyPoint> const& keypoints,
                     cv::Mat const& descriptors,
                     std::string const& output_filename);

  // Read features yaml file
  bool ReadFeatures(std::string const& input_filename,
                    std::string const& detector_name,
                    std::vector<cv::KeyPoint> * keypoints,
                    cv::Mat * descriptors);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_THIRD_PARTY_MAP_UTILITIES_H_
