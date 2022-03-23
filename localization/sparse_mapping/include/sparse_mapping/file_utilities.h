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
  // Adds yaml.gz or .txt extension, depending on descriptor
  std::string ImageToFeatureFile(std::string const& image_file,
                                 std::string const& detector_name);

  // The name of the file storing the list of images
  std::string DBImagesFile(std::string const& db_name);

  // The name of the matches file
  std::string MatchesFile(std::string const& map_file);

  // The name of the essential file
  std::string EssentialFile(std::string const& map_file);

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

  // Read SIFT features in Lowe's format
  int ReadFeaturesSIFT(std::string const& filename,
                       cv::Mat * descriptors,
                       std::vector<cv::KeyPoint> * keypoints);

  void MergePids(int repeat_index, int num_unique,
                 std::vector<std::map<int, int> > * pid_to_cid_fid);

  void PrintPidStats(std::vector<std::map<int, int> > const& pid_to_cid_fid);

  // Extract control points and the images they correspond to from
  // a hugin project file
  void ParseHuginControlPoints(std::string const& hugin_file,
                               std::vector<std::string> * images,
                               Eigen::MatrixXd * points);

  // Parse a file having on each line xyz coordinates
  void ParseXYZ(std::string const& xyz_file, Eigen::MatrixXd * xyz);

  // Parse a CSV file, with the first line having column names. Return
  // the results as columns in an std::map, with the column name being
  // the key. We assume all values are numbers (non-numbers are set to
  // 0).
  void ParseCSV(std::string const& csv_file,
                std::map< std::string, std::vector<double> > *cols);

  // Write the BAL format.
  bool WriteBAL(const std::string& filename, camera::CameraParameters const& camera_params,
                std::vector<std::map<int, int> > const& pid_to_cid_fid, std::vector<Eigen::Vector3d> const& pid_to_xyz,
                std::vector<Eigen::Affine3d> const& cid_to_cam_t_global,
                std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map);

void InitializeCidFidToPid(int num_cid,
                           std::vector<std::map<int, int> > const& pid_to_cid_fid,
                           std::vector<std::map<int, int> > * cid_fid_to_pid);

cv::Mat LoadImage(const std::string& filename);

  // I/O Functions for writing and reading affine solutions.
  void WriteAffineCSV(CIDPairAffineMap const& relative_affines,
                      std::string const& output_filename);
  void WriteAffineCSV(CIDAffineTupleVec const& relative_affines,
                      std::string const& output_filename);
  void ReadAffineCSV(std::string const& input_filename,
                     CIDPairAffineMap* relative_affines);
  void ReadAffineCSV(std::string const& input_filename,
                     CIDAffineTupleVec* relative_affines);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_FILE_UTILITIES_H_
