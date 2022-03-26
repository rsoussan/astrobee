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
  // The name of the matches file
  std::string MatchesFile(std::string const& map_file);

  // The name of the essential file
  std::string EssentialFile(std::string const& map_file);

  // Extract control points and the images they correspond to from
  // a hugin project file
  void ParseHuginControlPoints(std::string const& hugin_file,
                               std::vector<std::string> * images,
                               Eigen::MatrixXd * points);

  // Parse a file having on each line xyz coordinates
  void ParseXYZ(std::string const& xyz_file, Eigen::MatrixXd * xyz);

void InitializeCidFidToPid(int num_cid,
                           std::vector<std::map<int, int> > const& pid_to_cid_fid,
                           std::vector<std::map<int, int> > * cid_fid_to_pid);

cv::Mat LoadImage(const std::string& filename);
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_FILE_UTILITIES_H_
