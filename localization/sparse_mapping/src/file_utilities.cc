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

#include <sparse_mapping/file_utilities.h>
namespace {
  bool is_blank(std::string const& line) {
    return (line.find_first_not_of(" \t\n\v\f\r") == std::string::npos);
  }
}

namespace sparse_mapping {
void LoadHuginControlPoints(const std::string& hugin_file, std::vector<ControlPoint>& control_points,
                            std::vector<std::string>& image_names) {
  const std::ifstream filestream(hugin_file.c_str());
  if (!filestream.good())
    LOG(FATAL) << "ParseHuginControlPoints(): Could not open hugin file: " << hugin_file;

  std::string line;
  while (getline(filestream, line)) {
    // Load image names
    if (line.find("i ") == 0) {
      const int i = line.find("n\"");
      if (i == std::string::npos)
        LOG(FATAL) << "ParseHuginControlPoints(): Invalid line: " << line;
      i += 2;
      std::string image_name;
      while (i < line.size() && line[i] != '"') {
        image_name += line[i];
        ++i;
      }
      image_names.emplace_back(image_name);
    }

    // Load control points
    if (line.find("c ") == 0) {
      // First wipe all letters
      const std::string original_line = line;
      char * const pruned_line = const_cast<char*>(line.c_str());
      for (int i = 0; i < static_cast<int>(line.size()); ++i) {
        // Wipe some extra chars
        if ( (pruned_line[i] >= 'a' && pruned_line[i] <= 'z') ||
             (pruned_line[i] >= 'A' && pruned_line[i] <= 'Z') )
          pruned_line[i] = ' ';
      }

      // Out of a line like:
      // c n0 N1 x367 y240 X144.183010710425 Y243.04008545843 t0
      // we store the numbers, 0, 1, 367, 240, 144.183010710425 243.04008545843
      // as a column.
      // The stand for left image index, right image index,
      // left image x, left image y, right image x, right image y.
      ControlPoint control_point;
      if (sscanf(pruned_line, "%d %d %lf %lf %lf %lf", &control_point.cid_left, &control_point.cid_right,
                 &control_point.keypoint_left.x(), &control_point.keypoint_left.y(), &control_point.keypoint_right.x(),
                 &control_point.keypoint_right.y()) != 6)
        LOG(FATAL) << "ParseHuginControlPoints(): Could not scan line: " << line;

      // The left and right images must be different
      if (control_point.cid_left == control_point.cid_right)
        LOG(FATAL) << "The left and right images must be distinct. "
                   << "Offending line in " << hugin_file << " is:\n"
                   << original_line << "\n";
    }
  }

  return control_points;
}

void LoadPoints(const std::string& points_file, std::vector<Eigen::Vector3d>& points) {
  const std::ifstream filestream(points_file.c_str());
  if (!filestream.good())
    LOG(FATAL) << "LoadPoints(): Could not open hugin file: " << points_file;

  std::string line;
  while (getline(filestream, line)) {
    // Ignore lines starting with comments and empty lines
    if (line.find("#") == 0 || is_blank(line)) continue;

    // Apparently sometimes empty lines show up as if of length 1
    if (line.size() == 1)
      continue;

    // Replace commas with spaces
    char * const ptr = const_cast<char*>(line.c_str());
    for (int c = 0; c < static_cast<int>(line.size()); ++c)
      if (ptr[c] == ',') ptr[c] = ' ';
    double x, y, z;
    if (sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &z) != 3)
      LOG(FATAL) << "LoadPoints(): Could not scan line: '" << line << "'\n";
    points.emplace_back(Eigen::Vector3d(x, y, z));
  }
}

void LoadControlPoints(const std::vector<std::string>& files, std::vector<ControlPoint>& control_points,
                       std::vector < std::string & image_names) {
  std::vector<Eigen::Vector3d> global_t_points;
  for (const auto& file : files) {
    const std::string ext = ff_common::file_extension(file);
    if (ext == "pto") {
      LoadHuginControlPoints(file, control_points, image_names);
    } else if (ext == "txt") {
      LoadPoints(file, global_t_points);
    }
  }

  if (control_points.size() != global_t_points.size())
    LOG(FATAL) << "Could not parse an equal number of control "
               << "points and xyz coordinates. Their numbers are "
               << control_points.size() << " vs " << global_t_points.size() << ".\n";

  for (int i = 0; i < static_cast<int>(control_points.size()); ++i) {
    control_points[i].global_t_point = global_t_points[i];
  }
}

cv::Mat LoadImage(const std::string& filename) {
  const cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
  if (image.rows == 0 || image.cols == 0)
    LOG(FATAL) << "Found empty image in file: " << filename;
  return image;
}
}  // namespace sparse_mapping
