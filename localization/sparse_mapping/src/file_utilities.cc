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
  // A little helper function
  bool is_blank(std::string const& line) {
    return (line.find_first_not_of(" \t\n\v\f\r") == std::string::npos);
  }
}

namespace sparse_mapping {
std::vector<ControlPoints> LoadHuginControlPoints(const std::string& hugin_file) {
  const std::ifstream filestream(hugin_file.c_str());
  if (!filestream.good())
    LOG(FATAL) << "ParseHuginControlPoints(): Could not open hugin file: " << hugin_file;

  int num_points = 0;
  std::string line;
  std::vector<ControlPoints> control_points;
  std::vector<std::string> image_names;
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
      const std::string orig_line = line;
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
                   << orig_line << "\n";
    }
  }

  return control_points;
}

std::vector<Eigen::Vector3d> LoadPoints(const std::string& points_file) {
  std::vector<Eigen::Vector3d> points;

  const std::ifstream filestream(points_file.c_str());
  if (!filestream.good())
    LOG(FATAL) << "LoadPoints(): Could not open hugin file: " << points_file;

  int num_points = 0;
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
  return points;
}

cv::Mat LoadImage(const std::string& filename) {
  const cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
  if (image.rows == 0 || image.cols == 0)
    LOG(FATAL) << "Found empty image in file: " << filename;
  return image;
}

void LoadControlPoints(const std::vector<std::string>& files, std::vector<std::string>& image_filenames,
                       std::vector<Eigen::Matrix>& user_ip, user_xyz) {
  for (const auto& file : files) {
    const std::string ext = ff_common::file_extension(file);
    std::vector<std::string> curr_images;
    std::vector<Eigen::Vector3d> global_t_points;
    Eigen::MatrixXd curr_ip;

    if (ext == "pto") {
      ParseHuginControlPoints(file, &curr_images, &curr_ip);
      const int orig_num_img = images.size();

      // Append to the larger sets
      for (int i = 0; i < curr_images.size(); ++i)
        images.push_back(curr_images[i]);

      // Append to the larger set
      int orig_num_ip = user_ip.cols();
      Eigen::MatrixXd merged_ip(curr_ip.rows(),
                                user_ip.cols() + curr_ip.cols());
      if (user_ip.cols() > 0)
        merged_ip << user_ip, curr_ip;
      else
        merged_ip << curr_ip;
      user_ip = merged_ip;
      for (int pid = orig_num_ip; pid < user_ip.cols(); ++pid) {
        user_ip(0, pid) += orig_num_img;  // update the index of the left image
        user_ip(1, pid) += orig_num_img;  // update the index of the right image
      }
    } else if (ext == "txt") {
      const auto points = LoadPoints(file);
      global_t_points.insert(global_t_points.begin(), points.begin(), points.end());
    }
  }

  const int num_points = user_ip.cols();
  if (num_points != user_xyz.cols())
    LOG(FATAL) << "Could not parse an equal number of control "
               << "points and xyz coordinates. Their numbers are "
               << num_points << " vs " << user_xyz.cols() << ".\n";
}
}  // namespace sparse_mapping
