/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http:  //  www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <sparse_mapping/third_part_map_utilities.h>

namespace sparse_mapping {
// TODO(rsoussan): update code
// Form a sparse map by reading a text file from disk. This is for comparing
// nvm or theia maps.
SparseMap::SparseMap(const std::string& filename,
                     const std::vector<std::string>& all_image_files) {
  // these are placeholders and must be changed
  const camera::CameraParameters camera_params(Eigen::Vector2i(640, 480), Eigen::Vector2d::Constant(300),
                                               Eigen::Vector2d(320, 240));
  // TODO(rsoussan): how to get detector params???
  SetParams(detector, camera_params);
  std::string ext = ff_common::file_extension(filename);
  boost::to_lower(ext);

    sparse_mapping::ReadNVM(filename, &cid_to_keypoint_map_, &cid_to_filename_, &pid_to_cid_fid_, &pid_to_xyz_,
                            &cid_to_cam_T_global_);

    // Descriptors are not saved, so let them be empty
    cid_to_descriptor_map_.resize(cid_to_keypoint_map_.size());

    // When the NVM file is created by Theia, it saves the images
    // without a path, in random order, and it may not have used up
    // all the images, so need to adjust for that.

    std::map<std::string, std::string> base_to_full_path;
    std::map<int, std::string> orig_order;
    for (size_t it = 0; it < all_image_files.size(); it++) {
      std::string image = all_image_files[it];
      std::string base = boost::filesystem::path(image).filename().string();
      if (base_to_full_path.find(base) != base_to_full_path.end())
        LOG(FATAL) << "Duplicate image: " << base << std::endl;
      base_to_full_path[base] = image;
      orig_order[it] = base;
    }

    // Find the permutation which will tell how to reorder the images
    // in the nvm file to be in the original order.  This must happen
    // before we change cid_to_filename_ below.
    std::map<int, int> old_cid_to_new_cid;
    std::map<std::string, int> base2cid;
    for (size_t it = 0; it < cid_to_filename_.size(); it++) base2cid[cid_to_filename_[it]] = it;
    int new_cid = 0;
    for (auto order_it = orig_order.begin(); order_it != orig_order.end() ; order_it++) {
      auto base_it = base2cid.find(order_it->second);
      if (base_it == base2cid.end()) continue;  // Not all input images may be present in the map

      int old_cid = base_it->second;
      old_cid_to_new_cid[old_cid] = new_cid;
      new_cid++;
    }

    // Map the theia images to the actual image paths
    for (size_t it = 0; it < cid_to_filename_.size(); it++) {
      std::string base = cid_to_filename_[it];
      auto map_it = base_to_full_path.find(base);
      if (map_it == base_to_full_path.end())
        LOG(FATAL) << "The input file list is missing the nvm map image: " << base << std::endl;
      cid_to_filename_[it] = map_it->second;
    }

    // Apply the permutation
    reorderMap(old_cid_to_new_cid);

  // Initialize this convenient mapping
  InitializeCidFidToPid();
}

// Reorder the images in the map and the rest of the data accordingly
void SparseMap::reorderMap(std::map<int, int> const& old_cid_to_new_cid) {
  int num_cid = cid_to_filename_.size();

  // Sanity checks
  if (old_cid_to_new_cid.size() != cid_to_filename_.size())
    LOG(FATAL) << "Wrong size of the permutation in SparseMap::reorderMap().";
  for (auto it = old_cid_to_new_cid.begin(); it != old_cid_to_new_cid.end(); it++) {
    int new_cid = it->second;
    if (new_cid >= num_cid) LOG(FATAL) << "Out of bounds in the permutation in SparseMap::reorderMap().";
  }

  // Wipe things that we won't reorder
  // TODO(rsoussan): Make clear function to do this
  ClearImageDatabase();
  cid_to_cid_.clear();
  user_cid_to_keypoint_map_.clear();
  user_pid_to_cid_fid_.clear();
  user_pid_to_xyz_.clear();
  cid_fid_to_pid_.clear();  // Will recreate this later

  // TODO(rsoussan): Avoid all this by creating a new sparse map database and assigning it to the sparse map
  // Must create temporary structures
  std::vector<std::string>        new_cid_to_filename(num_cid);
  std::vector<Eigen::Matrix2Xd>   new_cid_to_keypoint_map(num_cid);
  std::vector<Eigen::Affine3d>    new_cid_to_cam_T_global(num_cid);
  std::vector<cv::Mat>            new_cid_to_descriptor_map(num_cid);
  std::vector<std::map<int, int>> new_pid_to_cid_fid(pid_to_cid_fid_.size());

  // Note that pid_to_xyz_ is not changed by this reordering

  // Copy the data in new order
  for (int old_cid = 0; old_cid < num_cid; old_cid++) {
    auto it = old_cid_to_new_cid.find(old_cid);
    if (it == old_cid_to_new_cid.end())
      LOG(FATAL) << "Cannot find desired index in permutation in SparseMap::reorderMap().";

    int new_cid = it->second;

    new_cid_to_filename[new_cid] = cid_to_filename_[old_cid];
    new_cid_to_keypoint_map[new_cid] = cid_to_keypoint_map_[old_cid];
    new_cid_to_cam_T_global[new_cid] = cid_to_cam_T_global_[old_cid];
    new_cid_to_descriptor_map[new_cid] = cid_to_descriptor_map_[old_cid];
  }

  // pid_to_cid_fid needs special treatment
  for (size_t pid = 0; pid < pid_to_cid_fid_.size(); pid++) {
    auto const& cid_fid = pid_to_cid_fid_[pid];  // alias

    std::map<int, int> new_cid_fid;
    for (auto cid_fid_it = cid_fid.begin(); cid_fid_it != cid_fid.end(); cid_fid_it++) {
      int old_cid = cid_fid_it->first;
      auto cid_it = old_cid_to_new_cid.find(old_cid);
      if (cid_it == old_cid_to_new_cid.end())
        LOG(FATAL) << "Bookkeeping error in SparseMap::reorderMap()";

      int new_cid = cid_it->second;
      new_cid_fid[new_cid] = cid_fid_it->second;
    }

    new_pid_to_cid_fid[pid] = new_cid_fid;
  }

  // Swap in the new values
  cid_to_filename_.swap(new_cid_to_filename);
  cid_to_keypoint_map_.swap(new_cid_to_keypoint_map);
  cid_to_cam_T_global_.swap(new_cid_to_cam_T_global);
  cid_to_descriptor_map_.swap(new_cid_to_descriptor_map);
  pid_to_cid_fid_.swap(new_pid_to_cid_fid);

  // Recreate cid_fid_to_pid_ from pid_to_cid_fid_.
  InitializeCidFidToPid();
}

// Writes the NVM control network format.
void WriteNVM(std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
                              std::vector<std::string> const& cid_to_filename,
                              std::vector<std::map<int, int> > const& pid_to_cid_fid,
                              std::vector<Eigen::Vector3d> const& pid_to_xyz,
                              std::vector<Eigen::Affine3d> const&
                              cid_to_cam_t_global,
                              double focal_length,
                              std::string const& output_filename) {
  std::fstream f(output_filename, std::ios::out);
  f << "NVM_V3\n";

  CHECK(cid_to_filename.size() == cid_to_keypoint_map.size())
    << "Unequal number of filenames and keypoints";
  CHECK(pid_to_cid_fid.size() == pid_to_xyz.size())
    << "Unequal number of pid_to_cid_fid and xyz measurements";
  CHECK(cid_to_filename.size() == cid_to_cam_t_global.size())
    << "Unequal number of filename and camera transforms";

  // Write camera information
  f << cid_to_filename.size() << std::endl;
  for (size_t cid = 0; cid < cid_to_filename.size(); cid++) {
    // Decompose cam_t_global so that we can write it into a WXY
    // quaternion and an XYZ camera position
    Eigen::Quaterniond q(cid_to_cam_t_global[cid].rotation());
    Eigen::Vector3d t(cid_to_cam_t_global[cid].translation());
    Eigen::Vector3d camera_center =
      - cid_to_cam_t_global[cid].rotation().inverse() * t;

    // The NVM format is a little crazy. When using a quaternion, we
    // write the camera center instead of the t from camera_t_global.
    f << cid_to_filename[cid] << " " << focal_length
      << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "
      << camera_center[0] << " " << camera_center[1] << " "
      << camera_center[2] << " " << "0.9 0\n";
  }

  // Write the number of points
  f << pid_to_cid_fid.size() << std::endl;

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    f << pid_to_xyz[pid][0] << " " << pid_to_xyz[pid][1] << " "
      << pid_to_xyz[pid][2] << " 0 0 0 "
      << pid_to_cid_fid[pid].size();

    CHECK(pid_to_cid_fid[pid].size() > 1)
      << "PID " << pid << " has " << pid_to_cid_fid[pid].size() << " measurements";

    for (std::map<int, int>::const_iterator it = pid_to_cid_fid[pid].begin();
         it != pid_to_cid_fid[pid].end(); it++) {
      f << " " << it->first << " " << it->second << " "
        << cid_to_keypoint_map[it->first].col(it->second)[0] << " "
        << cid_to_keypoint_map[it->first].col(it->second)[1];
    }
    f << std::endl;
  }

  // Close the file
  f.flush();
  f.close();
}

// Reads the NVM control network format.
void ReadNVM(std::string const& input_filename,
                             std::vector<Eigen::Matrix2Xd > * cid_to_keypoint_map,
                             std::vector<std::string> * cid_to_filename,
                             std::vector<std::map<int, int> > * pid_to_cid_fid,
                             std::vector<Eigen::Vector3d> * pid_to_xyz,
                             std::vector<Eigen::Affine3d> *
                             cid_to_cam_t_global) {
  std::ifstream f(input_filename, std::ios::in);
  std::string token;
  std::getline(f, token);

  // Assert that we start with our NVM token
  if (token.compare(0, 6, "NVM_V3") != 0) {
    LOG(FATAL) << "File doesn't start with NVM token";
  }

  // Read the number of cameras
  ptrdiff_t number_of_cid;
  f >> number_of_cid;
  if (number_of_cid < 1) {
    LOG(FATAL) << "NVM file is missing cameras";
  }

  // Resize all our structures to support the number of cameras we now expect
  cid_to_keypoint_map->resize(number_of_cid);
  cid_to_filename->resize(number_of_cid);
  cid_to_cam_t_global->resize(number_of_cid);
  for (ptrdiff_t cid = 0; cid < number_of_cid; cid++) {
    // Clear keypoints from map. We'll read these in shortly
    cid_to_keypoint_map->at(cid).resize(Eigen::NoChange_t(), 2);

    // Read the line that contains camera information
    double focal, dist1, dist2;
    Eigen::Quaterniond q;
    Eigen::Vector3d c;
    f >> token >> focal;
    f >> q.w() >> q.x() >> q.y() >> q.z();
    f >> c[0] >> c[1] >> c[2] >> dist1 >> dist2;
    cid_to_filename->at(cid) = token;

    // Solve for t, which is part of the affine transform
    Eigen::Matrix3d r = q.matrix();
    cid_to_cam_t_global->at(cid).linear() = r;
    cid_to_cam_t_global->at(cid).translation() = -r * c;
  }

  // Read the number of points
  ptrdiff_t number_of_pid;
  f >> number_of_pid;
  if (number_of_pid < 1) {
    LOG(FATAL) << "The NVM file has no triangulated points.";
  }

  // Read the point
  pid_to_cid_fid->resize(number_of_pid);
  pid_to_xyz->resize(number_of_pid);
  Eigen::Vector3d xyz;
  Eigen::Vector3i color;
  Eigen::Vector2d pt;
  ptrdiff_t cid, fid;
  for (ptrdiff_t pid = 0; pid < number_of_pid; pid++) {
    pid_to_cid_fid->at(pid).clear();

    ptrdiff_t number_of_measures;
    f >> xyz[0] >> xyz[1] >> xyz[2] >>
      color[0] >> color[1] >> color[2] >> number_of_measures;
    pid_to_xyz->at(pid) = xyz;
    for (ptrdiff_t m = 0; m < number_of_measures; m++) {
      f >> cid >> fid >> pt[0] >> pt[1];

      pid_to_cid_fid->at(pid)[cid] = fid;

      if (cid_to_keypoint_map->at(cid).cols() <= fid) {
        cid_to_keypoint_map->at(cid).conservativeResize(Eigen::NoChange_t(), fid + 1);
      }
      cid_to_keypoint_map->at(cid).col(fid) = pt;
    }

    if (!f.good())
      LOG(FATAL) << "Unable to correctly read PID: " << pid;
  }
}
}  // namespace sparse_mapping
