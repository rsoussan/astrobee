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
// bundler, nvm or theia maps.
SparseMap::SparseMap(bool bundler_format, std::string const& filename,
                     std::vector<std::string> const& all_image_files) {
  // these are placeholders and must be changed
  const camera::CameraParameters camera_params(Eigen::Vector2i(640, 480), Eigen::Vector2d::Constant(300),
                                               Eigen::Vector2d(320, 240));
  // TODO(rsoussan): how to get detector params???
  SetParams(detector, camera_params);
  std::string ext = ff_common::file_extension(filename);
  boost::to_lower(ext);

  if (ext == "nvm") {
    std::cout << "NVM format detected." << std::endl;

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

  } else if (bundler_format) {
    std::cout << "Bundler format detected." << std::endl;

    int num_cams = 0;

    std::ifstream is(filename.c_str());
    std::string line;
    std::getline(is, line);  // empty line
    is >> num_cams;
    cid_to_filename_ = all_image_files;
    cid_to_cam_T_global_.resize(num_cams);
    cid_to_filename_.resize(num_cams);

    for (int i = 0; i < num_cams; i++) {
      std::string line;
      std::getline(is, line);  // empty line
      std::getline(is, line);  // focal length, etc.

      // Rotation
      Eigen::Matrix3d T;
      for (int row = 0; row < T.rows(); row++) {
        for (int col = 0; col < T.cols(); col++) {
          is >> T(row, col);
        }
      }
      // This is needed for when bundler fails
      if (T.determinant() < 1e-8)
        T = Eigen::Matrix3d::Identity();

      // Translation
      Eigen::Vector3d P;
      for (int row = 0; row < P.size(); row++)
        is >> P[row];

      cid_to_cam_T_global_[i].linear() = T;  // not sure
      cid_to_cam_T_global_[i].translation() = P;
    }

    // Initialize other data expected in the map
    cid_to_keypoint_map_.resize(num_cams);
    cid_to_descriptor_map_.resize(num_cams);
  }

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




}  // namespace sparse_mapping
