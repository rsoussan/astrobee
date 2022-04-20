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

#include <sparse_mapping/utilities.h>
namespace {
std::string print_vec(double a) {
  char st[256];
  snprintf(st, sizeof(st), "%7.4f", a);
  return std::string(st);
}
std::string print_vec(Eigen::Vector3d a) {
  char st[256];
  snprintf(st, sizeof(st), "%7.4f %7.4f %7.4f", a[0], a[1], a[2]);
  return std::string(st);
}
}  // namespace

namespace sparse_mapping {
// Logic for implementing if two histogram equalization flags are compatible.
// This flag can be either 0 (false), 1 (true), or 2 (unknown). Be tolerant
// of unknown values, but intolerant when true and false are mixed.
void HistogramEqualizationCheck(int histogram_equalization1,
                                                int histogram_equalization2) {
  if ( (histogram_equalization1 == 0 && histogram_equalization2 == 1) ||
       (histogram_equalization1 == 1 && histogram_equalization2 == 0) )
    LOG(FATAL) << "Incompatible values of histogram equalization detected.";
}

bool IsBinaryDescriptor(std::string const& descriptor) {
  if (descriptor == "OPENSIFT" || descriptor == "SIFT" || descriptor == "SURF")
    return false;
  return true;
}

std::string CvMatTypeStr(cv::Mat const& Mat) {
  int type = Mat.type();
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  std::string r;
  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void ListToListMap(std::vector<std::string> const& big_list,
                                   std::vector<std::string> const& small_list,
                                   std::map<int, int> * map) {
  // Given a big list, and a smaller subset of it, for each index i in
  // the small list find the index j in the big list so that
  // small_list[i] equals big_list[j].  Define the map as map[j] = i.
  (*map).clear();

  std::map<std::string, int> str2int;
  for (size_t i = 0; i < big_list.size(); i++)
    str2int[big_list[i]] = i;

  for (size_t i = 0; i < small_list.size(); i++) {
    std::map<std::string, int>::iterator it = str2int.find(small_list[i]);
    if (it == str2int.end())
      LOG(FATAL) << "Could not query image: " << small_list[i];

    (*map)[it->second] = i;
  }
}

// Take a map. Form a map with only a subset of the images.
// Bundle adjustment will happen later.
void ExtractSubmap(std::vector<std::string> * keep_ptr,
                   sparse_mapping::SparseMap * map_ptr) {
  // Create aliases to not use pointers all the time.
  sparse_mapping::SparseMap & map = *map_ptr;
  std::vector<std::string> & keep = *keep_ptr;

  // Wipe things that we won't merge (or not yet)
  map.ClearImageDatabase();
  map.pid_to_global_t_point_.clear();
  map.cid_to_fid_to_pid_.clear();
  map.cid_to_cid_.clear();
  map.user_cid_to_keypoints_.clear();
  map.user_pid_to_feature_track_.clear();
  map.user_pid_to_global_t_point_.clear();

  // Sanity check. The images to keep must exist in the original map.
  std::map<std::string, int> image2cid;
  for (size_t cid = 0; cid < map.cid_to_filename_.size(); cid++)
    image2cid[map.cid_to_filename_[cid]] = cid;
  for (size_t cid = 0; cid < keep.size(); cid++) {
    if (image2cid.find(keep[cid]) == image2cid.end())
      LOG(WARNING) << "Could not find in the input map the image: " << keep[cid];
  }

  // To extract the submap-in place, it is simpler to reorder the images
  // to extract to be in the same order as in the map
  {
    std::set<std::string> keep_set;
    for (size_t cid = 0; cid < keep.size(); cid++)
      keep_set.insert(keep[cid]);
    std::vector<std::string> keep2;
    for (size_t cid = 0; cid < map.cid_to_filename_.size(); cid++) {
      if (keep_set.find(map.cid_to_filename_[cid]) != keep_set.end()) {
        keep2.push_back(map.cid_to_filename_[cid]);
      }
    }
    keep = keep2;
  }

  // Map each image we keep to its index
  std::map<std::string, int> keep2cid;
  for (size_t cid = 0; cid < keep.size(); cid++)
    keep2cid[keep[cid]] = cid;

  // The map from the old cid to the new cid
  std::map<int, int> cid2cid;
  for (size_t cid = 0; cid < map.cid_to_filename_.size(); cid++) {
    auto it = keep2cid.find(map.cid_to_filename_[cid]);
    if (it == keep2cid.end()) continue;  // current image is not in the final submap
    cid2cid[cid] = it->second;
  }

  // Sanity checks. All the kept images must be represented in cid2cid,
  // and the values in cid2cid must be consecutive.
  if (cid2cid.size() != keep.size() || cid2cid.empty())
    LOG(FATAL) << "Cannot extract a submap. Check your inputs.";
  for (auto it = cid2cid.begin(); it != cid2cid.end(); it++) {
    auto it2 = it; it2++;
    if (it2 == cid2cid.end()) continue;
    if (it->second + 1 != it2->second || cid2cid.begin()->second != 0 )
      LOG(FATAL) << "Cannot extract a submap. Check if the images "
                 << "you want to keep are in the same order as in the original map.";
  }

  // Over-write the data in-place. Should be safe with the checks done above.
  int num_cid = keep.size();
  for (size_t cid = 0; cid < map.cid_to_filename_.size(); cid++) {
    if (cid2cid.find(cid) == cid2cid.end()) continue;
    size_t new_cid = cid2cid[cid];
    map.cid_to_filename_[new_cid]             = map.cid_to_filename_[cid];
    map.cid_to_keypoints_[new_cid]         = map.cid_to_keypoints_[cid];
    map.cid_to_cam_T_global_[new_cid]         = map.cid_to_cam_T_global_[cid];
    map.cid_to_descriptors_[new_cid]       = map.cid_to_descriptors_[cid];
  }
  map.cid_to_filename_             .resize(num_cid);
  map.cid_to_keypoints_         .resize(num_cid);
  map.cid_to_cam_T_global_         .resize(num_cid);
  map.cid_to_descriptors_       .resize(num_cid);

  // Create new pid_to_feature_track_.
  std::vector<std::map<int, int> > pid_to_feature_track;
  std::vector<Eigen::Vector3d> pid_to_global_t_point;
  for (int pid = 0; pid < static_cast<int>(map.pid_to_feature_track_.size()); pid++) {
    auto const& cid_fid = map.pid_to_feature_track_[pid];  // alias
    std::map<int, int> cid_fid2;
    for (auto it = cid_fid.begin(); it != cid_fid.end(); it++) {
      int cid = it->first;
      if (cid2cid.find(cid) == cid2cid.end()) continue;  // not an image we want to keep
      cid_fid2[cid2cid[cid]] = it->second;
    }
    if (cid_fid2.size() <= 1) continue;  // tracks must have size at least 2
    pid_to_feature_track.push_back(cid_fid2);
    pid_to_global_t_point.push_back(map.pid_to_global_t_point_[pid]);
  }
  map.pid_to_feature_track_ = pid_to_feature_track;
  map.pid_to_global_t_point_ = pid_to_global_t_point;

  // Recreate cid_to_fid_to_pid_ from pid_to_feature_track_. This must happen
  // after the merging is complete but before using the new map.
  map.InitializeCidFidPidMap();

  LOG(INFO) << "Number of images in the extracted map: " << map.cid_to_filename_.size();
  LOG(INFO) << "Number of tracks in the extracted map: " << map.pid_to_feature_track_.size();
  // map.Save(output_map + ".extracted.map");

  return;
}

// Register a map to world coordinates from user-supplied data, or simply
// verify how well the map performs with this data.
double RegistrationOrVerification(std::vector<std::string> const& data_files,
                                bool verification,
                                sparse_mapping::SparseMap * map) {
  // Get the interest points in the images, and their positions in
  // the world coordinate system, as supplied by a user.
  // Parse and concatenate that information from multiple files.
  std::vector<std::string> images;
  Eigen::MatrixXd user_ip;
  Eigen::Matrix3Xd user_xyz;
  for (size_t file_id = 0; file_id < data_files.size(); file_id++) {
    std::string file = data_files[file_id];
    std::string ext = ff_common::file_extension(file);
    std::vector<std::string> curr_images;
    Eigen::MatrixXd curr_ip, curr_xyz;

    if (ext == "pto") {
      sparse_mapping::ParseHuginControlPoints(file, &curr_images, &curr_ip);

      int orig_num_img = images.size();

      // Append to the larger sets
      for (size_t it = 0; it < curr_images.size(); it++)
        images.push_back(curr_images[it]);

      // Append to the larger set
      int orig_num_ip = user_ip.cols();
      Eigen::MatrixXd merged_ip(curr_ip.rows(),
                                user_ip.cols() + curr_ip.cols());
      if (user_ip.cols() > 0)
        merged_ip << user_ip, curr_ip;
      else
        merged_ip << curr_ip;
      user_ip = merged_ip;
      for (int pid = orig_num_ip; pid < user_ip.cols(); pid++) {
        user_ip(0, pid) += orig_num_img;  // update the index of the left image
        user_ip(1, pid) += orig_num_img;  // update the index of the right image
      }
    } else if (ext == "txt") {
      sparse_mapping::ParseXYZ(file, &curr_xyz);

      // Append to the larger set
      Eigen::Matrix3Xd merged_xyz(curr_xyz.rows(),
                                 user_xyz.cols() + curr_xyz.cols());
      if (user_xyz.cols() > 0)
        merged_xyz << user_xyz, curr_xyz;
      else
        merged_xyz << curr_xyz;
      user_xyz = merged_xyz;
    }
  }

  int num_points = user_ip.cols();
  if (num_points != user_xyz.cols())
    LOG(FATAL) << "Could not parse an equal number of control "
               << "points and xyz coordinates. Their numbers are "
               << num_points << " vs " << user_xyz.cols() << ".\n";

  std::map<std::string, int> filename_to_cid;
  for (size_t cid = 0; cid < map->cid_to_filename_.size(); cid++)
    filename_to_cid[map->cid_to_filename_[cid]] = cid;

  // Wipe images that are missing from the map
  std::map<int, int> cid2cid;
  int good_cid = 0;
  for (size_t cid = 0; cid < images.size(); cid++) {
    std::string image = images[cid];
    if (filename_to_cid.find(image) == filename_to_cid.end()) {
      LOG(WARNING) << "Will ignore image missing from map: " << image;
      continue;
    }
    cid2cid[cid] = good_cid;
    images[good_cid] = images[cid];
    good_cid++;
  }
  images.resize(good_cid);

  // Remove points corresponding to images missing from map
  int good_pid = 0;
  for (int pid = 0; pid < num_points; pid++) {
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);
    if (cid2cid.find(id1) == cid2cid.end() || cid2cid.find(id2) == cid2cid.end()) {
      continue;
    }
    user_ip.col(good_pid) = user_ip.col(pid);
    user_xyz.col(good_pid) = user_xyz.col(pid);
    good_pid++;
  }
  user_ip.conservativeResize(Eigen::NoChange_t(), good_pid);
  user_xyz.conservativeResize(Eigen::NoChange_t(), good_pid);
  num_points = good_pid;
  for (int pid = 0; pid < num_points; pid++) {
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);
    if (cid2cid.find(id1) == cid2cid.end() || cid2cid.find(id2) == cid2cid.end())
      LOG(FATAL) << "Book-keeping failure in registration.";
    user_ip(0, pid) = cid2cid[id1];
    user_ip(1, pid) = cid2cid[id2];
  }

  // Iterate over the control points in the hugin file. Copy the
  // control points to the list of user keypoints, and create the
  // corresponding user_pid_to_feature_track_.
  map->user_cid_to_keypoints_.resize(map->cid_to_filename_.size());
  map->user_pid_to_feature_track_.resize(num_points);
  for (int pid = 0; pid < num_points; pid++) {
    // Left and right image indices
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);

    // Sanity check
    if (id1 < 0 || id2 < 0 ||
        id1 >= static_cast<int>(images.size()) ||
        id2 >= static_cast<int>(images.size()) )
      LOG(FATAL) << "Invalid image indices in the hugin file: " << id1 << ' ' << id2;

    // Find the corresponding indices in the map where these keypoints will go to
    if (filename_to_cid.find(images[id1]) == filename_to_cid.end())
      LOG(FATAL) << "File missing from map: " << images[id1];
    if (filename_to_cid.find(images[id2]) == filename_to_cid.end())
      LOG(FATAL) << "File missing from map: " << images[id2];
    int cid1 = filename_to_cid[images[id1]];
    int cid2 = filename_to_cid[images[id2]];

    // Append to the keypoints for cid1
    Eigen::Matrix<double, 2, -1> &M1 = map->user_cid_to_keypoints_[cid1];  // alias
    Eigen::Matrix<double, 2, -1> N1(M1.rows(), M1.cols()+1);
    N1 << M1, user_ip.block(2, pid, 2, 1);  // left image pixel x and pixel y
    M1.swap(N1);

    // Append to the keypoints for cid2
    Eigen::Matrix<double, 2, -1> &M2 = map->user_cid_to_keypoints_[cid2];  // alias
    Eigen::Matrix<double, 2, -1> N2(M2.rows(), M2.cols()+1);
    N2 << M2, user_ip.block(4, pid, 2, 1);  // right image pixel x and pixel y
    M2.swap(N2);

    // The corresponding user_pid_to_feature_track_
    map->user_pid_to_feature_track_[pid][cid1] = map->user_cid_to_keypoints_[cid1].cols()-1;
    map->user_pid_to_feature_track_[pid][cid2] = map->user_cid_to_keypoints_[cid2].cols()-1;
  }

  // Shift the keypoints. Undistort if necessary.
  Eigen::Vector2d output;
  for (size_t cid = 0; cid < map->user_cid_to_keypoints_.size(); cid++) {
    for (int i = 0; i < map->user_cid_to_keypoints_[cid].cols(); i++) {
      map->camera_params_.Convert<camera::DISTORTED, camera::UNDISTORTED_C>
        (map->user_cid_to_keypoints_[cid].col(i), &output);
      map->user_cid_to_keypoints_[cid].col(i) = output;
    }
  }

  // Initialize user_pid_to_global_t_point_
  map->user_pid_to_global_t_point_.resize(user_xyz.cols());
  for (int i = 0; i < user_xyz.cols(); i++)
    map->user_pid_to_global_t_point_[i] = user_xyz.col(i);

  // Triangulate to find the coordinates of the current points
  // in the virtual coordinate system
  std::vector<Eigen::Vector3d> pid_to_global_t_point;
  std::vector<std::map<int, int> > cid_to_fid_to_pid_local;
  const bool remove_invalid_points = false;  // there should be nothing to remove hopefully
  TriangulateAllPoints(remove_invalid_points,
                              map->camera_params_.GetFocalLength(),
                              map->cid_to_cam_T_global_,
                              map->user_cid_to_keypoints_,
                              &(map->user_pid_to_feature_track_),
                              &pid_to_global_t_point,
                              &cid_to_fid_to_pid_local);

  double mean_err = 0;
  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = pid_to_global_t_point[i];
    Eigen::Vector3d b = user_xyz.col(i);
    mean_err += (a-b).norm();
  }
  mean_err /= user_xyz.cols();

  if (verification) {
    std::cout << "Mean absolute error on verification: " << mean_err << " meters" << std::endl;
    std::cout << "computed xyz -- measured xyz -- error diff -- error norm (meters)" << std::endl;
  } else {
    std::cout << "Mean absolute error before registration: " << mean_err << " meters" << std::endl;
    std::cout << "Un-transformed computed xyz -- measured xyz -- error diff -- error norm (meters)" << std::endl;
  }

  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = pid_to_global_t_point[i];
    Eigen::Vector3d b = user_xyz.col(i);
    std::cout << print_vec(a) << " -- "
              << print_vec(b) << " -- "
              << print_vec(a-b) << " -- "
              << print_vec((a - b).norm())
              << std::endl;
  }

  if (verification)
    return 0;

  // Find the transform from the computed map coordinate system
  // to the world coordinate system.
  int np = pid_to_global_t_point.size();
  Eigen::Matrix3Xd in(3, np);
  for (int i = 0; i < np; i++)
    in.col(i) = pid_to_global_t_point[i];
  Eigen::Affine3d world_transform;
  sparse_mapping::Find3DAffineTransform(in, user_xyz, &world_transform);

  // Transform the map to the world coordinate system
  sparse_mapping::TransformCamerasAndPoints(world_transform,
                                            &(map->cid_to_cam_T_global_),
                                            &(map->pid_to_global_t_point_));

  mean_err = 0.0;
  for (int i = 0; i < user_xyz.cols(); i++)
    mean_err += (world_transform*in.col(i) - user_xyz.col(i)).norm();
  mean_err /= user_xyz.cols();

  // We don't use LOG(INFO) below, as it does not play well with
  // Eigen.
  double scale = pow(world_transform.linear().determinant(), 1.0 / 3.0);
  std::cout << "Transform to world coordinates." << std::endl;
  std::cout << "Rotation:\n" << world_transform.linear() / scale << std::endl;
  std::cout << "Scale:\n" << scale << std::endl;
  std::cout << "Translation:\n" << world_transform.translation().transpose()
            << std::endl;

  std::cout << "Mean absolute error after registration and before final bundle adjustment: "
            << mean_err << " meters" << std::endl;

  std::cout << "Transformed computed xyz -- measured xyz -- error diff - error norm (meters)" << std::endl;
  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = world_transform*in.col(i);
    Eigen::Vector3d b = user_xyz.col(i);
    int id1 = user_ip(0, i);
    int id2 = user_ip(1, i);

    std::cout << print_vec(a) << " -- "
              << print_vec(b) << " -- "
              << print_vec(a - b) << " -- "
              << print_vec((a - b).norm()) << " -- "
              << images[id1] << ' '
              << images[id2] << std::endl;
  }
  return scale;
}
}  // namespace sparse_mapping
