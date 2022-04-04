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

#include <sparse_mapping/bundle_adjustment_utilities.h>
#include <sparse_mapping/estimate_pose_utilities.h>
#include <sparse_mapping/ransac.h>
#include <sparse_mapping/sparse_map.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/core/core.hpp>

#include <set>
#include <vector>

DEFINE_bool(skip_filtering, false,
            "Skip filtering of outliers after bundle adjustment.");
DEFINE_double(reproj_thresh, 5.0,
              "Filter points with re-projection error higher than this.");

// bundle adjustment phase parameters
DEFINE_int32(max_num_iterations, 1000,
             "Maximum number of iterations for bundle adjustment solver.");
DEFINE_int32(num_ba_passes, 5,
             "How many times to run bundle adjustment, removing outliers each time.");
DEFINE_string(cost_function, "Cauchy",
              "Choose a bundle adjustment cost function from: Cauchy, PseudoHuber, Huber, L1, L2.");
DEFINE_double(cost_function_threshold, 2.0,
              "Threshold to use with some cost functions, e.g., Cauchy.");
DEFINE_int32(first_ba_index, 0,
             "Vary only cameras starting with this index during bundle adjustment.");
DEFINE_int32(last_ba_index, std::numeric_limits<int>::max(),
             "Vary only cameras ending with this index during bundle adjustment.");

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
void BundleAdjust(bool fix_all_cameras, sparse_mapping::SparseMap * map,
                  std::set<int> const& fixed_cameras) {
  for (int i = 0; i < FLAGS_num_ba_passes; i++) {
    LOG(INFO) << "Beginning bundle adjustment, pass: " << i << ".\n";

    // perform bundle adjustment
    ceres::Solver::Options options;
    // options.linear_solver_type = ceres::SPARSE_SCHUR; // Need to be building SuiteSparse
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    // What should the preconditioner be?
    options.num_threads = FLAGS_num_threads;
    options.max_num_iterations = FLAGS_max_num_iterations;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::LossFunction* loss = sparse_mapping::GetLossFunction(FLAGS_cost_function,
                                                                FLAGS_cost_function_threshold);
    sparse_mapping::BundleAdjustment(map, loss, options, &summary,
                                     FLAGS_first_ba_index, FLAGS_last_ba_index,
                                     fix_all_cameras, fixed_cameras);

    LOG(INFO) << summary.FullReport() << "\n";
    LOG(INFO) << "Starting average reprojection error: "
              << summary.initial_cost / map->GetNumObservations();
    LOG(INFO) << "Final average reprojection error:    "
              << summary.final_cost / map->GetNumObservations();
  }
}

void BundleAdjustment(sparse_mapping::SparseMap * s,
                      ceres::LossFunction* loss,
                      const ceres::Solver::Options & options,
                      ceres::Solver::Summary* summary,
                      int first, int last, bool fix_all_cameras,
                      std::set<int> const& fixed_cameras) {
  sparse_mapping::BundleAdjust(s->pid_to_cid_fid_, s->cid_to_keypoint_map_,
                               s->camera_params_.GetFocalLength(), &(s->cid_to_cam_t_global_),
                               &(s->pid_to_xyz_),
                               s->user_pid_to_cid_fid_, s->user_cid_to_keypoint_map_,
                               &(s->user_pid_to_xyz_),
                               loss, options, summary, first, last, fix_all_cameras,
                               fixed_cameras);

  // First do BA, and only afterwards remove outliers.
  if (!FLAGS_skip_filtering) {
    FilterPID(FLAGS_reproj_thresh,  s->camera_params_, s->cid_to_cam_t_global_,
              s->cid_to_keypoint_map_, &(s->pid_to_cid_fid_), &(s->pid_to_xyz_));
    s->InitializeCidFidToPid();
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
  map.pid_to_xyz_.clear();
  map.cid_fid_to_pid_.clear();
  map.cid_to_cid_.clear();
  map.user_cid_to_keypoint_map_.clear();
  map.user_pid_to_cid_fid_.clear();
  map.user_pid_to_xyz_.clear();

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
    map.cid_to_keypoint_map_[new_cid]         = map.cid_to_keypoint_map_[cid];
    map.cid_to_cam_t_global_[new_cid]         = map.cid_to_cam_t_global_[cid];
    map.cid_to_descriptor_map_[new_cid]       = map.cid_to_descriptor_map_[cid];
  }
  map.cid_to_filename_             .resize(num_cid);
  map.cid_to_keypoint_map_         .resize(num_cid);
  map.cid_to_cam_t_global_         .resize(num_cid);
  map.cid_to_descriptor_map_       .resize(num_cid);

  // Create new pid_to_cid_fid_.
  std::vector<std::map<int, int> > pid_to_cid_fid;
  std::vector<Eigen::Vector3d> pid_to_xyz;
  for (int pid = 0; pid < static_cast<int>(map.pid_to_cid_fid_.size()); pid++) {
    auto const& cid_fid = map.pid_to_cid_fid_[pid];  // alias
    std::map<int, int> cid_fid2;
    for (auto it = cid_fid.begin(); it != cid_fid.end(); it++) {
      int cid = it->first;
      if (cid2cid.find(cid) == cid2cid.end()) continue;  // not an image we want to keep
      cid_fid2[cid2cid[cid]] = it->second;
    }
    if (cid_fid2.size() <= 1) continue;  // tracks must have size at least 2
    pid_to_cid_fid.push_back(cid_fid2);
    pid_to_xyz.push_back(map.pid_to_xyz_[pid]);
  }
  map.pid_to_cid_fid_ = pid_to_cid_fid;
  map.pid_to_xyz_ = pid_to_xyz;

  // Recreate cid_fid_to_pid_ from pid_to_cid_fid_. This must happen
  // after the merging is complete but before using the new map.
  map.InitializeCidFidToPid();

  LOG(INFO) << "Number of images in the extracted map: " << map.cid_to_filename_.size();
  LOG(INFO) << "Number of tracks in the extracted map: " << map.pid_to_cid_fid_.size();
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
  // corresponding user_pid_to_cid_fid_.
  map->user_cid_to_keypoint_map_.resize(map->cid_to_filename_.size());
  map->user_pid_to_cid_fid_.resize(num_points);
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
    Eigen::Matrix<double, 2, -1> &M1 = map->user_cid_to_keypoint_map_[cid1];  // alias
    Eigen::Matrix<double, 2, -1> N1(M1.rows(), M1.cols()+1);
    N1 << M1, user_ip.block(2, pid, 2, 1);  // left image pixel x and pixel y
    M1.swap(N1);

    // Append to the keypoints for cid2
    Eigen::Matrix<double, 2, -1> &M2 = map->user_cid_to_keypoint_map_[cid2];  // alias
    Eigen::Matrix<double, 2, -1> N2(M2.rows(), M2.cols()+1);
    N2 << M2, user_ip.block(4, pid, 2, 1);  // right image pixel x and pixel y
    M2.swap(N2);

    // The corresponding user_pid_to_cid_fid_
    map->user_pid_to_cid_fid_[pid][cid1] = map->user_cid_to_keypoint_map_[cid1].cols()-1;
    map->user_pid_to_cid_fid_[pid][cid2] = map->user_cid_to_keypoint_map_[cid2].cols()-1;
  }

  // Shift the keypoints. Undistort if necessary.
  Eigen::Vector2d output;
  for (size_t cid = 0; cid < map->user_cid_to_keypoint_map_.size(); cid++) {
    for (int i = 0; i < map->user_cid_to_keypoint_map_[cid].cols(); i++) {
      map->camera_params_.Convert<camera::DISTORTED, camera::UNDISTORTED_C>
        (map->user_cid_to_keypoint_map_[cid].col(i), &output);
      map->user_cid_to_keypoint_map_[cid].col(i) = output;
    }
  }

  // Initialize user_pid_to_xyz_
  map->user_pid_to_xyz_.resize(user_xyz.cols());
  for (int i = 0; i < user_xyz.cols(); i++)
    map->user_pid_to_xyz_[i] = user_xyz.col(i);

  // Triangulate to find the coordinates of the current points
  // in the virtual coordinate system
  std::vector<Eigen::Vector3d> pid_to_xyz;
  std::vector<std::map<int, int> > cid_fid_to_pid_local;
  bool rm_invalid_xyz = false;  // there should be nothing to remove hopefully
  sparse_mapping::Triangulate(rm_invalid_xyz,
                              map->camera_params_.GetFocalLength(),
                              map->cid_to_cam_t_global_,
                              map->user_cid_to_keypoint_map_,
                              &(map->user_pid_to_cid_fid_),
                              &pid_to_xyz,
                              &cid_fid_to_pid_local);

  double mean_err = 0;
  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = pid_to_xyz[i];
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
    Eigen::Vector3d a = pid_to_xyz[i];
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
  int np = pid_to_xyz.size();
  Eigen::Matrix3Xd in(3, np);
  for (int i = 0; i < np; i++)
    in.col(i) = pid_to_xyz[i];
  Eigen::Affine3d world_transform;
  sparse_mapping::Find3DAffineTransform(in, user_xyz, &world_transform);

  // Transform the map to the world coordinate system
  sparse_mapping::TransformCamerasAndPoints(world_transform,
                                            &(map->cid_to_cam_t_global_),
                                            &(map->pid_to_xyz_));

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

// TODO(rsoussan): Pass sparse map database instead of all these individual params?
  // Add another function that takes sparse map database!!
void BundleAdjust(std::vector<std::map<int, int> > const& pid_to_cid_fid,
                  std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map, double focal_length,
                  std::vector<Eigen::Affine3d>* cid_to_cam_t_global, std::vector<Eigen::Vector3d>* pid_to_xyz,
                  std::vector<std::map<int, int> > const& user_pid_to_cid_fid,
                  std::vector<Eigen::Matrix2Xd> const& user_cid_to_keypoint_map,
                  std::vector<Eigen::Vector3d>* user_pid_to_xyz, ceres::LossFunction* loss,
                  ceres::Solver::Options const& options, ceres::Solver::Summary* summary, int first, int last,
                  bool fix_all_cameras, std::set<int> const& fixed_cameras) {
  // Perform bundle adjustment. Keep fixed all cameras with cid
  // not within [first, last] and all xyz points which project only
  // onto fixed cameras.

  // If provided, use user-set registration points in the second pass.

  // Allocate space for the angle axis representation of rotation
  std::vector<double> camera_aa_storage(3 * cid_to_cam_t_global->size());
  for (size_t cid = 0; cid < cid_to_cam_t_global->size(); cid++) {
    Eigen::Map<Eigen::Vector3d> aa_storage(camera_aa_storage.data() + 3 * cid);
    Eigen::Vector3d vec;
    camera::RotationToRodrigues(cid_to_cam_t_global->at(cid).linear(),
                               &vec);
    aa_storage = vec;
  }

  // Build problem
  ceres::Problem problem;

  // Ideally the block inside of the loop below must be a function call,
  // but the compiler does not handle that correctly with ceres.
  // So do this by changing where things are pointing.

  int num_passes = 1;
  if (!user_pid_to_xyz->empty()) num_passes = 2;  // A second pass using control points

  for (int pass = 0; pass < num_passes; pass++) {
    std::vector<std::map<int, int> > const * p_pid_to_cid_fid;
    std::vector<Eigen::Matrix2Xd >   const * p_cid_to_keypoint_map;
    std::vector<Eigen::Vector3d>           * p_pid_to_xyz;
    ceres::LossFunction * local_loss;
    if (pass == 0) {
      local_loss            = loss;  // outside-supplied loss
      p_pid_to_cid_fid      = &pid_to_cid_fid;
      p_cid_to_keypoint_map = &cid_to_keypoint_map;
      p_pid_to_xyz          = pid_to_xyz;
    } else {
      local_loss            = NULL;  // l2, as user-supplied data is reliable
      p_pid_to_cid_fid      = &user_pid_to_cid_fid;
      p_cid_to_keypoint_map = &user_cid_to_keypoint_map;
      p_pid_to_xyz          = user_pid_to_xyz;
    }

    for (size_t pid = 0; pid < p_pid_to_xyz->size(); pid++) {
      if ((*p_pid_to_cid_fid)[pid].size() < 2)
        LOG(FATAL) << "Found a track of size < 2.";

      // Don't vary points which project only into cameras which we don't vary.
      bool fix_pid = true;
      for (std::map<int, int>::value_type const& cid_fid : (*p_pid_to_cid_fid)[pid]) {
        if (cid_fid.first >= first && cid_fid.first <= last)
          fix_pid = false;
      }

      for (std::map<int, int>::value_type const& cid_fid : (*p_pid_to_cid_fid)[pid]) {
        ceres::CostFunction* cost_function =
          ReprojectionError::Create((*p_cid_to_keypoint_map)[cid_fid.first].col(cid_fid.second));

        problem.AddResidualBlock(cost_function,
                                 local_loss,
                                 &cid_to_cam_t_global->at(cid_fid.first).translation()[0],
                                 &camera_aa_storage[3 * cid_fid.first],
                                 &p_pid_to_xyz->at(pid)[0],
                                 &focal_length);

        if (fix_all_cameras || (cid_fid.first < first || cid_fid.first > last) ||
            fixed_cameras.find(cid_fid.first) != fixed_cameras.end()) {
          problem.SetParameterBlockConstant(&cid_to_cam_t_global->at(cid_fid.first).translation()[0]);
          problem.SetParameterBlockConstant(&camera_aa_storage[3 * cid_fid.first]);
        }
      }
      if (fix_pid || pass == 1) {
        // Fix pids which don't project in cameras that are floated.
        // Also, must not float points given by the user, those are measurements
        // we are supposed to reference ourselves against, and floating
        // them can make us lose the real world scale.
        problem.SetParameterBlockConstant(&p_pid_to_xyz->at(pid)[0]);
      }
    }
    problem.SetParameterBlockConstant(&focal_length);
  }

  // Solve the problem
  ceres::Solve(options, &problem, summary);

  // Write the rotations back to the transform
  for (size_t cid = 0; cid < cid_to_cam_t_global->size(); cid++) {
    Eigen::Map<Eigen::Vector3d> aa_storage
      (camera_aa_storage.data() + 3 * cid);
    Eigen::Matrix3d r;
    camera::RodriguesToRotation(aa_storage, &r);
    cid_to_cam_t_global->at(cid).linear() = r;
  }
}

// This is a very specialized function
void BundleAdjustSmallSet(std::vector<Eigen::Matrix2Xd> const& features_n,
                          double focal_length,
                          std::vector<Eigen::Affine3d> * cam_t_global_n,
                          Eigen::Matrix3Xd * pid_to_xyz,
                          ceres::LossFunction * loss,
                          ceres::Solver::Options const& options,
                          ceres::Solver::Summary * summary) {
  CHECK(cam_t_global_n) << "Variable cam_t_global_n needs to be defined";
  CHECK(cam_t_global_n->size() == features_n.size())
    << "Variables features_n and cam_t_global_n need to agree on the number of cameras";
  CHECK(cam_t_global_n->size() > 1) << "Bundle adjust needs at least 2 or more cameras";
  CHECK(pid_to_xyz->cols() == features_n[0].cols())
    << "There should be an equal amount of XYZ points as there are feature observations";
  for (size_t i = 1; i < features_n.size(); i++) {
    CHECK(features_n[0].cols() == features_n[i].cols())
      << "The same amount of features should be seen in all cameras";
  }

  const size_t n_cameras = features_n.size();

  // Allocate space for the angle axis representation of rotation
  std::vector<Eigen::Vector3d> aa(n_cameras);
  for (size_t cid = 0; cid < n_cameras; cid++) {
    camera::RotationToRodrigues(cam_t_global_n->at(cid).linear(), &aa[cid]);
  }

  // Build the problem
  ceres::Problem problem;
  for (ptrdiff_t pid = 0; pid < pid_to_xyz->cols(); pid++) {
    for (size_t cid = 0; cid < n_cameras; cid++) {
      ceres::CostFunction* cost_function = ReprojectionError::Create(features_n[cid].col(pid));
      problem.AddResidualBlock(cost_function, loss,
                               &cam_t_global_n->at(cid).translation()[0],
                               &aa.at(cid)[0],
                               &pid_to_xyz->col(pid)[0],
                               &focal_length);
    }
  }
  problem.SetParameterBlockConstant(&focal_length);

  // Solve the problem
  ceres::Solve(options, &problem, summary);

  // Write the rotations back to the transform
  Eigen::Matrix3d r;
  for (size_t cid = 0; cid < n_cameras; cid++) {
    camera::RodriguesToRotation(aa[cid], &r);
    cam_t_global_n->at(cid).linear() = r;
  }
}

}  // namespace sparse_mapping
