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
#include <sparse_mapping/utilities.h>

namespace sparse_mapping {
double RegistrationOrVerification(const std::vector<std::string>& files) {
  std::vector<ControlPoint> control_points;
  std::vector<std::string> image_names;
  LoadControlPoints(files, control_points, image_names);

  // Remove control points which contain images not contained in the map
  for (auto control_point = control_points.begin(); control_point != control_points.end(); ++control_point) {
    if (!ContainsImage(image_names[control_point->cid_left]) || !ContainsImage(image_names[control_point->cid_right])) {
      control_point = control_points.erase(control_point);
      continue;
    } else {
      ++control_point;
    }
  }

  // TODO(rsoussan): Add function to do this in database?? (undistort first in sparse map call)
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
    std::cout << a.matrix() << " -- "
              << b.matrix() << " -- "
              << (a-b).matrix() << " -- "
              << (a - b).norm()
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

    std::cout << a.matrix() << " -- "
              << b.matrix() << " -- "
              << (a - b).matrix() << " -- "
              << (a - b).norm() << " -- "
              << images[id1] << ' '
              << images[id2] << std::endl;
  }
  return scale;
}
}  // namespace sparse_mapping
