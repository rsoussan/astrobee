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
  // TODO(rsoussan): undistort control points first!!
  AddControlPoints(control_points);

  // Triangulate to find the coordinates of the current points
  // in the virtual coordinate system
  std::vector<Eigen::Vector3d> pid_to_global_t_point;
  std::vector<std::map<int, int> > cid_to_fid_to_pid_local;
  const bool remove_invalid_points = false;  // there should be nothing to remove hopefully
  TriangulateAllPoints(remove_invalid_points,
                              map->camera_params_.GetFocalLength(),
                              map->cid_to_cam_T_global_,
                              map->control_point_cid_to_keypoints_,
                              &(map->control_point_pid_to_feature_track_),
                              &pid_to_global_t_point,
                              &cid_to_fid_to_pid_local);

  double mean_err = 0;
  for (int i = 0; i < control_point_xyz.cols(); i++) {
    Eigen::Vector3d a = pid_to_global_t_point[i];
    Eigen::Vector3d b = control_point_xyz.col(i);
    mean_err += (a-b).norm();
  }
  mean_err /= control_point_xyz.cols();

  if (verification) {
    std::cout << "Mean absolute error on verification: " << mean_err << " meters" << std::endl;
    std::cout << "computed xyz -- measured xyz -- error diff -- error norm (meters)" << std::endl;
  } else {
    std::cout << "Mean absolute error before registration: " << mean_err << " meters" << std::endl;
    std::cout << "Un-transformed computed xyz -- measured xyz -- error diff -- error norm (meters)" << std::endl;
  }

  for (int i = 0; i < control_point_xyz.cols(); i++) {
    Eigen::Vector3d a = pid_to_global_t_point[i];
    Eigen::Vector3d b = control_point_xyz.col(i);
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
  sparse_mapping::Find3DAffineTransform(in, control_point_xyz, &world_transform);

  // Transform the map to the world coordinate system
  sparse_mapping::TransformCamerasAndPoints(world_transform,
                                            &(map->cid_to_cam_T_global_),
                                            &(map->pid_to_global_t_point_));

  mean_err = 0.0;
  for (int i = 0; i < control_point_xyz.cols(); i++)
    mean_err += (world_transform*in.col(i) - control_point_xyz.col(i)).norm();
  mean_err /= control_point_xyz.cols();

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
  for (int i = 0; i < control_point_xyz.cols(); i++) {
    Eigen::Vector3d a = world_transform*in.col(i);
    Eigen::Vector3d b = control_point_xyz.col(i);
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
