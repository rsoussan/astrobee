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
#ifndef POINT_CLOUD_COMMON_CORRESPONDENCES_3D_H_
#define POINT_CLOUD_COMMON_CORRESPONDENCES_3D_H_

#include <ff_common/eigen_vectors.h>
#include <point_cloud_common/utilities.h>

#include <pcl/correspondence.h>

#include <vector>

namespace point_cloud_common {
struct Correspondences3d {
  Correspondences3d(const std::vector<Eigen::Vector3d>& source_points,
                    const std::vector<Eigen::Vector3d>& target_points,
                    const std::vector<Eigen::Vector3d>& target_normals)
      : source_points(source_points), target_points(target_points), target_normals(target_normals) {}
  template <typename PointWithNormalType>
  Correspondences3d(const pcl::Correspondences& correspondences,
                    const pcl::PointCloud<PointWithNormalType>& source_cloud,
                    const pcl::PointCloud<PointWithNormalType>& target_cloud) {
    for (const auto& correspondence : correspondences) {
      const auto& pcl_source_point = source_cloud[correspondence.index_query];
      const auto& pcl_target_point = target_cloud[correspondence.index_match];
      source_points.emplace_back(Vector3d(pcl_source_point));
      target_points.emplace_back(Vector3d(pcl_target_point));
      target_normals.emplace_back(NormalVector3d(pcl_target_point));
    }
  }

  size_t size() const { return source_points.size(); }

  std::vector<Eigen::Vector3d> source_points;
  std::vector<Eigen::Vector3d> target_points;
  std::vector<Eigen::Vector3d> target_normals;
};
}  // namespace point_cloud_common

#endif  // POINT_CLOUD_COMMON_CORRESPONDENCES_3D_H_
