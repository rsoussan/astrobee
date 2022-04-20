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

namespace {
bool FixedCamera(const BundleAdjustmentParams& params, const int cid) {
  if (params.fix_all_cameras || params.fixed_cameras.count(cid) > 0) return true;
  const bool in_optimize_range =
    params.optimize_camera_range && (cid >= params.first_optimized_camera && cid <= params.last_optimized_camera);
  if (!in_optimize_range) return true;
  return false;
}

bool FixedPoint(const BundleAdjustmentParams& params, const int pid,
                const PidFeatureTrackMap& pid_to_feature_track) {
  if (params.fixed_points.count(pid) > 0) return true;
  // Points which project into cameras that are not fixed are also not fixed
  const auto& feature_track = pid_to_feature_track[pid];
  for (const auto& cid_fid_pair : feature_track) {
    const int cid = cid_fid.first;
    if (!FixedCamera(cid)) return false;
  }
  return true;
}
}  // namespace

namespace sparse_mapping {
namespace oc = optimization_common;

ceres::Solver::Summary BundleAdjust(const BundleAdjustmentParams& params,
                                    const std::vector<Eigen::Matrix2Xd>& cid_to_keypoints,
                                    PidPoseMap* cid_to_cam_T_global,
                                    PidFeatureTrackMap* pid_to_feature_track,
                                    PidPointMap* pid_to_global_t_point,
                                    CidFidPidMap* cid_to_fid_to_pid) {
  std::vector<Eigen::Matrix<double, 7, 1>> camera_T_globals;
  camera_T_globals.reserve(cid_to_cam_T_global->size());
  for (int cid = 0; cid < cid_to_cam_T_global->size(); ++cid) {
    camera_T_globals.emplace_back(oc::VectorFromAffine3d(cid_to_cam_T_global->at(cid)));
  }

  ceres::Problem problem;
  // Centered, undistored camera
  const Eigen::Vector2d zero_principal_points(Eigen::Vector2d::Zero());
  const Eigen::VectorXd zero_distortion(1);
  const Eigen::Vector2d focal_lengths = camera.GetFocalVector();
  oc::AddConstantParameterBlock(2, zero_principal_points.data(), problem);
  oc::AddConstantParameterBlock(1, zero_distortion.data(), problem);
  oc::AddConstantParameterBlock(2, focal_lengths.data(), problem);
    for (int pid = 0; pid < static_cast<int>(pid_to_global_t_point->size()); ++pid) {
      if (pid_to_feature_track[pid].size() < 2)
        LOG(FATAL) << "Found a track of size < 2.";

      auto& point_3d = pid_to_global_t_point->at(pid);
      const bool fixed_point = FixedPoint(params, pid, pid_to_feature_track);
      oc::AddParameterBlock(3, point_3d.data(), problem, fixed_point);
       for (const auto& cid_fid : pid_to_feature_track[pid]) {
        const int cid = cid_fid.first;
        const int fid = cid_fid.second;
        const auto& image_point = cid_to_keypoints[cid].col(fid);
        auto& camera_T_global = camera_T_globals[cid];

      const bool fixed_camera = FixedCamera(params, cid);
      oc::AddAffine3ParameterBlock(camera_T_global.data(), problem, fixed_camera);
      if (!params.optimize_scale) {
        // TODO(rsoussan): Optimize for scale??? test!! switch to subset manifold?? Can you add two local
        // parameterizations to one param block??
        ceres::SubsetParameterization* constant_scale_parameterization = new ceres::SubsetParameterization(7, {6});
        problem.SetParameterization(camera_T_global.data(), constant_scale_parameterization);
      }

      oc::ReprojectionError<vc::IdentityDistorter, oc::AffineFunctor>::AddCostFunction(
        image_point, point_3d, camera_T_global, const_cast<Eigen::Vector2d&>(focal_lengths),
        const_cast<Eigen::Vector2d&>(zero_principal_points), const_cast<Eigen::VectorXd&>(zero_distortion), problem,
        params.LossFunction());
      }
    }
  ceres::Solver::Summary summary;
  ceres::Solve(params.options, &problem, &summary);

  // Write the rotations back to the transform
  for (int cid = 0; cid < cid_to_cam_T_global->size(); ++cid) {
    cid_to_cam_T_global->at(cid) = oc::Affine3d(camera_T_globals[cid]);
  }

  if (params.remove_invalid_points_and_detections) {
    RemoveInvalidPointsAndDetections(params.remove_invalid_points_and_detections_params,
                 *cid_to_cam_T_global,
                 cid_to_keypoints,
                 pid_to_feature_track,
                 pid_to_global_t_point, cid_to_fid_to_pid);
  }

  return summary;
}

// This is a very specialized function
// TODO(rsoussan): Combine this with other bundle adjust function!!!!!
void BundleAdjustSmallSet(std::vector<Eigen::Matrix2Xd> const& features_n,
                          double focal_length,
                          std::vector<Eigen::Affine3d> * cam_T_global_n,
                          Eigen::Matrix3Xd * pid_to_global_t_point,
                          ceres::LossFunction * loss,
                          ceres::Solver::Options const& options,
                          ceres::Solver::Summary * summary) {
  CHECK(cam_T_global_n) << "Variable cam_T_global_n needs to be defined";
  CHECK(cam_T_global_n->size() == features_n.size())
    << "Variables features_n and cam_T_global_n need to agree on the number of cameras";
  CHECK(cam_T_global_n->size() > 1) << "Bundle adjust needs at least 2 or more cameras";
  CHECK(pid_to_global_t_point->cols() == features_n[0].cols())
    << "There should be an equal amount of XYZ points as there are feature observations";
  for (size_t i = 1; i < features_n.size(); i++) {
    CHECK(features_n[0].cols() == features_n[i].cols())
      << "The same amount of features should be seen in all cameras";
  }

  const size_t n_cameras = features_n.size();

  // Allocate space for the angle axis representation of rotation
  std::vector<Eigen::Vector3d> aa(n_cameras);
  for (size_t cid = 0; cid < n_cameras; cid++) {
    camera::RotationToRodrigues(cam_T_global_n->at(cid).linear(), &aa[cid]);
  }

  // Build the problem
  ceres::Problem problem;
  for (ptrdiff_t pid = 0; pid < pid_to_global_t_point->cols(); pid++) {
    for (size_t cid = 0; cid < n_cameras; cid++) {
      ceres::CostFunction* cost_function = ReprojectionError::Create(features_n[cid].col(pid));
      problem.AddResidualBlock(cost_function, loss,
                               &cam_T_global_n->at(cid).translation()[0],
                               &aa.at(cid)[0],
                               &pid_to_global_t_point->col(pid)[0],
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
    cam_T_global_n->at(cid).linear() = r;
  }
}

}  // namespace sparse_mapping
