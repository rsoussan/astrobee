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

DEFINE_int32(num_ba_passes, 5,
             "How many times to run bundle adjustment, removing outliers each time.");
namespace {
bool FixedCamera(const BundleAdjustmentParams& params, const int cid) {
  // TODO(rsoussan): Why would cid be out of range? would cam_t_global still be valid then?
  const bool in_optimize_range =
    params.optimize_camera_range && (cid >= params.first_optimized_camera && cid <= params.last_optimized_camera);
  if (params.fix_all_cameras || !in_optimize_range || fixed_cameras.find(cid) != fixed_cameras.end()) {
    return true;
  }
  return false;
}

// Vary points which project into cameras that are not fixed
bool FixedPoint(const BundleAdjustmentParams& params, const int pid,
                const std::vector<std::map<int, int>>& pid_to_cid_fid) {
  for (const auto& cid_fid : pid_to_cid_fid[pid]) {
    const int cid = cid_fid.first;
    if (cid >= params.first_optimized_camera && cid <= params.last_optimized_camera) return false;
  }
  return true;
}
}  // namespace

namespace sparse_mapping {
namespace oc = optimization_common;

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


void BundleAdjust(const BundleAdjustmentParams& params, const std::vector<std::map<int, int> >& pid_to_cid_fid,
                  const std::vector<Eigen::Matrix2Xd>& cid_to_keypoint_map, const double focal_length,
                  std::vector<Eigen::Affine3d>* cid_to_cam_t_global, std::vector<Eigen::Vector3d>* pid_to_xyz,
                  ceres::Solver::Summary* summary) {
  std::vector<Eigen::Matrix<double, 7, 1>> camera_T_globals;
  camera_T_globals.reserve(cid_to_cam_t_global->size());
  for (int cid = 0; cid < cid_to_cam_t_global->size(); ++cid) {
    camera_T_globals.emplace_back(oc::VectorFromAffine3d(cid_to_cam_t_global->at(cid)));
  }

  ceres::Problem problem;
  // Centered, undistored camera
  const Eigen::Vector2d zero_principal_points(Eigen::Vector2d::Zero());
  const Eigen::VectorXd zero_distortion(1);
  const Eigen::Vector2d focal_lengths(focal_length, focal_length);
  oc::AddConstantParameterBlock(2, zero_principal_points.data(), problem);
  oc::AddConstantParameterBlock(1, zero_distortion.data(), problem);
  oc::AddConstantParameterBlock(2, focal_lengths.data(), problem);
    for (int pid = 0; pid < static_cast<int>(pid_to_xyz->size()); ++pid) {
      if (pid_to_cid_fid[pid].size() < 2)
        LOG(FATAL) << "Found a track of size < 2.";

      auto& point_3d = pid_to_xyz->at(pid);
       for (const auto& cid_fid : pid_to_cid_fid[pid]) {
        const int cid = cid_fid.first;
        const int fid = cid_fid.second;
        const auto& image_point = cid_to_keypoint_map[cid].col(fid);
        auto& camera_T_global = camera_T_globals[cid];

      const bool fixed_camera = FixedCamera(params, cid);
      oc::AddAffine3ParameterBlock(camera_T_global.data(), problem, fixed_camera);
      // TODO(rsoussan): Optimize for scale??? test!! switch to subset manifold?? Can you add two local
      // parameterizations to one param block??
      ceres::SubsetParameterization* constant_scale_parameterization = new ceres::SubsetParameterization(7, {6});
      problem.SetParameterization(camera_T_global.data(), constant_scale_parameterization);

      oc::ReprojectionError<vc::IdentityDistorter, oc::AffineFunctor>::AddCostFunction(
        image_point, point_3d, camera_T_global, const_cast<Eigen::Vector2d&>(focal_lengths),
        const_cast<Eigen::Vector2d&>(zero_principal_points), const_cast<Eigen::VectorXd&>(zero_distortion), problem,
        params.LossFunction());
      }
        if (FixedPoint(params, pid, pid_to_cid_fid)) problem.SetParameterBlockConstant((pid_to_xyz->at(pid)).data());
    }
  ceres::Solve(params.options, &problem, summary);

  // Write the rotations back to the transform
  for (int cid = 0; cid < cid_to_cam_t_global->size(); ++cid) {
    cid_to_cam_t_global->at(cid) = oc::Affine3d(camera_T_globals[cid]);
  }
}

// This is a very specialized function
// TODO(rsoussan): Combine this with other bundle adjust function!!!!!
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
