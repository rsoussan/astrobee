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

#ifndef SPARSE_MAPPING_BUNDLE_ADJUSTMENT_PARAMS_H_
#define SPARSE_MAPPING_BUNDLE_ADJUSTMENT_PARAMS_H_

#include <limits>
#include <set>
#include <string>

namespace sparse_mapping {
struct BundleAdjustementParams {
  // If false, all cameras other than those in fixed_cameras set are optimized
  bool optimize_camera_range = false;
  // Only used if optimize_camera_range is set to true
  int first_optimized_camera = 0;
  int last_optimized_camera = std::numeric_limits<int>::max();

  bool fix_all_cameras = false;
  std::unordered_set<int> fixed_cameras;
  bool optimize_scale = false;

  // l1, l2, cauchy, or huber
  // l2 is equivalent to no loss since ceres is a least squares solver
  std::string loss_function = "cauchy";
  double loss_threshold = 2.0;
  ceres::Solver::Options solver_options = DefaultSolverOptions();

  ceres::LossFunction* LossFunction() const;
  static ceres::Solver::Options DefaultSolverOptions() const;
};

inline ceres::LossFunction* BundleAdjustmentParams::LossFunction() {
  if (loss_function == "l1") return new ceres::SoftLOneLoss(loss_threshold);
  else if (loss_function == "l2")
    return nullptr;
  else if (loss_function == "cauchy")
    return ceres::CauchyLoss(loss_threshold);
  else if (loss_function == "huber")
    return ceres::HuberLoss(loss_threshold);
  else
    LOG(FATAL) << "Invalid loss function provided: " << loss_function;
}

inline ceres::Solver::Options BunleAdjustmentParams::DefaultSolverOptions() {
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  // TODO(rsoussan): Set preconditioner? Set other options?
  options.num_threads = 1;
  options.max_num_iterations = 1000;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  return options;
}
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_BUNDLE_ADJUSTMENT_PARAMS_H_
