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

#ifndef SPARSE_MAPPING_ESTIMATE_POSE_PARAMS_H_
#define SPARSE_MAPPING_ESTIMATE_POSE_PARAMS_H_

namespace sparse_mapping {
struct EstimatePoseParams : vision_common::ReprojectionPoseEstimateParams {
    EstimatePoseParams();
      // TODO(rsoussan): Change this to a bool?
    // TODO(rsoussan): should this be here or in sparse map params????
    int histogram_equalization = false;
    int max_num_total_feature_matches = 100;
    bool check_point_3d_exists = true;
    int max_image_matches = 20;
};

inline EstimatePoseParams::EstimatePoseParams() {
  // ReprojectionPoseEstimateParams
  // Optimization
  optimization.solver_options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  optimization.solver_options.num_threads = 1;
  optimization.solver_options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = false;
  optimization.verbose = false;
  optimization.huber_loss = 1.0;
  // Ransac Pnp
  ransac_pnp.max_inlier_threshold = 5;
  ransac_pnp.num_iterations = 1000;
  ransac_pnp.min_num_inliers = 10;
  ransac_pnp.pnp_method = cv::SOLVEPNP_P3P;
  // Other
  optimize_estimate = true;
  max_inlier_threshold = 5;
}
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_ESTIMATE_POSE_PARAMS_H_
