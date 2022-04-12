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

#include <sparse_mapping/ransac_estimate_affine_3d.h>

#include <glog/logging.h>

namespace sparse_mapping {
  Eigen::Affine3d TranslationRotationScaleFittingFunctor::operator() (std::vector<Eigen::Vector3d> const& in_vec,
                          std::vector<Eigen::Vector3d> const& out_vec) const {
    // check consistency
    if (in_vec.size() != out_vec.size())
      LOG(FATAL) << "There must be as many inputs as outputs to be "
                 << "able to compute a transform between them.\n";
    if (in_vec.size() < min_elements_needed_for_fit())
      LOG(FATAL) << "Cannot compute a transformation. Insufficient data.\n";

    Eigen::Matrix3Xd in_mat  = Eigen::MatrixXd(3, in_vec.size());
    Eigen::Matrix3Xd out_mat = Eigen::MatrixXd(3, in_vec.size());
    for (size_t it = 0; it < in_vec.size(); it++) {
      in_mat.col(it)  = in_vec[it];
      out_mat.col(it) = out_vec[it];
    }
    Eigen::Affine3d out_trans;
    Find3DAffineTransform(in_mat, out_mat, &out_trans);
    return out_trans;
  }

  double TransformError::operator() (Eigen::Affine3d const& T, Eigen::Vector3d const& p1,
                     Eigen::Vector3d const& p2) const {
    return (T*p1 - p2).norm();
  }
}  // namespace sparse_mapping
