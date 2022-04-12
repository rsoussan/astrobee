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

#ifndef SPARSE_MAPPING_RANSAC_ESTIMATE_AFFINE_3D_H_
#define SPARSE_MAPPING_RANSAC_ESTIMATE_AFFINE_3D_H_

#include <sparse_mapping/ransac.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace sparse_mapping {
using RansacEstimateAffine3d = RandomSampleConsus<TranslationRotationScaleFittingFunctor, TransformError>;

// This fitting functor attempts to find a rotation + translation + scale transformation
// between two vectors of points.
struct TranslationRotationScaleFittingFunctor {
  typedef Eigen::Affine3d result_type;

  /// A transformation requires 3 inputs and 3 outputs to make a fit.
  size_t min_elements_needed_for_fit() const { return 3; }

  result_type operator()(std::vector<Eigen::Vector3d> const& in_vec, std::vector<Eigen::Vector3d> const& out_vec) const;
};

// How well does the given transform do to map p1 to p2.
struct TransformError {
  double operator()(Eigen::Affine3d const& T, Eigen::Vector3d const& p1, Eigen::Vector3d const& p2) const;
};
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_RANSAC_ESTIMATE_AFFINE_3D_H_
