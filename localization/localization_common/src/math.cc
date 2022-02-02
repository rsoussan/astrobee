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

#include <localization_common/math.h>

namespace localization_common {
bool MatrixEquality2(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs, const double tolerance) {
  // Seperately check for zero matrices since isApprox fails for these
  if (lhs.isZero(tolerance) || rhs.isZero(tolerance)) {
    return lhs.isZero(tolerance) && rhs.isZero(tolerance);
  }
  return lhs.isApprox(rhs, tolerance);
}

bool Equals(const double lhs, const double rhs, const double tolerance) { return std::abs(lhs - rhs) < tolerance; }
}  // namespace localization_common
