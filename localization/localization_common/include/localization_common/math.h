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

#ifndef LOCALIZATION_COMMON_MATH_H_
#define LOCALIZATION_COMMON_MATH_H_

#include <Eigen/Core>

// TODO(rsoussan): Move more functions from localization common here
namespace localization_common {
// TODO(rsoussan): Rename to MatrixEquality when other version removed
bool MatrixEquality2(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs, const double tolerance);

bool Equals(const double lhs, const double rhs, const double tolerance);
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_MATH_H_
