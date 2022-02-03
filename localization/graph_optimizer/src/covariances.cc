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
#include <graph_optimizer/covariances.h>
#include <localization_common/logger.h>

namespace graph_optimizer {
Covariances::Covariances(const CovariancesParams& params) : params_(params) {}

bool Covariances::Update(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values) {
  return UpdateMarginals(factors, values);
}

boost::optional<gtsam::noiseModel::Gaussian::shared_ptr> Covariances::Get(const gtsam::Key key) const {
  if (!marginals_) return boost::none;
  try {
    return gtsam::noiseModel::Gaussian::Covariance(marginals_->marginalCovariance(key));
  } catch (...) {
    LogOptionallyFatal("Get: Unable to get covariance for key " << key << ".", params_.fatal_failures);
    return boost::none;
  }
}

bool Covariances::UpdateMarginals(const gtsam::NonlinearFactorGraph& factors, const gtsam::Values& values) {
  try {
    marginals_ = gtsam::Marginals(factors, values, params_.marginals_factorization);
  } catch (gtsam::IndeterminantLinearSystemException) {
    LogOptionallyFatal("Update: Indeterminant linear system error during computation of marginals.",
                       params_.fatal_failures);
    marginals_ = boost::none;
    return false;
  } catch (const std::exception& exception) {
    LogOptionallyFatal("Update: Computing marginals failed. " + std::string(exception.what()), params_.fatal_failures);
    marginals_ = boost::none;
    return false;
  } catch (...) {
    LogOptionallyFatal("Update: Computing marginals failed.", params_.fatal_failures);
    marginals_ = boost::none;
    return false;
  }
  return true;
}
}  // namespace graph_optimizer
