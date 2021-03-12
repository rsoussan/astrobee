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
#ifndef IMU_AUGMENTOR_KALMAN_FILTER_UTILITIES_H_
#define IMU_AUGMENTOR_KALMAN_FILTER_UTILITIES_H_

#include <utility>

namespace imu_augmentor {
template <typename State, typename StateCovariance, typename TransitionMatrix, typename TransitionCovariance>
std::pair<State, StateCovariance> Predict(const State& x, const StateCovariance& P, const TransitionMatrix& F,
                                          const TransitionCovariance& Q) {
  const State predicted_x = F * x;
  const StateCovariance predicted_P = F * P * F.transpose() + Q;
  return {predicted_x, predicted_P};
}

template <typename State, typename StateCovariance, typename Measurement, typename ObservationMatrix,
          typename ObservationCovariance>
std::pair<State, StateCovariance> Update(const State& predicted_x, const StateCovariance& predicted_P,
                                         const Measurement& z, const ObservationMatrix& H,
                                         const ObservationCovariance& R) {
  // Innovation residual
  const Measurement y = z - H * predicted_x;
  // Innovation covariance
  const ObservationCovariance S = H * predicted_P * H.transpose() + R;
  // Kalman Gain
  const auto K = predicted_P * H.transpose() * S.inverse();
  // State Estimate
  const State estimated_x = predicted_x + K * y;
  // Covariance Estimate
  const StateCovariance estimated_P = (StateCovariance::Identity() - K * H) * estimated_P;
  return {estimated_x, estimated_P};
}

template <typename State, typename StateCovariance, typename TransitionMatrix, typename TransitionCovariance,
          typename Measurement, typename ObservationMatrix, typename ObservationCovariance>
std::pair<State, StateCovariance> KalmanFilterEstimate(const State& x, const StateCovariance& P,
                                                       const TransitionMatrix& F, const TransitionCovariance& Q,
                                                       const Measurement& z, const ObservationMatrix& H,
                                                       const ObservationCovariance& R) {
  const auto predicted_state_and_covariance = Predict(x, P, F, Q);
  return Update(predicted_state_and_covariance.first, predicted_state_and_covariance.second, z, H, R);
}
}  // namespace imu_augmentor

#endif  // IMU_AUGMENTOR_KALMAN_FILTER_UTILITIES_H_
