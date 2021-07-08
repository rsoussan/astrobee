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

#ifndef GRAPH_LOCALIZER_ACCELERATION_COMMAND_FACTOR_H_
#define GRAPH_LOCALIZER_ACCELERATION_COMMAND_FACTOR_H_

#include <graph_localizer/utilities.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtsam {
class AccelerationCommandFactor : public NoiseModelFactor5<Pose3, Velocity3, imuBias::ConstantBias, Pose3, Velocity3> {
  typedef NoiseModelFactor5<Pose3, Velocity3, imuBias::ConstantBias, Pose3, Velocity3> Base;
  typedef AccelerationCommandFactor This;

 public:
  AccelerationCommandFactor() {}

  AccelerationCommandFactor(const PreintegratedCombinedMeasurements& pim, const double huber_k, Key pose_a_key,
                            Key velocity_a_key, Key imu_bias_a_key, Key pose_b_key, Key velocity_b_key)
      : Base(graph_localizer::Robust(noiseModel::Gaussian::Covariance(pim.preintMeasCov()), huber_k), pose_a_key,
             velocity_a_key, imu_bias_a_key, pose_b_key, velocity_b_key),
        // Use dummy keys for member variable combined_imu_factor_ since only the error function from this is being used
        combined_imu_factor_(Key(), Key(), Key(), Key(), Key(), Key(), pim) {}

  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "AccelerationCommandFactor, z = ";
    combined_imu_factor_.print("", keyFormatter);
    Base::print("", keyFormatter);
  }

  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) && combined_imu_factor_.equals(e->combined_imu_factor(), tol);
  }

  Vector evaluateError(const Pose3& world_T_body_a, const Velocity3& world_F_world_v_body_a,
                       const imuBias::ConstantBias& imu_biases_a, const Pose3& world_T_body_b,
                       const Velocity3& world_F_world_v_body_b,
                       boost::optional<Matrix&> d_e_d_world_T_body_a = boost::none,
                       boost::optional<Matrix&> d_e_d_world_F_world_v_body_a = boost::none,
                       boost::optional<Matrix&> d_e_d_biases_a = boost::none,
                       boost::optional<Matrix&> d_e_d_world_T_body_b = boost::none,
                       boost::optional<Matrix&> d_e_d_world_F_world_v_body_b = boost::none) const override {
    // TODO(rsoussan): pass imu_biases_b???????
    const auto combined_imu_factor_error = combined_imu_factor_.evaluateError(
      world_T_body_a, world_F_world_v_body_a, world_T_body_b, world_F_world_v_body_b, imu_biases_a, imu_biases_a,
      d_e_d_world_T_body_a, d_e_d_world_F_world_v_body_a, d_e_d_world_T_body_b, d_e_d_world_F_world_v_body_b,
      d_e_d_biases_a, boost::none);
    // Calculate Jacobians
    if (d_e_d_world_T_body_a) {
      //*d_e_d_world_T_body_a = d_e_d_world_R_body_a * d_world_R_body_a_d_world_T_body_a;
    }
    if (d_e_d_world_F_world_v_body_a) {
      *d_e_d_world_F_world_v_body_a = I_3x3;
    }
    if (d_e_d_biases_a) {
      *d_e_d_biases_a = Eigen::Matrix<double, 3, 6>::Zero();
    }
    if (d_e_d_world_T_body_b) {
      *d_e_d_world_T_body_b = Eigen::Matrix<double, 3, 6>::Zero();
    }
    if (d_e_d_world_F_world_v_body_b) {
      *d_e_d_world_F_world_v_body_b = -1.0 * I_3x3;
    }

    // return error;
    return Vector();
  }

  const CombinedImuFactor& combined_imu_factor() const { return combined_imu_factor_; }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(combined_imu_factor_);
  }

  CombinedImuFactor combined_imu_factor_;

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_ACCELERATION_COMMAND_FACTOR_H_
