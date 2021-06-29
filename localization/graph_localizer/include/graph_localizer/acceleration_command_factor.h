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

#include <localization_measurements/acceleration_command.h>

#include <gtsam/geometry/Pose3.h>
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

  AccelerationCommandFactor(const localization_measurements::AccelerationCommand& acceleration_command, const double dt,
                            const SharedNoiseModel& model, Key pose_a_key, Key velocity_a_key, Key imu_bias_a_key,
                            Key pose_b_key, Key velocity_b_key)
      : Base(model, pose_a_key, velocity_a_key, imu_bias_a_key, pose_b_key, velocity_b_key),
        acceleration_command_(acceleration_command),
        dt_(dt) {}

  void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << s << "AccelerationCommandFactor, z = ";
    traits<localization_measurements::AccelerationCommand>::Print(acceleration_command_);
    std::cout << " dt: " << dt_ << std::endl;
    Base::print("", keyFormatter);
  }

  bool equals(const NonlinearFactor& p, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&p);
    return e && Base::equals(p, tol) &&
           traits<localization_measurements::AccelerationCommand>::Equals(this->acceleration_command(),
                                                                          e->acceleration_command(), tol) &&
           std::abs(dt_ - e->dt()) < tol;
  }

  Vector evaluateError(const Pose3& world_T_body_a, const Velocity3& world_F_world_v_body_a,
                       const imuBias::ConstantBias& imu_biases_a, const Pose3& world_T_body_b,
                       const Velocity3& world_F_world_v_body_b,
                       boost::optional<Matrix&> d_e_d_world_T_body_a = boost::none,
                       boost::optional<Matrix&> d_e_d_world_F_world_v_body_a = boost::none,
                       boost::optional<Matrix&> d_e_d_biases_a = boost::none,
                       boost::optional<Matrix&> d_e_d_world_T_body_b = boost::none,
                       boost::optional<Matrix&> d_e_d_world_F_world_v_body_b = boost::none) const override {
    const gtsam::Rot3 world_R_body_a = world_T_body_a.rotation();
    const Vector3 measured_delta_velocity = world_R_body_a * acceleration_command_.linear_acceleration * dt_;
    const Vector3 expected_delta_velocity = world_F_world_v_body_b - world_F_world_v_body_a;
    const Vector3 error = measured_delta_velocity - expected_delta_velocity;
    // Calculate Jacobians
    if (d_e_d_world_T_body_a) {
      gtsam::Matrix d_e_d_world_R_body_a;
      world_R_body_a.rotate(acceleration_command_.linear_acceleration * dt_, d_e_d_world_R_body_a);
      gtsam::Matrix d_world_R_body_a_d_world_T_body_a;
      world_T_body_a.rotation(d_world_R_body_a_d_world_T_body_a);
      *d_e_d_world_T_body_a = d_e_d_world_R_body_a * d_world_R_body_a_d_world_T_body_a;
    }
    if (d_e_d_world_F_world_v_body_a) {
      *d_e_d_world_F_world_v_body_a << -1.0 * I_3x3;
    }
    if (d_e_d_biases_a) {
      *d_e_d_biases_a << Z_3x3;
    }
    if (d_e_d_world_T_body_b) {
      *d_e_d_world_T_body_b << Eigen::Matrix<double, 3, 6>::Zero();
    }
    if (d_e_d_world_F_world_v_body_b) {
      *d_e_d_world_F_world_v_body_b << I_3x3;
    }

    return error;
  }

  const localization_measurements::AccelerationCommand& acceleration_command() const { return acceleration_command_; }
  const double dt() const { return dt_; }

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(acceleration_command_);
    ar& BOOST_SERIALIZATION_NVP(dt_);
  }

  localization_measurements::AccelerationCommand acceleration_command_;
  double dt_;

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace gtsam

#endif  // GRAPH_LOCALIZER_ACCELERATION_COMMAND_FACTOR_H_
