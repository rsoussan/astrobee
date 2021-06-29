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
                       const Velocity3& world_F_world_v_body_b, boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none, boost::optional<Matrix&> H3 = boost::none,
                       boost::optional<Matrix&> H4 = boost::none,
                       boost::optional<Matrix&> H5 = boost::none) const override {
    const Vector3 measured_delta_velocity = world_T_body_a.rotation() * acceleration_command_.linear_acceleration * dt_;
    const Vector3 expected_delta_velocity = world_F_world_v_body_b - world_F_world_v_body_a;
    return measured_delta_velocity - expected_delta_velocity;
    /*if (H) {
      Matrix66 d_world_T_sensor_d_world_T_body;
      Matrix36 d_world_t_point_d_world_T_sensor;
      Matrix13 d_distance_d_world_t_point;
      const auto error = getError(world_T_body, d_world_T_sensor_d_world_T_body, d_world_t_point_d_world_T_sensor,
                                  d_distance_d_world_t_point);
      *H = d_distance_d_world_t_point * d_world_t_point_d_world_T_sensor * d_world_T_sensor_d_world_T_body;
      return error;
    }
    return getError(world_T_body);*/
    return Vector3();
  }

  /*Vector getError(const Pose3& world_T_body, OptionalJacobian<6, 6> d_world_T_sensor_d_world_T_body = boost::none,
                  OptionalJacobian<3, 6> d_world_t_point_d_world_T_sensor = boost::none,
                  OptionalJacobian<1, 3> d_distance_d_world_t_point = boost::none) const {
    const Pose3 world_T_sensor = world_T_body.transformPoseFrom(body_T_sensor_, d_world_T_sensor_d_world_T_body);
    const Point3 world_t_point = world_T_sensor.transformFrom(sensor_t_point_, d_world_t_point_d_world_T_sensor);
    const double distance = world_T_plane_.Distance(world_t_point, d_distance_d_world_t_point);
    Vector error(1);
    error << distance;
    return error;
  }*/

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
