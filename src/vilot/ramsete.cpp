#include "ramsete.hpp"

namespace vilot {

using meter_t = units::length::meter_t;
using meters_per_second_t = units::velocity::meters_per_second_t;
using meters_per_second_squared_t =
    units::acceleration::meters_per_second_squared_t;
using radians_per_second_t = units::angular_velocity::radians_per_second_t;

std::tuple<meters_per_second_t, radians_per_second_t>
RamseteController::calculate(
    Pose2d _actual, Pose2d _desired,
    meters_per_second_t desired_linear_velocity,
    radians_per_second_t desired_angular_velocity) noexcept {
  Eigen::Vector3f actual(_actual.x(), _actual.y(), _actual.theta());
  Eigen::Vector3f desired(_desired.x(), _desired.y(), _desired.theta());
  Eigen::Vector3f global_error = desired - actual;
  Eigen::Matrix3f transformation;
  transformation << std::cosf(actual.z()), std::sinf(actual.z()), 0,
      -std::sinf(actual.z()), std::cosf(actual.z()), 0, 0, 0, 1;
  Eigen::Vector3f local_error = transformation * global_error;
  const float k =
      this->k_gain(desired_linear_velocity, desired_angular_velocity);
  float out_linear_velocity =
      desired_linear_velocity() * std::cosf(local_error.z()) +
      k * local_error.x();
  float out_angular_velocity = desired_angular_velocity() +
                               k * local_error.z() +
                               (this->b * desired_linear_velocity() *
                                std::sinf(local_error.z()) * local_error.y()) /
                                   local_error.z();
  return std::tuple<meters_per_second_t, radians_per_second_t>(
      out_linear_velocity, out_angular_velocity);
}

float RamseteController::k_gain(
    meters_per_second_t desired_linear_velocity,
    radians_per_second_t desired_angular_velocity) noexcept {
  return 2 * this->zeta *
         std::sqrtf(desired_angular_velocity() * desired_angular_velocity() +
                    this->b * desired_linear_velocity() *
                        desired_linear_velocity());
}

} // namespace vilot