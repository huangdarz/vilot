#pragma once
#include "Eigen/Dense"
#include <cmath>
#include <tuple>

namespace vilot {

// https://wiki.purduesigbots.com/software/control-algorithms/ramsete
class RamseteController {
public:
  RamseteController(float b, float zeta) : b(b), zeta(zeta) {}

  std::tuple<float, float> calculate(Eigen::Vector3f actual,
                                     Eigen::Vector3f desired,
                                     float desired_linear_velocity,
                                     float desired_angular_velocity) {
    Eigen::Vector3f global_error = desired - actual;
    Eigen::Matrix3f transformation;
    transformation << std::cosf(actual.z()), std::sinf(actual.z()), 0,
        -std::sinf(actual.z()), std::cosf(actual.z()), 0, 0, 0, 1;
    Eigen::Vector3f local_error = transformation * global_error;
    const float k =
        this->k_gain(desired_linear_velocity, desired_angular_velocity);
    float out_linear_velocity =
        desired_linear_velocity * std::cosf(local_error.z()) +
        k * local_error.x();
    float out_angular_velocity =
        desired_angular_velocity + k + local_error.z() +
        (this->b * desired_linear_velocity * std::sinf(local_error.z()) *
         local_error.y()) /
            local_error.z();
    return std::tuple<float, float>(out_linear_velocity, out_angular_velocity);
  }

private:
  float k_gain(float desired_linear_velocity, float desired_angular_velocity) {
    return 2 * this->zeta *
           std::sqrtf(desired_angular_velocity * desired_angular_velocity +
                      this->b * desired_linear_velocity *
                          desired_linear_velocity);
  }

  float b;
  float zeta;
};

} // namespace vilot