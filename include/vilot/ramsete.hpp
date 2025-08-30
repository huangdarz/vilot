#pragma once
#include "Eigen/Dense"
#include "units.h"
#include <cmath>
#include <tuple>

namespace vilot {

// https://wiki.purduesigbots.com/software/control-algorithms/ramsete
class RamseteController {
  using meter_t = units::length::meter_t;
  using meters_per_second_t = units::velocity::meters_per_second_t;
  using meters_per_second_squared_t =
      units::acceleration::meters_per_second_squared_t;
  using radian_t = units::angle::radian_t;
  using radians_per_second_t = units::angular_velocity::radians_per_second_t;

public:
  struct State {
    meter_t x;
    meter_t y;
    radian_t theta;
  };

  RamseteController(float b, float zeta) : b(b), zeta(zeta) {}

  std::tuple<meters_per_second_t, radians_per_second_t>
  calculate(State actual, State desired,
            meters_per_second_t desired_linear_velocity,
            radians_per_second_t desired_angular_velocity) noexcept;

private:
  float k_gain(meters_per_second_t desired_linear_velocity,
               radians_per_second_t desired_angular_velocity) noexcept;

  float b;
  float zeta;
};

} // namespace vilot