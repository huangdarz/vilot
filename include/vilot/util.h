#pragma once
#include "Eigen/Dense"
#include "units.h"

// Eigen should not be used with units.h

namespace vilot {

struct Pose2d {
  using meter_t = units::length::meter_t;
  using radian_t = units::angle::radian_t;

  Pose2d(Eigen::Vector2f vec, radian_t theta);
  Pose2d(meter_t x, meter_t y, radian_t theta);

  Eigen::Vector2f as_vec();

  meter_t x; // x forward, y left
  meter_t y;
  radian_t theta; // 90 left, 0 forward, -90 right
};

struct Twist2d {
  using meters_per_second_t = units::velocity::meters_per_second_t;
  using radians_per_second_t = units::angular_velocity::radians_per_second_t;

  Twist2d(Eigen::Vector2f vec, radians_per_second_t theta);
  Twist2d(meters_per_second_t x, meters_per_second_t y,
          radians_per_second_t theta);

  Eigen::Vector2f as_vec();

  meters_per_second_t x;
  meters_per_second_t y;
  radians_per_second_t theta; // 90 left, 0 forward, -90 right
};

} // namespace vilot