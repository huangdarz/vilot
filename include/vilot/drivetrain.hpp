#pragma once

#include "pros/motor_group.hpp"
#include "units.h"

namespace vilot::chassis {

struct Differential {
  using meter_t = units::length::meter_t;
  using millivolt_t = units::voltage::millivolt_t;
  using revolutions_per_minute_t =
      units::angular_velocity::revolutions_per_minute_t;

  Differential() = delete;

  template <typename... Args>
  Differential(const std::initializer_list<int8_t> left_motors,
               const std::initializer_list<int8_t> right_motors,
               const float gear_ratio_in_out, Args &&...shared_args)
      : left(left_motors, std::forward<Args>(shared_args)...),
        right(right_motors, std::forward<Args>(shared_args)...),
        gear_ratio_in_out(gear_ratio_in_out) {}

  pros::MotorGroup left;
  pros::MotorGroup right;
  float gear_ratio_in_out;
};

} // namespace vilot::chassis

namespace vilot::drivetrain {

class Differential {
  using meters_per_second_t = units::velocity::meters_per_second_t;
  using radians_per_second_t = units::angular_velocity::radians_per_second_t;
  using meter_t = units::length::meter_t;
  using radian_t = units::angle::radian_t;

public:
  template <typename... Args>
  Differential(meter_t track_width, meter_t wheel_radius,
               Args &&...chassis_args)
      : track_width(track_width), wheel_radius(wheel_radius),
        chassis(std::forward<Args>(chassis_args)...) {}

  void move(meters_per_second_t x, radians_per_second_t theta);

private:
  chassis::Differential chassis;
  meter_t track_width;
  meter_t wheel_radius;
};

} // namespace vilot::drivetrain