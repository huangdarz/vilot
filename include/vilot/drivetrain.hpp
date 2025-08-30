#pragma once

#include "pros/motor_group.hpp"
#include "units.h"
#include "vilot/localisation.hpp"

namespace vilot {

#define MAX_VOLTAGE_MV 12000.f

#define MOTORS(...)                                                            \
  std::initializer_list<int8_t> { __VA_ARGS__ }

struct DifferentialChassis {
  using meter_t = units::length::meter_t;
  using millivolt_t = units::voltage::millivolt_t;
  using revolutions_per_minute_t =
      units::angular_velocity::revolutions_per_minute_t;

  DifferentialChassis() = delete;

  template <typename... Args>
  DifferentialChassis(const std::initializer_list<int8_t> left_motors,
                      const std::initializer_list<int8_t> right_motors,
                      const float gear_ratio_in_out, Args &&...shared_args)
      : left(left_motors, std::forward<Args>(shared_args)...),
        right(right_motors, std::forward<Args>(shared_args)...),
        gear_ratio_in_out(gear_ratio_in_out) {}

  pros::MotorGroup left;
  pros::MotorGroup right;
  float gear_ratio_in_out;
};

class DifferentialDrivetrain {
  using meters_per_second_t = units::velocity::meters_per_second_t;
  using meters_per_second_squared_t =
      units::acceleration::meters_per_second_squared_t;
  using radians_per_second_t = units::angular_velocity::radians_per_second_t;
  using meter_t = units::length::meter_t;
  using radian_t = units::angle::radian_t;
  using millivolt_t = units::voltage::millivolt_t;

public:
  template <typename... Args>
  DifferentialDrivetrain(meter_t track_width, meter_t wheel_radius,
                         device::Odometry &odometry, Args &&...chassis_args)
      : track_width(track_width), wheel_radius(wheel_radius),
        chassis(std::forward<Args>(chassis_args)...), odometry(odometry) {}

  void move(meters_per_second_t x, radians_per_second_t theta);
  void move(millivolt_t forward, millivolt_t turn);

  void follow(meter_t distance, meters_per_second_t max_velocity,
              meters_per_second_squared_t acceleration,
              meters_per_second_squared_t deceleration,
              float follow_strength = 0.5, float follow_dampen = 0.05);

  void tank(millivolt_t left, millivolt_t right);

  void stop();

private:
  DifferentialChassis chassis;
  meter_t track_width;
  meter_t wheel_radius;
  device::Odometry &odometry;
};

} // namespace vilot