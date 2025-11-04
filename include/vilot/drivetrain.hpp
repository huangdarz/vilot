#pragma once

#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "units.h"
#include "vilot/localisation.hpp"
#include "vilot/pid.hpp"

namespace vilot {

#define MAX_VOLTAGE_MV 12000.f

#define PORTS(...)                \
  std::initializer_list<int8_t> { \
    __VA_ARGS__                   \
  }

struct DifferentialChassis {
  using meter_t = units::length::meter_t;
  using millivolt_t = units::voltage::millivolt_t;
  using revolutions_per_minute_t =
      units::angular_velocity::revolutions_per_minute_t;
  using meters_per_second_t = units::velocity::meters_per_second_t;
  using radians_per_second_t = units::angular_velocity::radians_per_second_t;

  DifferentialChassis() = delete;

  template <typename... Args>
  DifferentialChassis(const std::initializer_list<int8_t> left_motors,
                      const std::initializer_list<int8_t> right_motors,
                      const float gear_ratio_out_in, const meter_t track_width,
                      const meter_t wheel_radius, Args&&... shared_args)
      : left(left_motors, std::forward<Args>(shared_args)...),
        right(right_motors, std::forward<Args>(shared_args)...),
        gear_ratio_out_in(gear_ratio_out_in),
        track_width(track_width),
        wheel_radius(wheel_radius) {}

  void tank(millivolt_t left_, millivolt_t right_) const noexcept;

  void move(meters_per_second_t x, radians_per_second_t theta) const noexcept;

  void move(millivolt_t forward, millivolt_t turn) const noexcept;

  void stop(pros::motor_brake_mode_e_t mode =
                pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE) const noexcept;

  pros::MotorGroup left;
  pros::MotorGroup right;
  const float gear_ratio_out_in;
  const meter_t track_width;
  const meter_t wheel_radius;
};

}  // namespace vilot
