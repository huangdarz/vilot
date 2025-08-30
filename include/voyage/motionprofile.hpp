#pragma once
#include "units.h"
#include "voyage/gcem/gcem.hpp" // IWYU pragma: keep

namespace voyage {

class TrapezoidalMotionProfile {
  using second_t = units::time::second_t;
  using meter_t = units::length::meter_t;
  using meters_per_second_t = units::velocity::meters_per_second_t;
  using meters_per_second_squared_t =
      units::acceleration::meters_per_second_squared_t;

public:
  struct ProfilePoint {
    second_t t;
    meter_t position;
    meters_per_second_t velocity;
    meters_per_second_squared_t acceleration;
  };

  TrapezoidalMotionProfile(meter_t target, meters_per_second_t max_velocity,
                           meters_per_second_squared_t acceleration,
                           meters_per_second_squared_t deceleration) noexcept;

  ProfilePoint sample(second_t t) noexcept;

  second_t motion_total_time() noexcept;

private:
  meter_t target;
  meters_per_second_t max_velocity;
  meters_per_second_squared_t acceleration;
  meters_per_second_squared_t deceleration;

  second_t accel_time;
  second_t decel_time;
  meter_t accel_distance;
  meter_t decel_distance;
  second_t cruise_time;
  second_t total_time;
};

class SMotionProfile {

public:
  struct ProfilePoint {
    float t;
    float position;
    float velocity;
    float acceleration;
    float jerk;
  };

private:
};

} // namespace voyage