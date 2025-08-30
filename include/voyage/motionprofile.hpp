#pragma once
#include "units.h"
#include "voyage/gcem/gcem.hpp" // IWYU pragma: keep

namespace voyage {

template <typename Unit> class TrapezoidalMotionProfile {
  using second_t = units::time::second_t;
  using position_t = units::unit_t<Unit>;
  using velocity_t = units::unit_t<
      units::compound_unit<Unit, units::inverse<units::time::seconds>>>;
  using acceleration_t = units::unit_t<units::compound_unit<
      Unit, units::inverse<units::squared<units::time::seconds>>>>;

public:
  struct ProfilePoint {
    second_t t;
    position_t position;
    velocity_t velocity;
    acceleration_t acceleration;
  };

  TrapezoidalMotionProfile(position_t target, velocity_t max_velocity,
                           acceleration_t acceleration,
                           acceleration_t deceleration) noexcept {
    this->accel_time = this->max_velocity / this->acceleration;
    this->decel_time = this->max_velocity / this->deceleration;
    this->accel_distance =
        0.5f * this->acceleration * this->accel_time * this->accel_time;
    this->decel_distance =
        0.5f * this->deceleration * this->decel_time * this->decel_time;
    position_t min_dist_max_vel = accel_distance + decel_distance;

    if (target >= min_dist_max_vel) {
      position_t cruise_distance =
          target - this->accel_distance - this->decel_distance;
      this->cruise_time = cruise_distance / this->max_velocity;
    } else {
      velocity_t peak_velocity = units::math::sqrt(
          2.0f * this->target * this->acceleration * this->deceleration /
          (this->acceleration + this->deceleration));
      this->accel_time = peak_velocity / this->acceleration;
      this->decel_time = peak_velocity / this->deceleration;
      this->accel_distance =
          0.5f * this->acceleration * this->accel_time * this->accel_time;
      this->decel_distance =
          0.5f * this->deceleration * this->decel_time * this->decel_time;
      this->max_velocity = peak_velocity;
      this->cruise_time = second_t(0.0);
    }

    this->total_time = accel_time + cruise_time + decel_time;
  }

  ProfilePoint sample(second_t t) noexcept {
    ProfilePoint point;
    point.t = t;

    // Clamp time to valid range
    if (t <= second_t(0.0)) {
      point.position = position_t(0.0);
      point.velocity = velocity_t(0.0);
      point.acceleration = acceleration_t(0.0);
      return point;
    }

    if (t >= total_time) {
      point.position = target;
      point.velocity = velocity_t(0.0);
      point.acceleration = acceleration_t(0.0);
      return point;
    }

    if (t <= accel_time) {
      // Acceleration phase
      point.acceleration = acceleration;
      point.velocity = acceleration * t;
      point.position = 0.5f * acceleration * t * t;
    } else if (t <= accel_time + cruise_time) {
      // Cruise phase
      second_t cruise_t = t - accel_time;
      point.acceleration = acceleration_t(0.0);
      point.velocity = max_velocity;
      point.position = accel_distance + max_velocity * cruise_t;
    } else {
      // Deceleration phase
      second_t decel_t = t - accel_time - cruise_time;
      point.acceleration = -deceleration;
      point.velocity = max_velocity - deceleration * decel_t;
      point.position = accel_distance + max_velocity * cruise_time +
                       max_velocity * decel_t -
                       0.5f * deceleration * decel_t * decel_t;
    }

    return point;
  }

  second_t motion_total_time() noexcept { return this->total_time; }

private:
  position_t target;
  velocity_t max_velocity;
  acceleration_t acceleration;
  acceleration_t deceleration;

  second_t accel_time;
  second_t decel_time;
  position_t accel_distance;
  position_t decel_distance;
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