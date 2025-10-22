#include "voyage/motionprofile.hpp"

namespace voyage {

using namespace units::literals;
using second_t = units::time::second_t;
using meter_t = units::length::meter_t;
using meters_per_second_t = units::velocity::meters_per_second_t;
using meters_per_second_squared_t =
    units::acceleration::meters_per_second_squared_t;

TrapezoidalMotionProfile::TrapezoidalMotionProfile(
    meter_t target, meters_per_second_t max_velocity,
    meters_per_second_squared_t acceleration,
    meters_per_second_squared_t deceleration) noexcept
    : target(target), max_velocity(max_velocity), acceleration(acceleration),
      deceleration(deceleration) {
  this->accel_time = this->max_velocity / this->acceleration;
  this->decel_time = this->max_velocity / this->deceleration;
  this->accel_distance =
      0.5f * this->acceleration * this->accel_time * this->accel_time;
  this->decel_distance =
      0.5f * this->deceleration * this->decel_time * this->decel_time;
  meter_t min_dist_max_vel = accel_distance + decel_distance;

  if (target >= min_dist_max_vel) {
    meter_t cruise_distance =
        target - this->accel_distance - this->decel_distance;
    this->cruise_time = cruise_distance / this->max_velocity;
  } else {
    meters_per_second_t peak_velocity = units::math::sqrt(
        2.0f * this->target * this->acceleration * this->deceleration /
        (this->acceleration + this->deceleration));
    this->accel_time = peak_velocity / this->acceleration;
    this->decel_time = peak_velocity / this->deceleration;
    this->accel_distance =
        0.5f * this->acceleration * this->accel_time * this->accel_time;
    this->decel_distance =
        0.5f * this->deceleration * this->decel_time * this->decel_time;
    this->max_velocity = peak_velocity;
    this->cruise_time = 0.0_s;
  }

  this->total_time = accel_time + cruise_time + decel_time;
}

TrapezoidalMotionProfile::ProfilePoint
TrapezoidalMotionProfile::sample(second_t t) noexcept {
  ProfilePoint point;
  point.t = t;

  // Clamp time to valid range
  if (t <= 0.0_s) {
    point.position = 0.0_m;
    point.velocity = 0.0_mps;
    point.acceleration = 0.0_mps_sq;
    return point;
  }

  if (t >= total_time) {
    point.position = target;
    point.velocity = 0.0_mps;
    point.acceleration = 0.0_mps_sq;
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
    point.acceleration = 0.0_mps_sq;
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

second_t TrapezoidalMotionProfile::motion_total_time() noexcept {
  return this->total_time;
}

} // namespace voyage