#pragma once
#include "voyage/gcem/gcem.hpp" // IWYU pragma: keep

namespace voyage {

class TrapezoidalMotionProfile {
public:
  struct ProfilePoint {
    float t;
    float position;
    float velocity;
    float acceleration;
  };

  constexpr TrapezoidalMotionProfile(float target, float max_velocity,
                                     float acceleration,
                                     float deceleration) noexcept
      : target(target), max_velocity(max_velocity), acceleration(acceleration),
        deceleration(deceleration) {
    this->accel_time = this->max_velocity / this->acceleration;
    this->decel_time = this->max_velocity / this->deceleration;
    this->accel_distance =
        0.5f * this->acceleration * this->accel_time * this->accel_time;
    this->decel_distance =
        0.5f * this->deceleration * this->decel_time * this->decel_time;
    float min_dist_max_vel = accel_distance + decel_distance;

    if (target >= min_dist_max_vel) {
      float cruise_distance =
          target - this->accel_distance - this->decel_distance;
      this->cruise_time = cruise_distance / this->max_velocity;
    } else {
      float peak_velocity = gcem::sqrt(
          2.0f * this->target * this->acceleration * this->deceleration /
          (this->acceleration + this->deceleration));
      this->accel_time = peak_velocity / this->acceleration;
      this->decel_time = peak_velocity / this->deceleration;
      this->accel_distance =
          0.5f * this->acceleration * this->accel_time * this->accel_time;
      this->decel_distance =
          0.5f * this->deceleration * this->decel_time * this->decel_time;
      this->max_velocity = peak_velocity;
      this->cruise_time = 0.0f;
    }

    this->total_time = accel_time + cruise_time + decel_time;
  }

  constexpr ProfilePoint sample(float t) noexcept {
    ProfilePoint point;
    point.t = t;

    // Clamp time to valid range
    if (t <= 0.0f) {
      point.position = 0.0f;
      point.velocity = 0.0f;
      point.acceleration = 0.0f;
      return point;
    }

    if (t >= total_time) {
      point.position = target;
      point.velocity = 0.0f;
      point.acceleration = 0.0f;
      return point;
    }

    if (t <= accel_time) {
      // Acceleration phase
      point.acceleration = acceleration;
      point.velocity = acceleration * t;
      point.position = 0.5f * acceleration * t * t;
    } else if (t <= accel_time + cruise_time) {
      // Cruise phase
      float cruise_t = t - accel_time;
      point.acceleration = 0.0f;
      point.velocity = max_velocity;
      point.position = accel_distance + max_velocity * cruise_t;
    } else {
      // Deceleration phase
      float decel_t = t - accel_time - cruise_time;
      point.acceleration = -deceleration;
      point.velocity = max_velocity - deceleration * decel_t;
      point.position = accel_distance + max_velocity * cruise_time +
                       max_velocity * decel_t -
                       0.5f * deceleration * decel_t * decel_t;
    }

    return point;
  }

  constexpr float motion_total_time() noexcept { return this->total_time; }

private:
  float target;
  float max_velocity;
  float acceleration;
  float deceleration;

  float accel_time;
  float decel_time;
  float accel_distance;
  float decel_distance;
  float cruise_time;
  float total_time;
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