#pragma once
#include "units.h"
#include "voyage/gcem/gcem.hpp"  // IWYU pragma: keep

namespace voyage {

class TrapezoidalMotionProfile {
  using second_t = units::time::second_t;
  using meter_t = units::length::meter_t;
  using meters_per_second_t = units::velocity::meters_per_second_t;
  using meters_per_second_squared_t =
      units::acceleration::meters_per_second_squared_t;

 public:
  struct Constraints {
    meters_per_second_t max_velocity;
    meters_per_second_squared_t acceleration;
    meters_per_second_squared_t deceleration;
  };

  struct ProfilePoint {
    second_t t;
    meter_t position;
    meters_per_second_t velocity;
    meters_per_second_squared_t acceleration;
  };

  TrapezoidalMotionProfile(meter_t target, meters_per_second_t max_velocity,
                           meters_per_second_squared_t acceleration,
                           meters_per_second_squared_t deceleration) noexcept;

  TrapezoidalMotionProfile(meter_t target,
                           const Constraints& constraints) noexcept;

  [[nodiscard]] ProfilePoint sample(second_t t) const noexcept;

  [[nodiscard]] second_t motion_total_time() const noexcept;

  [[nodiscard]] size_t iterations_from_step(second_t step_size) const noexcept;

  template <typename Func>
  void step_iterate(const second_t step_size, Func&& callback) const noexcept {
    const size_t iterations = this->iterations_from_step(step_size);
    for (size_t i = 0; i <= iterations; i++) {
      const auto t = second_t(static_cast<float>(i) * step_size());
      const auto point = sample(t);
      callback(point, t);
    }
  }

 private:
  meter_t target;
  meters_per_second_t max_velocity;
  meters_per_second_squared_t acceleration;
  meters_per_second_squared_t deceleration;

  second_t accel_time{};
  second_t decel_time{};
  meter_t accel_distance{};
  meter_t decel_distance{};
  second_t cruise_time{};
  second_t total_time{};
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

}  // namespace voyage