#pragma once
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include "units.h"
#include "vilot/filter.h"
#include "vilot/odometry.h"
#include "vilot/pid.h"
#include <queue>
#include <utility>
#include <variant>

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

  void move(const millivolt_t left, const millivolt_t right) noexcept;
  void move(const revolutions_per_minute_t left,
            const revolutions_per_minute_t right) noexcept;

  pros::MotorGroup left;
  pros::MotorGroup right;
  float gear_ratio_in_out;
};

} // namespace vilot::chassis

namespace vilot::drivetrain {

template <typename... Callable> struct Visitor : Callable... {
  using Callable::operator()...;
};

struct DifferentialController {
  using meter_t = units::length::meter_t;
  using millimeter_t = units::length::millimeter_t;
  using radian_t = units::angle::radian_t;
  using meters_per_second_t = units::velocity::meters_per_second_t;
  using radians_per_second_t = units::angular_velocity::radians_per_second_t;

  std::pair<Twist2d, bool> calculate_rotation(Pose2d current, Pose2d target) {
    // TODO settled condition
    float out = rotation.calculate(current.theta(), target.theta());
    return std::pair<Twist2d, bool>{
        {Eigen::Vector2f::Zero(), radians_per_second_t(out)}, false};
  }

  std::pair<Twist2d, bool> calculate_forward(Pose2d current, Pose2d target) {
    // TODO settled condition
    auto current_dist = current.as_vec().norm();
    auto target_dist = target.as_vec().norm();
    float for_out = forward.calculate(current_dist, target_dist);
    return std::pair<Twist2d, bool>{{meters_per_second_t(for_out),
                                     meters_per_second_t(0),
                                     radians_per_second_t(0)},
                                    false};
  }

  PidController forward;
  PidController rotation;
};

class Differential {
  using meter_t = units::length::meter_t;
  using radian_t = units::angle::radian_t;
  using meters_per_second_t = units::velocity::meters_per_second_t;
  using radians_per_second_t = units::angular_velocity::radians_per_second_t;
  using millivolt_t = units::voltage::millivolt_t;
  using revolutions_per_minute_t =
      units::angular_velocity::revolutions_per_minute_t;
  using millisecond_t = units::time::millisecond_t;

public:
  Differential() = delete;
  Differential(const Differential &) = delete;
  Differential &operator=(const Differential &) = delete;
  Differential(Differential &&) = delete;
  Differential &operator=(Differential &&) = delete;

  template <typename... Args>
  Differential(meter_t track_width, meter_t wheel_radius,
               DifferentialController controller, device::Odometry *odometry,
               float decay, Args &&...chassis_args)
      : track_width(track_width), wheel_radius(wheel_radius),
        controller(controller), chassis(std::forward<Args>(chassis_args)...),
        odometry(odometry), decay_filter(decay), task(pros::Task::current()) {
    this->task = pros::Task([this]() { this->update(); });
  }

  void move(const millivolt_t left, const millivolt_t right) noexcept;
  void move(const revolutions_per_minute_t left,
            const revolutions_per_minute_t right) noexcept;
  void move(meters_per_second_t forward,
            radians_per_second_t rotation) noexcept;

  void forward_towards(meter_t forward) noexcept;
  void rotate_towards(radian_t rotation) noexcept;
  void wait_until_finished(millisecond_t timeout) noexcept;

  chassis::Differential chassis;
  meter_t track_width;
  meter_t wheel_radius;
  DifferentialController controller;
  device::Odometry *odometry;
  ExpDecayFilter decay_filter;

private:
  void __move(meters_per_second_t forward,
              radians_per_second_t rotation) noexcept;
  void update();

  pros::Task task;
  pros::Mutex mut;
  std::queue<std::variant<meter_t, radian_t>> targets; // need lock?
  // std::queue<Eigen::Vector2<meter_t>> targets;
};

} // namespace vilot::drivetrain