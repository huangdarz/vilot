#include "vilot/chassis.h"
#include "pros/rtos.hpp"
#include "units.h"
#include <algorithm>
#include <variant>

#define MAX_VOLTAGE_MV 12000.f

using namespace units::voltage;
using namespace units::angular_velocity;
using namespace units::angle;
using namespace units::length;

namespace vilot::chassis {

void Differential::move(const millivolt_t left,
                        const millivolt_t right) noexcept {
  this->left.move_voltage(std::clamp(left(), -MAX_VOLTAGE_MV, MAX_VOLTAGE_MV));
  this->right.move_voltage(
      std::clamp(right(), -MAX_VOLTAGE_MV, MAX_VOLTAGE_MV));
}

void Differential::move(const revolutions_per_minute_t left,
                        const revolutions_per_minute_t right) noexcept {
  this->left.move_velocity(left() * this->gear_ratio_in_out);
  this->right.move_velocity(right() * this->gear_ratio_in_out);
}

} // namespace vilot::chassis

namespace vilot::drivetrain {

void Differential::move(const millivolt_t left,
                        const millivolt_t right) noexcept {
  this->mut.lock();
  this->chassis.move(left, right);
  this->mut.unlock();
}

void Differential::move(const revolutions_per_minute_t left,
                        const revolutions_per_minute_t right) noexcept {
  this->mut.lock();
  this->chassis.move(left, right);
  this->mut.unlock();
}

void Differential::move(meters_per_second_t forward,
                        radians_per_second_t rotation) noexcept {
  this->mut.lock();
  this->__move(forward, rotation);
  this->mut.unlock();
}

void Differential::__move(meters_per_second_t forward,
                          radians_per_second_t rotation) noexcept {
  float left_vel = forward() + (rotation() * this->track_width());
  float right_vel = forward() - (rotation() * this->track_width());

  revolutions_per_minute_t left_rot = revolutions_per_minute_t(
      units::convert<radians_per_second, revolutions_per_minute>(
          left_vel / this->wheel_radius()));
  revolutions_per_minute_t right_rot = revolutions_per_minute_t(
      units::convert<radians_per_second, revolutions_per_minute>(
          right_vel / this->wheel_radius()));

  this->chassis.move(left_rot, right_rot);
}

void Differential::forward_towards(meter_t forward) noexcept {
  // local frame go to "forward"
  this->targets.emplace(forward);
  this->task.notify();
}

void Differential::rotate_towards(radian_t rotation) noexcept {
  // local frame go to "rotation"
  this->targets.emplace(rotation);
  this->task.notify();
}

void Differential::wait_until_finished(millisecond_t timeout) noexcept {
  while (!this->targets.empty()) {
    pros::Task::delay(10);
  }
  // timeout
}

void Differential::update() {
  for (;;) {
    pros::Task::notify_take(false, TIMEOUT_MAX);

    if (this->targets.empty()) {
      continue;
    }
    auto target = this->targets.front();
    this->targets.pop();

    this->mut.lock();

    Visitor visitor = {
        [this](meter_t forward) {
          bool exit = false;
          auto start = pros::millis();
          while (!exit) {
            // exit when aborted
            auto pose = this->odometry->get_pose();
            Pose2d current = pose;
            Pose2d target = {current.as_vec() +
                                 (current.as_vec().normalized() * forward()),
                             pose.theta};
            auto [vel, settled] =
                this->controller.calculate_forward(current, target);
            auto out = meters_per_second_t(
                decay_filter.apply(vel.x(), units::time::millisecond_t(10)));
            this->__move(out, radians_per_second_t(0));
            exit = settled;
            pros::Task::delay_until(&start, 10);
          }
        },
        [this](radian_t rotation) {
          bool exit = false;
          auto start = pros::millis();
          while (!exit) {
            // exit when aborted
            auto pose = this->odometry->get_pose();
            Pose2d current = pose;
            Pose2d target = {pose.as_vec(), pose.theta + rotation};
            auto [vel, settled] =
                this->controller.calculate_rotation(current, target);
            auto out = radians_per_second_t(decay_filter.apply(
                vel.theta(), units::time::millisecond_t(10)));
            this->__move(meters_per_second_t(0), out);
            exit = settled;
            pros::Task::delay_until(&start, 10);
          }
        },
    };
    std::visit(visitor, target);

    this->mut.unlock();
    pros::Task::delay(10);
  }
}

} // namespace vilot::drivetrain