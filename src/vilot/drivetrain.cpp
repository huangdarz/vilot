#include "vilot/drivetrain.hpp"
#include "pid.h"
#include "pros/rtos.hpp"
#include "ramsete.hpp"
#include "units.h"
#include "voyage/motionprofile.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace vilot {

using namespace units::literals;
using namespace units::angular_velocity;

// #define RPS_TO_RPM 9.5492

void DifferentialDrivetrain::move(meters_per_second_t x,
                                  radians_per_second_t theta) {
  // float left_vel = x() / (this->wheel_radius() * 2 * std::numbers::pi) * 60.0
  // +
  //                  (theta() * this->track_width() / 2.0f);
  // float right_vel = x() / (this->wheel_radius() * 2 * std::numbers::pi)
  // * 60.0 -
  //                   (theta() * this->track_width() / 2.0f);
  //
  // revolutions_per_minute_t left_rot = revolutions_per_minute_t(left_vel);
  // revolutions_per_minute_t right_rot = revolutions_per_minute_t(right_vel);

  float linear = (x() * this->chassis.gear_ratio_in_out * 60.0) /
                 (std::numbers::pi * 2 * this->wheel_radius());
  float rotate =
      (theta() * this->track_width() * this->chassis.gear_ratio_in_out * 60.0) /
      (std::numbers::pi * 2 * this->wheel_radius());

  // revolutions_per_minute_t left_rot = revolutions_per_minute_t(
  //     units::convert<radians_per_second, revolutions_per_minute>(left_vel *
  //                                                                RPS_TO_RPM));
  // revolutions_per_minute_t right_rot = revolutions_per_minute_t(
  //     units::convert<radians_per_second, revolutions_per_minute>(right_vel *
  //                                                                RPS_TO_RPM));

  this->chassis.left.move_velocity(linear + rotate);
  this->chassis.right.move_velocity(linear - rotate);
}

void DifferentialDrivetrain::move(millivolt_t forward, millivolt_t turn) {
  this->tank(forward + turn, forward - turn);
}

void DifferentialDrivetrain::follow(meter_t distance,
                                    meters_per_second_t max_velocity,
                                    meters_per_second_squared_t acceleration,
                                    meters_per_second_squared_t deceleration,
                                    float follow_strength,
                                    float follow_dampen) {
  assert(max_velocity() > 0 && "Max velocity must be positive");
  assert(acceleration() > 0 && "Acceleration must be positive");
  assert(deceleration() > 0 && "Deceleration must be positive");
  assert(follow_strength > 0 && "Follow strength must be positive");
  assert(follow_dampen > 0 && "Follow dampen must be positive");

  using namespace units::math;

  auto state = this->odometry.get_state();
  const auto start_state = state;
  auto motion = voyage::TrapezoidalMotionProfile(abs(distance), max_velocity,
                                                 acceleration, deceleration);
  auto controller = vilot::RamseteController(follow_strength, follow_dampen);

  units::time::millisecond_t total_ms = motion.motion_total_time();
  units::time::millisecond_t step_size_period = 10_ms;
  int iterations = total_ms / step_size_period;

  for (int i = 0; i < iterations; i++) {
    auto t =
        static_cast<float>(i) * motion.motion_total_time() / (iterations - 1);
    auto pp = motion.sample(units::time::millisecond_t(t));
    auto [lin, ang] = controller.calculate(
        state.pose,
        {start_state.pose.x + meter_t(std::copysign(pp.position(), distance())),
         start_state.pose.y, start_state.pose.theta},
        meters_per_second_t(std::copysign(pp.velocity(), distance())), 0_rps);
    this->move(lin, ang);
    state = this->odometry.get_state();
    pros::Task::delay(10);
  }
  this->stop();
}

bool DifferentialDrivetrain::rotate_to(degree_t target, PidConstants constants,
                                       degree_t tolerance,
                                       millisecond_t timeout) {
  // TODO check if need minimum output speed to turn robot

  using namespace units::math;

  vilot::ExpDecayFilter decay(1);

  PidController controller(constants);
  controller.set_continuous_input(true);
  controller.set_max_input(180);
  controller.set_min_input(-180);
  controller.set_abs_max_output(MAX_VOLTAGE_MV);

  auto state = this->odometry.get_state();
  auto start_time = pros::millis();
  while (pros::millis() - start_time < timeout() &&
         abs(target - state.pose.theta) > tolerance) {
    auto ang =
        controller.calculate(this->odometry.get_state()
                                 .pose.theta.convert<units::angle::degrees>()(),
                             target());
    float turn = decay.apply(degrees_per_second_t(ang)(),
                             units::time::millisecond_t(10));
    this->move(0_mps, degrees_per_second_t(-turn));
    state = this->odometry.get_state();
    pros::Task::delay(10);
  }
  this->stop();

  return abs(target - state.pose.theta) <= tolerance;
}

void DifferentialDrivetrain::tank(millivolt_t left, millivolt_t right) {
  this->chassis.left.move(std::clamp(left(), -MAX_VOLTAGE_MV, MAX_VOLTAGE_MV));
  this->chassis.right.move(
      std::clamp(right(), -MAX_VOLTAGE_MV, MAX_VOLTAGE_MV));
}

void DifferentialDrivetrain::stop(const pros::motor_brake_mode_e_t mode) {
  auto curr_mode_left = this->chassis.left.get_brake_mode();
  auto curr_mode_right = this->chassis.right.get_brake_mode();
  this->chassis.left.set_brake_mode_all(mode);
  this->chassis.right.set_brake_mode_all(mode);
  this->chassis.left.brake();
  this->chassis.right.brake();
  this->chassis.left.set_brake_mode_all(curr_mode_left);
  this->chassis.right.set_brake_mode_all(curr_mode_right);
}

} // namespace vilot
