#include "vilot/drivetrain.hpp"
#include "pid.h"
#include "pros/rtos.hpp"
#include "ramsete.hpp"
#include "units.h"
#include "voyage/motionprofile.hpp"
#include <algorithm>
#include <numbers>

namespace vilot {

using namespace units::literals;
using namespace units::angular_velocity;

// #define RPS_TO_RPM 9.5492

void DifferentialDrivetrain::move(meters_per_second_t x,
                                  radians_per_second_t theta) {
  float left_vel = x() / (this->wheel_radius() * 2 * std::numbers::pi) +
                   (theta() * this->track_width() / 2.0f);
  float right_vel = x() / (this->wheel_radius() * 2 * std::numbers::pi) -
                    (theta() * this->track_width() / 2.0f);

  revolutions_per_minute_t left_rot = radians_per_second_t(left_vel);
  revolutions_per_minute_t right_rot = radians_per_second_t(right_vel);

  // revolutions_per_minute_t left_rot = revolutions_per_minute_t(
  //     units::convert<radians_per_second, revolutions_per_minute>(left_vel *
  //                                                                RPS_TO_RPM));
  // revolutions_per_minute_t right_rot = revolutions_per_minute_t(
  //     units::convert<radians_per_second, revolutions_per_minute>(right_vel *
  //                                                                RPS_TO_RPM));

  this->chassis.left.move_velocity(left_rot() *
                                   this->chassis.gear_ratio_in_out);
  this->chassis.right.move_velocity(right_rot() *
                                    this->chassis.gear_ratio_in_out);
}

void DifferentialDrivetrain::move(millivolt_t forward, millivolt_t turn) {
  this->tank(forward + turn, forward - turn);
}

// TODO add reverse
void DifferentialDrivetrain::follow(meter_t distance,
                                    meters_per_second_t max_velocity,
                                    meters_per_second_squared_t acceleration,
                                    meters_per_second_squared_t deceleration,
                                    float follow_strength,
                                    float follow_dampen) {
  auto state = this->odometry.get_state();
  const auto start_state = state;
  auto motion = voyage::TrapezoidalMotionProfile<units::length::meter>(
      distance, max_velocity, acceleration, deceleration);
  auto controller = vilot::RamseteController(follow_strength, follow_dampen);

  units::time::millisecond_t total_ms = motion.motion_total_time();
  units::time::millisecond_t step_size_period = 10_ms;
  int iterations = total_ms / step_size_period;

  for (int i = 0; i < iterations; i++) {
    auto t =
        static_cast<float>(i) * motion.motion_total_time() / (iterations - 1);
    auto pp = motion.sample(units::time::millisecond_t(t));
    auto [lin, ang] =
        controller.calculate(state.pose,
                             {start_state.pose.x + pp.position,
                              start_state.pose.y, start_state.pose.theta},
                             pp.velocity, 0_rps);
    this->move(units::velocity::meters_per_second_t(lin),
               units::angular_velocity::radians_per_second_t(ang));
    state = this->odometry.get_state();
    pros::Task::delay(10);
  }
  this->stop();
}

// TODO add slew rate limiter
void DifferentialDrivetrain::rotate_to(degree_t target,
                                       NonLinearPidConstants constants,
                                       degree_t tolerance,
                                       millisecond_t timeout) {
  NonlinearPidController controller(constants);
  controller.set_continuous_input(true);
  controller.set_max_input(180);
  controller.set_min_input(-180);
  controller.set_abs_max_output(MAX_VOLTAGE_MV);
  auto state = this->odometry.get_state();
  auto start_time = pros::millis();
  while (units::math::abs(state.pose.theta - target) > tolerance &&
         pros::millis() - start_time < timeout()) {
    auto ang = controller.calculate(
        state.pose.theta.convert<units::angle::degrees>()(), target());
    this->move(0_MV, millivolt_t(ang));
    state = this->odometry.get_state();
    pros::Task::delay(10);
  }
  this->stop();
}

void DifferentialDrivetrain::rotate_for(degree_t amount,
                                        NonLinearPidConstants constants,
                                        degree_t tolerance,
                                        millisecond_t timeout) {
  auto state = this->odometry.get_state();
  this->rotate_to(state.pose.theta + amount, constants, tolerance, timeout);
}

void DifferentialDrivetrain::tank(millivolt_t left, millivolt_t right) {
  this->chassis.left.move(std::clamp(left(), -MAX_VOLTAGE_MV, MAX_VOLTAGE_MV));
  this->chassis.right.move(
      std::clamp(right(), -MAX_VOLTAGE_MV, MAX_VOLTAGE_MV));
}

void DifferentialDrivetrain::stop() {
  this->chassis.left.brake();
  this->chassis.right.brake();
}

} // namespace vilot