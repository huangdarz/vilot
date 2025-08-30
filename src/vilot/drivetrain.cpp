#include "vilot/drivetrain.hpp"
#include <algorithm>
#include <numbers>

namespace vilot {

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